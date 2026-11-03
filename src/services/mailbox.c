#include "services/mailbox.h"

#include "hal/hsem.h"
#include "platform_e2e.h"
#include "platform_mailbox.h"

/* A bike image needs the board-confirmed region address (D-093 item 3). */
#if defined(VEHICLE_SAFETY_BIKE_IMAGE) && (PLATFORM_MAILBOX_ADDR_VERIFIED == 0)
#error "the mailbox region address is not verified on the board (D-093 item 3)"
#endif

_Static_assert(PLATFORM_MAILBOX_SLOT_COUNT == 6u, "k_slots below lists every gen/ slot: update it with the layout");

/* Slot table, in slot order (checked in mailbox_init()). Everything comes from gen/: the
 * index, HSEM, length and direction of each slot and the gen/ E2E timeout of its message
 * (used for the stale flag of the M7->M4 slots). */
typedef struct {
    uint8_t index;
    uint8_t hsem_id;
    uint8_t length;
    uint8_t direction;
    uint8_t cls;
    uint32_t timeout_ms;
} mbx_slot_def_t;

static const mbx_slot_def_t k_slots[PLATFORM_MAILBOX_SLOT_COUNT] = {
    {(uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_INDEX, (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_HSEM_ID,
     (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_LENGTH, (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_DIRECTION,
     (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_CLASS, PLATFORM_EKF_LEAN_E2E_TIMEOUT_MS},
    {(uint8_t)PLATFORM_MAILBOX_SLOT_EKF_FRICTION_MASS_INDEX, (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_FRICTION_MASS_HSEM_ID,
     (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_FRICTION_MASS_LENGTH,
     (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_FRICTION_MASS_DIRECTION,
     (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_FRICTION_MASS_CLASS, PLATFORM_EKF_FRICTION_MASS_E2E_TIMEOUT_MS},
    {(uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_INDEX, (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_HSEM_ID,
     (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_LENGTH,
     (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_DIRECTION,
     (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_CLASS, PLATFORM_HEARTBEAT_RT_CORE_E2E_TIMEOUT_MS},
    {(uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_REQUEST_INDEX, (uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_REQUEST_HSEM_ID,
     (uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_REQUEST_LENGTH, (uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_REQUEST_DIRECTION,
     (uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_REQUEST_CLASS, PLATFORM_SAFETY_REQUEST_E2E_TIMEOUT_MS},
    {(uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_INDEX,
     (uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_HSEM_ID,
     (uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_LENGTH,
     (uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_DIRECTION,
     (uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_CLASS, PLATFORM_SAFETY_WARNING_STATUS_E2E_TIMEOUT_MS},
    {(uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_INDEX, (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_HSEM_ID,
     (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_LENGTH,
     (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_DIRECTION,
     (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_CLASS, PLATFORM_HEARTBEAT_SAFETY_E2E_TIMEOUT_MS},
};

/* Grace time for the M7 layout word: the shortest E2E timeout of a safety M7->M4 message
 * (0x020). A mismatch that outlasts it is a wrong image pair, not a wiped word that the
 * M7 rewrites within its next pass. */
#define MBX_LAYOUT_GRACE_MS PLATFORM_EKF_LEAN_E2E_TIMEOUT_MS

/* The newest frame read from an M7->M4 slot and not yet taken. */
typedef struct {
    uint32_t data[MAILBOX_PAYLOAD_WORDS];
    bool fresh;
} mbx_rx_buf_t;

typedef struct {
    volatile mailbox_region_t* region;
    mailbox_reader_t reader[PLATFORM_MAILBOX_SLOT_COUNT];
    mailbox_pending_t pending[PLATFORM_MAILBOX_SLOT_COUNT];
    mbx_rx_buf_t rx[PLATFORM_MAILBOX_SLOT_COUNT];
    bool peer_seen;           /* the M7 word matched at least once since init */
    uint32_t layout_ok_at_ms; /* last pass the M7 word matched */
    uint32_t read_frames;
    uint32_t take_overruns;
    uint32_t skipped_steps;
} mbx_state_t;

static mbx_state_t s_mbx;

static void mbx_sat_inc(uint32_t* counter)
{
    if (*counter < UINT32_MAX) {
        (*counter)++;
    }
}

static uint32_t mbx_sat_add(uint32_t a, uint32_t b)
{
    return ((UINT32_MAX - a) < b) ? UINT32_MAX : (a + b);
}

static bool mbx_is_rx(uint8_t slot)
{
    return (slot < PLATFORM_MAILBOX_SLOT_COUNT) && (k_slots[slot].direction == PLATFORM_MAILBOX_DIR_M7_TO_M4);
}

static bool mbx_is_tx(uint8_t slot)
{
    return (slot < PLATFORM_MAILBOX_SLOT_COUNT) && (k_slots[slot].direction == PLATFORM_MAILBOX_DIR_M4_TO_M7);
}

static volatile uint32_t* mbx_slot_words(uint8_t slot)
{
    const uint32_t first = (PLATFORM_MAILBOX_HEADER_BYTES / MAILBOX_WORD_BYTES) + ((uint32_t)slot * MAILBOX_SLOT_WORDS);
    return &s_mbx.region->word[first];
}

static bool mbx_table_ok(void)
{
    for (uint32_t i = 0u; i < PLATFORM_MAILBOX_SLOT_COUNT; i++) {
        const mbx_slot_def_t* d = &k_slots[i];
        const bool dir_ok = (d->direction == PLATFORM_MAILBOX_DIR_M7_TO_M4) ||
                            (d->direction == PLATFORM_MAILBOX_DIR_M4_TO_M7);
        const bool cls_ok = (d->cls == PLATFORM_MAILBOX_CLASS_SAFETY) || (d->cls == PLATFORM_MAILBOX_CLASS_QM);
        if ((d->index != i) || (d->hsem_id < HAL_HSEM_ID_MIN) || (d->hsem_id > HAL_HSEM_ID_MAX) ||
            (d->length == 0u) || (d->length > MAILBOX_PAYLOAD_BYTES) || !dir_ok || !cls_ok) {
            return false;
        }
    }
    return true;
}

bool mailbox_init(volatile mailbox_region_t* region, uint32_t now_ms)
{
    s_mbx.region = NULL;
    for (uint32_t i = 0u; i < PLATFORM_MAILBOX_SLOT_COUNT; i++) {
        mailbox_reader_init(&s_mbx.reader[i], k_slots[i].length);
        mailbox_pending_init(&s_mbx.pending[i]);
        s_mbx.rx[i].data[0] = 0u;
        s_mbx.rx[i].data[1] = 0u;
        s_mbx.rx[i].fresh = false;
    }
    s_mbx.peer_seen = false;
    s_mbx.layout_ok_at_ms = now_ms;
    s_mbx.read_frames = 0u;
    s_mbx.take_overruns = 0u;
    s_mbx.skipped_steps = 0u;
    if ((region == NULL) || !mbx_table_ok()) {
        return false;
    }
    for (uint32_t i = 0u; i < PLATFORM_MAILBOX_SLOT_COUNT; i++) {
        hal_hsem_release_own(k_slots[i].hsem_id); /* a lock left from before an M4 reset */
    }
    s_mbx.region = region;
    s_mbx.region->word[MAILBOX_M4_LAYOUT_WORD] = PLATFORM_MAILBOX_LAYOUT_ID;
    return true;
}

bool mailbox_offer(uint8_t slot, const uint8_t* data, uint8_t len)
{
    if ((s_mbx.region == NULL) || !mbx_is_tx(slot) || (data == NULL) || (len != k_slots[slot].length)) {
        return false;
    }
    return mailbox_pending_offer(&s_mbx.pending[slot], data, len);
}

static void mbx_write(uint8_t slot, bool peer_layout_ok)
{
    mailbox_pending_t* p = &s_mbx.pending[slot];
    if (!p->pending) {
        return;
    }
    if (!peer_layout_ok) {
        mailbox_pending_defer(p); /* never write into a peer with another layout */
        return;
    }
    hal_hsem_key_t key;
    if (hal_hsem_take(k_slots[slot].hsem_id, &key)) {
        volatile uint32_t* words = mbx_slot_words(slot);
        mailbox_slot_store(p, words);
        hal_hsem_barrier();
        mailbox_slot_commit(words);
        hal_hsem_release(k_slots[slot].hsem_id, &key);
        mailbox_pending_done(p);
    } else {
        mailbox_pending_defer(p); /* next pass; never a wait or a retry loop here */
    }
}

static void mbx_read(uint8_t slot, bool peer_layout_ok, uint32_t now_ms)
{
    mailbox_raw_t raw;
    hal_hsem_key_t key;
    if (!hal_hsem_take(k_slots[slot].hsem_id, &key)) {
        mailbox_reader_busy(&s_mbx.reader[slot]);
        return;
    }
    mailbox_slot_load(mbx_slot_words(slot), &raw);
    hal_hsem_release(k_slots[slot].hsem_id, &key);
    if (mailbox_reader_accept(&s_mbx.reader[slot], &raw, peer_layout_ok, now_ms) == MAILBOX_READ_NEW) {
        mbx_rx_buf_t* b = &s_mbx.rx[slot];
        if (b->fresh) {
            mbx_sat_inc(&s_mbx.take_overruns); /* the consumer skipped a frame: its E2E sees the gap */
        }
        b->data[0] = raw.data[0];
        b->data[1] = raw.data[1];
        b->fresh = true;
        mbx_sat_inc(&s_mbx.read_frames);
    }
}

void mailbox_step(uint32_t now_ms)
{
    if (s_mbx.region == NULL) {
        mbx_sat_inc(&s_mbx.skipped_steps);
        return;
    }
    /* Own word every pass: a wiped word recovers within one pass (D-093 item 5). */
    s_mbx.region->word[MAILBOX_M4_LAYOUT_WORD] = PLATFORM_MAILBOX_LAYOUT_ID;
    const bool peer_layout_ok = (s_mbx.region->word[MAILBOX_M7_LAYOUT_WORD] == PLATFORM_MAILBOX_LAYOUT_ID);
    if (peer_layout_ok) {
        s_mbx.peer_seen = true;
        s_mbx.layout_ok_at_ms = now_ms;
    }
    for (uint8_t i = 0u; i < (uint8_t)PLATFORM_MAILBOX_SLOT_COUNT; i++) {
        if (mbx_is_tx(i)) {
            mbx_write(i, peer_layout_ok);
        } else {
            mbx_read(i, peer_layout_ok, now_ms);
        }
    }
}

bool mailbox_take(uint8_t slot, uint8_t out[MAILBOX_PAYLOAD_BYTES])
{
    if ((s_mbx.region == NULL) || !mbx_is_rx(slot) || (out == NULL) || !s_mbx.rx[slot].fresh) {
        return false;
    }
    mailbox_payload_bytes(s_mbx.rx[slot].data, out);
    s_mbx.rx[slot].fresh = false;
    return true;
}

mailbox_health_t mailbox_health(uint32_t now_ms)
{
    mailbox_health_t h = {0};
    h.initialised = (s_mbx.region != NULL);
    h.skipped_steps = s_mbx.skipped_steps;
    if (!h.initialised) {
        return h;
    }
    h.peer_wait_ms = (uint32_t)(now_ms - s_mbx.layout_ok_at_ms);
    if (!s_mbx.peer_seen) {
        h.peer = MAILBOX_PEER_NEVER_MATCHED;
    } else if (h.peer_wait_ms > MBX_LAYOUT_GRACE_MS) {
        h.peer = MAILBOX_PEER_MISMATCH;
    } else {
        h.peer = MAILBOX_PEER_OK;
    }
    for (uint8_t i = 0u; i < (uint8_t)PLATFORM_MAILBOX_SLOT_COUNT; i++) {
        if (mbx_is_rx(i)) {
            const mailbox_reader_t* r = &s_mbx.reader[i];
            if (mailbox_reader_stale(r, now_ms, k_slots[i].timeout_ms)) {
                if (k_slots[i].cls == PLATFORM_MAILBOX_CLASS_SAFETY) {
                    h.safety_stale_slots |= (1u << i);
                } else {
                    h.qm_stale_slots |= (1u << i);
                }
            }
            h.read_busy = mbx_sat_add(h.read_busy, r->busy);
            h.read_rejected = mbx_sat_add(h.read_rejected, mbx_sat_add(r->bad_layout, r->bad_len));
        } else {
            const mailbox_pending_t* p = &s_mbx.pending[i];
            h.write_done = mbx_sat_add(h.write_done, p->done);
            h.write_deferred = mbx_sat_add(h.write_deferred, p->deferred);
            h.write_overruns = mbx_sat_add(h.write_overruns, p->overruns);
        }
    }
    h.read_frames = s_mbx.read_frames;
    h.take_overruns = s_mbx.take_overruns;
    return h;
}
