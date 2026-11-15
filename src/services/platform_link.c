#include "services/platform_link.h"

#include "hal/can.h"
#include "platform.h"
#include "platform_e2e.h"

#include <stddef.h>
#include <string.h>

_Static_assert(PLATFORM_EKF_LEAN_LENGTH <= PLATFORM_LINK_PAYLOAD_BYTES, "frame fits the buffer");
_Static_assert(PLATFORM_EKF_FRICTION_MASS_LENGTH <= PLATFORM_LINK_PAYLOAD_BYTES, "frame fits the buffer");
_Static_assert(PLATFORM_HEARTBEAT_RT_CORE_LENGTH <= PLATFORM_LINK_PAYLOAD_BYTES, "frame fits the buffer");
_Static_assert(PLATFORM_SAFETY_REQUEST_LENGTH <= PLATFORM_LINK_PAYLOAD_BYTES, "frame fits the buffer");
_Static_assert(PLATFORM_SAFETY_WARNING_STATUS_LENGTH <= HAL_CAN_MAX_DLC, "frame fits a classic CAN frame");
_Static_assert(PLATFORM_HEARTBEAT_SAFETY_LENGTH <= HAL_CAN_MAX_DLC, "frame fits a classic CAN frame");
_Static_assert(PLATFORM_LINK_PAYLOAD_BYTES == HAL_CAN_MAX_DLC, "payload buffer = classic CAN frame");

typedef vehicle_e2e_status_t (*link_check_fn_t)(const uint8_t* data, size_t size, vehicle_e2e_rx_state_t* state,
                                                uint32_t now_ms);
typedef vehicle_e2e_status_t (*link_timeout_fn_t)(vehicle_e2e_rx_state_t* state, uint32_t now_ms);

/* One row per received message: everything comes from the generated code. */
typedef struct {
    uint32_t id;
    uint8_t length;
    link_check_fn_t check;
    link_timeout_fn_t check_timeout;
} link_rx_desc_t;

static const link_rx_desc_t k_rx[PLATFORM_LINK_RX_COUNT] = {
    [PLATFORM_LINK_RX_EKF_LEAN] = {PLATFORM_EKF_LEAN_FRAME_ID, PLATFORM_EKF_LEAN_LENGTH, platform_ekf_lean_e2e_check,
                                   platform_ekf_lean_e2e_check_timeout},
    [PLATFORM_LINK_RX_EKF_FRICTION_MASS] = {PLATFORM_EKF_FRICTION_MASS_FRAME_ID, PLATFORM_EKF_FRICTION_MASS_LENGTH,
                                            platform_ekf_friction_mass_e2e_check,
                                            platform_ekf_friction_mass_e2e_check_timeout},
    [PLATFORM_LINK_RX_HEARTBEAT_RT_CORE] = {PLATFORM_HEARTBEAT_RT_CORE_FRAME_ID, PLATFORM_HEARTBEAT_RT_CORE_LENGTH,
                                            platform_heartbeat_rt_core_e2e_check,
                                            platform_heartbeat_rt_core_e2e_check_timeout},
    [PLATFORM_LINK_RX_SAFETY_REQUEST] = {PLATFORM_SAFETY_REQUEST_FRAME_ID, PLATFORM_SAFETY_REQUEST_LENGTH,
                                         platform_safety_request_e2e_check,
                                         platform_safety_request_e2e_check_timeout},
};

typedef struct {
    uint32_t id;
    uint8_t length;
} link_tx_desc_t;

typedef struct {
    vehicle_e2e_rx_state_t e2e;
    bool pending;                        /* a frame arrived since the last take */
    vehicle_e2e_status_t pending_status; /* the first failure since the last take, else the newest */
    uint8_t payload[PLATFORM_LINK_PAYLOAD_BYTES];
} link_rx_slot_t;

typedef struct {
    bool ready;
    link_rx_slot_t slot[PLATFORM_LINK_RX_COUNT];
    platform_link_health_t health;
} link_ctx_t;

static link_ctx_t lk;

bool platform_link_init(uint32_t now_ms)
{
    (void)now_ms;
    (void)memset(&lk, 0, sizeof lk);
    for (uint32_t i = 0u; i < (uint32_t)PLATFORM_LINK_RX_COUNT; i++) {
        vehicle_e2e_rx_init(&lk.slot[i].e2e);
        lk.slot[i].pending_status = VEHICLE_E2E_NO_NEW_DATA;
        lk.health.rx[i].last_status = VEHICLE_E2E_NO_NEW_DATA;
    }
    lk.ready = hal_can_init();
    lk.health.initialised = lk.ready;
    return lk.ready;
}

/* Records the verdict of one received frame of slot i (a failure stays until the take). */
static void note_frame(uint32_t i, const uint8_t* data, uint8_t len, vehicle_e2e_status_t st, uint32_t now_ms)
{
    link_rx_slot_t* s = &lk.slot[i];
    platform_link_rx_health_t* h = &lk.health.rx[i];

    if (s->pending) {
        h->take_overruns++;
        if (vehicle_e2e_is_valid(s->pending_status)) {
            s->pending_status = st;
        }
    } else {
        s->pending = true;
        s->pending_status = st;
    }
    (void)memset(s->payload, 0, sizeof s->payload);
    (void)memcpy(s->payload, data, len);
    h->seen = true;
    h->last_rx_ms = now_ms;
    h->last_status = st;
}

static void handle_frame(const hal_can_frame_t* f, uint32_t now_ms)
{
    uint32_t i = 0u;
    bool known = false;

    if (!f->extended) {
        for (; i < (uint32_t)PLATFORM_LINK_RX_COUNT; i++) {
            if (k_rx[i].id == f->id) {
                known = true;
                break;
            }
        }
    }
    if (!known) {
        lk.health.rx_unknown++;
    } else if (f->dlc != k_rx[i].length) {
        /* Never passed to the E2E check: it would read bytes that are not there. The consumer
         * sees a failure, so a broken or foreign sender on a safety ID is never usable. */
        lk.health.rx[i].bad_dlc++;
        note_frame(i, f->data, (uint8_t)((f->dlc < k_rx[i].length) ? f->dlc : k_rx[i].length),
                   VEHICLE_E2E_BAD_ARGUMENT, now_ms);
    } else {
        vehicle_e2e_status_t st = k_rx[i].check(f->data, k_rx[i].length, &lk.slot[i].e2e, now_ms);
        lk.health.rx[i].frames++;
        if (!vehicle_e2e_is_valid(st)) {
            lk.health.rx[i].e2e_rejected++;
        }
        note_frame(i, f->data, k_rx[i].length, st, now_ms);
    }
}

void platform_link_step(uint32_t now_ms)
{
    hal_can_frame_t f;

    if (!lk.ready) {
        lk.health.skipped_steps++;
        return;
    }
    for (uint32_t n = 0u; n < PLATFORM_LINK_RX_BURST_MAX; n++) {
        if (!hal_can_poll(&f)) {
            break;
        }
        handle_frame(&f, now_ms);
    }
}

bool platform_link_take(platform_link_rx_t rx, uint8_t out[PLATFORM_LINK_PAYLOAD_BYTES], vehicle_e2e_status_t* status)
{
    bool taken = false;
    if (lk.ready && ((uint32_t)rx < (uint32_t)PLATFORM_LINK_RX_COUNT) && (out != NULL) && (status != NULL) &&
        lk.slot[rx].pending) {
        (void)memcpy(out, lk.slot[rx].payload, PLATFORM_LINK_PAYLOAD_BYTES);
        *status = lk.slot[rx].pending_status;
        lk.slot[rx].pending = false;
        taken = true;
    }
    return taken;
}

vehicle_e2e_status_t platform_link_check_timeout(platform_link_rx_t rx, uint32_t now_ms)
{
    vehicle_e2e_status_t st = VEHICLE_E2E_NO_NEW_DATA;
    if (lk.ready && ((uint32_t)rx < (uint32_t)PLATFORM_LINK_RX_COUNT)) {
        st = k_rx[rx].check_timeout(&lk.slot[rx].e2e, now_ms);
    }
    return st;
}

bool platform_link_send(platform_link_tx_t tx, const uint8_t* data, uint8_t len)
{
    static const link_tx_desc_t k_tx[PLATFORM_LINK_TX_COUNT] = {
        [PLATFORM_LINK_TX_SAFETY_WARNING_STATUS] = {PLATFORM_SAFETY_WARNING_STATUS_FRAME_ID,
                                                    PLATFORM_SAFETY_WARNING_STATUS_LENGTH},
        [PLATFORM_LINK_TX_HEARTBEAT_SAFETY] = {PLATFORM_HEARTBEAT_SAFETY_FRAME_ID, PLATFORM_HEARTBEAT_SAFETY_LENGTH},
    };
    bool ok = false;
    hal_can_frame_t f;

    if (!lk.ready) {
        return false;
    }
    if (((uint32_t)tx >= (uint32_t)PLATFORM_LINK_TX_COUNT) || (data == NULL) || (len != k_tx[tx].length)) {
        lk.health.tx_bad_argument++;
        return false;
    }
    (void)memset(&f, 0, sizeof f);
    f.id = k_tx[tx].id;
    f.extended = false;
    f.dlc = len;
    (void)memcpy(f.data, data, len);
    ok = hal_can_send(&f);
    if (ok) {
        lk.health.tx_sent[tx]++;
    } else {
        lk.health.tx_refused[tx]++;
    }
    return ok;
}

platform_link_health_t platform_link_health(void)
{
    return lk.health;
}
