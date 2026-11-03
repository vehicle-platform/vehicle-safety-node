/*
 * Deterministic phase-locked contention test of the M4 mailbox glue with the generated E2E
 * check on every delivered frame (D-093 item 6: "no WRONG_SEQUENCE under forced
 * phase-locked contention"; safety-reviewer MAJOR-1 on safety-node PR1).
 *
 * A 1 ms simulation clock; in each ms the M7 model acts first, then the M4 pass (every
 * M4_PASS_MS, the D-093 item 5 bound: four reads per 20 ms writer cycle), then the M7's
 * poll of the M4->M7 slots. The HSEM is forced busy for exactly the M4's take on k
 * consecutive passes per writer cycle, starting at the first pass that could see a new
 * frame (the worst phase), and every phase offset between the two cores is swept.
 *
 *   incoming 0x020 (20 ms): k = 1..3 refused reads per cycle give zero WRONG_SEQUENCE and
 *     no timeout; one cycle refused on all four reads gives exactly one counted gap
 *     (one WRONG_SEQUENCE at the receiver), then the stream resyncs.
 *   outgoing 0x023 (20 ms) and 0x082 (100 ms): k = 1..3 refused writes per cycle with the
 *     M7 polling at a quarter of 0x023's cycle give zero WRONG_SEQUENCE at the M7.
 */
#include "services/mailbox.h"

#include "hal/hsem.h"
#include "hal/host/hsem_host.h"
#include "platform_e2e.h"
#include "platform_mailbox.h"
#include "vehicle_e2e.h"

#include <string.h>
#include <unity.h>

#define M4_PASS_MS 5u
#define M7_POLL_MS 5u /* a quarter of 0x023's 20 ms cycle (D-093 item 5) */
#define LEAN_CYCLE_MS 20u
#define WARN_CYCLE_MS 20u
#define HBS_CYCLE_MS 100u
#define CYCLES 50u
#define START_MS 1000u
#define HEADER_WORDS (PLATFORM_MAILBOX_HEADER_BYTES / MAILBOX_WORD_BYTES)

#define LEAN PLATFORM_MAILBOX_SLOT_EKF_LEAN_INDEX
#define LEAN_HSEM ((uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_HSEM_ID)
#define LEAN_LEN ((uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_LENGTH)
#define WARN PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_INDEX
#define WARN_HSEM ((uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_HSEM_ID)
#define WARN_LEN ((uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_LENGTH)
#define HBS PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_INDEX
#define HBS_HSEM ((uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_HSEM_ID)
#define HBS_LEN ((uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_LENGTH)

_Static_assert(PLATFORM_EKF_LEAN_E2E_MAX_DELTA_COUNTER == 1u, "a gap of one frame is WRONG_SEQUENCE");
_Static_assert((LEAN_CYCLE_MS / M4_PASS_MS) == 4u, "four M4 reads per 0x020 cycle");

static mailbox_region_t region;

typedef struct {
    uint32_t ok;
    uint32_t wrong_sequence;
    uint32_t other_bad; /* REPEATED, WRONG_CRC, NO_NEW_DATA at a take, BAD_ARGUMENT */
    uint32_t timeouts;  /* NO_NEW_DATA from the per-pass timeout check after the first frame */
} e2e_count_t;

static volatile uint32_t* slot_words(uint32_t slot)
{
    return &region.word[HEADER_WORDS + (slot * MAILBOX_SLOT_WORDS)];
}

static void count(e2e_count_t* c, vehicle_e2e_status_t st)
{
    if ((st == VEHICLE_E2E_OK) || (st == VEHICLE_E2E_INITIAL)) {
        c->ok++;
    } else if (st == VEHICLE_E2E_WRONG_SEQUENCE) {
        c->wrong_sequence++;
    } else {
        c->other_bad++;
    }
}

/* The M7's writer of one slot, as D-093 item 5 prescribes (never refused here: the
 * injection targets the M4's take only). */
static void m7_write(uint32_t slot, uint8_t hsem, const uint8_t* d, uint8_t len)
{
    mailbox_pending_t p;
    hal_hsem_key_t key;
    mailbox_pending_init(&p);
    TEST_ASSERT_TRUE(mailbox_pending_offer(&p, d, len));
    hsem_host_set_core(HSEM_HOST_CORE_M7);
    TEST_ASSERT_TRUE(hal_hsem_take(hsem, &key));
    mailbox_slot_store(&p, slot_words(slot));
    hal_hsem_barrier();
    mailbox_slot_commit(slot_words(slot));
    hal_hsem_release(hsem, &key);
    hsem_host_set_core(HSEM_HOST_CORE_M4);
}

/* The M7's reader of one M4->M7 slot (its forward path) with the receiver's E2E check. */
static void m7_poll(uint32_t slot, uint8_t hsem, mailbox_reader_t* r, uint16_t data_id, uint8_t max_delta,
                    uint32_t timeout_ms, vehicle_e2e_rx_state_t* rx, uint32_t now, e2e_count_t* c)
{
    mailbox_raw_t raw;
    hal_hsem_key_t key;
    hsem_host_set_core(HSEM_HOST_CORE_M7);
    TEST_ASSERT_TRUE(hal_hsem_take(hsem, &key));
    mailbox_slot_load(slot_words(slot), &raw);
    hal_hsem_release(hsem, &key);
    hsem_host_set_core(HSEM_HOST_CORE_M4);
    if (mailbox_reader_accept(r, &raw, true, now) == MAILBOX_READ_NEW) {
        uint8_t d[MAILBOX_PAYLOAD_BYTES];
        mailbox_payload_bytes(raw.data, d);
        count(c, vehicle_e2e_check(data_id, max_delta, timeout_ms, d, r->expected_len, rx, now));
    }
}

static void start(void)
{
    hsem_host_reset();
    (void)memset(&region, 0, sizeof region);
    region.word[MAILBOX_M7_LAYOUT_WORD] = PLATFORM_MAILBOX_LAYOUT_ID;
    TEST_ASSERT_TRUE(mailbox_init(&region, START_MS - 1u));
    mailbox_step(START_MS - 1u); /* the M4's baselines, before the M7's first frame */
}

/* Incoming 0x020: refused M4 reads on `k` consecutive passes of every cycle, and on all
 * four passes of the cycles listed by `full_loss_cycle` (UINT32_MAX: none). */
static e2e_count_t run_incoming(uint32_t phase, uint32_t k, uint32_t full_loss_cycle)
{
    start();
    vehicle_e2e_tx_state_t tx;
    vehicle_e2e_rx_state_t rx;
    vehicle_e2e_tx_init(&tx);
    vehicle_e2e_rx_init(&rx);
    e2e_count_t c = {0};
    bool seen_frame = false;
    uint32_t refuse_left = 0u;
    for (uint32_t ms = 0u; ms < (CYCLES * LEAN_CYCLE_MS); ms++) {
        const uint32_t now = START_MS + ms;
        if ((ms % LEAN_CYCLE_MS) == 0u) {
            uint8_t d[MAILBOX_PAYLOAD_BYTES] = {0};
            d[2] = (uint8_t)(ms / LEAN_CYCLE_MS);
            TEST_ASSERT_EQUAL(VEHICLE_E2E_OK, vehicle_e2e_protect(PLATFORM_EKF_LEAN_E2E_DATA_ID, d, LEAN_LEN, &tx));
            m7_write(LEAN, LEAN_HSEM, d, LEAN_LEN);
            const uint32_t cycle = ms / LEAN_CYCLE_MS;
            refuse_left = (cycle == full_loss_cycle) ? (LEAN_CYCLE_MS / M4_PASS_MS) : k;
        }
        if (((ms + phase) % M4_PASS_MS) == 0u) {
            if (refuse_left > 0u) {
                hsem_host_force_busy(LEAN_HSEM, 1u); /* exactly this pass's read */
                refuse_left--;
            }
            mailbox_step(now);
            uint8_t d[MAILBOX_PAYLOAD_BYTES];
            if (mailbox_take(LEAN, d)) {
                count(&c, platform_ekf_lean_e2e_check(d, LEAN_LEN, &rx, now));
                seen_frame = true;
            } else if (seen_frame &&
                       (platform_ekf_lean_e2e_check_timeout(&rx, now) == VEHICLE_E2E_NO_NEW_DATA)) {
                c.timeouts++;
            }
        }
    }
    return c;
}

static void test_up_to_three_refused_reads_per_cycle_lose_nothing(void)
{
    for (uint32_t phase = 0u; phase < M4_PASS_MS; phase++) {
        for (uint32_t k = 0u; k <= 3u; k++) {
            const e2e_count_t c = run_incoming(phase, k, UINT32_MAX);
            TEST_ASSERT_EQUAL_UINT32_MESSAGE(0u, c.wrong_sequence, "a refused read lost a 0x020 frame");
            TEST_ASSERT_EQUAL_UINT32(0u, c.other_bad);
            TEST_ASSERT_EQUAL_UINT32(0u, c.timeouts);
            TEST_ASSERT_EQUAL_UINT32(CYCLES, c.ok);
        }
    }
}

static void test_a_whole_refused_cycle_is_exactly_one_counted_gap(void)
{
    for (uint32_t phase = 0u; phase < M4_PASS_MS; phase++) {
        const e2e_count_t c = run_incoming(phase, 0u, 10u);
        TEST_ASSERT_EQUAL_UINT32(1u, c.wrong_sequence); /* the receiver reports the gap */
        TEST_ASSERT_EQUAL_UINT32(0u, c.other_bad);
        TEST_ASSERT_EQUAL_UINT32(0u, c.timeouts);       /* 2 cycles < the 60 ms timeout */
        TEST_ASSERT_EQUAL_UINT32(CYCLES - 2u, c.ok);    /* frame 10 lost, 11 is the gap, then OK */
        TEST_ASSERT_TRUE(mailbox_health(START_MS).read_busy >= (LEAN_CYCLE_MS / M4_PASS_MS));
    }
}

/* Outgoing 0x023 and 0x082: the M4 offers each at its cycle; its write is refused on `k`
 * consecutive passes after each offer; the M7 polls both slots every M7_POLL_MS. */
static void run_outgoing(uint32_t m4_phase, uint32_t m7_phase, uint32_t k, e2e_count_t* warn, e2e_count_t* hbs)
{
    start();
    vehicle_e2e_tx_state_t tx_warn;
    vehicle_e2e_tx_state_t tx_hbs;
    vehicle_e2e_rx_state_t rx_warn;
    vehicle_e2e_rx_state_t rx_hbs;
    mailbox_reader_t r_warn;
    mailbox_reader_t r_hbs;
    vehicle_e2e_tx_init(&tx_warn);
    vehicle_e2e_tx_init(&tx_hbs);
    vehicle_e2e_rx_init(&rx_warn);
    vehicle_e2e_rx_init(&rx_hbs);
    mailbox_reader_init(&r_warn, WARN_LEN);
    mailbox_reader_init(&r_hbs, HBS_LEN);
    mailbox_raw_t raw; /* the M7's baselines before the M4 writes anything */
    mailbox_slot_load(slot_words(WARN), &raw);
    (void)mailbox_reader_accept(&r_warn, &raw, true, START_MS);
    mailbox_slot_load(slot_words(HBS), &raw);
    (void)mailbox_reader_accept(&r_hbs, &raw, true, START_MS);
    (void)memset(warn, 0, sizeof *warn);
    (void)memset(hbs, 0, sizeof *hbs);
    uint32_t refuse_warn = 0u;
    uint32_t refuse_hbs = 0u;
    for (uint32_t ms = 0u; ms < (CYCLES * WARN_CYCLE_MS); ms++) {
        const uint32_t now = START_MS + ms;
        if (((ms + m4_phase) % M4_PASS_MS) == 0u) {
            uint8_t d[MAILBOX_PAYLOAD_BYTES] = {0};
            if ((ms % WARN_CYCLE_MS) < M4_PASS_MS) { /* the first pass of each 0x023 cycle */
                d[2] = (uint8_t)ms;
                TEST_ASSERT_EQUAL(VEHICLE_E2E_OK, platform_safety_warning_status_e2e_protect(d, WARN_LEN, &tx_warn));
                TEST_ASSERT_TRUE(mailbox_offer(WARN, d, WARN_LEN));
                refuse_warn = k;
            }
            if ((ms % HBS_CYCLE_MS) < M4_PASS_MS) {
                (void)memset(d, 0, sizeof d);
                TEST_ASSERT_EQUAL(VEHICLE_E2E_OK, platform_heartbeat_safety_e2e_protect(d, HBS_LEN, &tx_hbs));
                TEST_ASSERT_TRUE(mailbox_offer(HBS, d, HBS_LEN));
                refuse_hbs = k;
            }
            if (refuse_warn > 0u) {
                hsem_host_force_busy(WARN_HSEM, 1u);
                refuse_warn--;
            }
            if (refuse_hbs > 0u) {
                hsem_host_force_busy(HBS_HSEM, 1u);
                refuse_hbs--;
            }
            mailbox_step(now);
        }
        if (((ms + m7_phase) % M7_POLL_MS) == 0u) {
            m7_poll(WARN, WARN_HSEM, &r_warn, PLATFORM_SAFETY_WARNING_STATUS_E2E_DATA_ID,
                    (uint8_t)PLATFORM_SAFETY_WARNING_STATUS_E2E_MAX_DELTA_COUNTER,
                    PLATFORM_SAFETY_WARNING_STATUS_E2E_TIMEOUT_MS, &rx_warn, now, warn);
            m7_poll(HBS, HBS_HSEM, &r_hbs, PLATFORM_HEARTBEAT_SAFETY_E2E_DATA_ID,
                    (uint8_t)PLATFORM_HEARTBEAT_SAFETY_E2E_MAX_DELTA_COUNTER,
                    PLATFORM_HEARTBEAT_SAFETY_E2E_TIMEOUT_MS, &rx_hbs, now, hbs);
        }
    }
}

static void test_up_to_three_refused_writes_per_cycle_lose_nothing(void)
{
    for (uint32_t m4_phase = 0u; m4_phase < M4_PASS_MS; m4_phase++) {
        for (uint32_t m7_phase = 0u; m7_phase < M7_POLL_MS; m7_phase++) {
            for (uint32_t k = 0u; k <= 3u; k++) {
                e2e_count_t warn;
                e2e_count_t hbs;
                run_outgoing(m4_phase, m7_phase, k, &warn, &hbs);
                TEST_ASSERT_EQUAL_UINT32_MESSAGE(0u, warn.wrong_sequence, "a deferred 0x023 was lost");
                TEST_ASSERT_EQUAL_UINT32_MESSAGE(0u, hbs.wrong_sequence, "a deferred 0x082 was lost");
                TEST_ASSERT_EQUAL_UINT32(0u, warn.other_bad + hbs.other_bad);
                TEST_ASSERT_TRUE(warn.ok >= (CYCLES - 1u)); /* the last offer may still be pending */
                TEST_ASSERT_TRUE(hbs.ok >= ((CYCLES * WARN_CYCLE_MS) / HBS_CYCLE_MS) - 1u);
                TEST_ASSERT_EQUAL_UINT32(0u, mailbox_health(START_MS).write_overruns);
            }
        }
    }
}

void setUp(void) {}
void tearDown(void)
{
    (void)mailbox_init(NULL, 0u);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_up_to_three_refused_reads_per_cycle_lose_nothing);
    RUN_TEST(test_a_whole_refused_cycle_is_exactly_one_counted_gap);
    RUN_TEST(test_up_to_three_refused_writes_per_cycle_lose_nothing);
    return UNITY_END();
}
