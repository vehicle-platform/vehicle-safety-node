/*
 * L0 tests of the M4 side of the mailbox, services/mailbox.c (D-093 items 5-6 and the
 * 2026-11-03 amendment), on the host HSEM model with a model of the M7 in the test (the
 * thread switches its HSEM core to M7 for the M7's accesses). Covered: inert before init,
 * init touching only the M4 layout word and the M4's own locks, the per-pass rewrite of a
 * wiped layout word, baseline (a frame from before an M4 reset is never used), new frames
 * delivered once, layout and len mismatches (dropped, never delivered later), a busy HSEM
 * ("no new frame" this pass, no wait), the writer (store, write_seq, defer, overrun), the
 * ownership rule (the M4 never writes an M7-owned word) and the health flags.
 */
#include "services/mailbox.h"

#include "hal/hsem.h"
#include "hal/host/hsem_host.h"
#include "platform.h"
#include "platform_e2e.h"
#include "platform_mailbox.h"

#include <string.h>
#include <unity.h>

#define T0 1000u
#define LEAN PLATFORM_MAILBOX_SLOT_EKF_LEAN_INDEX
#define LEAN_LEN ((uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_LENGTH)
#define LEAN_HSEM ((uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_HSEM_ID)
#define HB PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_INDEX
#define WARN PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_INDEX
#define WARN_LEN ((uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_LENGTH)
#define WARN_HSEM ((uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_HSEM_ID)
#define HBS PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_INDEX
#define HBS_LEN ((uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_LENGTH)
#define HEADER_WORDS (PLATFORM_MAILBOX_HEADER_BYTES / MAILBOX_WORD_BYTES)
#define FILL 0xC3C3C3C3u

static mailbox_region_t region;

/* Per slot, from gen/ only: the HSEM ID and the direction. */
static const uint8_t k_hsem[PLATFORM_MAILBOX_SLOT_COUNT] = {
    (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_HSEM_ID,          (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_FRICTION_MASS_HSEM_ID,
    (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_HSEM_ID, (uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_REQUEST_HSEM_ID,
    (uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_HSEM_ID,
    (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_HSEM_ID,
};
static const uint8_t k_dir[PLATFORM_MAILBOX_SLOT_COUNT] = {
    (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_DIRECTION,          (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_FRICTION_MASS_DIRECTION,
    (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_DIRECTION, (uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_REQUEST_DIRECTION,
    (uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_DIRECTION,
    (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_DIRECTION,
};
_Static_assert(PLATFORM_MAILBOX_SLOT_EKF_LEAN_INDEX == 0u, "table order");
_Static_assert(PLATFORM_MAILBOX_SLOT_EKF_FRICTION_MASS_INDEX == 1u, "table order");
_Static_assert(PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_INDEX == 2u, "table order");
_Static_assert(PLATFORM_MAILBOX_SLOT_SAFETY_REQUEST_INDEX == 3u, "table order");
_Static_assert(PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_INDEX == 4u, "table order");
_Static_assert(PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_INDEX == 5u, "table order");

/* Bit i set for each M7->M4 slot i. */
static uint32_t rx_mask(void)
{
    uint32_t m = 0u;
    for (uint32_t i = 0u; i < PLATFORM_MAILBOX_SLOT_COUNT; i++) {
        if (k_dir[i] == PLATFORM_MAILBOX_DIR_M7_TO_M4) {
            m |= (1u << i);
        }
    }
    return m;
}

static uint32_t rx_count(void)
{
    return (uint32_t)__builtin_popcount(rx_mask());
}

static uint32_t slot_first_word(uint32_t slot)
{
    return HEADER_WORDS + (slot * MAILBOX_SLOT_WORDS);
}

static volatile uint32_t* slot_words(uint32_t slot)
{
    return &region.word[slot_first_word(slot)];
}

static void frame(uint8_t k, uint8_t len, uint8_t out[MAILBOX_PAYLOAD_BYTES])
{
    (void)memset(out, 0, MAILBOX_PAYLOAD_BYTES);
    for (uint8_t j = 0u; j < len; j++) {
        out[j] = (uint8_t)(k + (uint8_t)(j * 29u));
    }
}

/* The M7's writer, as D-093 item 5 prescribes; len is what lands in the len byte. */
static void m7_write(uint32_t slot, uint8_t k, uint8_t len)
{
    uint8_t d[MAILBOX_PAYLOAD_BYTES];
    mailbox_pending_t p;
    hal_hsem_key_t key;
    frame(k, len, d);
    mailbox_pending_init(&p);
    TEST_ASSERT_TRUE(mailbox_pending_offer(&p, d, len));
    hsem_host_set_core(HSEM_HOST_CORE_M7);
    TEST_ASSERT_TRUE(hal_hsem_take(k_hsem[slot], &key));
    mailbox_slot_store(&p, slot_words(slot));
    hal_hsem_barrier();
    mailbox_slot_commit(slot_words(slot));
    hal_hsem_release(k_hsem[slot], &key);
    hsem_host_set_core(HSEM_HOST_CORE_M4);
}

static void m7_layout(uint32_t value)
{
    region.word[MAILBOX_M7_LAYOUT_WORD] = value;
}

static void expect_frame(uint32_t slot, uint8_t k, uint8_t len)
{
    uint8_t want[MAILBOX_PAYLOAD_BYTES];
    uint8_t got[MAILBOX_PAYLOAD_BYTES];
    frame(k, len, want);
    TEST_ASSERT_TRUE(mailbox_take((uint8_t)slot, got));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(want, got, MAILBOX_PAYLOAD_BYTES);
    TEST_ASSERT_FALSE(mailbox_take((uint8_t)slot, got)); /* once */
}

void setUp(void)
{
    hsem_host_reset();
    (void)memset(&region, 0, sizeof region);
    m7_layout(PLATFORM_MAILBOX_LAYOUT_ID);
}

void tearDown(void)
{
    (void)mailbox_init(NULL, 0u);
}

static void test_the_slot_tables_match_gen(void)
{
    /* The M7 model and the counts below use these tables; gen/ owns the values. */
    TEST_ASSERT_EQUAL_UINT32(LEAN_HSEM, k_hsem[LEAN]);
    TEST_ASSERT_EQUAL_UINT32(WARN_HSEM, k_hsem[WARN]);
    for (uint32_t i = 0u; i < PLATFORM_MAILBOX_SLOT_COUNT; i++) {
        TEST_ASSERT_TRUE((k_hsem[i] >= HAL_HSEM_ID_MIN) && (k_hsem[i] <= HAL_HSEM_ID_MAX));
        for (uint32_t j = i + 1u; j < PLATFORM_MAILBOX_SLOT_COUNT; j++) {
            TEST_ASSERT_NOT_EQUAL(k_hsem[i], k_hsem[j]);
        }
    }
    TEST_ASSERT_EQUAL_UINT32(4u, rx_count()); /* 0x020, 0x022, 0x081, 0x120 (D-093 item 2) */
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_MAILBOX_DIR_M4_TO_M7, k_dir[WARN]);
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_MAILBOX_DIR_M4_TO_M7, k_dir[HBS]);
}

static void test_inert_before_init(void)
{
    uint8_t d[MAILBOX_PAYLOAD_BYTES] = {0};
    TEST_ASSERT_FALSE(mailbox_init(NULL, T0));
    TEST_ASSERT_FALSE(mailbox_offer(WARN, d, WARN_LEN));
    mailbox_step(T0);
    mailbox_step(T0 + 1u);
    TEST_ASSERT_FALSE(mailbox_take(LEAN, d));
    const mailbox_health_t h = mailbox_health(T0);
    TEST_ASSERT_FALSE(h.initialised);
    TEST_ASSERT_EQUAL_UINT32(2u, h.skipped_steps);
    TEST_ASSERT_EQUAL_UINT32(0u, hsem_host_taken_count());
}

static void test_init_writes_only_the_m4_layout_word(void)
{
    for (uint32_t i = 0u; i < MAILBOX_REGION_WORDS; i++) {
        region.word[i] = FILL;
    }
    TEST_ASSERT_TRUE(mailbox_init(&region, T0));
    for (uint32_t i = 0u; i < MAILBOX_REGION_WORDS; i++) {
        const uint32_t want = (i == MAILBOX_M4_LAYOUT_WORD) ? PLATFORM_MAILBOX_LAYOUT_ID : FILL;
        TEST_ASSERT_EQUAL_HEX32_MESSAGE(want, region.word[i], "init never clears the region");
    }
}

static void test_init_frees_only_the_m4s_own_locks(void)
{
    hsem_host_seize(LEAN_HSEM, HSEM_HOST_CORE_M4); /* left from before an M4 reset */
    hsem_host_seize(WARN_HSEM, HSEM_HOST_CORE_M7); /* the M7 holds it now */
    TEST_ASSERT_TRUE(mailbox_init(&region, T0));
    TEST_ASSERT_EQUAL(HSEM_HOST_FREE, hsem_host_owner(LEAN_HSEM));
    TEST_ASSERT_EQUAL(HSEM_HOST_CORE_M7, hsem_host_owner(WARN_HSEM));
    TEST_ASSERT_EQUAL(HSEM_HOST_FREE, hsem_host_owner(0u)); /* ID 0 never touched */
}

static void test_a_wiped_layout_word_recovers_in_one_pass(void)
{
    TEST_ASSERT_TRUE(mailbox_init(&region, T0));
    region.word[MAILBOX_M4_LAYOUT_WORD] = 0u;
    mailbox_step(T0 + 1u);
    TEST_ASSERT_EQUAL_HEX32(PLATFORM_MAILBOX_LAYOUT_ID, region.word[MAILBOX_M4_LAYOUT_WORD]);
}

static void test_a_frame_from_before_the_reset_is_never_used(void)
{
    uint8_t d[MAILBOX_PAYLOAD_BYTES];
    m7_write(LEAN, 7u, LEAN_LEN); /* committed before the M4 started */
    TEST_ASSERT_TRUE(mailbox_init(&region, T0));
    mailbox_step(T0);
    mailbox_step(T0 + 1u);
    TEST_ASSERT_FALSE(mailbox_take(LEAN, d));
    m7_write(LEAN, 8u, LEAN_LEN);
    mailbox_step(T0 + 2u);
    expect_frame(LEAN, 8u, LEAN_LEN);
}

static void test_each_new_frame_is_delivered_once_and_the_newest_wins(void)
{
    TEST_ASSERT_TRUE(mailbox_init(&region, T0));
    mailbox_step(T0); /* baselines */
    m7_write(LEAN, 1u, LEAN_LEN);
    m7_write(HB, 2u, (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_LENGTH);
    mailbox_step(T0 + 1u);
    expect_frame(LEAN, 1u, LEAN_LEN);
    expect_frame(HB, 2u, (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_LENGTH);
    mailbox_step(T0 + 2u); /* write_seq unchanged: nothing new */
    uint8_t d[MAILBOX_PAYLOAD_BYTES];
    TEST_ASSERT_FALSE(mailbox_take(LEAN, d));
    m7_write(LEAN, 3u, LEAN_LEN);
    mailbox_step(T0 + 3u);
    m7_write(LEAN, 4u, LEAN_LEN);
    mailbox_step(T0 + 4u); /* no take in between: 3 is replaced, counted */
    expect_frame(LEAN, 4u, LEAN_LEN);
    TEST_ASSERT_EQUAL_UINT32(1u, mailbox_health(T0 + 4u).take_overruns);
}

static void test_a_layout_mismatch_frame_is_dropped_for_good(void)
{
    uint8_t d[MAILBOX_PAYLOAD_BYTES];
    TEST_ASSERT_TRUE(mailbox_init(&region, T0));
    mailbox_step(T0);
    m7_layout(PLATFORM_MAILBOX_LAYOUT_ID ^ 1u);
    m7_write(LEAN, 5u, LEAN_LEN);
    mailbox_step(T0 + 1u);
    TEST_ASSERT_FALSE(mailbox_take(LEAN, d));
    m7_layout(PLATFORM_MAILBOX_LAYOUT_ID);
    mailbox_step(T0 + 2u);
    TEST_ASSERT_FALSE(mailbox_take(LEAN, d)); /* absorbed, never delivered later */
    TEST_ASSERT_EQUAL_UINT32(1u, mailbox_health(T0 + 2u).read_rejected);
}

static void test_peer_state_never_matched_ok_then_mismatch(void)
{
    m7_layout(0u); /* the M7 has not written its word yet (the M4 boots on its own) */
    TEST_ASSERT_TRUE(mailbox_init(&region, T0));
    mailbox_step(T0 + 1u);
    TEST_ASSERT_EQUAL(MAILBOX_PEER_NEVER_MATCHED, mailbox_health(T0 + 500u).peer); /* no false mismatch */
    TEST_ASSERT_EQUAL_UINT32(500u, mailbox_health(T0 + 500u).peer_wait_ms);         /* for the boot bound */
    m7_layout(PLATFORM_MAILBOX_LAYOUT_ID);
    mailbox_step(T0 + 600u);
    TEST_ASSERT_EQUAL(MAILBOX_PEER_OK, mailbox_health(T0 + 600u).peer);
    m7_layout(PLATFORM_MAILBOX_LAYOUT_ID ^ 1u);
    mailbox_step(T0 + 601u);
    TEST_ASSERT_EQUAL(MAILBOX_PEER_OK, mailbox_health(T0 + 600u + PLATFORM_EKF_LEAN_E2E_TIMEOUT_MS).peer);
    TEST_ASSERT_EQUAL(MAILBOX_PEER_MISMATCH, mailbox_health(T0 + 601u + PLATFORM_EKF_LEAN_E2E_TIMEOUT_MS).peer);
    m7_layout(PLATFORM_MAILBOX_LAYOUT_ID);
    mailbox_step(T0 + 700u);
    TEST_ASSERT_EQUAL(MAILBOX_PEER_OK, mailbox_health(T0 + 700u).peer);
}

static void test_a_one_pass_wipe_of_the_m7_word_costs_one_frame_and_does_not_latch(void)
{
    TEST_ASSERT_TRUE(mailbox_init(&region, T0));
    mailbox_step(T0);
    m7_write(LEAN, 1u, LEAN_LEN);
    m7_layout(0u); /* wiped for one pass; the M7 rewrites it in its next pass */
    mailbox_step(T0 + 5u);
    m7_layout(PLATFORM_MAILBOX_LAYOUT_ID);
    m7_write(LEAN, 2u, LEAN_LEN);
    mailbox_step(T0 + 10u);
    expect_frame(LEAN, 2u, LEAN_LEN);
    const mailbox_health_t h = mailbox_health(T0 + 10u);
    TEST_ASSERT_EQUAL_UINT32(1u, h.read_rejected);
    TEST_ASSERT_EQUAL(MAILBOX_PEER_OK, h.peer);
}

static void test_no_write_while_the_peer_layout_differs(void)
{
    /* D-093 amendment 2: no store, the frame dropped and counted, and never
     * written after the match; the next frame is. */
    uint8_t d[MAILBOX_PAYLOAD_BYTES];
    TEST_ASSERT_TRUE(mailbox_init(&region, T0));
    m7_layout(PLATFORM_MAILBOX_LAYOUT_ID ^ 1u);
    frame(3u, WARN_LEN, d);
    TEST_ASSERT_TRUE(mailbox_offer(WARN, d, WARN_LEN));
    frame(4u, HBS_LEN, d);
    TEST_ASSERT_TRUE(mailbox_offer(HBS, d, HBS_LEN));
    mailbox_step(T0 + 1u);
    mailbox_step(T0 + 2u);
    TEST_ASSERT_EQUAL_UINT32(0u, slot_words(WARN)[MAILBOX_SEQ_WORD]);
    TEST_ASSERT_EQUAL_UINT32(0u, slot_words(WARN)[MAILBOX_DATA_WORD]);
    TEST_ASSERT_EQUAL_UINT32(0u, slot_words(HBS)[MAILBOX_SEQ_WORD]);
    mailbox_health_t h = mailbox_health(T0 + 2u);
    TEST_ASSERT_EQUAL_UINT32(2u, h.write_dropped);
    TEST_ASSERT_EQUAL_UINT32(0u, h.write_deferred);
    TEST_ASSERT_EQUAL_UINT32(0u, h.write_overruns);
    m7_layout(PLATFORM_MAILBOX_LAYOUT_ID);
    mailbox_step(T0 + 3u);
    TEST_ASSERT_EQUAL_UINT32(0u, slot_words(WARN)[MAILBOX_SEQ_WORD]); /* the dropped frame stays dropped */
    TEST_ASSERT_EQUAL_UINT32(0u, slot_words(HBS)[MAILBOX_SEQ_WORD]);
    frame(5u, WARN_LEN, d);
    TEST_ASSERT_TRUE(mailbox_offer(WARN, d, WARN_LEN));
    mailbox_step(T0 + 4u);
    TEST_ASSERT_EQUAL_UINT32(1u, slot_words(WARN)[MAILBOX_SEQ_WORD]);
    h = mailbox_health(T0 + 4u);
    TEST_ASSERT_EQUAL_UINT32(1u, h.write_done);
    TEST_ASSERT_EQUAL_UINT32(2u, h.write_dropped);
}

static void test_an_m4_reset_inside_a_write_continues_the_sequence(void)
{
    uint8_t d[MAILBOX_PAYLOAD_BYTES];
    TEST_ASSERT_TRUE(mailbox_init(&region, T0));
    frame(1u, WARN_LEN, d);
    TEST_ASSERT_TRUE(mailbox_offer(WARN, d, WARN_LEN));
    mailbox_step(T0 + 1u);
    TEST_ASSERT_EQUAL_UINT32(1u, slot_words(WARN)[MAILBOX_SEQ_WORD]);
    /* The M4 resets after its next store, before the commit: it still holds the lock and
     * the payload is new but write_seq is not, so the M7 never sees that frame. */
    hsem_host_seize(WARN_HSEM, HSEM_HOST_CORE_M4);
    slot_words(WARN)[MAILBOX_DATA_WORD] = 0xDEADBEEFu;
    TEST_ASSERT_TRUE(mailbox_init(&region, T0 + 2u));
    TEST_ASSERT_EQUAL(HSEM_HOST_FREE, hsem_host_owner(WARN_HSEM));
    frame(2u, WARN_LEN, d);
    TEST_ASSERT_TRUE(mailbox_offer(WARN, d, WARN_LEN));
    mailbox_step(T0 + 3u);
    TEST_ASSERT_EQUAL_UINT32(2u, slot_words(WARN)[MAILBOX_SEQ_WORD]); /* continues, never repeats */
    TEST_ASSERT_EQUAL_HEX8(d[0], (uint8_t)slot_words(WARN)[MAILBOX_DATA_WORD]);
}
static void test_a_len_mismatch_is_no_frame(void)
{
    uint8_t d[MAILBOX_PAYLOAD_BYTES];
    TEST_ASSERT_TRUE(mailbox_init(&region, T0));
    mailbox_step(T0);
    m7_write(LEAN, 9u, (uint8_t)(LEAN_LEN - 1u));
    mailbox_step(T0 + 1u);
    TEST_ASSERT_FALSE(mailbox_take(LEAN, d));
    TEST_ASSERT_EQUAL_UINT32(1u, mailbox_health(T0 + 1u).read_rejected);
}

static void test_a_busy_hsem_is_no_new_frame_this_pass(void)
{
    TEST_ASSERT_TRUE(mailbox_init(&region, T0));
    mailbox_step(T0);
    m7_write(LEAN, 10u, LEAN_LEN);
    hsem_host_seize(LEAN_HSEM, HSEM_HOST_CORE_M7); /* the M7 is inside its locked section */
    const uint32_t refused = hsem_host_refused_count();
    mailbox_step(T0 + 1u);
    uint8_t d[MAILBOX_PAYLOAD_BYTES];
    TEST_ASSERT_FALSE(mailbox_take(LEAN, d));
    TEST_ASSERT_EQUAL_UINT32(refused + 1u, hsem_host_refused_count()); /* one try, no spin */
    TEST_ASSERT_EQUAL_UINT32(1u, mailbox_health(T0 + 1u).read_busy);
    hsem_host_release_as(LEAN_HSEM, HSEM_HOST_CORE_M7);
    mailbox_step(T0 + 2u);
    expect_frame(LEAN, 10u, LEAN_LEN);
}

static void test_every_hsem_busy_still_ends_the_pass(void)
{
    TEST_ASSERT_TRUE(mailbox_init(&region, T0));
    uint8_t d[MAILBOX_PAYLOAD_BYTES];
    frame(1u, WARN_LEN, d);
    TEST_ASSERT_TRUE(mailbox_offer(WARN, d, WARN_LEN));
    for (uint32_t i = 0u; i < PLATFORM_MAILBOX_SLOT_COUNT; i++) {
        hsem_host_seize(k_hsem[i], HSEM_HOST_CORE_M7);
    }
    const uint32_t refused = hsem_host_refused_count();
    mailbox_step(T0 + 1u);
    /* One take per read slot and per pending write, nothing more. */
    TEST_ASSERT_EQUAL_UINT32(refused + rx_count() + 1u, hsem_host_refused_count());
    TEST_ASSERT_EQUAL_UINT32(1u, mailbox_health(T0 + 1u).write_deferred);
}

static void test_offer_checks_slot_direction_and_length(void)
{
    uint8_t d[MAILBOX_PAYLOAD_BYTES] = {0};
    TEST_ASSERT_TRUE(mailbox_init(&region, T0));
    TEST_ASSERT_FALSE(mailbox_offer(LEAN, d, LEAN_LEN));                    /* M7-owned slot */
    TEST_ASSERT_FALSE(mailbox_offer(WARN, d, (uint8_t)(WARN_LEN - 1u)));    /* wrong length */
    TEST_ASSERT_FALSE(mailbox_offer(WARN, NULL, WARN_LEN));
    TEST_ASSERT_FALSE(mailbox_offer((uint8_t)PLATFORM_MAILBOX_SLOT_COUNT, d, WARN_LEN));
    TEST_ASSERT_FALSE(mailbox_take(WARN, d));                               /* M4-owned slot */
    TEST_ASSERT_FALSE(mailbox_take(LEAN, NULL));
    TEST_ASSERT_TRUE(mailbox_offer(WARN, d, WARN_LEN));
    TEST_ASSERT_TRUE(mailbox_offer(HBS, d, HBS_LEN));
}

static void test_the_writer_stores_then_commits_one_sequence(void)
{
    uint8_t d[MAILBOX_PAYLOAD_BYTES];
    TEST_ASSERT_TRUE(mailbox_init(&region, T0));
    slot_words(WARN)[MAILBOX_SEQ_WORD] = 41u; /* any start value */
    frame(0x5Au, WARN_LEN, d);
    TEST_ASSERT_TRUE(mailbox_offer(WARN, d, WARN_LEN));
    mailbox_step(T0 + 1u);
    TEST_ASSERT_EQUAL_UINT32(42u, slot_words(WARN)[MAILBOX_SEQ_WORD]);
    TEST_ASSERT_EQUAL_UINT32(WARN_LEN, slot_words(WARN)[MAILBOX_LEN_WORD]);
    uint8_t got[MAILBOX_PAYLOAD_BYTES];
    const uint32_t words[MAILBOX_PAYLOAD_WORDS] = {slot_words(WARN)[MAILBOX_DATA_WORD],
                                                   slot_words(WARN)[MAILBOX_DATA_WORD + 1u]};
    mailbox_payload_bytes(words, got);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(d, got, MAILBOX_PAYLOAD_BYTES);
    TEST_ASSERT_EQUAL(HSEM_HOST_FREE, hsem_host_owner(WARN_HSEM)); /* released */
    mailbox_step(T0 + 2u);
    TEST_ASSERT_EQUAL_UINT32(42u, slot_words(WARN)[MAILBOX_SEQ_WORD]); /* nothing pending */
    TEST_ASSERT_EQUAL_UINT32(1u, mailbox_health(T0 + 2u).write_done);
}

static void test_a_refused_write_waits_for_the_next_pass_and_a_newer_frame_overruns(void)
{
    uint8_t d1[MAILBOX_PAYLOAD_BYTES];
    uint8_t d2[MAILBOX_PAYLOAD_BYTES];
    TEST_ASSERT_TRUE(mailbox_init(&region, T0));
    frame(1u, WARN_LEN, d1);
    frame(2u, WARN_LEN, d2);
    TEST_ASSERT_TRUE(mailbox_offer(WARN, d1, WARN_LEN));
    hsem_host_force_busy(WARN_HSEM, 1u);
    mailbox_step(T0 + 1u);
    TEST_ASSERT_EQUAL_UINT32(0u, slot_words(WARN)[MAILBOX_SEQ_WORD]);
    TEST_ASSERT_TRUE(mailbox_offer(WARN, d2, WARN_LEN)); /* replaces d1: an overrun */
    mailbox_step(T0 + 2u);
    TEST_ASSERT_EQUAL_UINT32(1u, slot_words(WARN)[MAILBOX_SEQ_WORD]);
    TEST_ASSERT_EQUAL_HEX32(d2[0] | ((uint32_t)d2[1] << 8) | ((uint32_t)d2[2] << 16) | ((uint32_t)d2[3] << 24),
                            slot_words(WARN)[MAILBOX_DATA_WORD]);
    const mailbox_health_t h = mailbox_health(T0 + 2u);
    TEST_ASSERT_EQUAL_UINT32(1u, h.write_deferred);
    TEST_ASSERT_EQUAL_UINT32(1u, h.write_overruns);
    TEST_ASSERT_EQUAL_UINT32(1u, h.write_done);
}

static void test_the_m4_never_writes_an_m7_owned_word(void)
{
    uint8_t d[MAILBOX_PAYLOAD_BYTES] = {0};
    for (uint32_t i = 0u; i < MAILBOX_REGION_WORDS; i++) {
        region.word[i] = FILL;
    }
    m7_layout(PLATFORM_MAILBOX_LAYOUT_ID); /* a matching peer, so the M4 does write */
    TEST_ASSERT_TRUE(mailbox_init(&region, T0));
    for (uint32_t pass = 0u; pass < 50u; pass++) {
        frame((uint8_t)pass, WARN_LEN, d);
        (void)mailbox_offer(WARN, d, WARN_LEN);
        frame((uint8_t)(pass + 100u), HBS_LEN, d);
        (void)mailbox_offer(HBS, d, HBS_LEN);
        mailbox_step(T0 + pass);
        (void)mailbox_take(LEAN, d);
    }
    for (uint32_t i = 0u; i < MAILBOX_REGION_WORDS; i++) {
        const bool m4_word = (i == MAILBOX_M4_LAYOUT_WORD) ||
                             ((i >= slot_first_word(WARN)) && (i < slot_first_word(WARN) + MAILBOX_SLOT_WORDS)) ||
                             ((i >= slot_first_word(HBS)) && (i < slot_first_word(HBS) + MAILBOX_SLOT_WORDS));
        if (i == MAILBOX_M7_LAYOUT_WORD) {
            TEST_ASSERT_EQUAL_HEX32(PLATFORM_MAILBOX_LAYOUT_ID, region.word[i]);
        } else if (!m4_word) {
            TEST_ASSERT_EQUAL_HEX32_MESSAGE(FILL, region.word[i], "an M7-owned or reserved word changed");
        }
    }
}

static void test_stale_masks_split_safety_and_qm(void)
{
    const uint32_t qm = 1u << PLATFORM_MAILBOX_SLOT_SAFETY_REQUEST_INDEX;
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_MAILBOX_CLASS_QM, PLATFORM_MAILBOX_SLOT_SAFETY_REQUEST_CLASS);
    TEST_ASSERT_TRUE(mailbox_init(&region, T0));
    mailbox_health_t h = mailbox_health(T0); /* no baseline yet */
    TEST_ASSERT_EQUAL_HEX32(rx_mask() & ~qm, h.safety_stale_slots);
    TEST_ASSERT_EQUAL_HEX32(qm, h.qm_stale_slots);
    mailbox_step(T0);
    TEST_ASSERT_EQUAL_HEX32(0u, mailbox_health(T0 + PLATFORM_EKF_LEAN_E2E_TIMEOUT_MS).safety_stale_slots);
    const uint32_t later = T0 + PLATFORM_EKF_LEAN_E2E_TIMEOUT_MS + 1u;
    TEST_ASSERT_EQUAL_HEX32(1u << LEAN, mailbox_health(later).safety_stale_slots);
    m7_write(LEAN, 1u, LEAN_LEN);
    mailbox_step(later);
    TEST_ASSERT_EQUAL_HEX32(0u, mailbox_health(later).safety_stale_slots);
    /* A stale 0x120 shows only in the QM mask (D-072 item 7). Keep the safety slots fresh. */
    const uint32_t qm_late = T0 + PLATFORM_SAFETY_REQUEST_E2E_TIMEOUT_MS + 1u;
    m7_write(LEAN, 2u, LEAN_LEN);
    m7_write(PLATFORM_MAILBOX_SLOT_EKF_FRICTION_MASS_INDEX, 3u, (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_FRICTION_MASS_LENGTH);
    m7_write(HB, 4u, (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_LENGTH);
    mailbox_step(qm_late);
    h = mailbox_health(qm_late);
    TEST_ASSERT_EQUAL_HEX32(qm, h.qm_stale_slots);
    TEST_ASSERT_EQUAL_HEX32(0u, h.safety_stale_slots);
}
static void test_init_starts_over(void)
{
    uint8_t d[MAILBOX_PAYLOAD_BYTES] = {0};
    TEST_ASSERT_TRUE(mailbox_init(&region, T0));
    mailbox_step(T0);
    m7_write(LEAN, 1u, LEAN_LEN);
    mailbox_step(T0 + 1u); /* fresh, not taken */
    TEST_ASSERT_TRUE(mailbox_offer(WARN, d, WARN_LEN));
    TEST_ASSERT_TRUE(mailbox_init(&region, T0 + 2u));
    TEST_ASSERT_FALSE(mailbox_take(LEAN, d));
    mailbox_step(T0 + 3u); /* new baseline; the old pending frame is gone */
    TEST_ASSERT_EQUAL_UINT32(0u, slot_words(WARN)[MAILBOX_SEQ_WORD]);
    TEST_ASSERT_FALSE(mailbox_take(LEAN, d));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_the_slot_tables_match_gen);
    RUN_TEST(test_inert_before_init);
    RUN_TEST(test_init_writes_only_the_m4_layout_word);
    RUN_TEST(test_init_frees_only_the_m4s_own_locks);
    RUN_TEST(test_a_wiped_layout_word_recovers_in_one_pass);
    RUN_TEST(test_a_frame_from_before_the_reset_is_never_used);
    RUN_TEST(test_each_new_frame_is_delivered_once_and_the_newest_wins);
    RUN_TEST(test_a_layout_mismatch_frame_is_dropped_for_good);
    RUN_TEST(test_peer_state_never_matched_ok_then_mismatch);
    RUN_TEST(test_a_one_pass_wipe_of_the_m7_word_costs_one_frame_and_does_not_latch);
    RUN_TEST(test_no_write_while_the_peer_layout_differs);
    RUN_TEST(test_an_m4_reset_inside_a_write_continues_the_sequence);
    RUN_TEST(test_a_len_mismatch_is_no_frame);
    RUN_TEST(test_a_busy_hsem_is_no_new_frame_this_pass);
    RUN_TEST(test_every_hsem_busy_still_ends_the_pass);
    RUN_TEST(test_offer_checks_slot_direction_and_length);
    RUN_TEST(test_the_writer_stores_then_commits_one_sequence);
    RUN_TEST(test_a_refused_write_waits_for_the_next_pass_and_a_newer_frame_overruns);
    RUN_TEST(test_the_m4_never_writes_an_m7_owned_word);
    RUN_TEST(test_stale_masks_split_safety_and_qm);
    RUN_TEST(test_init_starts_over);
    return UNITY_END();
}
