/*
 * Two-thread stress test of the M4 side of the mailbox (D-093 items 5 and 6): the real M4
 * glue (services/mailbox.c on services/mailbox_core.c) in the main-loop thread, a model of
 * the M7 in another, sharing only the region and the slot semaphores of the host HSEM model.
 * Run under ASan + UBSan (host-tests) and ThreadSanitizer (host-tsan): TSan reports a region
 * word touched by both sides without the slot's HSEM; the checks report a torn, repeated or
 * out-of-order frame.
 *
 * The M7 model writes EKF_LEAN frames (take, store, barrier, commit, release; a refused
 * take defers to its next iteration) and reads HEARTBEAT_SAFETY (baseline, accept). The
 * M4 offers a HEARTBEAT_SAFETY frame every few passes and takes EKF_LEAN frames.
 * Layout words stay out of the concurrent phase (plain volatile words owned by one core,
 * single-copy atomic on the chip, which the host memory model cannot express): the M7
 * word is written before the threads start and the M4 rewrites only its own word, which
 * the M7 model read once before the threads.
 *
 * Frame k carries k in data[0..3] and k ^ CHECK_MASK in data[4..7]: a payload mixed from
 * two frames never decodes. Committed frames are never overwritten unread (a writer starts
 * a new frame only after the reader acknowledged the previous one), so every frame a writer
 * committed reaches its reader, and every gap is a counted writer overrun.
 * Unity asserts only in the main thread; every cross-thread value goes through the HSEMs
 * or __atomic builtins; all loops are bounded.
 */
#define _POSIX_C_SOURCE 200809L
#include "services/mailbox.h"

#include "hal/hsem.h"
#include "hal/host/hsem_host.h"
#include "platform_mailbox.h"

#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unity.h>

#define FRAMES 20000u
#define M4_GEN_EVERY 3u
#define HOOK_YIELD_EVERY 7u
#define MAX_ITERATIONS 200000000u
#define CHECK_MASK 0xA5A5A5A5u
#define DRAIN_PASSES 4u
#define LEAN PLATFORM_MAILBOX_SLOT_EKF_LEAN_INDEX
#define LEAN_HSEM ((uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_HSEM_ID)
#define HBS PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_INDEX
#define HBS_HSEM ((uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_HSEM_ID)
#define HEADER_WORDS (PLATFORM_MAILBOX_HEADER_BYTES / MAILBOX_WORD_BYTES)

_Static_assert(PLATFORM_MAILBOX_SLOT_EKF_LEAN_LENGTH == 8u, "frame k needs 8 bytes");
_Static_assert(PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_LENGTH == 8u, "frame k needs 8 bytes");

static mailbox_region_t region;
static mailbox_reader_t m7_reader; /* baseline taken before the threads start */

/* Cross-thread progress (atomics only). */
static uint32_t m7_committed;  /* EKF_LEAN frames the M7 committed (last k) */
static uint32_t m4_accepted;   /* last EKF_LEAN k the M4 took */
static uint32_t m7_seen;       /* last HEARTBEAT_SAFETY k the M7 read */
static uint32_t m7_done;       /* the M7 thread finished */

/* Per-side results, read by the main thread after the join. */
static uint32_t m4_torn, m4_order, m7_torn, m7_order, m7_overruns, m7_frames;
static uint32_t hold_count;

static volatile uint32_t* slot_words(uint32_t slot)
{
    return &region.word[HEADER_WORDS + (slot * MAILBOX_SLOT_WORDS)];
}

static void encode(uint32_t k, uint8_t d[MAILBOX_PAYLOAD_BYTES])
{
    const uint32_t c = k ^ CHECK_MASK;
    for (uint32_t i = 0u; i < 4u; i++) {
        d[i] = (uint8_t)(k >> (8u * i));
        d[i + 4u] = (uint8_t)(c >> (8u * i));
    }
}

static bool decode(const uint8_t d[MAILBOX_PAYLOAD_BYTES], uint32_t* k)
{
    uint32_t a = 0u;
    uint32_t c = 0u;
    for (uint32_t i = 0u; i < 4u; i++) {
        a |= (uint32_t)d[i] << (8u * i);
        c |= (uint32_t)d[i + 4u] << (8u * i);
    }
    *k = a;
    return (a ^ CHECK_MASK) == c;
}

static void held_hook(uint8_t id)
{
    (void)id;
    if ((__atomic_add_fetch(&hold_count, 1u, __ATOMIC_RELAXED) % HOOK_YIELD_EVERY) == 0u) {
        (void)sched_yield();
    }
}

static void* m7_thread(void* arg)
{
    (void)arg;
    hsem_host_set_core(HSEM_HOST_CORE_M7);
    mailbox_pending_t p;
    mailbox_pending_init(&p);
    uint32_t next = 1u;
    uint32_t last_hbs = 0u;
    for (uint32_t it = 0u; (it < MAX_ITERATIONS) && (__atomic_load_n(&m7_committed, __ATOMIC_ACQUIRE) < FRAMES);
         it++) {
        /* Writer: a new frame only after the M4 took the previous one. */
        if (!p.pending && (__atomic_load_n(&m4_accepted, __ATOMIC_ACQUIRE) == (next - 1u))) {
            uint8_t d[MAILBOX_PAYLOAD_BYTES];
            encode(next, d);
            (void)mailbox_pending_offer(&p, d, (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_LENGTH);
        }
        hal_hsem_key_t key;
        if (p.pending) {
            if (hal_hsem_take(LEAN_HSEM, &key)) {
                mailbox_slot_store(&p, slot_words(LEAN));
                hal_hsem_barrier();
                mailbox_slot_commit(slot_words(LEAN));
                hal_hsem_release(LEAN_HSEM, &key);
                mailbox_pending_done(&p);
                __atomic_store_n(&m7_committed, next, __ATOMIC_RELEASE);
                next++;
            } else {
                mailbox_pending_defer(&p);
            }
        }
        /* Reader of the M4's heartbeat slot. */
        if (hal_hsem_take(HBS_HSEM, &key)) {
            mailbox_raw_t raw;
            mailbox_slot_load(slot_words(HBS), &raw);
            hal_hsem_release(HBS_HSEM, &key);
            if (mailbox_reader_accept(&m7_reader, &raw, true, 0u) == MAILBOX_READ_NEW) {
                uint8_t d[MAILBOX_PAYLOAD_BYTES];
                uint32_t k = 0u;
                mailbox_payload_bytes(raw.data, d);
                if (!decode(d, &k)) {
                    m7_torn++;
                } else if (k <= last_hbs) {
                    m7_order++;
                } else {
                    m7_frames++;
                    m7_overruns += (k - last_hbs) - 1u;
                    last_hbs = k;
                    __atomic_store_n(&m7_seen, k, __ATOMIC_RELEASE);
                }
            }
        }
    }
    __atomic_store_n(&m7_done, 1u, __ATOMIC_RELEASE);
    return NULL;
}

static void test_two_cores_exchange_without_torn_or_reordered_frames(void)
{
    hsem_host_reset();
    hsem_host_set_held_hook(held_hook);
    (void)memset(&region, 0, sizeof region);
    region.word[MAILBOX_M7_LAYOUT_WORD] = PLATFORM_MAILBOX_LAYOUT_ID;
    TEST_ASSERT_TRUE(mailbox_init(&region, 0u));
    mailbox_step(0u); /* baselines before the M7 starts */
    mailbox_raw_t raw0;
    mailbox_reader_init(&m7_reader, (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_LENGTH);
    mailbox_slot_load(slot_words(HBS), &raw0);
    TEST_ASSERT_EQUAL(MAILBOX_READ_BASELINE, mailbox_reader_accept(&m7_reader, &raw0, true, 0u));

    pthread_t t;
    TEST_ASSERT_EQUAL_INT(0, pthread_create(&t, NULL, m7_thread, NULL));
    uint32_t last_lean = 0u;
    uint32_t hbs_next = 1u;
    uint32_t hbs_offered = 0u;
    uint32_t drain = 0u; /* a few passes after the M7 stopped, for its last frame */
    for (uint32_t pass = 1u; (pass < MAX_ITERATIONS) && (drain < DRAIN_PASSES); pass++) {
        if (__atomic_load_n(&m7_done, __ATOMIC_ACQUIRE) != 0u) {
            drain++;
        }
        /* A new heartbeat every few passes, once the M7 has read the previous one. */
        if (((pass % M4_GEN_EVERY) == 0u) && (__atomic_load_n(&m7_seen, __ATOMIC_ACQUIRE) + 1u >= hbs_next)) {
            uint8_t d[MAILBOX_PAYLOAD_BYTES];
            encode(hbs_next, d);
            if (mailbox_offer(HBS, d, (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_LENGTH)) {
                hbs_offered++;
                hbs_next++;
            }
        }
        mailbox_step(pass);
        uint8_t d[MAILBOX_PAYLOAD_BYTES];
        if (mailbox_take(LEAN, d)) {
            uint32_t k = 0u;
            if (!decode(d, &k)) {
                m4_torn++;
            } else if (k != (last_lean + 1u)) {
                m4_order++; /* the M7 waits for each ack: no gap, no repeat allowed */
            } else {
                last_lean = k;
                __atomic_store_n(&m4_accepted, k, __ATOMIC_RELEASE);
            }
        }
    }
    TEST_ASSERT_EQUAL_INT(0, pthread_join(t, NULL));
    hsem_host_set_held_hook(NULL);

    const mailbox_health_t h = mailbox_health(0u);
    TEST_ASSERT_EQUAL_UINT32(0u, m4_torn);
    TEST_ASSERT_EQUAL_UINT32(0u, m4_order);
    TEST_ASSERT_EQUAL_UINT32(0u, m7_torn);
    TEST_ASSERT_EQUAL_UINT32(0u, m7_order);
    TEST_ASSERT_EQUAL_UINT32(FRAMES, last_lean);
    TEST_ASSERT_TRUE(m7_frames > 0u);
    /* Every M4 frame not seen by the M7 is a counted overrun or still pending/unread. */
    TEST_ASSERT_TRUE(m7_overruns <= h.write_overruns);
    TEST_ASSERT_TRUE(hbs_offered >= h.write_done);
    TEST_ASSERT_EQUAL_UINT32(0u, h.take_overruns);
    TEST_ASSERT_EQUAL_UINT32(0u, h.read_rejected);
}

void setUp(void) {}
void tearDown(void) {}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_two_cores_exchange_without_torn_or_reordered_frames);
    return UNITY_END();
}
