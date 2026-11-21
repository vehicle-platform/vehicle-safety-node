/*
 * Heartbeat 0x081 golden wire vectors through the real receive path (T36, safety-reviewer
 * 2026-11-19), and the D-042 item 3 rule that a valid-CRC 0x081 whose NODE_MODE is not NORMAL
 * never selects the EKF source, whatever the value (SAFE_STATE, BOOTLOADER, DIAGNOSTIC and an
 * undefined 15 included, D-064 item 1).
 *
 * Part 1 injects literal frame bytes into the host CAN model and reads them back with the same
 * calls cornering.c uses: platform_link_step / platform_link_take (E2E check with Data ID,
 * CRC, counter) and the generated platform_heartbeat_rt_core_unpack. Part 2 drives the real
 * cornering feature with a 0x081 stream of each mode.
 */
#include "features/cornering/cornering.h"

#include "bus_model.h"
#include "services/platform_link.h"

#include <unity.h>

#define HB_LEN (8u)

typedef struct {
    uint8_t mode;
    uint8_t errors;
    uint16_t uptime;
    uint8_t counter;
    uint8_t bytes[HB_LEN];
} hb_vector_t;

/* From vehicle-defs heartbeat_vectors.json (v0.30.0), resync on repin. Message HeartbeatRtCore
 * (0x081, Data ID 0x1081). Built by the defs generator from cantools and the Python reference
 * E2E, never from the C pack; the bytes are byte 0 (CRC-8) first. Order as in the file. */
static const hb_vector_t VECTORS[] = {
    {0u, 0u, 0u, 0u, {0x70u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u}},
    {0u, 0u, 0u, 15u, {0x2Cu, 0x0Fu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u}},
    {0u, 0u, 65535u, 0u, {0x39u, 0x00u, 0x00u, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u}},
    {0u, 0u, 65535u, 15u, {0x65u, 0x0Fu, 0x00u, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u}},
    {0u, 255u, 0u, 0u, {0x79u, 0x00u, 0xFFu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u}},
    {0u, 255u, 0u, 15u, {0x25u, 0x0Fu, 0xFFu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u}},
    {0u, 255u, 65535u, 0u, {0x30u, 0x00u, 0xFFu, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u}},
    {0u, 255u, 65535u, 15u, {0x6Cu, 0x0Fu, 0xFFu, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u}},
    {1u, 0u, 0u, 0u, {0xC9u, 0x10u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u}},
    {1u, 0u, 0u, 15u, {0x95u, 0x1Fu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u}},
    {1u, 0u, 65535u, 0u, {0x80u, 0x10u, 0x00u, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u}},
    {1u, 0u, 65535u, 15u, {0xDCu, 0x1Fu, 0x00u, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u}},
    {1u, 255u, 0u, 0u, {0xC0u, 0x10u, 0xFFu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u}},
    {1u, 255u, 0u, 15u, {0x9Cu, 0x1Fu, 0xFFu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u}},
    {1u, 255u, 65535u, 0u, {0x89u, 0x10u, 0xFFu, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u}},
    {1u, 255u, 65535u, 15u, {0xD5u, 0x1Fu, 0xFFu, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u}},
    {2u, 0u, 0u, 0u, {0x1Fu, 0x20u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u}},
    {2u, 0u, 0u, 15u, {0x43u, 0x2Fu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u}},
    {2u, 0u, 65535u, 0u, {0x56u, 0x20u, 0x00u, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u}},
    {2u, 0u, 65535u, 15u, {0x0Au, 0x2Fu, 0x00u, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u}},
    {2u, 255u, 0u, 0u, {0x16u, 0x20u, 0xFFu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u}},
    {2u, 255u, 0u, 15u, {0x4Au, 0x2Fu, 0xFFu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u}},
    {2u, 255u, 65535u, 0u, {0x5Fu, 0x20u, 0xFFu, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u}},
    {2u, 255u, 65535u, 15u, {0x03u, 0x2Fu, 0xFFu, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u}},
    {3u, 0u, 0u, 0u, {0xA6u, 0x30u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u}},
    {4u, 0u, 0u, 0u, {0xAEu, 0x40u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u}},
    {5u, 0u, 0u, 0u, {0x17u, 0x50u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u}},
    {15u, 0u, 0u, 0u, {0xD9u, 0xF0u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u}},
};
#define VECTOR_COUNT (sizeof VECTORS / sizeof VECTORS[0])

/* HeartbeatSafety (0x082, Data ID 0x1082) frames from the same file, same provenance: the
 * safety node's own heartbeat arriving as a 0x081 is a masquerade and must fail the CRC. */
static const uint8_t SAFETY_HB_VECTORS[][HB_LEN] = {
    {0xDFu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u},
    {0x83u, 0x0Fu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u},
    {0x96u, 0x00u, 0x00u, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u},
    {0xCAu, 0x0Fu, 0x00u, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u},
    {0xD6u, 0x00u, 0xFFu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u},
    {0x8Au, 0x0Fu, 0xFFu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u},
    {0x9Fu, 0x00u, 0xFFu, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u},
    {0xC3u, 0x0Fu, 0xFFu, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u},
    {0x66u, 0x10u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u},
    {0x3Au, 0x1Fu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u},
    {0x2Fu, 0x10u, 0x00u, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u},
    {0x73u, 0x1Fu, 0x00u, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u},
    {0x6Fu, 0x10u, 0xFFu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u},
    {0x33u, 0x1Fu, 0xFFu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u},
    {0x26u, 0x10u, 0xFFu, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u},
    {0x7Au, 0x1Fu, 0xFFu, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u},
    {0xB0u, 0x20u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u},
    {0xECu, 0x2Fu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u},
    {0xF9u, 0x20u, 0x00u, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u},
    {0xA5u, 0x2Fu, 0x00u, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u},
    {0xB9u, 0x20u, 0xFFu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u},
    {0xE5u, 0x2Fu, 0xFFu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u},
    {0xF0u, 0x20u, 0xFFu, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u},
    {0xACu, 0x2Fu, 0xFFu, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u},
    {0x09u, 0x30u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u},
    {0x01u, 0x40u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u},
    {0xB8u, 0x50u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u},
    {0x76u, 0xF0u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u},
};
#define SAFETY_HB_COUNT (sizeof SAFETY_HB_VECTORS / sizeof SAFETY_HB_VECTORS[0])

#define PASS_MS 2u
#define MODE_NORMAL (PLATFORM_HEARTBEAT_RT_CORE_RT_CORE_NODE_MODE_NORMAL_CHOICE)

static uint32_t t_now;

void setUp(void)
{
    bus_reset();
    t_now = 0u;
    TEST_ASSERT_TRUE(platform_link_init(0u));
}
void tearDown(void) {}

/* One frame onto the bus, one link pass, one take: the path of cornering.c take_heartbeat(). */
static vehicle_e2e_status_t receive(const uint8_t bytes[HB_LEN], uint8_t payload[HB_LEN])
{
    vehicle_e2e_status_t st = VEHICLE_E2E_BAD_ARGUMENT;
    t_now += PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS;
    bus_send(BUS_HB, bytes);
    platform_link_step(t_now);
    TEST_ASSERT_TRUE(platform_link_take(PLATFORM_LINK_RX_HEARTBEAT_RT_CORE, payload, &st));
    return st;
}

static void check_fields(const hb_vector_t* v, const uint8_t payload[HB_LEN])
{
    struct platform_heartbeat_rt_core_t hb;
    TEST_ASSERT_EQUAL_INT(0, platform_heartbeat_rt_core_unpack(&hb, payload, PLATFORM_HEARTBEAT_RT_CORE_LENGTH));
    TEST_ASSERT_EQUAL_UINT8(v->mode, hb.rt_core_node_mode);
    TEST_ASSERT_EQUAL_UINT8(v->errors, hb.rt_core_error_count);
    TEST_ASSERT_EQUAL_UINT16(v->uptime, hb.rt_core_uptime);
    TEST_ASSERT_EQUAL_UINT8(v->counter, hb.e2_e_counter);
    TEST_ASSERT_EQUAL_UINT8(v->bytes[0], hb.e2_e_crc);
}

/* The first frame of every vector (counter 15 or 0 alike) passes CRC and Data ID: INITIAL,
 * the resync verdict, never WRONG_CRC; the payload decodes to the vector's fields. */
static void test_every_vector_passes_crc_and_decodes(void)
{
    for (uint32_t i = 0u; i < VECTOR_COUNT; i++) {
        uint8_t payload[PLATFORM_LINK_PAYLOAD_BYTES];
        TEST_ASSERT_EQUAL_MESSAGE(VEHICLE_E2E_INITIAL, receive(VECTORS[i].bytes, payload), "first frame");
        check_fields(&VECTORS[i], payload);
        setUp();
    }
}

/* Counter 15 then 0 is a legal wrap (delta 1): the second frame is OK, hence usable. Every
 * counter-0 vector is received behind a counter-15 vector. */
static void test_counter_wrap_pair_is_ok_and_decodes(void)
{
    uint32_t pairs = 0u;
    for (uint32_t i = 0u; i < VECTOR_COUNT; i++) {
        if (VECTORS[i].counter != 15u) {
            continue;
        }
        for (uint32_t j = 0u; j < VECTOR_COUNT; j++) {
            if (VECTORS[j].counter == 0u) {
                uint8_t payload[PLATFORM_LINK_PAYLOAD_BYTES];
                (void)receive(VECTORS[i].bytes, payload);
                TEST_ASSERT_EQUAL(VEHICLE_E2E_OK, receive(VECTORS[j].bytes, payload));
                check_fields(&VECTORS[j], payload);
                pairs++;
                setUp();
            }
        }
    }
    TEST_ASSERT_GREATER_THAN_UINT32(0u, pairs);
}

/* Any single flipped bit of any vector (CRC, counter, payload) is WRONG_CRC, including the
 * NODE_MODE nibble in byte 1. All vectors. */
static void test_single_bit_corruption_is_wrong_crc(void)
{
    for (uint32_t i = 0u; i < VECTOR_COUNT; i++) {
        for (uint32_t bit = 0u; bit < (HB_LEN * 8u); bit++) {
            uint8_t bad[HB_LEN];
            uint8_t payload[PLATFORM_LINK_PAYLOAD_BYTES];
            (void)memcpy(bad, VECTORS[i].bytes, HB_LEN);
            bad[bit / 8u] ^= (uint8_t)(1u << (bit % 8u));
            TEST_ASSERT_EQUAL(VEHICLE_E2E_WRONG_CRC, receive(bad, payload));
            setUp();
        }
    }
}

/* A 0x082 frame injected as 0x081: same layout, other Data ID, so the CRC fails. */
static void test_heartbeat_safety_frames_as_0x081_are_wrong_crc(void)
{
    for (uint32_t i = 0u; i < SAFETY_HB_COUNT; i++) {
        uint8_t payload[PLATFORM_LINK_PAYLOAD_BYTES];
        TEST_ASSERT_EQUAL(VEHICLE_E2E_WRONG_CRC, receive(SAFETY_HB_VECTORS[i], payload));
        setUp();
    }
}

/* ---- Part 2: NODE_MODE other than NORMAL never selects the EKF (D-042 item 3) ---- */

static rt_core_t rt;

static void run_to(uint32_t t_end)
{
    while (t_now < t_end) {
        t_now++;
        rt_core_tick(&rt, t_now);
        if ((t_now % PASS_MS) == 0u) {
            platform_link_step(t_now);
            cornering_step(t_now);
        }
    }
}

static void cornering_start(uint8_t mode)
{
    setUp();
    rt_core_init(&rt);
    rt.lean_raw = bus_lean_raw(5.0f);
    rt.hb_mode = mode;
    cornering_init();
    platform_link_step(0u);
    cornering_step(0u);
}

/* Long enough for the boot rule (hb OK from 200 ms, ten usable 0x020 frames, D-101 item 4) many times over. */
#define RUN_MS 1500u

static void assert_degraded_like(uint8_t mode)
{
    cornering_start(mode);
    for (uint32_t t = PASS_MS; t <= RUN_MS; t += PASS_MS) {
        run_to(t);
        TEST_ASSERT_NOT_EQUAL_MESSAGE(CORNERING_SOURCE_EKF, cornering_output().source, "EKF selected");
        TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, cornering_output().source);
        TEST_ASSERT_EQUAL(CORNERING_LEVEL_UNAVAILABLE, cornering_output().level);
        TEST_ASSERT_FALSE(cornering_status().hb_ok);
        TEST_ASSERT_TRUE(cornering_status().rt_core_inputs_lost);
    }
    /* The frames themselves are E2E-valid (CRC, counter): a refusal by mode, not by E2E. */
    TEST_ASSERT_TRUE(vehicle_e2e_is_valid(cornering_status().hb_e2e));
    TEST_ASSERT_GREATER_THAN_UINT32(0u, cornering_status().hb_rejected);
}

/* The reference: DEGRADED (the value rt-core sends today) gives exactly this behaviour. */
static void test_degraded_reference(void)
{
    assert_degraded_like(PLATFORM_HEARTBEAT_RT_CORE_RT_CORE_NODE_MODE_DEGRADED_CHOICE);
}
static void test_safe_state_acts_like_degraded(void) { assert_degraded_like(PLATFORM_HEARTBEAT_RT_CORE_RT_CORE_NODE_MODE_SAFE_STATE_CHOICE); }
static void test_bootloader_acts_like_degraded(void) { assert_degraded_like(PLATFORM_HEARTBEAT_RT_CORE_RT_CORE_NODE_MODE_BOOTLOADER_CHOICE); }
static void test_diagnostic_acts_like_degraded(void) { assert_degraded_like(PLATFORM_HEARTBEAT_RT_CORE_RT_CORE_NODE_MODE_DIAGNOSTIC_CHOICE); }
static void test_undefined_mode_15_acts_like_degraded(void) { assert_degraded_like(15u); }

/* From a running EKF source, a switch to each non-NORMAL mode drops it on the next frame and
 * the source never comes back while the mode stays. */
static void test_each_non_normal_mode_drops_a_running_ekf(void)
{
    static const uint8_t modes[] = {PLATFORM_HEARTBEAT_RT_CORE_RT_CORE_NODE_MODE_SAFE_STATE_CHOICE,
                                    PLATFORM_HEARTBEAT_RT_CORE_RT_CORE_NODE_MODE_BOOTLOADER_CHOICE,
                                    PLATFORM_HEARTBEAT_RT_CORE_RT_CORE_NODE_MODE_DIAGNOSTIC_CHOICE, 15u};
    for (uint32_t i = 0u; i < sizeof modes; i++) {
        cornering_start(MODE_NORMAL);
        run_to(380u);
        TEST_ASSERT_EQUAL(CORNERING_SOURCE_EKF, cornering_output().source);
        rt.hb_mode = modes[i];
        run_to(400u);
        TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, cornering_output().source);
        for (uint32_t t = 402u; t <= RUN_MS; t += PASS_MS) {
            run_to(t);
            TEST_ASSERT_NOT_EQUAL(CORNERING_SOURCE_EKF, cornering_output().source);
        }
        TEST_ASSERT_FALSE(cornering_status().hb_ok);
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_every_vector_passes_crc_and_decodes);
    RUN_TEST(test_counter_wrap_pair_is_ok_and_decodes);
    RUN_TEST(test_single_bit_corruption_is_wrong_crc);
    RUN_TEST(test_heartbeat_safety_frames_as_0x081_are_wrong_crc);
    RUN_TEST(test_degraded_reference);
    RUN_TEST(test_safe_state_acts_like_degraded);
    RUN_TEST(test_bootloader_acts_like_degraded);
    RUN_TEST(test_diagnostic_acts_like_degraded);
    RUN_TEST(test_undefined_mode_15_acts_like_degraded);
    RUN_TEST(test_each_non_normal_mode_drops_a_running_ekf);
    return UNITY_END();
}
