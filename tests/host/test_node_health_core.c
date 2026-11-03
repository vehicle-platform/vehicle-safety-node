/*
 * L0 tests of the 0x082 semantics (D-101 items 1-2): INIT only before the first completed
 * pass, DEGRADED for a non-EKF source or an own fault seen since the previous frame, own
 * fault onsets detected every pass, saturation, wrap-safe uptime.
 */
#include "app/node_health_core.h"

#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

static void pass(node_health_t* h, uint32_t lvl, uint32_t evt, uint32_t now)
{
    node_health_pass(h, lvl, evt, now);
    node_health_pass_done(h);
}

static void test_init_only_before_the_first_completed_pass(void)
{
    node_health_t h;
    node_health_init(&h, 0u);
    node_health_pass(&h, 0u, 0u, 1u);
    TEST_ASSERT_EQUAL_UINT8(NODE_MODE_INIT, node_health_mode(&h, true));
    TEST_ASSERT_EQUAL_UINT8(NODE_MODE_INIT, node_health_mode(&h, false));
    node_health_pass_done(&h);
    node_health_frame_sent(&h);
    pass(&h, 0u, 0u, 2u);
    TEST_ASSERT_EQUAL_UINT8(NODE_MODE_NORMAL, node_health_mode(&h, true));
    TEST_ASSERT_EQUAL_UINT8(NODE_MODE_DEGRADED, node_health_mode(&h, false));
}

static void test_event_fault_shows_in_one_frame(void)
{
    node_health_t h;
    node_health_init(&h, 0u);
    pass(&h, 0u, 0u, 1u);
    pass(&h, 0u, NODE_FAULT_PASS_GAP, 2u);
    pass(&h, 0u, 0u, 3u);
    TEST_ASSERT_EQUAL_UINT8(NODE_MODE_DEGRADED, node_health_mode(&h, true));
    TEST_ASSERT_EQUAL_UINT8(1u, node_health_error_count(&h));
    node_health_frame_sent(&h);
    pass(&h, 0u, 0u, 4u);
    TEST_ASSERT_EQUAL_UINT8(NODE_MODE_NORMAL, node_health_mode(&h, true));
}

static void test_level_fault_counts_onsets_every_pass(void)
{
    node_health_t h;
    node_health_init(&h, 0u);
    pass(&h, NODE_FAULT_PEER, 0u, 1u);
    pass(&h, NODE_FAULT_PEER, 0u, 2u);
    pass(&h, NODE_FAULT_PEER, 0u, 3u);
    TEST_ASSERT_EQUAL_UINT8(1u, node_health_error_count(&h));
    node_health_frame_sent(&h);
    pass(&h, NODE_FAULT_PEER, 0u, 4u);
    TEST_ASSERT_EQUAL_UINT8(NODE_MODE_DEGRADED, node_health_mode(&h, true)); /* still active */
    /* Toggling inside one 100 ms window: every onset counts (architecture-guard MAJOR-2). */
    pass(&h, 0u, 0u, 5u);
    pass(&h, NODE_FAULT_PEER, 0u, 6u);
    pass(&h, 0u, 0u, 7u);
    pass(&h, NODE_FAULT_PEER, 0u, 8u);
    TEST_ASSERT_EQUAL_UINT8(3u, node_health_error_count(&h));
    /* Several event faults in one pass count each; unknown bits are ignored. */
    pass(&h, NODE_FAULT_PEER, NODE_FAULT_PASS_GAP | NODE_FAULT_OUTPUT | (1u << 9), 9u);
    TEST_ASSERT_EQUAL_UINT8(5u, node_health_error_count(&h));
    node_health_frame_sent(&h);
    pass(&h, 0u, 0u, 10u);
    TEST_ASSERT_EQUAL_UINT8(NODE_MODE_NORMAL, node_health_mode(&h, true));
}

static void test_error_count_saturates(void)
{
    node_health_t h;
    node_health_init(&h, 0u);
    for (uint32_t i = 0u; i < 300u; i++) {
        pass(&h, 0u, NODE_FAULT_OUTPUT, i);
    }
    TEST_ASSERT_EQUAL_UINT8(255u, node_health_error_count(&h));
}

static void test_uptime(void)
{
    node_health_t h;
    node_health_init(&h, 100u);
    pass(&h, 0u, 0u, 1099u);
    TEST_ASSERT_EQUAL_UINT16(0u, node_health_uptime(&h));
    pass(&h, 0u, 0u, 1100u);
    TEST_ASSERT_EQUAL_UINT16(1u, node_health_uptime(&h));
    pass(&h, 0u, 0u, 3650u);
    TEST_ASSERT_EQUAL_UINT16(3u, node_health_uptime(&h));
    /* Across the 32-bit ms wrap: never decreases. */
    node_health_init(&h, UINT32_MAX - 499u);
    pass(&h, 0u, 0u, 500u);
    TEST_ASSERT_EQUAL_UINT16(1u, node_health_uptime(&h));
    /* Saturation and a huge delta. */
    node_health_init(&h, 0u);
    pass(&h, 0u, 0u, 70000000u);
    TEST_ASSERT_EQUAL_UINT16(65535u, node_health_uptime(&h));
    node_health_init(&h, 0u);
    pass(&h, 0u, 0u, 999u);
    pass(&h, 0u, 0u, 999u + UINT32_MAX); /* rem 999 + delta 2^32-1 */
    TEST_ASSERT_EQUAL_UINT16(65535u, node_health_uptime(&h));
}

static void test_null(void)
{
    node_health_init(NULL, 0u);
    node_health_pass(NULL, 1u, 1u, 0u);
    node_health_pass_done(NULL);
    node_health_frame_sent(NULL);
    TEST_ASSERT_EQUAL_UINT8(NODE_MODE_DEGRADED, node_health_mode(NULL, true));
    TEST_ASSERT_EQUAL_UINT8(0u, node_health_error_count(NULL));
    TEST_ASSERT_EQUAL_UINT16(0u, node_health_uptime(NULL));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_only_before_the_first_completed_pass);
    RUN_TEST(test_event_fault_shows_in_one_frame);
    RUN_TEST(test_level_fault_counts_onsets_every_pass);
    RUN_TEST(test_error_count_saturates);
    RUN_TEST(test_uptime);
    RUN_TEST(test_null);
    return UNITY_END();
}
