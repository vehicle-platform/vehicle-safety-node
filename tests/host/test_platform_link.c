/*
 * Host tests of services/platform_link (D-118) on the CAN host HAL: acceptance of the four RX
 * IDs with the generated E2E check on every frame (valid, timeout, counter jump, freeze, CRC,
 * wrong DLC, unknown and extended IDs ignored, two interleaved senders), the bounded RX burst,
 * the sticky failure between takes, and the TX path (0x023 / 0x082) with a refused send.
 */
#include "services/platform_link.h"

#include "bus_model.h"
#include "platform_e2e.h"

#include <unity.h>

static vehicle_e2e_tx_state_t req_tx;

static void build_req(uint8_t f[FB_LEN])
{
    (void)memset(f, 0, FB_LEN);
    (void)vehicle_e2e_protect(PLATFORM_SAFETY_REQUEST_E2E_DATA_ID, f, FB_LEN, &req_tx);
}

void setUp(void)
{
    bus_reset();
    (void)memset(&req_tx, 0, sizeof req_tx);
    vehicle_e2e_tx_init(&req_tx);
    TEST_ASSERT_TRUE(platform_link_init(0u));
}
void tearDown(void) {}

static vehicle_e2e_status_t take_status(platform_link_rx_t rx)
{
    uint8_t buf[PLATFORM_LINK_PAYLOAD_BYTES];
    vehicle_e2e_status_t st = VEHICLE_E2E_BAD_ARGUMENT;
    TEST_ASSERT_TRUE(platform_link_take(rx, buf, &st));
    return st;
}

static bool nothing_to_take(platform_link_rx_t rx)
{
    uint8_t buf[PLATFORM_LINK_PAYLOAD_BYTES];
    vehicle_e2e_status_t st;
    return !platform_link_take(rx, buf, &st);
}

/* ------------------------------------------------------------------ RX: valid frames */

static void test_all_four_ids_are_accepted_and_checked(void)
{
    vehicle_e2e_tx_state_t lean_tx, mu_tx, hb_tx;
    uint8_t f[FB_LEN];
    uint8_t buf[PLATFORM_LINK_PAYLOAD_BYTES];
    vehicle_e2e_status_t st;
    vehicle_e2e_tx_init(&lean_tx);
    vehicle_e2e_tx_init(&mu_tx);
    vehicle_e2e_tx_init(&hb_tx);

    for (uint32_t k = 0u; k < 2u; k++) {
        uint32_t t = 10u + (k * 20u);
        fb_lean(f, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_ESTIMATED_CHOICE, 1234, &lean_tx);
        bus_send(BUS_LEAN, f);
        fb_mu(f, PLATFORM_EKF_FRICTION_MASS_FRICTION_COEFF_STATE_DEFAULT_CHOICE, 50u, &mu_tx);
        bus_send(BUS_MU, f);
        fb_hb(f, PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_NORMAL_CHOICE, &hb_tx);
        bus_send(BUS_HB, f);
        build_req(f);
        bus_send(BUS_REQ, f);
        platform_link_step(t);
        for (uint32_t i = 0u; i < (uint32_t)PLATFORM_LINK_RX_COUNT; i++) {
            TEST_ASSERT_TRUE(platform_link_take((platform_link_rx_t)i, buf, &st));
            TEST_ASSERT_EQUAL((k == 0u) ? VEHICLE_E2E_INITIAL : VEHICLE_E2E_OK, st); /* first frame resyncs */
            TEST_ASSERT_TRUE(nothing_to_take((platform_link_rx_t)i));                 /* once per frame */
        }
    }
    /* The payload handed out is the frame (lean 12.34 deg state ESTIMATED, readable by gen unpack). */
    fb_lean(f, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_ESTIMATED_CHOICE, 1234, &lean_tx);
    bus_send(BUS_LEAN, f);
    platform_link_step(50u);
    TEST_ASSERT_TRUE(platform_link_take(PLATFORM_LINK_RX_EKF_LEAN, buf, &st));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(f, buf, FB_LEN);
    {
        platform_link_health_t h = platform_link_health();
        TEST_ASSERT_EQUAL_UINT32(3u, h.rx[PLATFORM_LINK_RX_EKF_LEAN].frames);
        TEST_ASSERT_EQUAL_UINT32(2u, h.rx[PLATFORM_LINK_RX_HEARTBEAT_RT_CORE].frames);
        TEST_ASSERT_EQUAL_UINT32(1u, h.rx[PLATFORM_LINK_RX_EKF_LEAN].e2e_rejected); /* the INITIAL */
        TEST_ASSERT_EQUAL_UINT32(50u, h.rx[PLATFORM_LINK_RX_EKF_LEAN].last_rx_ms);
        TEST_ASSERT_TRUE(h.rx[PLATFORM_LINK_RX_SAFETY_REQUEST].seen);
        TEST_ASSERT_EQUAL_UINT32(0u, h.rx_unknown);
    }
}

/* ------------------------------------------------------------------ RX: faults */

static void send_lean(vehicle_e2e_tx_state_t* tx)
{
    uint8_t f[FB_LEN];
    fb_lean(f, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_ESTIMATED_CHOICE, 100, tx);
    bus_send(BUS_LEAN, f);
}

static void sync_lean(vehicle_e2e_tx_state_t* tx)
{
    vehicle_e2e_tx_init(tx);
    send_lean(tx);
    platform_link_step(0u);
    TEST_ASSERT_EQUAL(VEHICLE_E2E_INITIAL, take_status(PLATFORM_LINK_RX_EKF_LEAN));
    send_lean(tx);
    platform_link_step(20u);
    TEST_ASSERT_EQUAL(VEHICLE_E2E_OK, take_status(PLATFORM_LINK_RX_EKF_LEAN));
}

static void test_timeout_then_resync(void)
{
    vehicle_e2e_tx_state_t tx;
    sync_lean(&tx);
    TEST_ASSERT_EQUAL(VEHICLE_E2E_OK, platform_link_check_timeout(PLATFORM_LINK_RX_EKF_LEAN, 20u + PLATFORM_EKF_LEAN_E2E_TIMEOUT_MS));
    TEST_ASSERT_EQUAL(VEHICLE_E2E_NO_NEW_DATA,
                      platform_link_check_timeout(PLATFORM_LINK_RX_EKF_LEAN, 20u + PLATFORM_EKF_LEAN_E2E_TIMEOUT_MS + 1u));
    send_lean(&tx); /* the first frame after a timeout resyncs: not valid */
    platform_link_step(200u);
    TEST_ASSERT_EQUAL(VEHICLE_E2E_INITIAL, take_status(PLATFORM_LINK_RX_EKF_LEAN));
    /* a message that never arrived is a timeout too */
    TEST_ASSERT_EQUAL(VEHICLE_E2E_NO_NEW_DATA, platform_link_check_timeout(PLATFORM_LINK_RX_EKF_FRICTION_MASS, 5u));
}

static void test_counter_jump_and_freeze(void)
{
    vehicle_e2e_tx_state_t tx;
    uint8_t f[FB_LEN];
    sync_lean(&tx);
    tx.counter = (uint8_t)(tx.counter + 2u); /* three frames lost: beyond MAX_DELTA 1 */
    send_lean(&tx);
    platform_link_step(40u);
    TEST_ASSERT_EQUAL(VEHICLE_E2E_WRONG_SEQUENCE, take_status(PLATFORM_LINK_RX_EKF_LEAN));
    /* a frozen sender: the same frame (same counter) again */
    fb_lean(f, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_ESTIMATED_CHOICE, 100, &tx);
    bus_send(BUS_LEAN, f);
    bus_send(BUS_LEAN, f);
    platform_link_step(60u);
    TEST_ASSERT_EQUAL(VEHICLE_E2E_REPEATED, take_status(PLATFORM_LINK_RX_EKF_LEAN)); /* the failure of the 2nd frame */
}

static void test_crc_error(void)
{
    vehicle_e2e_tx_state_t tx;
    uint8_t f[FB_LEN];
    sync_lean(&tx);
    fb_lean(f, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_ESTIMATED_CHOICE, 100, &tx);
    f[3] ^= 0x10u;
    bus_send(BUS_LEAN, f);
    platform_link_step(40u);
    TEST_ASSERT_EQUAL(VEHICLE_E2E_WRONG_CRC, take_status(PLATFORM_LINK_RX_EKF_LEAN));
}

static void test_wrong_dlc_is_never_valid_and_leaves_the_e2e_state(void)
{
    vehicle_e2e_tx_state_t tx;
    uint8_t f[FB_LEN];
    sync_lean(&tx);
    fb_lean(f, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_ESTIMATED_CHOICE, 100, &tx);
    bus_send_dlc(BUS_LEAN, f, 7u);
    platform_link_step(40u);
    TEST_ASSERT_EQUAL(VEHICLE_E2E_BAD_ARGUMENT, take_status(PLATFORM_LINK_RX_EKF_LEAN));
    bus_send_dlc(BUS_LEAN, f, 0u);
    platform_link_step(42u);
    TEST_ASSERT_EQUAL(VEHICLE_E2E_BAD_ARGUMENT, take_status(PLATFORM_LINK_RX_EKF_LEAN));
    bus_send_dlc(BUS_LEAN, f, 8u); /* the genuine frame arrives: the E2E counter state was not touched */
    platform_link_step(44u);
    TEST_ASSERT_EQUAL(VEHICLE_E2E_OK, take_status(PLATFORM_LINK_RX_EKF_LEAN));
    TEST_ASSERT_EQUAL_UINT32(2u, platform_link_health().rx[PLATFORM_LINK_RX_EKF_LEAN].bad_dlc);
}

static void test_unknown_and_extended_ids_are_ignored(void)
{
    uint8_t f[FB_LEN] = {0};
    hal_can_frame_t x;
    bus_send(0x021u, f); /* VehicleSpeed: not a safety-node input (D-041) */
    bus_send(0x082u, f); /* our own ID from another sender */
    bus_send(0x000u, f);
    bus_send(0x7FFu, f);
    (void)memset(&x, 0, sizeof x);
    x.id = BUS_LEAN; /* the same number as an extended ID is another frame */
    x.extended = true;
    x.dlc = 8u;
    TEST_ASSERT_TRUE(can_host_inject(&x));
    platform_link_step(10u);
    TEST_ASSERT_EQUAL_UINT32(5u, platform_link_health().rx_unknown);
    for (uint32_t i = 0u; i < (uint32_t)PLATFORM_LINK_RX_COUNT; i++) {
        TEST_ASSERT_TRUE(nothing_to_take((platform_link_rx_t)i));
        TEST_ASSERT_FALSE(platform_link_health().rx[i].seen);
    }
}

static void test_masquerade_two_senders_in_one_pass_is_never_valid(void)
{
    vehicle_e2e_tx_state_t real, fake;
    sync_lean(&real);
    vehicle_e2e_tx_init(&fake);
    fake.counter = 9u; /* another counter phase, the correct Data ID (worst case) */
    for (uint32_t n = 0u; n < 3u; n++) {
        send_lean(&real);
        send_lean(&fake);
        platform_link_step(40u + (n * 20u));
        vehicle_e2e_status_t st = take_status(PLATFORM_LINK_RX_EKF_LEAN);
        TEST_ASSERT_FALSE(vehicle_e2e_is_valid(st)); /* the failure survives the later frame */
        TEST_ASSERT_EQUAL(VEHICLE_E2E_WRONG_SEQUENCE, st);
    }
    TEST_ASSERT_TRUE(platform_link_health().rx[PLATFORM_LINK_RX_EKF_LEAN].take_overruns >= 3u);
}

static void test_a_failure_cannot_be_hidden_by_a_later_good_frame(void)
{
    vehicle_e2e_tx_state_t tx;
    uint8_t f[FB_LEN];
    sync_lean(&tx);
    fb_lean(f, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_ESTIMATED_CHOICE, 100, &tx);
    f[5] ^= 0x01u; /* corrupted */
    bus_send(BUS_LEAN, f);
    send_lean(&tx); /* good (counter continues): OK_SOME_LOST at the check */
    platform_link_step(40u);
    TEST_ASSERT_EQUAL(VEHICLE_E2E_WRONG_CRC, take_status(PLATFORM_LINK_RX_EKF_LEAN));
}

static void test_rx_burst_is_bounded(void)
{
    uint8_t f[FB_LEN] = {0};
    for (uint32_t i = 0u; i < 20u; i++) {
        bus_send(0x555u, f);
    }
    platform_link_step(1u);
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_LINK_RX_BURST_MAX, platform_link_health().rx_unknown);
    TEST_ASSERT_EQUAL_UINT32(20u - PLATFORM_LINK_RX_BURST_MAX, can_host_rx_pending());
    platform_link_step(2u);
    platform_link_step(3u);
    TEST_ASSERT_EQUAL_UINT32(20u, platform_link_health().rx_unknown);
    TEST_ASSERT_EQUAL_UINT32(0u, can_host_rx_pending());
}

static void test_take_rejects_bad_arguments(void)
{
    uint8_t buf[PLATFORM_LINK_PAYLOAD_BYTES];
    vehicle_e2e_status_t st;
    TEST_ASSERT_FALSE(platform_link_take(PLATFORM_LINK_RX_COUNT, buf, &st));
    TEST_ASSERT_FALSE(platform_link_take(PLATFORM_LINK_RX_EKF_LEAN, NULL, &st));
    TEST_ASSERT_FALSE(platform_link_take(PLATFORM_LINK_RX_EKF_LEAN, buf, NULL));
    TEST_ASSERT_EQUAL(VEHICLE_E2E_NO_NEW_DATA, platform_link_check_timeout(PLATFORM_LINK_RX_COUNT, 0u));
}

static void test_init_failure_is_inert_and_counted(void)
{
    uint8_t f[FB_LEN] = {0};
    uint8_t buf[PLATFORM_LINK_PAYLOAD_BYTES];
    vehicle_e2e_status_t st;
    bus_reset();
    can_host_set_init_fails(true);
    TEST_ASSERT_FALSE(platform_link_init(0u));
    TEST_ASSERT_FALSE(platform_link_health().initialised);
    platform_link_step(1u);
    platform_link_step(2u);
    TEST_ASSERT_EQUAL_UINT32(2u, platform_link_health().skipped_steps);
    TEST_ASSERT_FALSE(platform_link_send(PLATFORM_LINK_TX_HEARTBEAT_SAFETY, f, FB_LEN));
    TEST_ASSERT_FALSE(platform_link_take(PLATFORM_LINK_RX_EKF_LEAN, buf, &st));
    TEST_ASSERT_EQUAL(VEHICLE_E2E_NO_NEW_DATA, platform_link_check_timeout(PLATFORM_LINK_RX_EKF_LEAN, 5u));
}

/* ------------------------------------------------------------------ TX */

static void test_tx_goes_out_with_the_generated_id_and_length(void)
{
    uint8_t f[FB_LEN] = {1, 2, 3, 4, 5, 6, 7, 8};
    hal_can_frame_t out;
    TEST_ASSERT_TRUE(platform_link_send(PLATFORM_LINK_TX_SAFETY_WARNING_STATUS, f, FB_LEN));
    TEST_ASSERT_TRUE(platform_link_send(PLATFORM_LINK_TX_HEARTBEAT_SAFETY, f, FB_LEN));
    TEST_ASSERT_TRUE(can_host_tx_pop(&out));
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_SAFETY_WARNING_STATUS_FRAME_ID, out.id);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_SAFETY_WARNING_STATUS_LENGTH, out.dlc);
    TEST_ASSERT_FALSE(out.extended);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(f, out.data, FB_LEN);
    TEST_ASSERT_TRUE(can_host_tx_pop(&out));
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_HEARTBEAT_SAFETY_FRAME_ID, out.id);
    TEST_ASSERT_FALSE(can_host_tx_pop(&out));
    TEST_ASSERT_EQUAL_UINT32(1u, platform_link_health().tx_sent[PLATFORM_LINK_TX_SAFETY_WARNING_STATUS]);
    TEST_ASSERT_EQUAL_UINT32(1u, platform_link_health().tx_sent[PLATFORM_LINK_TX_HEARTBEAT_SAFETY]);
}

static void test_tx_refusal_is_counted_and_not_retried(void)
{
    uint8_t f[FB_LEN] = {0};
    hal_can_frame_t out;
    can_host_refuse_next_sends(1u);
    TEST_ASSERT_FALSE(platform_link_send(PLATFORM_LINK_TX_SAFETY_WARNING_STATUS, f, FB_LEN));
    TEST_ASSERT_EQUAL_UINT32(1u, platform_link_health().tx_refused[PLATFORM_LINK_TX_SAFETY_WARNING_STATUS]);
    TEST_ASSERT_EQUAL_UINT32(1u, can_host_tx_refused()); /* the HAL saw exactly one attempt */
    TEST_ASSERT_FALSE(can_host_tx_pop(&out));
    TEST_ASSERT_TRUE(platform_link_send(PLATFORM_LINK_TX_SAFETY_WARNING_STATUS, f, FB_LEN));
    TEST_ASSERT_EQUAL_UINT32(1u, platform_link_health().tx_sent[PLATFORM_LINK_TX_SAFETY_WARNING_STATUS]);
}

static void test_tx_bad_arguments_are_never_sent(void)
{
    uint8_t f[FB_LEN] = {0};
    TEST_ASSERT_FALSE(platform_link_send(PLATFORM_LINK_TX_COUNT, f, FB_LEN));
    TEST_ASSERT_FALSE(platform_link_send(PLATFORM_LINK_TX_HEARTBEAT_SAFETY, NULL, FB_LEN));
    TEST_ASSERT_FALSE(platform_link_send(PLATFORM_LINK_TX_HEARTBEAT_SAFETY, f, FB_LEN - 1u));
    TEST_ASSERT_EQUAL_UINT32(3u, platform_link_health().tx_bad_argument);
    TEST_ASSERT_EQUAL_UINT32(0u, can_host_tx_accepted());
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_all_four_ids_are_accepted_and_checked);
    RUN_TEST(test_timeout_then_resync);
    RUN_TEST(test_counter_jump_and_freeze);
    RUN_TEST(test_crc_error);
    RUN_TEST(test_wrong_dlc_is_never_valid_and_leaves_the_e2e_state);
    RUN_TEST(test_unknown_and_extended_ids_are_ignored);
    RUN_TEST(test_masquerade_two_senders_in_one_pass_is_never_valid);
    RUN_TEST(test_a_failure_cannot_be_hidden_by_a_later_good_frame);
    RUN_TEST(test_rx_burst_is_bounded);
    RUN_TEST(test_take_rejects_bad_arguments);
    RUN_TEST(test_init_failure_is_inert_and_counted);
    RUN_TEST(test_tx_goes_out_with_the_generated_id_and_length);
    RUN_TEST(test_tx_refusal_is_counted_and_not_retried);
    RUN_TEST(test_tx_bad_arguments_are_never_sent);
    return UNITY_END();
}
