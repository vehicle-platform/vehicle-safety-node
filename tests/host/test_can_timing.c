/*
 * L0 tests for hal/can_timing (pure FDCAN nominal bit-timing arithmetic and the NBTP read-back
 * decode): the cubemx/ image values (170 MHz kernel, prescaler 20, seg1 14, seg2 2, SJW 2 =
 * 500 kbit/s, sample point 88.2 %), wrong prescaler/clock, field ranges, SJW rule, sample window.
 */
#include "hal/can_timing.h"

#include <stdint.h>
#include <unity.h>

#define KERNEL_HZ 170000000u
#define BPS 500000u

/* NBTP as HAL_FDCAN_Init writes it: fields hold the value minus one. */
#define NBTP(sjw, brp, s1, s2) \
    ((((uint32_t)(sjw) - 1u) << 25) | (((uint32_t)(brp) - 1u) << 16) | (((uint32_t)(s1) - 1u) << 8) | ((uint32_t)(s2) - 1u))

void setUp(void) {}
void tearDown(void) {}

static void test_image_values(void)
{
    TEST_ASSERT_EQUAL_UINT32(BPS, can_timing_bitrate(KERNEL_HZ, 20u, 14u, 2u));
    TEST_ASSERT_TRUE(can_timing_ok(KERNEL_HZ, 20u, 14u, 2u, 2u, BPS));
}

static void test_wrong_values(void)
{
    TEST_ASSERT_FALSE(can_timing_ok(KERNEL_HZ, 10u, 14u, 2u, 2u, BPS));       /* 1 Mbit/s */
    TEST_ASSERT_FALSE(can_timing_ok(80000000u, 20u, 14u, 2u, 2u, BPS));       /* wrong clock tree */
    TEST_ASSERT_FALSE(can_timing_ok(0u, 20u, 14u, 2u, 2u, BPS));
    TEST_ASSERT_FALSE(can_timing_ok(KERNEL_HZ, 20u, 14u, 2u, 2u, 0u));
    TEST_ASSERT_FALSE(can_timing_ok(KERNEL_HZ, 20u, 14u, 2u, 3u, BPS));       /* sjw > seg2 */
    TEST_ASSERT_FALSE(can_timing_ok(KERNEL_HZ, 20u, 14u, 2u, 0u, BPS));       /* sjw 0 */
    TEST_ASSERT_FALSE(can_timing_ok(KERNEL_HZ, 0u, 14u, 2u, 2u, BPS));
    TEST_ASSERT_FALSE(can_timing_ok(KERNEL_HZ, 513u, 14u, 2u, 2u, BPS));
    TEST_ASSERT_FALSE(can_timing_ok(KERNEL_HZ, 20u, 257u, 2u, 2u, BPS));
    TEST_ASSERT_FALSE(can_timing_ok(KERNEL_HZ, 20u, 14u, 129u, 2u, BPS));
    TEST_ASSERT_EQUAL_UINT32(0u, can_timing_bitrate(KERNEL_HZ, 0u, 1u, 1u));
}

static void test_sample_point_window(void)
{
    /* 170 MHz / (10 * 34) = 500 kbit/s; seg2 = 8 -> sample (1+25)/34 = 76.5 % ok, seg2 = 12 -> too early. */
    TEST_ASSERT_TRUE(can_timing_ok(KERNEL_HZ, 10u, 25u, 8u, 2u, BPS));
    TEST_ASSERT_FALSE(can_timing_ok(KERNEL_HZ, 10u, 21u, 12u, 2u, BPS));
    TEST_ASSERT_FALSE(can_timing_ok(KERNEL_HZ, 10u, 32u, 1u, 1u, BPS));       /* 33/34 = 97 %: too late */
}

static void test_no_truncated_rate(void)
{
    /* 170 MHz / (21 * 17) = 476190 bit/s truncated: never accepted as 476190 unless exact. */
    TEST_ASSERT_FALSE(can_timing_ok(KERNEL_HZ, 21u, 14u, 2u, 2u, 476190u));
}

static void test_kernel_divider(void)
{
    TEST_ASSERT_EQUAL_UINT32(170000000u, can_timing_kernel_hz(170000000u, 0u)); /* PDIV 0: /1 */
    TEST_ASSERT_EQUAL_UINT32(85000000u, can_timing_kernel_hz(170000000u, 1u));  /* /2 */
    TEST_ASSERT_EQUAL_UINT32(5666666u, can_timing_kernel_hz(170000000u, 15u));  /* /30 */
    TEST_ASSERT_EQUAL_UINT32(0u, can_timing_kernel_hz(170000000u, 16u));        /* out of range */
    TEST_ASSERT_EQUAL_UINT32(0u, can_timing_kernel_hz(0u, 0u));
    /* the divided clock still has to match the NBTP: PDIV 1 with prescaler 10 gives 500 kbit/s */
    TEST_ASSERT_TRUE(can_timing_nbtp_ok(can_timing_kernel_hz(170000000u, 1u), NBTP(2u, 10u, 14u, 2u), BPS));
    TEST_ASSERT_FALSE(can_timing_nbtp_ok(can_timing_kernel_hz(170000000u, 1u), NBTP(2u, 20u, 14u, 2u), BPS));
}

static void test_nbtp_readback(void)
{
    TEST_ASSERT_TRUE(can_timing_nbtp_ok(KERNEL_HZ, NBTP(2u, 20u, 14u, 2u), BPS));
    TEST_ASSERT_FALSE(can_timing_nbtp_ok(KERNEL_HZ, NBTP(2u, 10u, 14u, 2u), BPS));
    TEST_ASSERT_FALSE(can_timing_nbtp_ok(KERNEL_HZ, NBTP(2u, 20u, 14u, 2u), 1000000u));
    TEST_ASSERT_FALSE(can_timing_nbtp_ok(0u, NBTP(2u, 20u, 14u, 2u), BPS));      /* unreadable clock */
    TEST_ASSERT_FALSE(can_timing_nbtp_ok(KERNEL_HZ, 0u, BPS));                    /* 1/1/1/1: 170 Mbit/s */
    TEST_ASSERT_FALSE(can_timing_nbtp_ok(KERNEL_HZ, 0x06000A33u, BPS));           /* M_CAN reset value */
    TEST_ASSERT_FALSE(can_timing_nbtp_ok(KERNEL_HZ, 0xFFFFFFFFu, BPS));           /* all ones, no wrap */
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_image_values);
    RUN_TEST(test_wrong_values);
    RUN_TEST(test_sample_point_window);
    RUN_TEST(test_no_truncated_rate);
    RUN_TEST(test_kernel_divider);
    RUN_TEST(test_nbtp_readback);
    return UNITY_END();
}
