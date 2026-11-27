/*
 * L0 tests for hal/reset_cause: the RCC_CSR decode (each flag alone, the multi-flag priority of
 * real resets, ignored bits), the IWDG reset counter with its magic + inverted-copy integrity
 * check (cold-boot garbage, corruption, saturation, clearing causes) and boot_should_safe_halt.
 */
#include "hal/reset_cause.h"

#include <stdint.h>
#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

static void test_single_flags(void)
{
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_IWDG, reset_cause_decode(RESET_CAUSE_CSR_IWDGRSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_WWDG, reset_cause_decode(RESET_CAUSE_CSR_WWDGRSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_LOW_POWER, reset_cause_decode(RESET_CAUSE_CSR_LPWRRSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_SOFTWARE, reset_cause_decode(RESET_CAUSE_CSR_SFTRSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_OPTION_BYTE_LOAD, reset_cause_decode(RESET_CAUSE_CSR_OBLRSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_POWER_ON_OR_BROWN_OUT, reset_cause_decode(RESET_CAUSE_CSR_BORRSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_PIN, reset_cause_decode(RESET_CAUSE_CSR_PINRSTF));
}

static void test_real_resets_set_the_pin_flag_too(void)
{
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_IWDG, reset_cause_decode(RESET_CAUSE_CSR_IWDGRSTF | RESET_CAUSE_CSR_PINRSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_SOFTWARE, reset_cause_decode(RESET_CAUSE_CSR_SFTRSTF | RESET_CAUSE_CSR_PINRSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_POWER_ON_OR_BROWN_OUT,
                          reset_cause_decode(RESET_CAUSE_CSR_BORRSTF | RESET_CAUSE_CSR_PINRSTF));
}

static void test_priority_order(void)
{
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_IWDG, reset_cause_decode(0xFE000000u));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_WWDG, reset_cause_decode(0x5E000000u));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_LOW_POWER, reset_cause_decode(0x9E000000u & ~RESET_CAUSE_CSR_IWDGRSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_SOFTWARE,
                          reset_cause_decode(RESET_CAUSE_CSR_SFTRSTF | RESET_CAUSE_CSR_OBLRSTF |
                                             RESET_CAUSE_CSR_BORRSTF | RESET_CAUSE_CSR_PINRSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_OPTION_BYTE_LOAD,
                          reset_cause_decode(RESET_CAUSE_CSR_OBLRSTF | RESET_CAUSE_CSR_BORRSTF));
}

static void test_real_rcc_csr_combinations(void)
{
    /* RM0440 RCC_CSR: the internal reset drives NRST, so PINRSTF is set on EVERY reset; a
     * power-on reset also sets BORRSTF; OBLRSTF accompanies an option-byte reload. Each case is
     * the flag set software really reads, with LSION (bit 0) and LSIRDY (bit 1) possibly set. */
    const uint32_t lsi = 0x3u;
    const uint32_t pin = RESET_CAUSE_CSR_PINRSTF;
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_POWER_ON_OR_BROWN_OUT, reset_cause_decode(pin | RESET_CAUSE_CSR_BORRSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_POWER_ON_OR_BROWN_OUT, reset_cause_decode(pin | RESET_CAUSE_CSR_BORRSTF | lsi));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_PIN, reset_cause_decode(pin)); /* external NRST: PIN only */
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_PIN, reset_cause_decode(pin | lsi));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_SOFTWARE, reset_cause_decode(pin | RESET_CAUSE_CSR_SFTRSTF | lsi));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_IWDG, reset_cause_decode(pin | RESET_CAUSE_CSR_IWDGRSTF | lsi));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_WWDG, reset_cause_decode(pin | RESET_CAUSE_CSR_WWDGRSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_LOW_POWER, reset_cause_decode(pin | RESET_CAUSE_CSR_LPWRRSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_OPTION_BYTE_LOAD, reset_cause_decode(pin | RESET_CAUSE_CSR_OBLRSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_OPTION_BYTE_LOAD, reset_cause_decode(pin | RESET_CAUSE_CSR_OBLRSTF | RESET_CAUSE_CSR_BORRSTF));
    /* Stale flags that were never cleared: the most specific one still wins. */
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_IWDG, reset_cause_decode(pin | RESET_CAUSE_CSR_IWDGRSTF | RESET_CAUSE_CSR_SFTRSTF | RESET_CAUSE_CSR_BORRSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_WWDG, reset_cause_decode(pin | RESET_CAUSE_CSR_WWDGRSTF | RESET_CAUSE_CSR_SFTRSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_LOW_POWER, reset_cause_decode(pin | RESET_CAUSE_CSR_LPWRRSTF | RESET_CAUSE_CSR_SFTRSTF | RESET_CAUSE_CSR_BORRSTF));
}

static void test_counter_follows_real_sequence(void)
{
    /* power-on -> IWDG -> IWDG (PIN accompanies each) -> software fault reset -> power cycle */
    reset_counter_t rc = {0u, 0u, 0u};
    const uint32_t pin = RESET_CAUSE_CSR_PINRSTF;
    TEST_ASSERT_EQUAL_UINT32(0u, reset_counter_update(&rc, reset_cause_decode(pin | RESET_CAUSE_CSR_BORRSTF)));
    TEST_ASSERT_EQUAL_UINT32(1u, reset_counter_update(&rc, reset_cause_decode(pin | RESET_CAUSE_CSR_IWDGRSTF)));
    TEST_ASSERT_EQUAL_UINT32(2u, reset_counter_update(&rc, reset_cause_decode(pin | RESET_CAUSE_CSR_IWDGRSTF)));
    TEST_ASSERT_EQUAL_UINT32(2u, reset_counter_update(&rc, reset_cause_decode(pin | RESET_CAUSE_CSR_SFTRSTF)));
    TEST_ASSERT_EQUAL_UINT32(0u, reset_counter_update(&rc, reset_cause_decode(pin | RESET_CAUSE_CSR_BORRSTF)));
}

static void test_unknown_and_ignored_bits(void)
{
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_UNKNOWN, reset_cause_decode(0u));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_UNKNOWN, reset_cause_decode(RESET_CAUSE_CSR_RMVF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_UNKNOWN, reset_cause_decode(0x00000001u)); /* LSION only */
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_IWDG, reset_cause_decode(RESET_CAUSE_CSR_IWDGRSTF | RESET_CAUSE_CSR_RMVF | 1u));
}

static void test_cold_boot_garbage_restarts_at_zero(void)
{
    const reset_counter_t garbage[] = {
        {0u, 0u, 0u},
        {0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu},
        {0xA5A5A5A5u, 7u, ~7u},               /* wrong magic */
        {RESET_COUNTER_MAGIC, 7u, 7u},         /* inverse mismatch */
        {RESET_COUNTER_MAGIC, 0u, 0u},         /* zeroed RAM: inverse of 0 is not 0 */
        {RESET_COUNTER_MAGIC, 0x10000u, ~0x10000u}, /* count beyond the limit */
    };
    for (unsigned i = 0; i < sizeof(garbage) / sizeof(garbage[0]); i++) {
        reset_counter_t rc = garbage[i];
        TEST_ASSERT_FALSE(reset_counter_valid(&rc));
        TEST_ASSERT_EQUAL_UINT32(0u, reset_counter_update(&rc, RESET_CAUSE_SOFTWARE));
        TEST_ASSERT_TRUE(reset_counter_valid(&rc));
        rc = garbage[i];
        TEST_ASSERT_EQUAL_UINT32(1u, reset_counter_update(&rc, RESET_CAUSE_IWDG)); /* garbage never inflates */
    }
}

static void test_iwdg_counts_consecutively(void)
{
    reset_counter_t rc = {0u, 0u, 0u};
    TEST_ASSERT_EQUAL_UINT32(0u, reset_counter_update(&rc, RESET_CAUSE_POWER_ON_OR_BROWN_OUT));
    TEST_ASSERT_EQUAL_UINT32(1u, reset_counter_update(&rc, RESET_CAUSE_IWDG));
    TEST_ASSERT_EQUAL_UINT32(2u, reset_counter_update(&rc, RESET_CAUSE_IWDG));
    TEST_ASSERT_EQUAL_UINT32(3u, reset_counter_update(&rc, RESET_CAUSE_IWDG));
    TEST_ASSERT_TRUE(reset_counter_valid(&rc));
}

static void test_other_causes_keep_or_clear(void)
{
    reset_counter_t rc;
    reset_counter_set(&rc, 4u);
    TEST_ASSERT_EQUAL_UINT32(4u, reset_counter_update(&rc, RESET_CAUSE_SOFTWARE));
    TEST_ASSERT_EQUAL_UINT32(4u, reset_counter_update(&rc, RESET_CAUSE_WWDG));
    TEST_ASSERT_EQUAL_UINT32(4u, reset_counter_update(&rc, RESET_CAUSE_LOW_POWER));
    TEST_ASSERT_EQUAL_UINT32(4u, reset_counter_update(&rc, RESET_CAUSE_OPTION_BYTE_LOAD));
    TEST_ASSERT_EQUAL_UINT32(4u, reset_counter_update(&rc, RESET_CAUSE_UNKNOWN));
    TEST_ASSERT_EQUAL_UINT32(0u, reset_counter_update(&rc, RESET_CAUSE_PIN));
    reset_counter_set(&rc, 4u);
    TEST_ASSERT_EQUAL_UINT32(0u, reset_counter_update(&rc, RESET_CAUSE_POWER_ON_OR_BROWN_OUT));
}

static void test_saturates_and_detects_corruption(void)
{
    reset_counter_t rc;
    reset_counter_set(&rc, RESET_COUNTER_MAX);
    TEST_ASSERT_EQUAL_UINT32(RESET_COUNTER_MAX, reset_counter_update(&rc, RESET_CAUSE_IWDG));
    reset_counter_set(&rc, 3u);
    rc.count ^= 0x4u; /* single bit flip */
    TEST_ASSERT_FALSE(reset_counter_valid(&rc));
    rc.count = 3u;
    rc.count_inv ^= 0x80000000u;
    TEST_ASSERT_FALSE(reset_counter_valid(&rc));
}

static void test_safe_halt_policy(void)
{
    TEST_ASSERT_FALSE(boot_should_safe_halt(0u, 3u));
    TEST_ASSERT_FALSE(boot_should_safe_halt(2u, 3u));
    TEST_ASSERT_TRUE(boot_should_safe_halt(3u, 3u));
    TEST_ASSERT_TRUE(boot_should_safe_halt(RESET_COUNTER_MAX, 3u));
    TEST_ASSERT_FALSE(boot_should_safe_halt(100u, 0u)); /* limit 0 = policy off */
    TEST_ASSERT_TRUE(boot_should_safe_halt(1u, 1u));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_single_flags);
    RUN_TEST(test_real_resets_set_the_pin_flag_too);
    RUN_TEST(test_priority_order);
    RUN_TEST(test_real_rcc_csr_combinations);
    RUN_TEST(test_counter_follows_real_sequence);
    RUN_TEST(test_unknown_and_ignored_bits);
    RUN_TEST(test_cold_boot_garbage_restarts_at_zero);
    RUN_TEST(test_iwdg_counts_consecutively);
    RUN_TEST(test_other_causes_keep_or_clear);
    RUN_TEST(test_saturates_and_detects_corruption);
    RUN_TEST(test_safe_halt_policy);
    return UNITY_END();
}
