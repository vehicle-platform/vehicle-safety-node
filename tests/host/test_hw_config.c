/*
 * L0 tests for hal/hw_config: FLASH_OPTR and IWDG register comparison against the configured
 * values of cubemx/README.md (nSWBOOT0=0 nBOOT0=1 BOR_LEV=4 IWDG_SW=0 NRST_MODE=3; IWDG /32, reload 99).
 */
#include "hal/hw_config.h"

#include <stdint.h>
#include <unity.h>

/* A FLASH_OPTR as the README command leaves it: BOR_LEV 4, IWDG_SW 0, nSWBOOT0 0, nBOOT0 1,
 * every other bit at an arbitrary factory-like value that must be ignored. */
#define OPTR_GOOD (0x38000400u)
#define OPTR_NOISE (0xF3FEE0FFu & ~(HW_CONFIG_OPTR_IWDG_SW_MASK | HW_CONFIG_OPTR_NSWBOOT0_MASK | \
                                   HW_CONFIG_OPTR_NBOOT0_MASK | 0x700u | 0x30000000u))

void setUp(void) {}
void tearDown(void) {}

static void test_configured_value_passes(void)
{
    TEST_ASSERT_EQUAL_UINT32(0u, hw_config_check_optr(OPTR_GOOD));
    TEST_ASSERT_EQUAL_UINT32(0u, hw_config_check_optr(OPTR_GOOD | OPTR_NOISE)); /* other fields ignored */
}

static void test_factory_default_is_flagged(void)
{
    /* RM0440 reset value 0xFFEFF8AA: BOR_LEV 0, IWDG_SW 1, nSWBOOT0 1. */
    const uint32_t bad = hw_config_check_optr(0xFFEFF8AAu);
    TEST_ASSERT_BITS(HW_CONFIG_BAD_BOR_LEV, HW_CONFIG_BAD_BOR_LEV, bad);
    TEST_ASSERT_BITS(HW_CONFIG_BAD_IWDG_SW, HW_CONFIG_BAD_IWDG_SW, bad);
    TEST_ASSERT_BITS(HW_CONFIG_BAD_BOOT_SRC, HW_CONFIG_BAD_BOOT_SRC, bad);
}

static void test_each_field_alone(void)
{
    TEST_ASSERT_EQUAL_UINT32(HW_CONFIG_BAD_BOR_LEV, hw_config_check_optr((OPTR_GOOD & ~0x700u) | 0x300u));
    TEST_ASSERT_EQUAL_UINT32(HW_CONFIG_BAD_BOR_LEV, hw_config_check_optr((OPTR_GOOD & ~0x700u) | 0x500u));
    TEST_ASSERT_EQUAL_UINT32(HW_CONFIG_BAD_IWDG_SW, hw_config_check_optr(OPTR_GOOD | HW_CONFIG_OPTR_IWDG_SW_MASK));
    TEST_ASSERT_EQUAL_UINT32(HW_CONFIG_BAD_BOOT_SRC, hw_config_check_optr(OPTR_GOOD | HW_CONFIG_OPTR_NSWBOOT0_MASK));
    TEST_ASSERT_EQUAL_UINT32(HW_CONFIG_BAD_BOOT_SRC, hw_config_check_optr(OPTR_GOOD & ~HW_CONFIG_OPTR_NBOOT0_MASK));
    TEST_ASSERT_EQUAL_UINT32(HW_CONFIG_BAD_NRST_MODE, hw_config_check_optr((OPTR_GOOD & ~0x30000000u) | 0x20000000u));
    TEST_ASSERT_EQUAL_UINT32(HW_CONFIG_BAD_NRST_MODE, hw_config_check_optr(OPTR_GOOD & ~0x30000000u));
}

static void test_iwdg_registers(void)
{
    TEST_ASSERT_EQUAL_UINT32(0u, hw_config_check_iwdg(3u, 99u, 4095u));
    TEST_ASSERT_EQUAL_UINT32(HW_CONFIG_BAD_IWDG_TIMING, hw_config_check_iwdg(4u, 99u, 4095u));
    TEST_ASSERT_EQUAL_UINT32(HW_CONFIG_BAD_IWDG_TIMING, hw_config_check_iwdg(3u, 4095u, 4095u)); /* reset value */
    TEST_ASSERT_EQUAL_UINT32(HW_CONFIG_BAD_IWDG_TIMING, hw_config_check_iwdg(3u, 99u, 50u));
    TEST_ASSERT_EQUAL_UINT32(HW_CONFIG_BAD_IWDG_TIMING, hw_config_check_iwdg(0u, 0u, 0u));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_configured_value_passes);
    RUN_TEST(test_factory_default_is_flagged);
    RUN_TEST(test_each_field_alone);
    RUN_TEST(test_iwdg_registers);
    return UNITY_END();
}
