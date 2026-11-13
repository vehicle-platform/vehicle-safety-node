/* L0 test of the indicator glue: the HAL read is mapped through the core. */
#include "features/indicator/indicator.h"
#include "hal/host/indicator_gpio_host.h"

#include <unity.h>

void setUp(void)
{
    indicator_gpio_host_set(true, true);
    indicator_gpio_host_set_fail(false);
    indicator_init();
}
void tearDown(void) {}

static void test_open_pins_read_none(void)
{
    uint32_t t;
    for (t = 0u; t < 5000u; t += 5u) {
        indicator_step(t);
    }
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, indicator_output().state);
    TEST_ASSERT_EQUAL_UINT8(0u, indicator_output().fault_flags);
}

static void test_low_pin_signals_and_init_resets(void)
{
    uint32_t t;
    indicator_gpio_host_set(true, false); /* IND_R lamp lit */
    for (t = 0u; t < 100u; t += 5u) {
        indicator_step(t);
    }
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_RIGHT, indicator_output().state);
    indicator_init();
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, indicator_output().state);
}

static void test_failed_hal_read_reads_as_open_wire(void)
{
    uint32_t t;
    indicator_gpio_host_set(false, false); /* the pins would say signalling ... */
    indicator_gpio_host_set_fail(true);    /* ... but the read fails */
    for (t = 0u; t < 6000u; t += 5u) {
        indicator_step(t);
    }
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, indicator_output().state);
    TEST_ASSERT_EQUAL_UINT8(0u, indicator_output().fault_flags);
    /* a read failing in the middle of signalling lets the hold expire */
    indicator_gpio_host_set_fail(false);
    for (t = 6000u; t < 6100u; t += 5u) {
        indicator_step(t);
    }
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_HAZARD, indicator_output().state);
    indicator_gpio_host_set_fail(true);
    for (t = 6100u; t < 6100u + 40u; t += 5u) {
        indicator_step(t);
    }
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_HAZARD, indicator_output().state); /* hold, not suppression */
    for (t = 6140u; t < 6100u + 1700u; t += 5u) {
        indicator_step(t);
    }
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, indicator_output().state);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_open_pins_read_none);
    RUN_TEST(test_low_pin_signals_and_init_resets);
    RUN_TEST(test_failed_hal_read_reads_as_open_wire);
    return UNITY_END();
}
