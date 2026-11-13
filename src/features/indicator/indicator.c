#include "features/indicator/indicator.h"

#include "hal/indicator_gpio.h"

#include <stdbool.h>

static indicator_core_t g_core;
static indicator_output_t g_out = {INDICATOR_STATE_NONE, 0u};

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the blind-spot feature, which does not exist yet
void indicator_init(void)
{
    indicator_core_init(&g_core);
    g_out.state = INDICATOR_STATE_NONE;
    g_out.fault_flags = 0u;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the blind-spot feature, which does not exist yet
void indicator_step(uint32_t now_ms)
{
    bool left_high = true;
    bool right_high = true;
    if (!hal_indicator_gpio_read(&left_high, &right_high)) {
        /* A failed read looks like an open wire: both inputs inactive. */
        left_high = true;
        right_high = true;
    }
    g_out = indicator_core_step(&g_core, now_ms, left_high, right_high);
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the blind-spot feature, which does not exist yet
indicator_output_t indicator_output(void)
{
    return g_out;
}
