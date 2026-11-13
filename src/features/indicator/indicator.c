#include "features/indicator/indicator.h"

#include "hal/indicator_gpio.h"

#include <stdbool.h>

static indicator_core_t g_core;
static indicator_output_t g_out = {INDICATOR_STATE_NONE, 0u};

void indicator_init(void)
{
    indicator_core_init(&g_core);
    g_out.state = INDICATOR_STATE_NONE;
    g_out.fault_flags = 0u;
}

void indicator_step(uint32_t now_ms)
{
    bool left_high = true; /* a failed read looks like an open wire: not signalling */
    bool right_high = true;
    hal_indicator_gpio_read(&left_high, &right_high);
    g_out = indicator_core_step(&g_core, now_ms, left_high, right_high);
}

indicator_output_t indicator_output(void)
{
    return g_out;
}
