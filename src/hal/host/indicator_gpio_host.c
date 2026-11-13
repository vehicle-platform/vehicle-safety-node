#include "hal/host/indicator_gpio_host.h"

#include <stddef.h>

static bool g_left_high = true;
static bool g_right_high = true;
static bool g_fail = false;

void indicator_gpio_host_set(bool left_high, bool right_high)
{
    g_left_high = left_high;
    g_right_high = right_high;
}

void indicator_gpio_host_set_fail(bool fail)
{
    g_fail = fail;
}

bool hal_indicator_gpio_read(bool *left_high, bool *right_high)
{
    if (g_fail) {
        return false;
    }
    if (left_high != NULL) {
        *left_high = g_left_high;
    }
    if (right_high != NULL) {
        *right_high = g_right_high;
    }
    return true;
}
