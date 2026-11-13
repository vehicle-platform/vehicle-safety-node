#ifndef HAL_HOST_INDICATOR_GPIO_HOST_H
#define HAL_HOST_INDICATOR_GPIO_HOST_H

/* Host model of hal/indicator_gpio.h for the tests: the pin levels are set by the test
 * (default both HIGH = not signalling, like an open wire with the pull-up). */

#include "hal/indicator_gpio.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void indicator_gpio_host_set(bool left_high, bool right_high);

/* Makes hal_indicator_gpio_read() fail (return false, outputs left untouched) when true. */
void indicator_gpio_host_set_fail(bool fail);

#ifdef __cplusplus
}
#endif

#endif
