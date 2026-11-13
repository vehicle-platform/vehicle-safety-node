#ifndef HAL_INDICATOR_GPIO_H
#define HAL_INDICATOR_GPIO_H

/*
 * The two turn-signal inputs IND_L and IND_R (D-111): M4 GPIOs behind PC817 opto-isolators
 * on C1-IO. HAL-free interface; the board port (CubeMX CM4 project, red zone, pin map
 * h755-pin-plan) implements it and the host model is hal/host/indicator_gpio_host.[ch].
 * One bounded, non-blocking read of the raw pin levels (true = HIGH); polarity and
 * debounce are the feature's business (features/indicator/indicator_config.h).
 */

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void hal_indicator_gpio_read(bool *left_high, bool *right_high);

#ifdef __cplusplus
}
#endif

#endif
