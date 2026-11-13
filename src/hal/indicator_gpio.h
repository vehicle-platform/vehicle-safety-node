#ifndef HAL_INDICATOR_GPIO_H
#define HAL_INDICATOR_GPIO_H

/*
 * The two turn-signal inputs IND_L and IND_R (D-075 item 4, D-082; D-111, provisional, defs#165): M4 GPIOs behind PC817 opto-isolators
 * on C1-IO. HAL-free interface; the board port (CubeMX CM4 project, red zone, pin map
 * h755-pin-plan) implements it and the host model is hal/host/indicator_gpio_host.[ch].
 * One bounded, non-blocking read of the raw pin levels (true = HIGH); polarity and
 * debounce are the feature's business (features/indicator/indicator_config.h).
 * Returns false if the pins could not be read; the outputs are then unspecified and the
 * caller treats both inputs as inactive (open-wire level).
 * Board port: besides the external pull-up on C1-IO, enable the MCU internal pull-up on
 * both indicator pins, so a lost connector or a floating pin reads HIGH = not signalling.
 */

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool hal_indicator_gpio_read(bool *left_high, bool *right_high);

#ifdef __cplusplus
}
#endif

#endif
