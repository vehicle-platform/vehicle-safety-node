#ifndef FEATURES_INDICATOR_INDICATOR_H
#define FEATURES_INDICATOR_INDICATOR_H

/*
 * Turn-signal input glue (D-111): reads IND_L / IND_R through hal/indicator_gpio.h once
 * per super-loop pass and feeds indicator_core. See indicator_core.h for the SAFETY
 * CONTRACT: the result may only escalate a blind-spot warning, never suppress one.
 * Not yet called from app/safety_app.c: the blind-spot decision that would consume it
 * does not exist (radar hardware pending), and a new input must not touch the cornering
 * pass until it has a consumer.
 */

#include "features/indicator/indicator_core.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Boot, before the super loop: NONE. */
void indicator_init(void);

/* Once per super-loop pass with the caller's monotonic now_ms. */
void indicator_step(uint32_t now_ms);

indicator_output_t indicator_output(void);

#ifdef __cplusplus
}
#endif

#endif
