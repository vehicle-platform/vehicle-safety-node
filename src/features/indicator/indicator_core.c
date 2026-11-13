#include "features/indicator/indicator_core.h"

#include "features/indicator/indicator_config.h"

#include <stddef.h>

static void side_init(indicator_side_t *s)
{
    s->cand_since_ms = 0u;
    s->last_edge_ms = 0u;
    s->active_since_ms = 0u;
    s->cand_count = 0u;
    s->primed = false;
    s->raw_active = false;
    s->debounced = false;
    s->has_edge = false;
    s->stuck = false;
}

static void side_step(indicator_side_t *s, uint32_t now_ms, bool active)
{
    if (!s->primed) {
        s->primed = true;
        s->raw_active = active;
        s->cand_since_ms = now_ms;
        s->cand_count = 1u;
    } else if (active != s->raw_active) {
        s->raw_active = active;
        s->cand_since_ms = now_ms;
        s->cand_count = 1u;
    } else if (s->cand_count < UINT8_MAX) {
        s->cand_count = (uint8_t)(s->cand_count + 1u);
    } else {
        /* saturated */
    }

    if ((active != s->debounced) && (s->cand_count >= INDICATOR_DEBOUNCE_SAMPLES) &&
        ((uint32_t)(now_ms - s->cand_since_ms) >= INDICATOR_DEBOUNCE_MS)) {
        s->debounced = active;
        if (active) {
            s->has_edge = true;
            s->last_edge_ms = now_ms;
            s->active_since_ms = now_ms;
        } else if (s->stuck) {
            /* The release of a stuck input is not a signalling edge. */
            s->stuck = false;
            s->has_edge = false;
        } else {
            s->has_edge = true;
            s->last_edge_ms = now_ms;
        }
    }

    if (s->debounced && !s->stuck &&
        ((uint32_t)(now_ms - s->active_since_ms) >= INDICATOR_STUCK_MS)) {
        s->stuck = true;
    }

    /* Drop an expired hold at once so a wrap of now_ms can never revive it. */
    if (s->has_edge &&
        ((s->stuck) || ((uint32_t)(now_ms - s->last_edge_ms) >= INDICATOR_HOLD_MS))) {
        s->has_edge = false;
    }
}

void indicator_core_init(indicator_core_t *c)
{
    if (c == NULL) {
        return;
    }
    side_init(&c->left);
    side_init(&c->right);
}

indicator_output_t indicator_core_step(indicator_core_t *c, uint32_t now_ms,
                                       bool left_pin_high, bool right_pin_high)
{
    indicator_output_t out = {INDICATOR_STATE_NONE, 0u};
    if (c == NULL) {
        return out;
    }

#if INDICATOR_ACTIVE_LEVEL_LOW
    side_step(&c->left, now_ms, !left_pin_high);
    side_step(&c->right, now_ms, !right_pin_high);
#else
    side_step(&c->left, now_ms, left_pin_high);
    side_step(&c->right, now_ms, right_pin_high);
#endif

    if (c->left.has_edge && c->right.has_edge) {
        out.state = INDICATOR_STATE_HAZARD;
    } else if (c->left.has_edge) {
        out.state = INDICATOR_STATE_LEFT;
    } else if (c->right.has_edge) {
        out.state = INDICATOR_STATE_RIGHT;
    } else {
        out.state = INDICATOR_STATE_NONE;
    }
    if (c->left.stuck) {
        out.fault_flags = (uint8_t)(out.fault_flags | INDICATOR_FAULT_LEFT_STUCK);
    }
    if (c->right.stuck) {
        out.fault_flags = (uint8_t)(out.fault_flags | INDICATOR_FAULT_RIGHT_STUCK);
    }
    return out;
}
