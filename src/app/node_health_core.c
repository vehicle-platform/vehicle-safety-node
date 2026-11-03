#include "app/node_health_core.h"

#include "platform.h"

#include <stddef.h>

_Static_assert(NODE_MODE_INIT == PLATFORM_HEARTBEAT_SAFETY_NODE_MODE_INIT_CHOICE, "0x082 NODE_MODE choice");
_Static_assert(NODE_MODE_NORMAL == PLATFORM_HEARTBEAT_SAFETY_NODE_MODE_NORMAL_CHOICE, "0x082 NODE_MODE choice");
_Static_assert(NODE_MODE_DEGRADED == PLATFORM_HEARTBEAT_SAFETY_NODE_MODE_DEGRADED_CHOICE, "0x082 NODE_MODE choice");

#define NODE_ERROR_COUNT_MAX (255u) /* the 8-bit ERROR_COUNT, 0..255 in the DBC */
#define NODE_UPTIME_MAX (65535u)    /* the 16-bit UPTIME, 0..65535 s in the DBC */
#define NODE_MS_PER_S (1000u)
#define NODE_FAULT_BITS (3u)
_Static_assert(NODE_FAULT_ALL == ((1u << NODE_FAULT_BITS) - 1u), "count_onsets scans every fault bit");

void node_health_init(node_health_t* h, uint32_t now_ms)
{
    if (h != NULL) {
        h->pass_completed = false;
        h->level_prev = 0u;
        h->seen = 0u;
        h->error_count = 0u;
        h->uptime_s = 0u;
        h->rem_ms = 0u;
        h->last_ms = now_ms;
    }
}

static void count_onsets(node_health_t* h, uint32_t onsets)
{
    uint32_t i;
    for (i = 0u; i < NODE_FAULT_BITS; i++) {
        if (((onsets >> i) & 1u) != 0u) {
            if (h->error_count < NODE_ERROR_COUNT_MAX) {
                h->error_count++;
            }
        }
    }
}

void node_health_pass(node_health_t* h, uint32_t level_faults, uint32_t event_faults, uint32_t now_ms)
{
    uint32_t lvl;
    uint32_t evt;
    uint32_t total;
    if (h == NULL) {
        return;
    }
    lvl = level_faults & NODE_FAULT_ALL;
    evt = event_faults & NODE_FAULT_ALL;
    count_onsets(h, lvl & ~h->level_prev);
    count_onsets(h, evt);
    h->level_prev = lvl;
    h->seen |= (lvl | evt);

    total = h->rem_ms + (uint32_t)(now_ms - h->last_ms); /* wrap-safe delta */
    if (total < h->rem_ms) {
        total = UINT32_MAX; /* cannot happen with rem_ms < 1000 and a 32-bit delta, kept safe */
    }
    h->last_ms = now_ms;
    h->uptime_s += total / NODE_MS_PER_S;
    if (h->uptime_s > NODE_UPTIME_MAX) {
        h->uptime_s = NODE_UPTIME_MAX;
    }
    h->rem_ms = total % NODE_MS_PER_S;
}

void node_health_pass_done(node_health_t* h)
{
    if (h != NULL) {
        h->pass_completed = true;
    }
}

uint8_t node_health_mode(const node_health_t* h, bool source_ekf)
{
    uint8_t mode = (uint8_t)NODE_MODE_DEGRADED;
    if (h != NULL) {
        if (!h->pass_completed) {
            mode = (uint8_t)NODE_MODE_INIT;
        } else if (source_ekf && (h->seen == 0u)) {
            mode = (uint8_t)NODE_MODE_NORMAL;
        } else {
            mode = (uint8_t)NODE_MODE_DEGRADED;
        }
    }
    return mode;
}

void node_health_frame_sent(node_health_t* h)
{
    if (h != NULL) {
        h->seen = 0u;
    }
}

uint8_t node_health_error_count(const node_health_t* h)
{
    return (h != NULL) ? h->error_count : 0u;
}

uint16_t node_health_uptime(const node_health_t* h)
{
    return (h != NULL) ? (uint16_t)h->uptime_s : 0u;
}
