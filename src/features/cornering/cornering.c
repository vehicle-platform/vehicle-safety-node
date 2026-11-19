#include "features/cornering/cornering.h"

#include "platform.h"
#include "platform_e2e.h"
#include "platform_limits.h"
#include "services/platform_link.h"

#include <string.h>

typedef struct {
    cornering_select_t sel;
    cornering_hyst_t hyst;
    cornering_output_t out;
    float lean_deg; /* the latest usable lean, valid while status.lean_live */
    bool have_mu;       /* a valid 0x022 since boot */
    float mu_valid_eff; /* the D-029 result of the last valid 0x022 */
    cornering_status_t status;
} cornering_ctx_t;

static cornering_ctx_t ctx;

void cornering_init(void)
{
    (void)memset(&ctx, 0, sizeof ctx);
    cornering_select_init(&ctx.sel);
    cornering_hyst_reset(&ctx.hyst);
    ctx.out.level = CORNERING_LEVEL_UNAVAILABLE;
    ctx.out.source = CORNERING_SOURCE_NONE;
    ctx.status.lean_e2e = VEHICLE_E2E_NO_NEW_DATA;
    ctx.status.mu_e2e = VEHICLE_E2E_NO_NEW_DATA;
    ctx.status.hb_e2e = VEHICLE_E2E_NO_NEW_DATA;
    ctx.status.rt_core_inputs_lost = true;
    ctx.status.mu_eff = PLATFORM_LIMIT_FRICTION_COEFF_DEFAULT;
}

static void take_heartbeat(uint32_t now_ms)
{
    uint8_t buf[PLATFORM_LINK_PAYLOAD_BYTES];
    struct platform_heartbeat_rt_core_t hb;
    if (platform_link_take(PLATFORM_LINK_RX_HEARTBEAT_RT_CORE, buf, &ctx.status.hb_e2e)) {
        bool ok = false;
        if (vehicle_e2e_is_valid(ctx.status.hb_e2e) &&
            (platform_heartbeat_rt_core_unpack(&hb, buf, PLATFORM_HEARTBEAT_RT_CORE_LENGTH) == 0) &&
            (hb.rt_core_node_mode == PLATFORM_HEARTBEAT_RT_CORE_RT_CORE_NODE_MODE_NORMAL_CHOICE)) {
            ok = true;
        } else {
            ctx.status.hb_rejected++;
        }
        ctx.status.hb_ok = ok;
    }
    if (platform_link_check_timeout(PLATFORM_LINK_RX_HEARTBEAT_RT_CORE, now_ms) == VEHICLE_E2E_NO_NEW_DATA) {
        if (ctx.status.hb_ok) {
            ctx.status.hb_timeouts++;
        }
        ctx.status.hb_ok = false;
    }
}

static bool take_friction(uint32_t now_ms)
{
    uint8_t buf[PLATFORM_LINK_PAYLOAD_BYTES];
    struct platform_ekf_friction_mass_t m;
    bool fresh = false;
    if (platform_link_take(PLATFORM_LINK_RX_EKF_FRICTION_MASS, buf, &ctx.status.mu_e2e)) {
        fresh = true;
        if (!vehicle_e2e_is_valid(ctx.status.mu_e2e)) {
            ctx.status.mu_live = false; /* D-101 item 5: frame lost -> min(last valid, default) */
            ctx.status.mu_e2e_rejected++;
        } else if ((platform_ekf_friction_mass_unpack(&m, buf, PLATFORM_EKF_FRICTION_MASS_LENGTH) == 0) &&
                   cornering_mu_value(m.friction_coeff_state, m.friction_coeff, &ctx.mu_valid_eff)) {
            ctx.have_mu = true;
            ctx.status.mu_live = true;
        } else {
            ctx.status.mu_live = false; /* D-101 item 5: value unusable -> min(last valid, default) */
            ctx.status.mu_value_rejected++;
        }
    }
    if (platform_link_check_timeout(PLATFORM_LINK_RX_EKF_FRICTION_MASS, now_ms) == VEHICLE_E2E_NO_NEW_DATA) {
        if (ctx.status.mu_live) {
            ctx.status.mu_timeouts++;
            fresh = true; /* mu_eff becomes min(last valid, default): re-evaluate */
        }
        ctx.status.mu_live = false;
    }
    return fresh;
}

/* Returns true if a 0x020 frame was taken; *usable is its result. */
static bool take_lean(uint32_t now_ms, bool* usable)
{
    uint8_t buf[PLATFORM_LINK_PAYLOAD_BYTES];
    struct platform_ekf_lean_t m;
    float deg = 0.0f;
    bool taken = false;
    bool ok = false;
    if (platform_link_take(PLATFORM_LINK_RX_EKF_LEAN, buf, &ctx.status.lean_e2e)) {
        taken = true;
        if (vehicle_e2e_is_valid(ctx.status.lean_e2e) &&
            (platform_ekf_lean_unpack(&m, buf, PLATFORM_EKF_LEAN_LENGTH) == 0) &&
            cornering_lean_usable(m.lean_angle_state, m.lean_angle, &deg)) {
            ok = true;
            ctx.lean_deg = deg;
        } else {
            ctx.status.lean_rejected++;
        }
        ctx.status.lean_live = ok;
    }
    if (platform_link_check_timeout(PLATFORM_LINK_RX_EKF_LEAN, now_ms) == VEHICLE_E2E_NO_NEW_DATA) {
        if (ctx.status.lean_live) {
            ctx.status.lean_timeouts++;
        }
        ctx.status.lean_live = false;
    }
    *usable = ok;
    return taken;
}

void cornering_step(uint32_t now_ms)
{
    bool lean_usable = false;
    bool new_lean;
    bool new_mu;
    cornering_source_t prev = ctx.sel.source;
    cornering_source_t src;

    take_heartbeat(now_ms); /* first, so a 0x020 of this pass sees the newest heartbeat */
    new_mu = take_friction(now_ms);
    new_lean = take_lean(now_ms, &lean_usable);

    src = cornering_select_step(&ctx.sel, new_lean, lean_usable, ctx.status.lean_live, ctx.status.hb_ok);
    if (src == CORNERING_SOURCE_EKF) {
        if ((prev != CORNERING_SOURCE_EKF) || new_lean || new_mu) {
            ctx.status.mu_eff = ctx.status.mu_live ? ctx.mu_valid_eff
                                                   : cornering_mu_eff_lost(ctx.have_mu, ctx.mu_valid_eff);
            (void)cornering_decide(&ctx.hyst, ctx.lean_deg, ctx.status.mu_eff, now_ms);
        }
        ctx.out.level = ctx.hyst.level;
        ctx.out.source = CORNERING_SOURCE_EKF;
        ctx.status.rt_core_inputs_lost = false;
    } else {
        if (prev == CORNERING_SOURCE_EKF) {
            ctx.status.inputs_lost_onsets++;
        }
        cornering_hyst_reset(&ctx.hyst);
        ctx.out.level = CORNERING_LEVEL_UNAVAILABLE;
        ctx.out.source = CORNERING_SOURCE_NONE;
        ctx.status.rt_core_inputs_lost = true;
    }
    ctx.status.good_frames = ctx.sel.good_frames;
}

cornering_output_t cornering_output(void)
{
    return ctx.out;
}

cornering_status_t cornering_status(void)
{
    return ctx.status;
}
