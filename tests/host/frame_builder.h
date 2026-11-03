#ifndef TESTS_HOST_FRAME_BUILDER_H
#define TESTS_HOST_FRAME_BUILDER_H

/*
 * Test-only frame builder for the M7 side. Node SAFETY's gen/ has unpack (no pack) for the
 * M7->M4 messages and pack (no unpack) for the M4->M7 ones. To build and read frames without
 * hand-writing any bit layout (invariant 2), each signal's bit positions are discovered from
 * the generated functions themselves: single-bit probe buffers through unpack (RX) and
 * single-bit raw values through pack (TX). E2E bytes use the generated protect/check.
 */

#include "platform.h"
#include "platform_e2e.h"
#include "vehicle_e2e.h"

#include <stdint.h>
#include <string.h>

#define FB_LEN (8u)

typedef uint32_t (*fb_rx_get_t)(const uint8_t* frame);
typedef void (*fb_tx_pack_t)(uint8_t* frame, uint32_t raw);

/* Sets raw into the frame bits that unpack maps onto the field read by get. */
static inline void fb_rx_set(uint8_t frame[FB_LEN], fb_rx_get_t get, uint32_t raw)
{
    for (uint32_t b = 0u; b < (FB_LEN * 8u); b++) {
        uint8_t probe[FB_LEN] = {0};
        probe[b / 8u] = (uint8_t)(1u << (b % 8u));
        uint32_t v = get(probe);
        if ((v != 0u) && ((v & (v - 1u)) == 0u) && ((raw & v) != 0u)) {
            frame[b / 8u] |= (uint8_t)(1u << (b % 8u));
        }
    }
}

/* Reads a field of a packed TX frame through the bit that pack puts each raw bit into. */
static inline uint32_t fb_tx_get(const uint8_t frame[FB_LEN], fb_tx_pack_t pack, uint32_t width)
{
    uint32_t raw = 0u;
    for (uint32_t k = 0u; k < width; k++) {
        uint8_t probe[FB_LEN] = {0};
        pack(probe, 1u << k);
        for (uint32_t i = 0u; i < FB_LEN; i++) {
            if ((probe[i] & frame[i]) != 0u) {
                raw |= (1u << k);
            }
        }
    }
    return raw;
}

/* ---- 0x020 EkfLean ---- */
static inline uint32_t fb_lean_state_get(const uint8_t* f)
{
    struct platform_ekf_lean_t m;
    (void)platform_ekf_lean_unpack(&m, f, FB_LEN);
    return m.lean_angle_state;
}
static inline uint32_t fb_lean_angle_get(const uint8_t* f)
{
    struct platform_ekf_lean_t m;
    (void)platform_ekf_lean_unpack(&m, f, FB_LEN);
    return (uint16_t)m.lean_angle;
}
static inline void fb_lean(uint8_t frame[FB_LEN], uint8_t state, int16_t raw_lean, vehicle_e2e_tx_state_t* tx)
{
    (void)memset(frame, 0, FB_LEN);
    fb_rx_set(frame, fb_lean_state_get, state);
    fb_rx_set(frame, fb_lean_angle_get, (uint16_t)raw_lean);
    (void)vehicle_e2e_protect(PLATFORM_EKF_LEAN_E2E_DATA_ID, frame, FB_LEN, tx);
}

/* ---- 0x022 EkfFrictionMass ---- */
static inline uint32_t fb_mu_state_get(const uint8_t* f)
{
    struct platform_ekf_friction_mass_t m;
    (void)platform_ekf_friction_mass_unpack(&m, f, FB_LEN);
    return m.friction_coeff_state;
}
static inline uint32_t fb_mu_get(const uint8_t* f)
{
    struct platform_ekf_friction_mass_t m;
    (void)platform_ekf_friction_mass_unpack(&m, f, FB_LEN);
    return m.friction_coeff;
}
static inline void fb_mu(uint8_t frame[FB_LEN], uint8_t state, uint16_t raw_mu, vehicle_e2e_tx_state_t* tx)
{
    (void)memset(frame, 0, FB_LEN);
    fb_rx_set(frame, fb_mu_state_get, state);
    fb_rx_set(frame, fb_mu_get, raw_mu);
    (void)vehicle_e2e_protect(PLATFORM_EKF_FRICTION_MASS_E2E_DATA_ID, frame, FB_LEN, tx);
}

/* ---- 0x081 HeartbeatRtCore ---- */
static inline uint32_t fb_hb_mode_get(const uint8_t* f)
{
    struct platform_heartbeat_rt_core_t m;
    (void)platform_heartbeat_rt_core_unpack(&m, f, FB_LEN);
    return m.node_mode;
}
static inline void fb_hb(uint8_t frame[FB_LEN], uint8_t mode, vehicle_e2e_tx_state_t* tx)
{
    (void)memset(frame, 0, FB_LEN);
    fb_rx_set(frame, fb_hb_mode_get, mode);
    (void)vehicle_e2e_protect(PLATFORM_HEARTBEAT_RT_CORE_E2E_DATA_ID, frame, FB_LEN, tx);
}

/* ---- 0x023 SafetyWarningStatus (read back) ---- */
static inline void fb_warn_level_pack(uint8_t* f, uint32_t raw)
{
    struct platform_safety_warning_status_t m;
    (void)memset(&m, 0, sizeof m);
    m.cornering_level = (uint8_t)raw;
    (void)platform_safety_warning_status_pack(f, &m, FB_LEN);
}
static inline void fb_warn_source_pack(uint8_t* f, uint32_t raw)
{
    struct platform_safety_warning_status_t m;
    (void)memset(&m, 0, sizeof m);
    m.cornering_source = (uint8_t)raw;
    (void)platform_safety_warning_status_pack(f, &m, FB_LEN);
}
static inline void fb_warn_bsm_left_pack(uint8_t* f, uint32_t raw)
{
    struct platform_safety_warning_status_t m;
    (void)memset(&m, 0, sizeof m);
    m.bsm_left = (uint8_t)raw;
    (void)platform_safety_warning_status_pack(f, &m, FB_LEN);
}
static inline void fb_warn_bsm_right_pack(uint8_t* f, uint32_t raw)
{
    struct platform_safety_warning_status_t m;
    (void)memset(&m, 0, sizeof m);
    m.bsm_right = (uint8_t)raw;
    (void)platform_safety_warning_status_pack(f, &m, FB_LEN);
}
static inline void fb_warn_immobilizer_pack(uint8_t* f, uint32_t raw)
{
    struct platform_safety_warning_status_t m;
    (void)memset(&m, 0, sizeof m);
    m.immobilizer_state = (uint8_t)raw;
    (void)platform_safety_warning_status_pack(f, &m, FB_LEN);
}

/* ---- 0x082 HeartbeatSafety (read back) ---- */
static inline void fb_hbs_mode_pack(uint8_t* f, uint32_t raw)
{
    struct platform_heartbeat_safety_t m;
    (void)memset(&m, 0, sizeof m);
    m.node_mode = (uint8_t)raw;
    (void)platform_heartbeat_safety_pack(f, &m, FB_LEN);
}
static inline void fb_hbs_errors_pack(uint8_t* f, uint32_t raw)
{
    struct platform_heartbeat_safety_t m;
    (void)memset(&m, 0, sizeof m);
    m.error_count = (uint8_t)raw;
    (void)platform_heartbeat_safety_pack(f, &m, FB_LEN);
}
static inline void fb_hbs_uptime_pack(uint8_t* f, uint32_t raw)
{
    struct platform_heartbeat_safety_t m;
    (void)memset(&m, 0, sizeof m);
    m.uptime = (uint16_t)raw;
    (void)platform_heartbeat_safety_pack(f, &m, FB_LEN);
}

#endif /* TESTS_HOST_FRAME_BUILDER_H */
