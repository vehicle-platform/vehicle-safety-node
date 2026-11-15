#ifndef TESTS_HOST_BUS_MODEL_H
#define TESTS_HOST_BUS_MODEL_H

/*
 * Test-only model of the platform CAN bus around the safety node (D-118): periodic senders
 * of 0x081 / 0x022 / 0x020 (what rt-core puts on the bus, built with frame_builder.h and
 * injected into the host HAL RX FIFO) and sinks that read the node's own transmissions
 * (0x023 / 0x082) back from the host HAL TX FIFO and check them with the generated E2E.
 */

#include "frame_builder.h"
#include "hal/host/can_host.h"
#include "platform.h"

#include <string.h>
#include <unity.h>

#define BUS_LEAN ((uint32_t)PLATFORM_EKF_LEAN_FRAME_ID)
#define BUS_MU ((uint32_t)PLATFORM_EKF_FRICTION_MASS_FRAME_ID)
#define BUS_HB ((uint32_t)PLATFORM_HEARTBEAT_RT_CORE_FRAME_ID)
#define BUS_REQ ((uint32_t)PLATFORM_SAFETY_REQUEST_FRAME_ID)
#define BUS_WARN ((uint32_t)PLATFORM_SAFETY_WARNING_STATUS_FRAME_ID)
#define BUS_HBS ((uint32_t)PLATFORM_HEARTBEAT_SAFETY_FRAME_ID)

static inline void bus_reset(void)
{
    can_host_reset();
}

/* A frame of id with dlc data bytes arrives at the node. */
static inline void bus_send_dlc(uint32_t id, const uint8_t* d, uint8_t dlc)
{
    hal_can_frame_t f;
    (void)memset(&f, 0, sizeof f);
    f.id = id;
    f.dlc = dlc;
    (void)memcpy(f.data, d, dlc);
    TEST_ASSERT_TRUE(can_host_inject(&f));
}

static inline void bus_send(uint32_t id, const uint8_t* d)
{
    bus_send_dlc(id, d, (uint8_t)FB_LEN);
}

/* Lean in raw 0.01 deg units, derived from the gen decoder. */
static inline int16_t bus_lean_raw(float deg)
{
    float lsb = platform_ekf_lean_lean_angle_decode(1);
    return (int16_t)((deg / lsb) + ((deg >= 0.0f) ? 0.5f : -0.5f));
}

static inline uint16_t bus_mu_raw(float mu)
{
    float lsb = platform_ekf_friction_mass_friction_coeff_decode(1u);
    return (uint16_t)((mu / lsb) + 0.5f);
}

/* Periodic rt-core model: what it sends each cycle; *_on false stops that stream. */
typedef struct {
    bool lean_on;
    bool mu_on;
    bool hb_on;
    uint8_t lean_state;
    int16_t lean_raw;
    uint8_t mu_state;
    uint16_t mu_raw;
    uint8_t hb_mode;
    vehicle_e2e_tx_state_t lean_tx;
    vehicle_e2e_tx_state_t mu_tx;
    vehicle_e2e_tx_state_t hb_tx;
} rt_core_t;

static inline void rt_core_init(rt_core_t* m)
{
    (void)memset(m, 0, sizeof *m);
    m->lean_on = true;
    m->mu_on = true;
    m->hb_on = true;
    m->lean_state = PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_ESTIMATED_CHOICE;
    m->mu_state = PLATFORM_EKF_FRICTION_MASS_FRICTION_COEFF_STATE_DEFAULT_CHOICE;
    m->mu_raw = bus_mu_raw(0.5f);
    m->hb_mode = PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_NORMAL_CHOICE;
}

/* Sends whatever is due at now (each stream on its gen/ cycle, phase 0). */
static inline void rt_core_tick(rt_core_t* m, uint32_t now)
{
    uint8_t f[FB_LEN];
    if (m->hb_on && ((now % PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS) == 0u)) {
        fb_hb(f, m->hb_mode, &m->hb_tx);
        bus_send(BUS_HB, f);
    }
    if (m->mu_on && ((now % PLATFORM_EKF_FRICTION_MASS_CYCLE_TIME_MS) == 0u)) {
        fb_mu(f, m->mu_state, m->mu_raw, &m->mu_tx);
        bus_send(BUS_MU, f);
    }
    if (m->lean_on && ((now % PLATFORM_EKF_LEAN_CYCLE_TIME_MS) == 0u)) {
        fb_lean(f, m->lean_state, m->lean_raw, &m->lean_tx);
        bus_send(BUS_LEAN, f);
    }
}

/* What the node transmitted, as a receiver on the bus sees it (generated E2E check). */
typedef struct {
    vehicle_e2e_rx_state_t rx;
    uint32_t frames;
    uint32_t ok;
    uint32_t initial; /* boot only in these tests */
    uint32_t wrong_sequence;
    uint32_t other_bad;
    uint32_t bad; /* E2E results other than OK / INITIAL */
    vehicle_e2e_status_t last_st;
    uint8_t last[FB_LEN];
    uint32_t last_ms;
} bus_sink_t;

static inline void bus_sink_init(bus_sink_t* s)
{
    (void)memset(s, 0, sizeof *s);
    vehicle_e2e_rx_init(&s->rx);
}

static inline void bus_sink_feed(bus_sink_t* s, uint16_t data_id, uint8_t max_delta, uint32_t timeout,
                                 const uint8_t* d, uint32_t now)
{
    const vehicle_e2e_status_t st = vehicle_e2e_check(data_id, max_delta, timeout, d, FB_LEN, &s->rx, now);
    s->frames++;
    if (st == VEHICLE_E2E_OK) {
        s->ok++;
    } else if (st == VEHICLE_E2E_INITIAL) {
        s->initial++;
    } else if (st == VEHICLE_E2E_WRONG_SEQUENCE) {
        s->wrong_sequence++;
        s->bad++;
    } else {
        s->other_bad++;
        s->bad++;
    }
    s->last_st = st;
    s->last_ms = now;
    (void)memcpy(s->last, d, FB_LEN);
}

/* Reads every frame the node transmitted so far: 0x023 into warn, 0x082 into hbs. Anything
 * else on the TX side is a test failure (the node transmits these two IDs only), as is a
 * DLC other than the generated length. */
static inline void bus_drain_tx(bus_sink_t* warn, bus_sink_t* hbs, uint32_t now)
{
    hal_can_frame_t f;
    while (can_host_tx_pop(&f)) {
        TEST_ASSERT_FALSE(f.extended);
        TEST_ASSERT_EQUAL_UINT8(FB_LEN, f.dlc);
        if (f.id == BUS_WARN) {
            bus_sink_feed(warn, PLATFORM_SAFETY_WARNING_STATUS_E2E_DATA_ID,
                          (uint8_t)PLATFORM_SAFETY_WARNING_STATUS_E2E_MAX_DELTA_COUNTER,
                          PLATFORM_SAFETY_WARNING_STATUS_E2E_TIMEOUT_MS, f.data, now);
        } else if (f.id == BUS_HBS) {
            bus_sink_feed(hbs, PLATFORM_HEARTBEAT_SAFETY_E2E_DATA_ID,
                          (uint8_t)PLATFORM_HEARTBEAT_SAFETY_E2E_MAX_DELTA_COUNTER,
                          PLATFORM_HEARTBEAT_SAFETY_E2E_TIMEOUT_MS, f.data, now);
        } else {
            TEST_FAIL_MESSAGE("the safety node transmitted an ID other than 0x023 / 0x082");
        }
    }
}

#endif /* TESTS_HOST_BUS_MODEL_H */
