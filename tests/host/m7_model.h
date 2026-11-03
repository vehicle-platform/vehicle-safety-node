#ifndef TESTS_HOST_M7_MODEL_H
#define TESTS_HOST_M7_MODEL_H

/*
 * Test-only model of the M7 side of the mailbox (the D-093 item 5 writer and reader, as in
 * test_mailbox_timing.c) plus periodic senders of 0x081 / 0x022 / 0x020 built with
 * frame_builder.h. One region per test executable.
 */

#include "frame_builder.h"
#include "hal/hsem.h"
#include "hal/host/hsem_host.h"
#include "platform_mailbox.h"
#include "services/mailbox_core.h"

#include <string.h>
#include <unity.h>

#define M7_HEADER_WORDS (PLATFORM_MAILBOX_HEADER_BYTES / MAILBOX_WORD_BYTES)
#define M7_LEAN ((uint32_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_INDEX)
#define M7_MU ((uint32_t)PLATFORM_MAILBOX_SLOT_EKF_FRICTION_MASS_INDEX)
#define M7_HB ((uint32_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_INDEX)
#define M7_WARN ((uint32_t)PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_INDEX)
#define M7_HBS ((uint32_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_INDEX)

static mailbox_region_t m7_region;

static const uint8_t m7_hsem[PLATFORM_MAILBOX_SLOT_COUNT] = {
    (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_HSEM_ID,
    (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_FRICTION_MASS_HSEM_ID,
    (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_HSEM_ID,
    (uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_REQUEST_HSEM_ID,
    (uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_HSEM_ID,
    (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_HSEM_ID,
};

static inline volatile uint32_t* m7_slot_words(uint32_t slot)
{
    return &m7_region.word[M7_HEADER_WORDS + (slot * MAILBOX_SLOT_WORDS)];
}

/* Fresh region with the M7's layout word set (or not, to model a missing/wrong M7). */
static inline void m7_reset(bool layout_ok)
{
    hsem_host_reset();
    (void)memset(&m7_region, 0, sizeof m7_region);
    m7_region.word[MAILBOX_M7_LAYOUT_WORD] = layout_ok ? PLATFORM_MAILBOX_LAYOUT_ID : 0u;
}

static inline void m7_set_layout(uint32_t word)
{
    m7_region.word[MAILBOX_M7_LAYOUT_WORD] = word;
}

static inline void m7_write(uint32_t slot, const uint8_t* d)
{
    mailbox_pending_t p;
    hal_hsem_key_t key;
    mailbox_pending_init(&p);
    TEST_ASSERT_TRUE(mailbox_pending_offer(&p, d, (uint8_t)FB_LEN));
    hsem_host_set_core(HSEM_HOST_CORE_M7);
    TEST_ASSERT_TRUE(hal_hsem_take(m7_hsem[slot], &key));
    mailbox_slot_store(&p, m7_slot_words(slot));
    hal_hsem_barrier();
    mailbox_slot_commit(m7_slot_words(slot));
    hal_hsem_release(m7_hsem[slot], &key);
    hsem_host_set_core(HSEM_HOST_CORE_M4);
}

/* The M7's reader of an M4->M7 slot: true and the payload when a new frame is there. */
static inline bool m7_read(uint32_t slot, mailbox_reader_t* r, uint32_t now, uint8_t out[MAILBOX_PAYLOAD_BYTES])
{
    mailbox_raw_t raw;
    hal_hsem_key_t key;
    hsem_host_set_core(HSEM_HOST_CORE_M7);
    TEST_ASSERT_TRUE(hal_hsem_take(m7_hsem[slot], &key));
    mailbox_slot_load(m7_slot_words(slot), &raw);
    hal_hsem_release(m7_hsem[slot], &key);
    hsem_host_set_core(HSEM_HOST_CORE_M4);
    if (mailbox_reader_accept(r, &raw, true, now) == MAILBOX_READ_NEW) {
        mailbox_payload_bytes(raw.data, out);
        return true;
    }
    return false;
}

/* Lean in raw 0.01 deg units, derived from the gen decoder. */
static inline int16_t m7_lean_raw(float deg)
{
    float lsb = platform_ekf_lean_lean_angle_decode(1);
    return (int16_t)((deg / lsb) + ((deg >= 0.0f) ? 0.5f : -0.5f));
}

static inline uint16_t m7_mu_raw(float mu)
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
} m7_rt_core_t;

static inline void m7_rt_core_init(m7_rt_core_t* m)
{
    (void)memset(m, 0, sizeof *m);
    m->lean_on = true;
    m->mu_on = true;
    m->hb_on = true;
    m->lean_state = PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_ESTIMATED_CHOICE;
    m->mu_state = PLATFORM_EKF_FRICTION_MASS_FRICTION_COEFF_STATE_DEFAULT_CHOICE;
    m->mu_raw = m7_mu_raw(0.5f);
    m->hb_mode = PLATFORM_HEARTBEAT_RT_CORE_NODE_MODE_NORMAL_CHOICE;
}

/* Sends whatever is due at now (each stream on its gen/ cycle, phase 0). */
static inline void m7_rt_core_tick(m7_rt_core_t* m, uint32_t now)
{
    uint8_t f[FB_LEN];
    if (m->hb_on && ((now % PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS) == 0u)) {
        fb_hb(f, m->hb_mode, &m->hb_tx);
        m7_write(M7_HB, f);
    }
    if (m->mu_on && ((now % PLATFORM_EKF_FRICTION_MASS_CYCLE_TIME_MS) == 0u)) {
        fb_mu(f, m->mu_state, m->mu_raw, &m->mu_tx);
        m7_write(M7_MU, f);
    }
    if (m->lean_on && ((now % PLATFORM_EKF_LEAN_CYCLE_TIME_MS) == 0u)) {
        fb_lean(f, m->lean_state, m->lean_raw, &m->lean_tx);
        m7_write(M7_LEAN, f);
    }
}

#endif /* TESTS_HOST_M7_MODEL_H */
