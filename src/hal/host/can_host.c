#include "hal/host/can_host.h"

#include <string.h>

typedef struct {
    hal_can_frame_t slot[CAN_HOST_FIFO_DEPTH_MAX];
    uint32_t head;
    uint32_t count;
} can_host_fifo_t;

static can_host_fifo_t rx_fifo[HAL_CAN_RX_CLASS_COUNT];
static uint32_t rx_lost[HAL_CAN_RX_CLASS_COUNT];
static can_host_fifo_t tx_fifo;
static uint32_t rx_depth = CAN_HOST_FIFO_DEPTH_DEFAULT;
static uint32_t tx_depth = CAN_HOST_FIFO_DEPTH_DEFAULT;
static bool initialised;
static bool init_fails;
static bool bus_is_off;
static uint32_t bus_off_episodes;
static uint32_t refuse_sends;
static uint32_t tx_accepted_count;
static uint32_t tx_refused_count;
static uint32_t tx_cancelled_count;

static uint32_t clamp_depth(uint32_t d)
{
    uint32_t r = d;
    if (r < 1u) {
        r = 1u;
    }
    if (r > CAN_HOST_FIFO_DEPTH_MAX) {
        r = CAN_HOST_FIFO_DEPTH_MAX;
    }
    return r;
}

static bool fifo_push(can_host_fifo_t* f, uint32_t depth, const hal_can_frame_t* frame)
{
    bool ok = false;
    if (f->count < depth) {
        f->slot[(f->head + f->count) % CAN_HOST_FIFO_DEPTH_MAX] = *frame;
        f->count++;
        ok = true;
    }
    return ok;
}

static bool fifo_pop(can_host_fifo_t* f, hal_can_frame_t* frame)
{
    bool ok = false;
    if (f->count > 0u) {
        *frame = f->slot[f->head];
        f->head = (f->head + 1u) % CAN_HOST_FIFO_DEPTH_MAX;
        f->count--;
        ok = true;
    }
    return ok;
}

/* Removes the pending frames of the ID (the board's TXBCR); returns how many. */
static uint32_t tx_cancel_id(uint32_t id, bool all)
{
    can_host_fifo_t keep;
    hal_can_frame_t f;
    uint32_t cancelled = 0u;
    (void)memset(&keep, 0, sizeof keep);
    while (fifo_pop(&tx_fifo, &f)) {
        if (all || (f.id == id)) {
            cancelled++;
        } else {
            (void)fifo_push(&keep, CAN_HOST_FIFO_DEPTH_MAX, &f);
        }
    }
    tx_fifo = keep;
    tx_cancelled_count += cancelled;
    return cancelled;
}

void can_host_reset(void)
{
    (void)memset(rx_fifo, 0, sizeof rx_fifo);
    (void)memset(rx_lost, 0, sizeof rx_lost);
    (void)memset(&tx_fifo, 0, sizeof tx_fifo);
    rx_depth = CAN_HOST_FIFO_DEPTH_DEFAULT;
    tx_depth = CAN_HOST_FIFO_DEPTH_DEFAULT;
    initialised = false;
    init_fails = false;
    bus_is_off = false;
    bus_off_episodes = 0u;
    refuse_sends = 0u;
    tx_accepted_count = 0u;
    tx_refused_count = 0u;
    tx_cancelled_count = 0u;
}

void can_host_set_rx_depth(uint32_t depth)
{
    rx_depth = clamp_depth(depth);
}

void can_host_set_tx_depth(uint32_t depth)
{
    tx_depth = clamp_depth(depth);
}

bool hal_can_init(void)
{
    (void)memset(rx_fifo, 0, sizeof rx_fifo);
    (void)memset(rx_lost, 0, sizeof rx_lost);
    (void)memset(&tx_fifo, 0, sizeof tx_fifo);
    initialised = !init_fails;
    return initialised;
}

hal_can_send_t hal_can_send(const hal_can_frame_t* frame)
{
    hal_can_send_t r = HAL_CAN_SEND_OK;
    if (!initialised) {
        r = HAL_CAN_SEND_NOT_INITIALISED;
    } else if ((frame == NULL) || (frame->dlc > HAL_CAN_MAX_DLC)) {
        r = HAL_CAN_SEND_BAD_ARGUMENT;
    } else if (bus_is_off) {
        r = HAL_CAN_SEND_BUS_OFF;
    } else if (refuse_sends > 0u) {
        refuse_sends--;
        r = HAL_CAN_SEND_FIFO_FULL;
    } else {
        (void)tx_cancel_id(frame->id, false); /* never a late stale frame of an earlier cycle */
        if (!fifo_push(&tx_fifo, tx_depth, frame)) {
            r = HAL_CAN_SEND_FIFO_FULL;
        }
    }
    if (r == HAL_CAN_SEND_OK) {
        tx_accepted_count++;
    } else {
        tx_refused_count++;
    }
    return r;
}

bool hal_can_poll(hal_can_rx_class_t rx_class, hal_can_frame_t* frame)
{
    return initialised && ((uint32_t)rx_class < (uint32_t)HAL_CAN_RX_CLASS_COUNT) && (frame != NULL) &&
           fifo_pop(&rx_fifo[rx_class], frame);
}

uint32_t hal_can_rx_lost(hal_can_rx_class_t rx_class)
{
    return ((uint32_t)rx_class < (uint32_t)HAL_CAN_RX_CLASS_COUNT) ? rx_lost[rx_class] : 0u;
}

uint32_t hal_can_bus_off_count(void)
{
    return bus_off_episodes;
}

bool can_host_inject(hal_can_rx_class_t rx_class, const hal_can_frame_t* frame)
{
    bool ok = false;
    if (((uint32_t)rx_class < (uint32_t)HAL_CAN_RX_CLASS_COUNT) && (frame != NULL)) {
        ok = fifo_push(&rx_fifo[rx_class], rx_depth, frame);
        if (!ok) {
            rx_lost[rx_class]++;
        }
    }
    return ok;
}

bool can_host_tx_pop(hal_can_frame_t* frame)
{
    return fifo_pop(&tx_fifo, frame);
}

void can_host_refuse_next_sends(uint32_t n)
{
    refuse_sends = n;
}

void can_host_set_bus_off(bool bus_off)
{
    if (bus_off && !bus_is_off) {
        bus_off_episodes++;
    }
    bus_is_off = bus_off;
    if (bus_off) {
        (void)tx_cancel_id(0u, true);
    }
}

void can_host_set_init_fails(bool fails)
{
    init_fails = fails;
}

uint32_t can_host_rx_pending(hal_can_rx_class_t rx_class)
{
    return ((uint32_t)rx_class < (uint32_t)HAL_CAN_RX_CLASS_COUNT) ? rx_fifo[rx_class].count : 0u;
}

uint32_t can_host_tx_pending(void)
{
    return tx_fifo.count;
}

uint32_t can_host_tx_accepted(void)
{
    return tx_accepted_count;
}

uint32_t can_host_tx_refused(void)
{
    return tx_refused_count;
}

uint32_t can_host_tx_cancelled(void)
{
    return tx_cancelled_count;
}
