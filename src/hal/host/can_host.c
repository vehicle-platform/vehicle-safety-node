#include "hal/host/can_host.h"

#include <string.h>

typedef struct {
    hal_can_frame_t slot[CAN_HOST_FIFO_DEPTH];
    uint32_t head;
    uint32_t count;
} can_host_fifo_t;

static can_host_fifo_t rx_fifo;
static can_host_fifo_t tx_fifo;
static bool initialised;
static bool init_fails;
static uint32_t refuse_sends;
static uint32_t rx_overflow_count;
static uint32_t tx_accepted_count;
static uint32_t tx_refused_count;

static bool fifo_push(can_host_fifo_t* f, const hal_can_frame_t* frame)
{
    bool ok = false;
    if (f->count < CAN_HOST_FIFO_DEPTH) {
        f->slot[(f->head + f->count) % CAN_HOST_FIFO_DEPTH] = *frame;
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
        f->head = (f->head + 1u) % CAN_HOST_FIFO_DEPTH;
        f->count--;
        ok = true;
    }
    return ok;
}

void can_host_reset(void)
{
    (void)memset(&rx_fifo, 0, sizeof rx_fifo);
    (void)memset(&tx_fifo, 0, sizeof tx_fifo);
    initialised = false;
    init_fails = false;
    refuse_sends = 0u;
    rx_overflow_count = 0u;
    tx_accepted_count = 0u;
    tx_refused_count = 0u;
}

bool hal_can_init(void)
{
    (void)memset(&rx_fifo, 0, sizeof rx_fifo);
    (void)memset(&tx_fifo, 0, sizeof tx_fifo);
    initialised = !init_fails;
    return initialised;
}

bool hal_can_send(const hal_can_frame_t* frame)
{
    bool ok = false;
    if (initialised && (frame != NULL) && (frame->dlc <= HAL_CAN_MAX_DLC)) {
        if (refuse_sends > 0u) {
            refuse_sends--;
        } else {
            ok = fifo_push(&tx_fifo, frame);
        }
    }
    if (ok) {
        tx_accepted_count++;
    } else {
        tx_refused_count++;
    }
    return ok;
}

bool hal_can_poll(hal_can_frame_t* frame)
{
    return initialised && (frame != NULL) && fifo_pop(&rx_fifo, frame);
}

bool can_host_inject(const hal_can_frame_t* frame)
{
    bool ok = fifo_push(&rx_fifo, frame);
    if (!ok) {
        rx_overflow_count++;
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

void can_host_set_init_fails(bool fails)
{
    init_fails = fails;
}

uint32_t can_host_rx_pending(void)
{
    return rx_fifo.count;
}

uint32_t can_host_tx_pending(void)
{
    return tx_fifo.count;
}

uint32_t can_host_rx_overflows(void)
{
    return rx_overflow_count;
}

uint32_t can_host_tx_accepted(void)
{
    return tx_accepted_count;
}

uint32_t can_host_tx_refused(void)
{
    return tx_refused_count;
}
