#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/structs/sio.h"
#include "pico/multicore.h"

#include "flexray_bss_streamer.pio.h"
#include "flexray_bss_streamer.h"
#include "flexray_forwarder_with_injector.h"
#include "flexray_frame.h"

// ===================== FR1/FR2 (primary) stream state =====================
uint dma_data_from_fr1_chan;
uint dma_data_from_fr2_chan;
static uint dma_rearm_fr1_chan;
static uint dma_rearm_fr2_chan;

static PIO streamer_pio;
static uint streamer_sm_fr1;
static uint streamer_sm_fr2;

volatile uint8_t fr1_ring_buffer[FR1_RING_SIZE_BYTES] __attribute__((aligned(FR1_RING_SIZE_BYTES)));
volatile uint8_t fr2_ring_buffer[FR2_RING_SIZE_BYTES] __attribute__((aligned(FR2_RING_SIZE_BYTES)));

static volatile uint32_t fr1_prev_write_idx = 0;
static volatile uint32_t fr2_prev_write_idx = 0;

// ===================== FR3/FR4 (secondary) stream state =====================
static uint dma_data_from_fr3_chan;
static uint dma_data_from_fr4_chan;
static uint dma_rearm_fr3_chan;
static uint dma_rearm_fr4_chan;

static PIO streamer_pio_fr34;
static uint streamer_sm_fr3;
static uint streamer_sm_fr4;

volatile uint8_t fr3_ring_buffer[FR3_RING_SIZE_BYTES] __attribute__((aligned(FR3_RING_SIZE_BYTES)));
volatile uint8_t fr4_ring_buffer[FR4_RING_SIZE_BYTES] __attribute__((aligned(FR4_RING_SIZE_BYTES)));

static volatile uint32_t fr3_prev_write_idx = 0;
static volatile uint32_t fr4_prev_write_idx = 0;
static volatile uint8_t fr34_detected_source = FROM_UNKNOWN;

// ===================== Shared state =====================
#define DMA_BLOCK_COUNT_BYTES  (4096u | 0x10000000) // self trigger
// IRQ4 follows the fifth RX autopush; allow DMA a bounded time to publish it.
#define HEADER_DMA_READY_RETRIES 8u

volatile uint32_t irq_counter = 0;
volatile uint32_t irq_handler_call_count = 0;

volatile void *buffer_addresses[2] = {
    (void *)fr1_ring_buffer,
    (void *)fr2_ring_buffer};

volatile int dma_inject_chan_to_fr1 = -1;
volatile int dma_inject_chan_to_fr2 = -1;
volatile int dma_inject_chan_to_fr3 = -1;
volatile int dma_inject_chan_to_fr4 = -1;

volatile uint32_t current_buffer_index = 0;

static inline uint32_t dma_ring_write_idx(uint dma_chan, volatile uint8_t *ring_base, uint32_t ring_mask)
{
    uint32_t wa = dma_channel_hw_addr(dma_chan)->write_addr;
    return (wa - (uint32_t)(uintptr_t)ring_base) & ring_mask;
}

// ===================== Cross-core notification ring =====================
#define NOTIFY_RING_SIZE 1024u
static volatile uint32_t notify_ring[NOTIFY_RING_SIZE];
static volatile uint16_t notify_head = 0;
static volatile uint16_t notify_tail = 0;
static volatile uint32_t notify_dropped = 0;

static inline uint16_t header_crc_from_header(const uint8_t *header)
{
    return (uint16_t)(((uint16_t)(header[2] & 0x01) << 10) |
                      ((uint16_t)header[3] << 2) |
                      ((header[4] >> 6) & 0x03));
}

static inline void ring_copy_header_from_start(uint8_t *header,
                                               const volatile uint8_t *ring_base,
                                               uint32_t start,
                                               uint32_t ring_mask)
{
    for (uint32_t i = 0; i < 5u; i++) {
        header[i] = ring_base[(start + i) & ring_mask];
    }
}

static inline void process_fr12_header_from_ring(const volatile uint8_t *ring_base,
                                                 uint32_t start,
                                                 uint32_t ring_mask)
{
    uint8_t header[5];
    ring_copy_header_from_start(header, ring_base, start, ring_mask);
    if (calculate_flexray_header_crc(header) != header_crc_from_header(header)) {
        return;
    }

    uint16_t frame_id = (uint16_t)(((uint16_t)(header[0] & 0x07) << 8) | header[1]);
    uint8_t cycle_count = (uint8_t)(header[4] & 0x3F);
    (void)prepare_inject_frame(frame_id, cycle_count);
}

static inline void service_fr12_header_irq(void)
{
    for (uint32_t retry = 0; retry < HEADER_DMA_READY_RETRIES; retry++) {
        uint32_t fr1_delta = (dma_ring_write_idx(dma_data_from_fr1_chan,
                                                 fr1_ring_buffer,
                                                 FR1_RING_MASK) -
                              fr1_prev_write_idx) &
                             FR1_RING_MASK;
        uint32_t fr2_delta = (dma_ring_write_idx(dma_data_from_fr2_chan,
                                                 fr2_ring_buffer,
                                                 FR2_RING_MASK) -
                              fr2_prev_write_idx) &
                             FR2_RING_MASK;

        if (fr2_delta >= 5u) {
            process_fr12_header_from_ring(fr2_ring_buffer,
                                          fr2_prev_write_idx,
                                          FR2_RING_MASK);
            return;
        }
        if (fr1_delta >= 5u) {
            process_fr12_header_from_ring(fr1_ring_buffer,
                                          fr1_prev_write_idx,
                                          FR1_RING_MASK);
            return;
        }
    }
}

static inline bool record_fr34_source_from_header(bool is_fr4,
                                                  const volatile uint8_t *ring_base,
                                                  uint32_t start,
                                                  uint32_t ring_mask)
{
    uint8_t header[5];
    ring_copy_header_from_start(header, ring_base, start, ring_mask);
    if (calculate_flexray_header_crc(header) != header_crc_from_header(header)) {
        fr34_detected_source = FROM_UNKNOWN;
        return false;
    }

    fr34_detected_source = is_fr4 ? FROM_FR4 : FROM_FR3;
    return true;
}

static inline void service_fr34_header_irq(void)
{
    for (uint32_t retry = 0; retry < HEADER_DMA_READY_RETRIES; retry++) {
        uint32_t fr3_delta = (dma_ring_write_idx(dma_data_from_fr3_chan,
                                                 fr3_ring_buffer,
                                                 FR3_RING_MASK) -
                              fr3_prev_write_idx) &
                             FR3_RING_MASK;
        uint32_t fr4_delta = (dma_ring_write_idx(dma_data_from_fr4_chan,
                                                 fr4_ring_buffer,
                                                 FR4_RING_MASK) -
                              fr4_prev_write_idx) &
                             FR4_RING_MASK;

        if (fr4_delta >= 5u) {
            (void)record_fr34_source_from_header(true, fr4_ring_buffer, fr4_prev_write_idx, FR4_RING_MASK);
            return;
        }
        if (fr3_delta >= 5u) {
            (void)record_fr34_source_from_header(false, fr3_ring_buffer, fr3_prev_write_idx, FR3_RING_MASK);
            return;
        }
    }

    fr34_detected_source = FROM_UNKNOWN;
}

void notify_queue_init(void)
{
    notify_head = 0;
    notify_tail = 0;
    notify_dropped = 0;
}

static inline bool notify_queue_push(uint32_t value)
{
    uint16_t head = notify_head;
    uint16_t next = (uint16_t)((head + 1u) & (NOTIFY_RING_SIZE - 1u));
    if (next == notify_tail)
    {
        notify_dropped++;
        return false;
    }
    notify_ring[head] = value;
    notify_head = next;
    __sev();
    return true;
}

bool notify_queue_pop(uint32_t *encoded)
{
    uint16_t tail = notify_tail;
    if (tail == notify_head)
    {
        return false;
    }
    *encoded = notify_ring[tail];
    notify_tail = (uint16_t)((tail + 1u) & (NOTIFY_RING_SIZE - 1u));
    return true;
}

uint32_t notify_queue_dropped(void)
{
    return notify_dropped;
}

// ===================== FR1/FR2 header IRQ handler =====================
void __time_critical_func(streamer_header_irq_handler)(void)
{
    sio_hw->gpio_set = (1u << 7);
    pio_interrupt_clear(streamer_pio, 4);
    discard_prepared_injection();
    service_fr12_header_irq();
    sio_hw->gpio_clr = (1u << 7);
}

// ===================== FR1/FR2 frame-end IRQ handler =====================
void __time_critical_func(streamer_frame_end_irq_handler)(void)
{
    sio_hw->gpio_set = (1u << 7);

    irq_handler_call_count++;
    pio_interrupt_clear(streamer_pio, 3);

    uint32_t fr1_idx_now = dma_ring_write_idx(dma_data_from_fr1_chan, fr1_ring_buffer, FR1_RING_MASK);
    uint32_t fr2_idx_now = dma_ring_write_idx(dma_data_from_fr2_chan, fr2_ring_buffer, FR2_RING_MASK);

    bool fr1_advanced = (fr1_idx_now != fr1_prev_write_idx);
    bool fr2_advanced = (fr2_idx_now != fr2_prev_write_idx);

    uint32_t idx = 0;
    bool is_fr2 = false;

    if (fr1_advanced && !fr2_advanced)
    {
        idx = fr1_idx_now;
        fr1_prev_write_idx = fr1_idx_now;
    }
    else if (!fr1_advanced && fr2_advanced)
    {
        idx = fr2_idx_now;
        is_fr2 = true;
        fr2_prev_write_idx = fr2_idx_now;
    }
    else
    {
        uint32_t fr1_delta = (fr1_idx_now - fr1_prev_write_idx) & FR1_RING_MASK;
        uint32_t fr2_delta = (fr2_idx_now - fr2_prev_write_idx) & FR2_RING_MASK;
        if (fr2_delta > fr1_delta)
        {
            idx = fr2_idx_now;
            is_fr2 = true;
            fr2_prev_write_idx = fr2_idx_now;
        }
        else
        {
            idx = fr1_idx_now;
            fr1_prev_write_idx = fr1_idx_now;
        }
    }

    uint8_t fr34_source = fr34_detected_source;
    fr34_detected_source = FROM_UNKNOWN;

    inject_prepared_frame();

    uint32_t encoded = notify_encode(is_fr2, fr34_source, ((irq_counter++) & 0x1FFFF), (uint16_t)idx);
    (void)notify_queue_push(encoded);
    sio_hw->gpio_clr = (1u << 7);
}

// ===================== FR3/FR4 header IRQ handler =====================
void __time_critical_func(streamer_fr34_header_irq_handler)(void)
{
    sio_hw->gpio_set = (1u << 7);
    pio_interrupt_clear(streamer_pio_fr34, 4);
    service_fr34_header_irq();
    sio_hw->gpio_clr = (1u << 7);
}

// ===================== FR3/FR4 frame-end IRQ handler =====================
void __time_critical_func(streamer_fr34_frame_end_irq_handler)(void)
{
    // sio_hw->gpio_set = (1u << 7);
    pio_interrupt_clear(streamer_pio_fr34, 3);

    uint32_t fr3_idx_now = dma_ring_write_idx(dma_data_from_fr3_chan, fr3_ring_buffer, FR3_RING_MASK);
    uint32_t fr4_idx_now = dma_ring_write_idx(dma_data_from_fr4_chan, fr4_ring_buffer, FR4_RING_MASK);

    if (fr3_idx_now != fr3_prev_write_idx) {
        fr3_prev_write_idx = fr3_idx_now;
    }

    if (fr4_idx_now != fr4_prev_write_idx) {
        fr4_prev_write_idx = fr4_idx_now;
    }

    // sio_hw->gpio_clr = (1u << 7);
}

// ===================== FR1/FR2 setup =====================
void setup_stream(PIO pio,
                  uint rx_pin_from_fr1, uint tx_en_pin_to_fr2,
                  uint rx_pin_from_fr2, uint tx_en_pin_to_fr1)
{
    streamer_pio = pio;

    uint offset = pio_add_program(pio, &flexray_bss_streamer_program);
    uint sm_fr1 = pio_claim_unused_sm(pio, true);
    uint sm_fr2 = pio_claim_unused_sm(pio, true);

    streamer_sm_fr1 = sm_fr1;
    streamer_sm_fr2 = sm_fr2;

    flexray_bss_streamer_program_init(pio, sm_fr1, offset, rx_pin_from_fr1, tx_en_pin_to_fr2);
    flexray_bss_streamer_program_init(pio, sm_fr2, offset, rx_pin_from_fr2, tx_en_pin_to_fr1);
    dma_data_from_fr1_chan = dma_claim_unused_channel(true);
    dma_data_from_fr2_chan = dma_claim_unused_channel(true);
    dma_channel_config dma_c_fr1 = dma_channel_get_default_config(dma_data_from_fr1_chan);
    dma_channel_config dma_c_fr2 = dma_channel_get_default_config(dma_data_from_fr2_chan);
    channel_config_set_transfer_data_size(&dma_c_fr1, DMA_SIZE_8);
    channel_config_set_transfer_data_size(&dma_c_fr2, DMA_SIZE_8);
    channel_config_set_read_increment(&dma_c_fr1, false);
    channel_config_set_read_increment(&dma_c_fr2, false);
    channel_config_set_write_increment(&dma_c_fr1, true);
    channel_config_set_write_increment(&dma_c_fr2, true);
    channel_config_set_dreq(&dma_c_fr1, pio_get_dreq(pio, sm_fr1, false));
    channel_config_set_dreq(&dma_c_fr2, pio_get_dreq(pio, sm_fr2, false));

    uint8_t fr1_ring_bits = 0;
    if (FR1_RING_SIZE_BYTES > 1)
    {
        fr1_ring_bits = 32 - __builtin_clz(FR1_RING_SIZE_BYTES - 1);
    }
    channel_config_set_ring(&dma_c_fr1, true, fr1_ring_bits);

    uint8_t fr2_ring_bits = 0;
    if (FR2_RING_SIZE_BYTES > 1)
    {
        fr2_ring_bits = 32 - __builtin_clz(FR2_RING_SIZE_BYTES - 1);
    }
    channel_config_set_ring(&dma_c_fr2, true, fr2_ring_bits);
    dma_rearm_fr1_chan = dma_claim_unused_channel(true);
    dma_rearm_fr2_chan = dma_claim_unused_channel(true);
    channel_config_set_chain_to(&dma_c_fr1, dma_rearm_fr1_chan);
    channel_config_set_chain_to(&dma_c_fr2, dma_rearm_fr2_chan);

    dma_channel_configure(dma_data_from_fr1_chan, &dma_c_fr1,
                          (void *)fr1_ring_buffer,
                          &pio->rxf[sm_fr1],
                          DMA_BLOCK_COUNT_BYTES,
                          true);
    dma_channel_configure(dma_data_from_fr2_chan, &dma_c_fr2,
                          (void *)fr2_ring_buffer,
                          &pio->rxf[sm_fr2],
                          DMA_BLOCK_COUNT_BYTES,
                          true);

    pio_interrupt_clear(pio, 3);
    pio_interrupt_clear(pio, 4);
    pio_interrupt_clear(pio, 7);

    pio_set_irq0_source_enabled(pio, pis_interrupt4, true);
    irq_set_exclusive_handler(pio_get_irq_num(pio, 0), streamer_header_irq_handler);
    irq_set_enabled(pio_get_irq_num(pio, 0), true);

    pio_set_irq1_source_enabled(pio, pis_interrupt3, true);
    irq_set_exclusive_handler(pio_get_irq_num(pio, 1), streamer_frame_end_irq_handler);
    irq_set_enabled(pio_get_irq_num(pio, 1), true);

    pio_sm_set_enabled(pio, sm_fr1, true);
    pio_sm_set_enabled(pio, sm_fr2, true);
}

// ===================== FR3/FR4 setup =====================
void setup_stream_fr34(PIO pio,
                       uint rx_pin_from_fr3, uint tx_en_pin_to_fr4,
                       uint rx_pin_from_fr4, uint tx_en_pin_to_fr3)
{
    streamer_pio_fr34 = pio;

    uint offset = pio_add_program(pio, &flexray_bss_streamer_program);
    uint sm_fr3 = pio_claim_unused_sm(pio, true);
    uint sm_fr4 = pio_claim_unused_sm(pio, true);

    streamer_sm_fr3 = sm_fr3;
    streamer_sm_fr4 = sm_fr4;

    flexray_bss_streamer_program_init(pio, sm_fr3, offset, rx_pin_from_fr3, tx_en_pin_to_fr4);
    flexray_bss_streamer_program_init(pio, sm_fr4, offset, rx_pin_from_fr4, tx_en_pin_to_fr3);

    dma_data_from_fr3_chan = dma_claim_unused_channel(true);
    dma_data_from_fr4_chan = dma_claim_unused_channel(true);
    dma_channel_config dma_c_fr3 = dma_channel_get_default_config(dma_data_from_fr3_chan);
    dma_channel_config dma_c_fr4 = dma_channel_get_default_config(dma_data_from_fr4_chan);
    channel_config_set_transfer_data_size(&dma_c_fr3, DMA_SIZE_8);
    channel_config_set_transfer_data_size(&dma_c_fr4, DMA_SIZE_8);
    channel_config_set_read_increment(&dma_c_fr3, false);
    channel_config_set_read_increment(&dma_c_fr4, false);
    channel_config_set_write_increment(&dma_c_fr3, true);
    channel_config_set_write_increment(&dma_c_fr4, true);
    channel_config_set_dreq(&dma_c_fr3, pio_get_dreq(pio, sm_fr3, false));
    channel_config_set_dreq(&dma_c_fr4, pio_get_dreq(pio, sm_fr4, false));

    uint8_t fr3_ring_bits = 0;
    if (FR3_RING_SIZE_BYTES > 1)
    {
        fr3_ring_bits = 32 - __builtin_clz(FR3_RING_SIZE_BYTES - 1);
    }
    channel_config_set_ring(&dma_c_fr3, true, fr3_ring_bits);

    uint8_t fr4_ring_bits = 0;
    if (FR4_RING_SIZE_BYTES > 1)
    {
        fr4_ring_bits = 32 - __builtin_clz(FR4_RING_SIZE_BYTES - 1);
    }
    channel_config_set_ring(&dma_c_fr4, true, fr4_ring_bits);

    dma_rearm_fr3_chan = dma_claim_unused_channel(true);
    dma_rearm_fr4_chan = dma_claim_unused_channel(true);
    channel_config_set_chain_to(&dma_c_fr3, dma_rearm_fr3_chan);
    channel_config_set_chain_to(&dma_c_fr4, dma_rearm_fr4_chan);

    dma_channel_configure(dma_data_from_fr3_chan, &dma_c_fr3,
                          (void *)fr3_ring_buffer,
                          &pio->rxf[sm_fr3],
                          DMA_BLOCK_COUNT_BYTES,
                          true);
    dma_channel_configure(dma_data_from_fr4_chan, &dma_c_fr4,
                          (void *)fr4_ring_buffer,
                          &pio->rxf[sm_fr4],
                          DMA_BLOCK_COUNT_BYTES,
                          true);

    pio_interrupt_clear(pio, 3);
    pio_interrupt_clear(pio, 4);
    pio_interrupt_clear(pio, 7);

    pio_set_irq0_source_enabled(pio, pis_interrupt4, true);
    irq_set_exclusive_handler(pio_get_irq_num(pio, 0), streamer_fr34_header_irq_handler);
    irq_set_enabled(pio_get_irq_num(pio, 0), true);

    pio_set_irq1_source_enabled(pio, pis_interrupt3, true);
    irq_set_exclusive_handler(pio_get_irq_num(pio, 1), streamer_fr34_frame_end_irq_handler);
    irq_set_enabled(pio_get_irq_num(pio, 1), true);

    pio_sm_set_enabled(pio, sm_fr3, true);
    pio_sm_set_enabled(pio, sm_fr4, true);
}
