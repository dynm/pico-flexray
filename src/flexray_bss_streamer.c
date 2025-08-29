#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"

#include <string.h>

#include "flexray_bss_streamer.pio.h"
#include "flexray_bss_streamer.h"
#include "flexray_injector.h"
#include "flexray_frame.h"

// --- Global State ---
uint dma_data_from_ecu_chan;
uint dma_data_from_vehicle_chan;
uint dma_rearm_ecu_chan;
uint dma_rearm_vehicle_chan;

// --- Global Config for Reset ---
static PIO streamer_pio;
static uint streamer_sm_from_ecu;
static uint streamer_sm_from_vehicle;

// Ring-ready buffers per source, power-of-two sized and aligned (for circular DMA)
// Sizes are defined in header

// DMA block size per data-channel before chaining to rearm
#define DMA_BLOCK_COUNT_BYTES  (4096u | 0x10000000) // self trigger

volatile uint8_t ecu_ring_buffer[ECU_RING_SIZE_BYTES] __attribute__((aligned(ECU_RING_SIZE_BYTES)));
volatile uint8_t vehicle_ring_buffer[VEH_RING_SIZE_BYTES] __attribute__((aligned(VEH_RING_SIZE_BYTES)));
volatile uint32_t irq_counter = 0;
volatile uint32_t irq_handler_call_count = 0;
// Keep existing buffer address indirection for current ping-pong logic
volatile void *buffer_addresses[2] = {
    (void *)ecu_ring_buffer,
    (void *)vehicle_ring_buffer};

// DMA to write injector payload to PIO2 SM3 TX FIFO
volatile int dma_inject_chan = -1;
static uint32_t injector_payload[7] = {
    23,
    0xFFFF11FF, 0xFFFFFFFF, 0xFFFFF0F0, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
};
static dma_channel_config injector_dc;

// injection/cache logic moved into flexray_injector.{h,c}

// Current buffer index no longer used with ring; keep for compatibility if needed
volatile uint32_t current_buffer_index = 0;

static inline uint32_t dma_ring_write_idx(uint dma_chan, volatile uint8_t *ring_base, uint32_t ring_mask)
{
    uint32_t wa = dma_channel_hw_addr(dma_chan)->write_addr;
    return (wa - (uint32_t)(uintptr_t)ring_base) & ring_mask;
}

// Track last-seen write indices to identify which DMA advanced at this IRQ
static volatile uint32_t ecu_prev_write_idx = 0;
static volatile uint32_t veh_prev_write_idx = 0;

// --- Cross-core notification ring (single-producer ISR on core1, single-consumer on core0) ---
#define NOTIFY_RING_SIZE 1024u
static volatile uint32_t notify_ring[NOTIFY_RING_SIZE];
static volatile uint16_t notify_head = 0; // producer writes head
static volatile uint16_t notify_tail = 0; // consumer advances tail
static volatile uint32_t notify_dropped = 0;

static volatile uint16_t current_frame_id = 0;
static volatile uint8_t current_cycle_count = 0;
static volatile uint8_t payload_length = 0;

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
        return false; // full
    }
    notify_ring[head] = value;
    notify_head = next;
    __sev(); // wake consumer after publishing head
    return true;
}

bool notify_queue_pop(uint32_t *encoded)
{
    uint16_t tail = notify_tail;
    if (tail == notify_head)
    {
        return false; // empty
    }
    *encoded = notify_ring[tail];
    notify_tail = (uint16_t)((tail + 1u) & (NOTIFY_RING_SIZE - 1u));
    return true;
}

uint32_t notify_queue_dropped(void)
{
    return notify_dropped;
}

// This is the DMA interrupt handler, which is much more efficient.
void streamer_irq0_handler()
{
    uint32_t start_idx = 0;

    irq_handler_call_count++;
    // Clear PIO IRQ source
    pio_interrupt_clear(streamer_pio, 3);

    // Determine source by which DMA write index advanced since last IRQ
    uint32_t ecu_idx_now = dma_ring_write_idx(dma_data_from_ecu_chan, ecu_ring_buffer, ECU_RING_MASK);
    uint32_t veh_idx_now = dma_ring_write_idx(dma_data_from_vehicle_chan, vehicle_ring_buffer, VEH_RING_MASK);

    bool ecu_advanced = (ecu_idx_now != ecu_prev_write_idx);
    bool veh_advanced = (veh_idx_now != veh_prev_write_idx);

    uint32_t idx = 0;
    bool is_vehicle = false;

    if (ecu_advanced && !veh_advanced)
    {
        start_idx = ecu_prev_write_idx; // frame start for ECU stream
        idx = ecu_idx_now;
        ecu_prev_write_idx = ecu_idx_now;
    }
    else if (!ecu_advanced && veh_advanced)
    {
        start_idx = veh_prev_write_idx; // frame start for VEH stream
        idx = veh_idx_now;
        is_vehicle = true;
        veh_prev_write_idx = veh_idx_now;
    }
    else
    {
        // Fallback: pick the one with larger delta (handles rare simultaneous cases)
        uint32_t ecu_delta = (ecu_idx_now - ecu_prev_write_idx) & ECU_RING_MASK;
        uint32_t veh_delta = (veh_idx_now - veh_prev_write_idx) & VEH_RING_MASK;
        if (veh_delta > ecu_delta)
        {
            start_idx = veh_prev_write_idx;
            idx = veh_idx_now;
            is_vehicle = true;
            veh_prev_write_idx = veh_idx_now;
        }
        else
        {
            start_idx = ecu_prev_write_idx;
            idx = ecu_idx_now;
            ecu_prev_write_idx = ecu_idx_now;
        }
    }

    // Fast-path: extract 5-byte header from ring buffer at start_idx
    {
        volatile uint8_t *ring_base = is_vehicle ? vehicle_ring_buffer : ecu_ring_buffer;
        uint32_t ring_mask = is_vehicle ? VEH_RING_MASK : ECU_RING_MASK;

        uint8_t h0 = ring_base[(start_idx + 0) & ring_mask];
        uint8_t h1 = ring_base[(start_idx + 1) & ring_mask];
        uint8_t h2 = ring_base[(start_idx + 2) & ring_mask];
        uint8_t h3 = ring_base[(start_idx + 3) & ring_mask];
        uint8_t h4 = ring_base[(start_idx + 4) & ring_mask];
        (void)h3; // silence unused warnings; kept for clarity/extension

        current_frame_id = (uint16_t)(((uint16_t)(h0 & 0x07) << 8) | h1);
        // byte2 bits [7:1] hold payload_length_words (7 bits)
        uint8_t payload_len_words = (uint8_t)((h2 >> 1) & 0x7F);
        payload_length = (uint8_t)(payload_len_words * 2u); // bytes
        current_cycle_count = (uint8_t)(h4 & 0x3F);
        dma_channel_abort(dma_inject_chan);

        // Caching moved to main loop after validation
        try_to_inject_frame(current_frame_id, current_cycle_count);
    }

    // Encode: [31]=source(1=VEH), [30:12]=seq(19 bits), [11:0]=ring index (4KB ring)
    uint32_t encoded = notify_encode(is_vehicle, ((irq_counter++) & 0x7FFFF), idx);
    (void)notify_queue_push(encoded);
}

// injection/cache functions are declared in flexray_injector.h and implemented in flexray_injector.c

void inject_frame(uint16_t frame_id, uint8_t cycle_count, uint16_t injector_payload_length)
{
    dma_channel_set_read_addr((uint)dma_inject_chan, (const void *)injector_payload, false);
    dma_channel_set_trans_count((uint)dma_inject_chan, injector_payload_length, true);
}


void setup_stream(PIO pio,
                  uint rx_pin_from_ecu, uint tx_en_pin_to_vehicle,
                  uint rx_pin_from_vehicle, uint tx_en_pin_to_ecu)
{
    // --- Store PIO and SM for reset ---
    streamer_pio = pio;

    // --- PIO Setup ---
    uint offset = pio_add_program(pio, &flexray_bss_streamer_program);
    uint sm_from_ecu = pio_claim_unused_sm(pio, true);
    uint sm_from_vehicle = pio_claim_unused_sm(pio, true);

    streamer_sm_from_ecu = sm_from_ecu;
    streamer_sm_from_vehicle = sm_from_vehicle;

    flexray_bss_streamer_program_init(pio, sm_from_ecu, offset, rx_pin_from_ecu, tx_en_pin_to_vehicle);
    flexray_bss_streamer_program_init(pio, sm_from_vehicle, offset, rx_pin_from_vehicle, tx_en_pin_to_ecu);
    dma_data_from_ecu_chan = dma_claim_unused_channel(true);
    dma_data_from_vehicle_chan = dma_claim_unused_channel(true);
    dma_channel_config dma_c_from_ecu = dma_channel_get_default_config(dma_data_from_ecu_chan);
    dma_channel_config dma_c_from_vehicle = dma_channel_get_default_config(dma_data_from_vehicle_chan);
    channel_config_set_transfer_data_size(&dma_c_from_ecu, DMA_SIZE_8);
    channel_config_set_transfer_data_size(&dma_c_from_vehicle, DMA_SIZE_8);
    channel_config_set_read_increment(&dma_c_from_ecu, false);                               // Always read from same FIFO
    channel_config_set_read_increment(&dma_c_from_vehicle, false);                           // Always read from same FIFO
    channel_config_set_write_increment(&dma_c_from_ecu, true);                               // Write to sequential buffer locations
    channel_config_set_write_increment(&dma_c_from_vehicle, true);                           // Write to sequential buffer locations
    channel_config_set_dreq(&dma_c_from_ecu, pio_get_dreq(pio, sm_from_ecu, false));         // Paced by PIO RX
    channel_config_set_dreq(&dma_c_from_vehicle, pio_get_dreq(pio, sm_from_vehicle, false)); // Paced by PIO RX

    // Configure write-address ring for continuous circular write
    uint8_t ecu_ring_bits = 0;
    if (ECU_RING_SIZE_BYTES > 1)
    {
        ecu_ring_bits = 32 - __builtin_clz(ECU_RING_SIZE_BYTES - 1);
    }
    channel_config_set_ring(&dma_c_from_ecu, true, ecu_ring_bits); // true = wrap write address

    uint8_t veh_ring_bits = 0;
    if (VEH_RING_SIZE_BYTES > 1)
    {
        veh_ring_bits = 32 - __builtin_clz(VEH_RING_SIZE_BYTES - 1);
    }
    channel_config_set_ring(&dma_c_from_vehicle, true, veh_ring_bits); // true = wrap write address
    // Chain each data channel to its rearm channel
    dma_rearm_ecu_chan = dma_claim_unused_channel(true);
    dma_rearm_vehicle_chan = dma_claim_unused_channel(true);
    channel_config_set_chain_to(&dma_c_from_ecu, dma_rearm_ecu_chan);
    channel_config_set_chain_to(&dma_c_from_vehicle, dma_rearm_vehicle_chan);

    // Configure data channels
    dma_channel_configure(dma_data_from_ecu_chan, &dma_c_from_ecu,
                          (void *)ecu_ring_buffer,       // Destination: ECU ring base
                          &pio->rxf[sm_from_ecu],        // Source: PIO RX FIFO
                          DMA_BLOCK_COUNT_BYTES,
                          true);
    dma_channel_configure(dma_data_from_vehicle_chan, &dma_c_from_vehicle,
                          (void *)vehicle_ring_buffer,   // Destination: VEHICLE ring base
                          &pio->rxf[sm_from_vehicle],    // Source: PIO RX FIFO
                          DMA_BLOCK_COUNT_BYTES,
                          true);

    // Configure rearm channels: write a 32-bit refill value to data channel's transfer_count_trig
    // static uint32_t ecu_refill_count = DMA_BLOCK_COUNT_BYTES;
    // static uint32_t veh_refill_count = DMA_BLOCK_COUNT_BYTES;


    pio_set_irq0_source_enabled(pio, pis_interrupt3, true);
    irq_set_exclusive_handler(pio_get_irq_num(pio, 0), streamer_irq0_handler);
    irq_set_enabled(pio_get_irq_num(pio, 0), true);

    // dma_channel_start(dma_data_from_ecu_chan);
    // dma_channel_start(dma_data_from_vehicle_chan);
    pio_interrupt_clear(pio, 3);
    pio_interrupt_clear(pio, 7);
    pio_sm_set_enabled(pio, sm_from_ecu, true);
    pio_sm_set_enabled(pio, sm_from_vehicle, true);

    if (dma_inject_chan < 0)
    {
        dma_inject_chan = (int)dma_claim_unused_channel(true);
    }
    if (!dma_channel_is_busy((uint)dma_inject_chan))
    {
        injector_dc = dma_channel_get_default_config((uint)dma_inject_chan);
        channel_config_set_transfer_data_size(&injector_dc, DMA_SIZE_32);
        channel_config_set_read_increment(&injector_dc, true);
        channel_config_set_write_increment(&injector_dc, false);
        channel_config_set_dreq(&injector_dc, pio_get_dreq(pio2, 1, true)); // TX pacing
        dma_channel_set_config((uint)dma_inject_chan, &injector_dc, false);        
        dma_channel_set_write_addr((uint)dma_inject_chan, (void *)&pio2->txf[1], false);
    }
}
