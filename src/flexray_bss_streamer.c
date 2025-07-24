#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"

#include "flexray_bss_streamer.pio.h"
#include "flexray_bss_streamer.h"
#include "flexray_frame.h"

// --- Global State ---
uint dma_data_from_ecu_chan;
uint dma_data_from_vehicle_chan;
uint dma_rearm_chan;
uint dma_control_chan; // New: for automatic buffer switching
static uint32_t rearm_data;

// Ping-pong buffers, max payload is 254, 8 bytes for header and trailer
volatile uint8_t capture_buffer_a[FRAME_BUF_SIZE_BYTES] __attribute__((aligned(32)));
volatile uint8_t capture_buffer_b[FRAME_BUF_SIZE_BYTES] __attribute__((aligned(32)));

// Address table for automatic buffer switching
volatile void *buffer_addresses[2] = {
    capture_buffer_a,
    capture_buffer_b};

// Current buffer index (for CPU to know which buffer was just filled)
volatile uint32_t current_buffer_index = 0;

// Flag to indicate that a new buffer is ready for processing
volatile bool g_new_data_available = false;

// Debug counter for interrupt handler
volatile uint32_t irq_handler_call_count = 0;

// This is the DMA interrupt handler, which is much more efficient.
void streamer_irq0_handler()
{
    // The interrupt is on PIO0's IRQ 0. We must clear this specific interrupt flag.
    // The previous code was using a system-level IRQ number, which is incorrect for this function.
    pio_interrupt_clear(pio0, 0);
    g_new_data_available = true;
    irq_handler_call_count++;

    uint32_t transfer_remaining_from_ecu = dma_channel_hw_addr(dma_data_from_ecu_chan)->transfer_count;
    uint32_t transfer_remaining_from_vehicle = dma_channel_hw_addr(dma_data_from_vehicle_chan)->transfer_count;
    if (transfer_remaining_from_ecu != FRAME_BUF_SIZE_BYTES) // from ecu transferred
    {
        ((uint8_t *)buffer_addresses[current_buffer_index])[FRAME_BUF_SIZE_BYTES - 1] = FROM_ECU;
    }
    if (transfer_remaining_from_vehicle != FRAME_BUF_SIZE_BYTES) // from vehicle transferred
    {
        ((uint8_t *)buffer_addresses[current_buffer_index])[FRAME_BUF_SIZE_BYTES - 1] = FROM_VEHICLE;
    }
    dma_channel_abort(dma_data_from_ecu_chan);
    dma_channel_abort(dma_data_from_vehicle_chan);
    current_buffer_index = 1 - current_buffer_index;

    dma_channel_set_write_addr(dma_data_from_ecu_chan, buffer_addresses[current_buffer_index], false);
    dma_channel_set_write_addr(dma_data_from_vehicle_chan, buffer_addresses[current_buffer_index], false);
    dma_channel_set_trans_count(dma_data_from_ecu_chan, FRAME_BUF_SIZE_BYTES, true);
    dma_channel_set_trans_count(dma_data_from_vehicle_chan, FRAME_BUF_SIZE_BYTES, true);
}

void setup_stream(PIO pio,
                  uint rx_pin_from_ecu, uint tx_en_pin_to_vehicle,
                  uint rx_pin_from_vehicle, uint tx_en_pin_to_ecu)
{
    // --- PIO Setup ---
    uint offset = pio_add_program(pio, &flexray_bss_streamer_program);
    uint sm_from_ecu = pio_claim_unused_sm(pio, true);
    uint sm_from_vehicle = pio_claim_unused_sm(pio, true);
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
    dma_channel_configure(dma_data_from_ecu_chan, &dma_c_from_ecu,
                          capture_buffer_a,       // Destination: Buffer A
                          &pio->rxf[sm_from_ecu], // Source: PIO RX FIFO
                          262,                     // Transfer count: 64 words (2048 bits)
                          false                   // Don't start yet
    );
    dma_channel_configure(dma_data_from_vehicle_chan, &dma_c_from_vehicle,
                          capture_buffer_b,           // Destination: Buffer B
                          &pio->rxf[sm_from_vehicle], // Source: PIO RX FIFO
                          262,                         // Transfer count: 64 words (2048 bits)
                          false                       // Don't start yet
    );
    pio_set_irq0_source_enabled(pio, pis_interrupt0, true);
    irq_set_exclusive_handler(pio_get_irq_num(pio, 0), streamer_irq0_handler);
    irq_set_enabled(pio_get_irq_num(pio, 0), true);

    dma_channel_start(dma_data_from_ecu_chan);
    dma_channel_start(dma_data_from_vehicle_chan);
    pio_sm_set_enabled(pio, sm_from_ecu, true);
    pio_sm_set_enabled(pio, sm_from_vehicle, true);
}