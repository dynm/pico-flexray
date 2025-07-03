#include <stdio.h>
#include <limits.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/regs/io_bank0.h"
#include "hardware/structs/iobank0.h"
#include "hardware/pll.h"
#include "hardware/xosc.h"

#define REAL_DATA 1

#include "flexray_bss_streamer.pio.h"
#include "replay_frame.h"
#include "flexray_frame.h"

// --- Configuration ---

// -- Mode 1: Streamer Pins --
#define REPLAY_TX_PIN 15
#define RXD_PIN  13
#define DEBUG_PIN 25

// -- Mode 2: Forwarder Pins --
#define FR_CH1_RX 8
#define FR_CH1_TX 9
#define FR_CH1_TXEN 10

#define FR_CH2_RX 11
#define FR_CH2_TX 12
#define FR_CH2_TXEN 13

// -- Mode 3: Interceptor Pins --
#define MITM_RX_PIN 8   // Receive from this pin
#define MITM_TX_PIN 9   // Transmit on this pin
#define MITM_TXEN_PIN 10 // Transmit enable for TX pin

// -- Mode 4 & 5: On-the-fly Override Pins --
#define OVR_RX_PIN 8
#define OVR_TX_PIN 9
#define OVR_TXEN_PIN 10
#define OVR_DECISION_PIN 11 // Internal pipeline communication pin for Mode 5

#define FLEXRAY_BIT_RATE_MBPS 10
#define OVERSAMPLE_RATE 10

// --- Streamer-specific definitions ---
// Constants like FRAME_BITS_TO_CAPTURE, FRAME_BUF_SIZE_WORDS, and TOTAL_FRAMES
// have been moved to flexray_frame.h for better modularity.

// --- Global State ---
uint dma_data_chan;
uint dma_rearm_chan;
uint dma_control_chan;  // New: for automatic buffer switching
static uint32_t rearm_data;

// Ping-pong buffers
volatile uint32_t capture_buffer_a[FRAME_BUF_SIZE_WORDS] __attribute__((aligned(FRAME_BITS_TO_CAPTURE/8)));
volatile uint32_t capture_buffer_b[FRAME_BUF_SIZE_WORDS] __attribute__((aligned(FRAME_BITS_TO_CAPTURE/8)));

// Address table for automatic buffer switching
volatile void *buffer_addresses[2] = {
    capture_buffer_a,
    capture_buffer_b
};

// Current buffer index (for CPU to know which buffer was just filled)
volatile uint32_t current_buffer_index = 0;

// Flag to indicate that a new buffer is ready for processing
volatile bool g_new_data_available = false;

// Debug counter for interrupt handler
volatile uint32_t irq_handler_call_count = 0;

// This is the DMA interrupt handler, which is much more efficient.
void streamer_irq0_handler() {
    // The interrupt is on PIO0's IRQ 0. We must clear this specific interrupt flag.
    // The previous code was using a system-level IRQ number, which is incorrect for this function.
    pio_interrupt_clear(pio0, 0);
    g_new_data_available = true;
    irq_handler_call_count++;
    
    uint32_t transfer_count = dma_channel_hw_addr(dma_data_chan)->transfer_count;
    dma_channel_abort(dma_data_chan);
    current_buffer_index = 1 - current_buffer_index;

    dma_channel_set_write_addr(dma_data_chan, buffer_addresses[current_buffer_index], false);
    dma_channel_set_trans_count(dma_data_chan, FRAME_BUF_SIZE_WORDS, true);
}

void setup_stream(PIO pio, uint rx_pin, uint debug_pin) {
    // --- PIO Setup ---
    uint offset = pio_add_program(pio, &flexray_bss_streamer_program);
    uint sm = pio_claim_unused_sm(pio, true);
    flexray_bss_streamer_program_init(pio, sm, offset, rx_pin, debug_pin);
    dma_data_chan = dma_claim_unused_channel(true);
    dma_channel_config dma_c = dma_channel_get_default_config(dma_data_chan);
    channel_config_set_transfer_data_size(&dma_c, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_c, false);  // Always read from same FIFO
    channel_config_set_write_increment(&dma_c, true);  // Write to sequential buffer locations
    channel_config_set_dreq(&dma_c, pio_get_dreq(pio, sm, false)); // Paced by PIO RX
    pio_set_irq0_source_enabled(pio, pis_interrupt0, true);
    dma_channel_configure(dma_data_chan, &dma_c,
        capture_buffer_a,           // Destination: Buffer A
        &pio->rxf[sm],           // Source: PIO RX FIFO
        64,      // Transfer count: 64 words (2048 bits)
        false                      // Don't start yet
    );
    
    irq_set_exclusive_handler(pio_get_irq_num(pio, 0), streamer_irq0_handler);
    irq_set_enabled(pio_get_irq_num(pio, 0), true);

    dma_channel_start(dma_data_chan);
    pio_sm_set_enabled(pio, sm, true);
}

int main() {
    // bool clock_configured = set_sys_clock_khz(100000, false);
    stdio_init_all();
    // --- Set system clock to 100MHz (RP2350) ---
    // make PIO clock div has no fraction, reduce jitter
    // if (!clock_configured) {
    //     printf("Warning: Failed to set system clock, using default\n");
    // } else {
    //     printf("System clock set to 100MHz\n");
    // }    
    
    
    printf("Actual system clock: %lu Hz\n", clock_get_hz(clk_sys));    
    printf("\n--- FlexRay Continuous Streaming Bridge (RX Mode) ---\n");
    

    uint dma_replay_chan = setup_replay(pio1, REPLAY_TX_PIN);
    setup_stream(pio0, RXD_PIN, DEBUG_PIN);


    printf("Connect GPIO %d to GPIO %d and send data\n", REPLAY_TX_PIN, RXD_PIN);
    printf("Waiting for %d bits (%d words) to be captured...\n", FRAME_BITS_TO_CAPTURE, FRAME_BUF_SIZE_WORDS);

    flexray_frame_t frame_snap[TOTAL_FRAMES] = {0};
    printf("size of frame_snap: %d\n", sizeof(flexray_frame_t));
    printf("size of total frames: %d\n", sizeof(frame_snap));
    bool frame_received[TOTAL_FRAMES] = {false}; // Tracks which frames have been validated
    uint32_t temp_buffer[FRAME_BUF_SIZE_WORDS];
    int print_trigger = 0;
    uint32_t main_loop_count = 0;
    uint32_t data_ready_count = 0;
    uint32_t irq_count_prev = 0;

    while (true) {
        main_loop_count++;
        // Wait for buffer index to change (indicating new data is ready)
        static uint32_t last_seen_buffer_index = 0;
        uint32_t current_seen_index = current_buffer_index;

        // check dma_replay_chan is stopped, restart it if it is
        if (!dma_channel_is_busy(dma_replay_chan)) {
            // To restart the DMA, we must reset its read address to the beginning
            // of the buffer and then trigger it.
            dma_channel_set_read_addr(dma_replay_chan, replay_buffer, true);
        }

        if (g_new_data_available) {
            last_seen_buffer_index = current_seen_index;
            data_ready_count++;
            // Copy the completed buffer (the one that's NOT currently being written to)
            uint32_t completed_buffer_index = 1 - current_seen_index;
            memcpy(temp_buffer, (const void *)buffer_addresses[completed_buffer_index], FRAME_BUF_SIZE_WORDS * sizeof(uint32_t));
            
            // Parse the frame from the copied data
            flexray_frame_t frame;
            parse_frame(temp_buffer, &frame);

            // Store the latest version of the frame in a snapshot array if it's valid
            // short circuit the check if the frame_id is received
            if (frame_received[frame.frame_id] || is_valid_frame(&frame, temp_buffer)) {
                 frame_snap[frame.frame_id] = frame;
                 frame_received[frame.frame_id] = true;
            }

            // Periodically print the collected frame data
            print_trigger++;
            if (print_trigger >= 5000) { // Print summary every 1000 captured frames
                print_trigger = 0; 
                // printf("Main Loop Count: %d, IRQ Count: %d, Data Ready Count: %d, Diff: %d\n", main_loop_count, irq_handler_call_count, data_ready_count,  irq_handler_call_count - data_ready_count);
                printf("ID,Len,HeaderCRC,Cyc,Data,PayloadCRC\n");
                for (int i = 0; i < TOTAL_FRAMES; i++) {
                    // Only print frames that have been received and validated
                    if (frame_received[i]) {
                        print_frame(&frame_snap[i]);
                    }
                }
            
            }
            g_new_data_available = false;
        }
        // For power efficiency, the CPU could wait for an interrupt here.
        // For example: __wfi(); 
    }


    return 0;
}

