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
#include "hardware/timer.h"

#include "flexray_override_pipeline.pio.h"
#include "replay_frame.h"
#include "flexray_frame.h"
#include "panda_usb.h"
#include "flexray_bss_streamer.h"

#define SRAM __attribute__((section(".data")))
#define FLASH __attribute__((section(".rodata")))
// --- Configuration ---

// -- Mode 1: Streamer Pins --
#define REPLAY_TX_PIN 15
#define RXD_FROM_ECU_PIN 13
#define TXEN_TO_VEHICLE_PIN 25
#define RXD_FROM_VEHICLE_PIN 14
#define TXEN_TO_ECU_PIN 16

// -- Mode 2: Forwarder Pins --
#define FR_RX_FROM_ECU_PIN 8
#define FR_TX_TO_VEHICLE_PIN 9
#define FR_TXEN_TO_VEHICLE_PIN 10

#define FR_RX_FROM_VEHICLE_PIN 11
#define FR_TX_TO_ECU_PIN 12
#define FR_TXEN_TO_ECU_PIN 13

// -- Mode 3: Interceptor Pins --
#define MITM_RX_PIN 8    // Receive from this pin
#define MITM_TX_PIN 9    // Transmit on this pin
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
// MOVED to flexray_bss_streamer.c

void setup_forwarder(PIO pio,
    uint rx_pin_from_ecu, uint tx_pin_to_vehicle, uint tx_en_pin_to_vehicle,
    uint rx_pin_from_vehicle, uint tx_pin_to_ecu, uint tx_en_pin_to_ecu)
{
    bool loopback_mode = true;
    uint offset = pio_add_program(pio, &flexray_forwarder_program);
    uint sm_from_ecu = pio_claim_unused_sm(pio, true);
    uint sm_from_vehicle = pio_claim_unused_sm(pio, true);
    flexray_forwarder_program_init(pio, sm_from_ecu, offset, rx_pin_from_ecu, tx_pin_to_vehicle, tx_en_pin_to_vehicle, loopback_mode);
    flexray_forwarder_program_init(pio, sm_from_vehicle, offset, rx_pin_from_vehicle, tx_pin_to_ecu, tx_en_pin_to_ecu, loopback_mode);

}

int main()
{
    // bool clock_configured = set_sys_clock_khz(100000, false);
    stdio_init_all();
    
    // Initialize Panda USB interface
    panda_usb_init();
    // --- Set system clock to 100MHz (RP2350) ---
    // make PIO clock div has no fraction, reduce jitter
    // if (!clock_configured) {
    //     printf("Warning: Failed to set system clock, using default\n");
    // } else {
    //     printf("System clock set to 100MHz\n");
    // }

    printf("FRAME_BUF_SIZE_WORDS: %u, flexray_frame_t size: %u\n", FRAME_BUF_SIZE_WORDS, sizeof(flexray_frame_t));

    printf("Actual system clock: %lu Hz\n", clock_get_hz(clk_sys));
    printf("\n--- FlexRay Continuous Streaming Bridge (RX Mode) ---\n");

    uint dma_replay_chan = setup_replay(pio1, REPLAY_TX_PIN);
    setup_stream(pio0, RXD_FROM_ECU_PIN, FR_TXEN_TO_VEHICLE_PIN, RXD_FROM_VEHICLE_PIN, TXEN_TO_ECU_PIN);
    // setup_forwarder(pio1, REPLAY_TX_PIN, FR_TX_TO_VEHICLE_PIN, TXEN_TO_VEHICLE_PIN, FR_RX_FROM_VEHICLE_PIN, FR_TX_TO_ECU_PIN, FR_TXEN_TO_ECU_PIN);
    printf("Connect GPIO %d to GPIO %d and send data\n", REPLAY_TX_PIN, RXD_FROM_ECU_PIN);
    printf("Waiting for %d bits (%d words) to be captured...\n", FRAME_BITS_TO_CAPTURE, FRAME_BUF_SIZE_WORDS);

    uint32_t temp_buffer[FRAME_BUF_SIZE_WORDS];
    uint32_t main_loop_count = 0;
    uint32_t data_ready_count = 0;
    uint32_t irq_count_prev = 0;
    absolute_time_t next_print_time = make_timeout_time_ms(1000);
    absolute_time_t next_usb_health_check = make_timeout_time_ms(5000); // Check USB health every 5 seconds
    
    printf("!!! FLEXRAY_FRAME_T_SIZE_IS: %u !!!\n", sizeof(flexray_frame_t));

    while (true)
    {
        main_loop_count++;
        
        // Handle Panda USB communication
        panda_usb_task();
        
        // Periodic USB health check
        if (time_reached(next_usb_health_check))
        {
            next_usb_health_check = make_timeout_time_ms(5000);
            
            // Check if USB is still mounted and working
            if (!tud_mounted())
            {
                printf("USB not mounted, attempting recovery...\n");
                // Force a complete USB reset
                tud_disconnect();
                sleep_ms(500);
                tud_init(0);
                sleep_ms(100);
                printf("USB recovery attempt completed\n");
            }
            
            // Print FIFO statistics periodically
            panda_print_fifo_stats();
        }
        
        // Wait for buffer index to change (indicating new data is ready)
        static uint32_t last_seen_buffer_index = 0;
        uint32_t current_seen_index = current_buffer_index;

        // check dma_replay_chan is stopped, restart it if it is
        if (!dma_channel_is_busy(dma_replay_chan))
        {
            // To restart the DMA, we must reset its read address to the beginning
            // of the buffer and then trigger it.
            dma_channel_set_read_addr(dma_replay_chan, replay_buffer, true);
        }

        if (g_new_data_available)
        {
            last_seen_buffer_index = current_seen_index;
            data_ready_count++;
            // Copy the completed buffer (the one that's NOT currently being written to)
            uint32_t completed_buffer_index = 1 - current_seen_index;
            memcpy(temp_buffer, (const void *)buffer_addresses[completed_buffer_index], FRAME_BUF_SIZE_WORDS * sizeof(uint32_t));

            // Parse the frame from the copied data with hardware clock measurement
            flexray_frame_t frame;
            // parse_frame(temp_buffer, &frame);
            parse_frame_fast(temp_buffer, &frame);

            // Store the latest version of the frame in a snapshot array if it's valid
            // short circuit the check if the frame_id is received
            if (is_valid_frame(&frame, temp_buffer))
            {
                // push the frame to the FIFO
                panda_flexray_fifo_push(&frame);
            }
            else
            {
                printf("Invalid frame: %d\n", frame.frame_id);
            }
            g_new_data_available = false;
        }
    }

    return 0;
}
