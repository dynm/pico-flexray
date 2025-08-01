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
#include "pico/multicore.h"

#include "flexray_override_pipeline.pio.h"
#include "replay_frame.h"
#include "flexray_frame.h"
#include "panda_usb.h"
#include "flexray_bss_streamer.h"

#define SRAM __attribute__((section(".data")))
#define FLASH __attribute__((section(".rodata")))
// --- Configuration ---

// -- Streamer Pins --
#define REPLAY_TX_PIN 15
#define BGE_PIN 2
#define STBN_PIN 3

#define RXD_FROM_ECU_PIN 4
#define TXD_TO_ECU_PIN 5
#define TXEN_TO_ECU_PIN 6

#define RXD_FROM_VEHICLE_PIN 28
#define TXD_TO_VEHICLE_PIN 27
#define TXEN_TO_VEHICLE_PIN 26

// -- Injector Pins, do not connect physical pins to these --
#define INJECT_SWITCH_TO_ECU_PIN 20 
#define INJECT_SWITCH_TO_VEHICLE_PIN 21


void setup_forwarder(PIO pio,
    uint rx_pin_from_ecu, uint tx_pin_to_vehicle, uint inject_switch_to_vehicle,
    uint rx_pin_from_vehicle, uint tx_pin_to_ecu, uint inject_switch_to_ecu)
{
    uint offset = pio_add_program(pio, &flexray_forwarder_program);
    uint sm_from_ecu = pio_claim_unused_sm(pio, true);
    uint sm_from_vehicle = pio_claim_unused_sm(pio, true);
    
    flexray_forwarder_program_init(pio, sm_from_ecu, offset, rx_pin_from_ecu, tx_pin_to_vehicle, inject_switch_to_vehicle);
    flexray_forwarder_program_init(pio, sm_from_vehicle, offset, rx_pin_from_vehicle, tx_pin_to_ecu, inject_switch_to_ecu);
}

void print_pin_assignments() {
    printf("Test Data Output Pin: %02d\n", REPLAY_TX_PIN);
    printf("STBN Pin: %02d\n", STBN_PIN);
    printf("ECU Transceiver Pins: RXD=%02d, TXD=%02d, TXEN=%02d\n", RXD_FROM_ECU_PIN, TXD_TO_ECU_PIN, TXEN_TO_ECU_PIN);
    printf("VEH Transceiver Pins: RXD=%02d, TXD=%02d, TXEN=%02d\n", RXD_FROM_VEHICLE_PIN, TXD_TO_VEHICLE_PIN, TXEN_TO_VEHICLE_PIN);
    printf("Injector Pins: ECU=%02d, VEH=%02d\n", INJECT_SWITCH_TO_ECU_PIN, INJECT_SWITCH_TO_VEHICLE_PIN);
}

void core1_entry() {
    setup_stream(pio0, 
        RXD_FROM_ECU_PIN, TXEN_TO_VEHICLE_PIN, 
        RXD_FROM_VEHICLE_PIN, TXEN_TO_ECU_PIN);

    uint32_t last_dma_count_ecu = 0;
    uint32_t last_dma_count_vehicle = 0;
    int stall_count_ecu = 0;
    int stall_count_vehicle = 0;

    while (1) {
        sleep_ms(10); // Check every 10ms

        // Check ECU stream
        uint32_t current_dma_count_ecu = dma_channel_hw_addr(dma_data_from_ecu_chan)->transfer_count;
        if (current_dma_count_ecu == last_dma_count_ecu) {
            stall_count_ecu++;
        } else {
            stall_count_ecu = 0; // Reset counter if there's activity
        }
        last_dma_count_ecu = current_dma_count_ecu;

        // Check Vehicle stream
        uint32_t current_dma_count_vehicle = dma_channel_hw_addr(dma_data_from_vehicle_chan)->transfer_count;
        if (current_dma_count_vehicle == last_dma_count_vehicle) {
            stall_count_vehicle++;
        } else {
            stall_count_vehicle = 0; // Reset counter if there's activity
        }
        last_dma_count_vehicle = current_dma_count_vehicle;

        // If either stream has stalled for too long, reset the whole streamer
        // char *fmt_str = "Core1: DMA stall detected on %s. Resetting streamer...\n";
        if (stall_count_ecu > 10) { // ~100ms timeout
            // printf(fmt_str, "ECU");
            reset_streamer(STREAMER_SM_ECU);
            stall_count_ecu = 0; 
        }
        if (stall_count_vehicle > 10) { // ~100ms timeout
            // printf(fmt_str, "VEHICLE");
            reset_streamer(STREAMER_SM_VEHICLE);
            stall_count_vehicle = 0;
        }
    }
}

void setup_pins() {
    gpio_init(BGE_PIN);
    gpio_set_dir(BGE_PIN, GPIO_OUT);
    gpio_put(BGE_PIN, 1);

    gpio_init(STBN_PIN);
    gpio_set_dir(STBN_PIN, GPIO_OUT);
    gpio_put(STBN_PIN, 1);

    gpio_pull_up(TXEN_TO_ECU_PIN);
    gpio_pull_up(TXEN_TO_VEHICLE_PIN);

    // pull up inject switches, stop injecting
    gpio_pull_up(INJECT_SWITCH_TO_ECU_PIN);
    gpio_pull_up(INJECT_SWITCH_TO_VEHICLE_PIN);

    gpio_init(RXD_FROM_ECU_PIN);
    gpio_set_dir(RXD_FROM_ECU_PIN, GPIO_IN);
    gpio_init(RXD_FROM_VEHICLE_PIN);
    gpio_set_dir(RXD_FROM_VEHICLE_PIN, GPIO_IN);
    gpio_pull_up(RXD_FROM_ECU_PIN);
    gpio_pull_up(RXD_FROM_VEHICLE_PIN);
}

int main()
{
    setup_pins();

    bool clock_configured = set_sys_clock_khz(100000, false);
    stdio_init_all();
    
    // Initialize Panda USB interface
    panda_usb_init();
    // --- Set system clock to 100MHz (RP2350) ---
    // make PIO clock div has no fraction, reduce jitter
    if (!clock_configured) {
        printf("Warning: Failed to set system clock, using default\n");
    } else {
        printf("System clock set to 100MHz\n");
    }

    print_pin_assignments();

    printf("Actual system clock: %lu Hz\n", clock_get_hz(clk_sys));
    printf("\n--- FlexRay Continuous Streaming Bridge (Forwarder Mode) ---\n");

    uint dma_replay_chan = setup_replay(pio1, REPLAY_TX_PIN);
    
    multicore_launch_core1(core1_entry);
    setup_forwarder(pio1, 
        RXD_FROM_ECU_PIN, TXD_TO_VEHICLE_PIN, INJECT_SWITCH_TO_VEHICLE_PIN,
        RXD_FROM_VEHICLE_PIN, TXD_TO_ECU_PIN, INJECT_SWITCH_TO_ECU_PIN);
    
    uint8_t temp_buffer[MAX_FRAME_BUF_SIZE_BYTES];
    uint32_t main_loop_count = 0;
    uint32_t data_ready_count = 0;
    uint32_t irq_count_prev = 0;
    absolute_time_t next_print_time = make_timeout_time_ms(1000);
    absolute_time_t next_usb_health_check = make_timeout_time_ms(5000); // Check USB health every 5 seconds
    
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
            // panda_print_fifo_stats();
        }
        
        // check dma_replay_chan is stopped, restart it if it is
        if (!dma_channel_is_busy(dma_replay_chan))
        {
            // To restart the DMA, we must reset its read address to the beginning
            // of the buffer and then trigger it.
            dma_channel_set_read_addr(dma_replay_chan, replay_buffer, true);
        }

        uint32_t written_buffer_index;
        if (multicore_fifo_pop_timeout_us(0, &written_buffer_index))
        {
            data_ready_count++;
            // Copy the completed buffer (the one that's NOT currently being written to)
            memcpy(temp_buffer, (const void *)buffer_addresses[written_buffer_index], FRAME_BUF_SIZE_BYTES);

            // Parse the frame from the copied data with hardware clock measurement
            flexray_frame_t frame;
            // parse_frame(temp_buffer, &frame);
            // parse_frame_fast(temp_buffer, &frame);
            parse_frame(temp_buffer, &frame);

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
        }
    }

    return 0;
}
