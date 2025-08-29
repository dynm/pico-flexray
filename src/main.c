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
#include <unistd.h>
#include "hardware/regs/addressmap.h"

extern char __end__;
extern char __StackTop;
extern char __StackLimit;

static inline uintptr_t get_sp(void) {
	uintptr_t sp;
	__asm volatile ("mov %0, sp" : "=r"(sp));
	return sp;
}

static void print_ram_usage(void) {
	void *heap_end = sbrk(0);
	uintptr_t sp = get_sp();

	uintptr_t heap_start = (uintptr_t)&__end__;
	uintptr_t stack_top = (uintptr_t)&__StackTop;
	uintptr_t stack_limit = (uintptr_t)&__StackLimit;

	size_t heap_used = (uintptr_t)heap_end - heap_start;
	size_t stack_used = stack_top - sp;
	size_t gap_heap_to_sp = sp - (uintptr_t)heap_end;   // remaining space between heap and sp
	size_t stack_free = sp - stack_limit;               // remaining space in stack

	printf("RAM usage: heap_used=%lu B, stack_used=%lu B, gap(heap->sp)=%lu B, stack_free=%lu B\n",
	       (unsigned long)heap_used,
	       (unsigned long)stack_used,
	       (unsigned long)gap_heap_to_sp,
	       (unsigned long)stack_free);
}

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

#define TXD_TO_ECU_PIN 4
#define TXEN_TO_ECU_PIN 5
#define RXD_FROM_ECU_PIN 6

#define TXD_TO_VEHICLE_PIN 28
#define TXEN_TO_VEHICLE_PIN 27
#define RXD_FROM_VEHICLE_PIN 26

// Forward declaration for the Core 1 counter
extern volatile uint32_t core1_sent_frame_count;


void setup_forwarder_with_injector(PIO pio,
    uint rx_pin_from_ecu, uint tx_pin_to_vehicle,
    uint rx_pin_from_vehicle, uint tx_pin_to_ecu)
{
    uint offset = pio_add_program(pio, &flexray_forwarder_with_injector_program);
    uint sm_from_ecu_to_vehicle = pio_claim_unused_sm(pio, true);
    uint sm_from_vehicle_to_ecu = pio_claim_unused_sm(pio, true);

    flexray_forwarder_with_injector_program_init(pio, sm_from_ecu_to_vehicle, offset, rx_pin_from_ecu, tx_pin_to_vehicle);
    flexray_forwarder_with_injector_program_init(pio, sm_from_vehicle_to_ecu, offset, rx_pin_from_vehicle, tx_pin_to_ecu);
}

void print_pin_assignments()
{
    printf("Test Data Output Pin: %02d\n", REPLAY_TX_PIN);
    printf("BGE Pin: %02d\n", BGE_PIN);
    printf("STBN Pin: %02d\n", STBN_PIN);
    printf("ECU Transceiver Pins: RXD=%02d, TXD=%02d, TXEN=%02d\n", RXD_FROM_ECU_PIN, TXD_TO_ECU_PIN, TXEN_TO_ECU_PIN);
    printf("VEH Transceiver Pins: RXD=%02d, TXD=%02d, TXEN=%02d\n", RXD_FROM_VEHICLE_PIN, TXD_TO_VEHICLE_PIN, TXEN_TO_VEHICLE_PIN);
}

typedef struct {
    uint32_t total_notif;
    uint32_t seq_gap;
    uint32_t parsed_ok;
    uint32_t valid;
    uint32_t len_mismatch;
    uint32_t len_ok;
    uint32_t parse_fail;
    uint32_t source_ecu;
    uint32_t source_veh;
    uint32_t overflow_len;
    uint32_t zero_len;
} stream_stats_t;

uint8_t FRAME_CACHE[262][10];

static void stats_print(const stream_stats_t *s, uint32_t prev_total, uint32_t prev_valid)
{
    // Use number of parsed frames (len_ok) to represent total frames per second
    uint32_t total_fps = (s->len_ok - prev_total) / 5; // 5s interval
    uint32_t valid_fps = (s->valid - prev_valid) / 5;       // 5s interval

    printf("Ring Stats: total=%lu seq_gap=%lu src[ECU=%lu,VEH=%lu] len_ok=%lu len_mis=%lu overflow=%lu zero=%lu parse_fail=%lu valid=%lu | fps[frames=%lu/s,valid=%lu/s]\n",
           s->total_notif, s->seq_gap, s->source_ecu, s->source_veh,
           s->len_ok, s->len_mismatch, s->overflow_len, s->zero_len,
           s->parse_fail, s->valid, total_fps, valid_fps);
    printf("Notify dropped=%lu\n", notify_queue_dropped());
}

void core1_entry()
{
    setup_stream(pio0,
                 RXD_FROM_ECU_PIN, TXEN_TO_VEHICLE_PIN,
                 RXD_FROM_VEHICLE_PIN, TXEN_TO_ECU_PIN);

    while (1)
    {
        __wfi();
    }
}

void setup_pins()
{
    // disable transceiver
    gpio_init(BGE_PIN);
    gpio_set_dir(BGE_PIN, GPIO_OUT);
    gpio_put(BGE_PIN, 0);

    gpio_init(STBN_PIN);
    gpio_set_dir(STBN_PIN, GPIO_OUT);
    gpio_put(STBN_PIN, 0);

    gpio_pull_up(TXEN_TO_ECU_PIN);
    gpio_pull_up(TXEN_TO_VEHICLE_PIN);

    gpio_init(RXD_FROM_ECU_PIN);
    gpio_set_dir(RXD_FROM_ECU_PIN, GPIO_IN);
    gpio_init(RXD_FROM_VEHICLE_PIN);
    gpio_set_dir(RXD_FROM_VEHICLE_PIN, GPIO_IN);
    gpio_pull_up(RXD_FROM_ECU_PIN);
    gpio_pull_up(RXD_FROM_VEHICLE_PIN);

    // delay enabling pins to avoid glitch
    sleep_ms(100);

    // enable transceiver
    gpio_put(BGE_PIN, 1);
    gpio_put(STBN_PIN, 1);
}

int main()
{
    setup_pins();

    bool clock_configured = set_sys_clock_khz(100000, true);
    stdio_init_all();
    printf("static_used=%lu B\n", (unsigned long)((uintptr_t)&__end__ - (uintptr_t)SRAM_BASE));
    print_ram_usage();
    // Initialize Panda USB interface
    panda_usb_init();
    // Initialize cross-core notification queue before starting streams
    notify_queue_init();
    // --- Set system clock to 100MHz (RP2350) ---
    // make PIO clock div has no fraction, reduce jitter
    if (!clock_configured)
    {
        printf("Warning: Failed to set system clock, using default\n");
    }
    else
    {
        printf("System clock set to 125MHz\n");
    }

    print_pin_assignments();

    printf("Actual system clock: %lu Hz\n", clock_get_hz(clk_sys));
    printf("\n--- FlexRay Continuous Streaming Bridge (Forwarder Mode) ---\n");

    setup_replay(pio1, REPLAY_TX_PIN);

    multicore_launch_core1(core1_entry);
    sleep_ms(500);


    setup_forwarder_with_injector(pio2,
                                  RXD_FROM_ECU_PIN, TXD_TO_VEHICLE_PIN,
                                  RXD_FROM_VEHICLE_PIN, TXD_TO_ECU_PIN);

    stream_stats_t stats = (stream_stats_t){0};

    uint8_t temp_buffer[MAX_FRAME_BUF_SIZE_BYTES];

    absolute_time_t next_stats_print_time = make_timeout_time_ms(5000);
    // Track previous len_ok to compute parsed-frames FPS
    uint32_t prev_total = 0;
    uint32_t prev_valid = 0;

    while (true)
    {
        panda_usb_task();
        if (time_reached(next_stats_print_time))
        {
            next_stats_print_time = make_timeout_time_ms(5000);
            stats_print(&stats, prev_total, prev_valid);
            prev_total = stats.len_ok;
            prev_valid = stats.valid;
            print_ram_usage();
        }

        // Consume frame-end notifications from core1 (encoded source+seq+ring index)
        static uint16_t last_end_idx_ecu = 0;
        static uint16_t last_end_idx_veh = 0;
        static uint32_t last_seq = 0;

        uint32_t encoded;
        if (!notify_queue_pop(&encoded))
        {
            // No pending notifications: keep USB serviced and wait
            panda_usb_task();
            __wfe();
            continue;
        }
        // Drain the queue including the first popped item
        do {
            notify_info_t info; notify_decode(encoded, &info);

            stats.total_notif++;
            if (stats.total_notif > 1 && ((info.seq - last_seq) & 0x7FFFF) != 1) stats.seq_gap++;
            last_seq = info.seq;
            if (info.is_vehicle) {
                stats.source_veh++;
            } else {
                stats.source_ecu++;
            }

            volatile uint8_t *ring_base = info.is_vehicle ? vehicle_ring_buffer : ecu_ring_buffer;
            uint16_t ring_mask = info.is_vehicle ? VEH_RING_MASK : ECU_RING_MASK;
            uint16_t prev_end = info.is_vehicle ? last_end_idx_veh : last_end_idx_ecu;
            uint16_t len = (uint16_t)((info.end_idx - prev_end) & ring_mask);

            if (len == 0 || len > MAX_FRAME_BUF_SIZE_BYTES)
            {
                // Update prev_end to avoid stalling if zero or oversized
                if (info.is_vehicle) {
                    last_end_idx_veh = info.end_idx;
                } else {
                    last_end_idx_ecu = info.end_idx;
                }
                if (len == 0) 
                {
                    stats.zero_len++;
                } else {
                    stats.overflow_len++;
                }
                continue;
            }

            // Copy [len] bytes ending at end_idx from ring (handle wrap)
            uint16_t start = (uint16_t)((info.end_idx - len) & ring_mask);
            uint16_t first = (uint16_t)((len <= (ring_mask + 1 - start)) ? len : (ring_mask + 1 - start));
            memcpy(temp_buffer, (const void *)(ring_base + start), first);
            if (first < len)
            {
                memcpy(temp_buffer + first, (const void *)ring_base, (size_t)(len - first));
            }

            // The chunk may contain multiple complete frames (e.g., if some notifications were missed).
            // Iterate and parse frames sequentially within [0, len).
            uint16_t pos = 0;
            while ((uint16_t)(len - pos) >= 8)
            {
                uint8_t *header = temp_buffer + pos;
                uint8_t payload_len_words = (header[2] >> 1) & 0x7F;
                uint16_t expected_len = (uint16_t)(5 + (payload_len_words * 2) + 3);
                if (expected_len == 0 || expected_len > FRAME_BUF_SIZE_BYTES) {
                    stats.len_mismatch++;
                    break;
                }
                if ((uint16_t)(len - pos) < expected_len) {
                    // Incomplete tail (shouldn't happen since end_idx is on frame end), stop.
                    break;
                }

                stats.len_ok++;

                flexray_frame_t frame;
                if (!parse_frame_from_slice(header, expected_len, info.is_vehicle ? FROM_VEHICLE : FROM_ECU, &frame))
                {
                    stats.parse_fail++;
                    // Parse failed: resync by advancing 1 byte and retry
                    pos = (uint16_t)(pos + 1);
                    continue;
                }
                else if (is_valid_frame(&frame, header))
                {
                    stats.valid++;
                    panda_flexray_fifo_push(&frame);
                }

                // Parsed (even if invalid CRC): consume this frame length
                pos = (uint16_t)(pos + expected_len);
            }

            if (info.is_vehicle) {
                last_end_idx_veh = info.end_idx;
            } else {
                last_end_idx_ecu = info.end_idx;
            }
        } while (notify_queue_pop(&encoded));
    }

    return 0;
}
