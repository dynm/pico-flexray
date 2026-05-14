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

#include "replay_frame.h"
#include "flexray_frame.h"
#include "panda_usb.h"
#include "flexray_bss_streamer.h"
#include "flexray_forwarder_with_injector.h"

#define SRAM __attribute__((section(".data")))
#define FLASH __attribute__((section(".rodata")))

#define ENABLE_PERIODIC_STATS 0
#define STATS_PERIOD_MS 5000
#define DIAG_FRAME_ID 0x10

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


// --- Configuration ---

// -- Streamer Pins --
#define REPLAY_TX_PIN 15
#define BGE_PIN 2
#define STBN_PIN 3

#define LED_PIN 20
#define RELAY_FR_1_2 17
#define RELAY_FR_3_4 18

#define TXD_FR_1_PIN 28
#define TXEN_FR_1_PIN 27
#define RXD_FR_1_PIN 26

#define TXD_FR_2_PIN 4
#define TXEN_FR_2_PIN 5
#define RXD_FR_2_PIN 6

#define TXD_FR_3_PIN 10
#define TXEN_FR_3_PIN 9
#define RXD_FR_3_PIN 8

#define TXD_FR_4_PIN 16
#define TXEN_FR_4_PIN 22
#define RXD_FR_4_PIN 21

// Forward declaration for the Core 1 counter
extern volatile uint32_t core1_sent_frame_count;


void print_pin_assignments(void)
{
    printf("Test Data Output Pin: %02d\n", REPLAY_TX_PIN);
    printf("BGE Pin: %02d\n", BGE_PIN);
    printf("STBN Pin: %02d\n", STBN_PIN);
    printf("FR1 Transceiver Pins: RXD=%02d, TXD=%02d, TXEN=%02d\n", RXD_FR_1_PIN, TXD_FR_1_PIN, TXEN_FR_1_PIN);
    printf("FR2 Transceiver Pins: RXD=%02d, TXD=%02d, TXEN=%02d\n", RXD_FR_2_PIN, TXD_FR_2_PIN, TXEN_FR_2_PIN);
    printf("FR3 Transceiver Pins: RXD=%02d, TXD=%02d, TXEN=%02d\n", RXD_FR_3_PIN, TXD_FR_3_PIN, TXEN_FR_3_PIN);
    printf("FR4 Transceiver Pins: RXD=%02d, TXD=%02d, TXEN=%02d\n", RXD_FR_4_PIN, TXD_FR_4_PIN, TXEN_FR_4_PIN);
}

typedef struct {
    uint32_t total_notif;
    uint32_t seq_gap;
    uint32_t parsed_ok;
    uint32_t valid;
    uint32_t len_mismatch;
    uint32_t len_ok;
    uint32_t parse_fail;
    uint32_t source_fr1;
    uint32_t source_fr2;
    uint32_t source_fr3;
    uint32_t source_fr4;
    uint32_t overflow_len;
    uint32_t zero_len;
    uint32_t panda_push_fail;
    uint32_t diag_seen;
    uint32_t diag_gap;
    uint32_t diag_repeat;
    uint32_t diag_backward;
    uint8_t diag_expected;
    uint8_t diag_last;
    bool diag_has_value;
} stream_stats_t;

uint8_t FRAME_CACHE[262][10];

#if ENABLE_PERIODIC_STATS
static void stats_print(const stream_stats_t *s, uint32_t prev_total, uint32_t prev_valid)
{
    uint32_t total_fps = (s->len_ok - prev_total) / (STATS_PERIOD_MS / 1000);
    uint32_t valid_fps = (s->valid - prev_valid) / (STATS_PERIOD_MS / 1000);

    printf("Ring Stats: total=%lu seq_gap=%lu src[FR1=%lu,FR2=%lu,FR3=%lu,FR4=%lu] len_ok=%lu len_mis=%lu overflow=%lu zero=%lu parse_fail=%lu valid=%lu push_fail=%lu | fps[frames=%lu/s,valid=%lu/s]\n",
           s->total_notif, s->seq_gap, s->source_fr1, s->source_fr2,
           s->source_fr3, s->source_fr4,
           s->len_ok, s->len_mismatch, s->overflow_len, s->zero_len,
           s->parse_fail, s->valid, s->panda_push_fail, total_fps, valid_fps);
    printf("Notify dropped=%lu diag_id=0x%02x seen=%lu gap=%lu repeat=%lu backward=%lu last=%u expected=%u\n",
           notify_queue_dropped(), DIAG_FRAME_ID, s->diag_seen, s->diag_gap,
           s->diag_repeat, s->diag_backward, s->diag_last, s->diag_expected);
}
#endif

static void diag_track_frame(stream_stats_t *s, const flexray_frame_t *frame)
{
    if (frame->frame_id != DIAG_FRAME_ID) {
        return;
    }

    uint8_t value = frame->cycle_count & 0x3F;
    if (!s->diag_has_value) {
        s->diag_has_value = true;
        s->diag_expected = (uint8_t)((value + 1u) & 0x3Fu);
        s->diag_last = value;
        s->diag_seen++;
        return;
    }

    if (value != s->diag_expected) {
        if (value == s->diag_last) {
            s->diag_repeat++;
        } else if (((value - s->diag_expected) & 0x3F) < 32) {
            s->diag_gap++;
        } else {
            s->diag_backward++;
        }
    }

    s->diag_expected = (uint8_t)((value + 1u) & 0x3Fu);
    s->diag_last = value;
    s->diag_seen++;
}

void core1_entry(void)
{
    setup_stream(pio0,
                 RXD_FR_1_PIN, TXEN_FR_2_PIN,
                 RXD_FR_2_PIN, TXEN_FR_1_PIN);

    setup_stream_fr34(pio1,
                      RXD_FR_3_PIN, TXEN_FR_4_PIN,
                      RXD_FR_4_PIN, TXEN_FR_3_PIN);
    while (1)
    {
        __wfi();
    }
}

void setup_pins(void)
{
    // disable transceiver
    gpio_init(BGE_PIN);
    gpio_set_dir(BGE_PIN, GPIO_OUT);
    gpio_put(BGE_PIN, 0);

    gpio_init(STBN_PIN);
    gpio_set_dir(STBN_PIN, GPIO_OUT);
    gpio_put(STBN_PIN, 0);

    gpio_pull_up(TXEN_FR_1_PIN);
    gpio_pull_up(TXEN_FR_2_PIN);
    gpio_pull_up(TXEN_FR_3_PIN);
    gpio_pull_up(TXEN_FR_4_PIN);

    gpio_init(RXD_FR_1_PIN);
    gpio_set_dir(RXD_FR_1_PIN, GPIO_IN);
    gpio_init(RXD_FR_2_PIN);
    gpio_set_dir(RXD_FR_2_PIN, GPIO_IN);

    gpio_init(RXD_FR_3_PIN);
    gpio_set_dir(RXD_FR_3_PIN, GPIO_IN);
    gpio_init(RXD_FR_4_PIN);
    gpio_set_dir(RXD_FR_4_PIN, GPIO_IN);

    gpio_pull_up(RXD_FR_1_PIN);
    gpio_pull_up(RXD_FR_2_PIN);
    gpio_pull_up(RXD_FR_3_PIN);
    gpio_pull_up(RXD_FR_4_PIN);

    gpio_init(RELAY_FR_1_2);
    gpio_set_dir(RELAY_FR_1_2, GPIO_OUT);
    gpio_put(RELAY_FR_1_2, 1);
    sleep_ms(500);
    gpio_init(RELAY_FR_3_4);
    gpio_set_dir(RELAY_FR_3_4, GPIO_OUT);
    gpio_put(RELAY_FR_3_4, 1);

    // delay enabling pins to avoid glitch
    sleep_ms(100);

    // enable transceiver
    gpio_put(BGE_PIN, 1);
    gpio_put(STBN_PIN, 1);

    // Debug profiling pin: GPIO7 low = idle, high = ISR processing
    gpio_init(7);
    gpio_set_dir(7, GPIO_OUT);
    gpio_put(7, 0);

	// On-board LED
	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);
}

int main(void)
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

    // setup_replay(pio1, REPLAY_TX_PIN);

    multicore_launch_core1(core1_entry);
    sleep_ms(500);


    setup_forwarder_with_injector(pio2,
                                  RXD_FR_1_PIN, TXD_FR_2_PIN,
                                  RXD_FR_2_PIN, TXD_FR_1_PIN,
                                  RXD_FR_3_PIN, TXD_FR_4_PIN,
                                  RXD_FR_4_PIN, TXD_FR_3_PIN);

    stream_stats_t stats = (stream_stats_t){0};

    uint8_t temp_buffer[MAX_FRAME_BUF_SIZE_BYTES];

	absolute_time_t next_stats_print_time = make_timeout_time_ms(STATS_PERIOD_MS);
	absolute_time_t next_led_toggle_time = make_timeout_time_ms(500);
	bool led_on = false;
#if ENABLE_PERIODIC_STATS
    // Track previous len_ok to compute parsed-frames FPS
    uint32_t prev_total = 0;
    uint32_t prev_valid = 0;
#endif

    while (true)
    {
        panda_usb_task();
		if (time_reached(next_led_toggle_time))
		{
			next_led_toggle_time = make_timeout_time_ms(500);
			led_on = !led_on;
			gpio_put(LED_PIN, led_on);
		}
        if (time_reached(next_stats_print_time))
        {
#if ENABLE_PERIODIC_STATS
            next_stats_print_time = make_timeout_time_ms(STATS_PERIOD_MS);
            stats_print(&stats, prev_total, prev_valid);
            prev_total = stats.len_ok;
            prev_valid = stats.valid;
            print_ram_usage();
#else
            next_stats_print_time = make_timeout_time_ms(STATS_PERIOD_MS);
#endif
        }

        // Consume frame-end notifications from core1 (FR1/FR2 only;
        // FR3/FR4 ISR just records frame IDs for channel demuxing)
        static uint16_t last_end_idx_fr1 = 0;
        static uint16_t last_end_idx_fr2 = 0;
        static uint32_t last_seq = 0;

        uint32_t encoded;
        if (!notify_queue_pop(&encoded))
        {
            panda_usb_task();
            __wfe();
            continue;
        }
        do {
            notify_info_t info; notify_decode(encoded, &info);

            stats.total_notif++;
            if (stats.total_notif > 1 && ((info.seq - last_seq) & 0x3FFFF) != 1) stats.seq_gap++;
            last_seq = info.seq;

            if (info.is_fr2) stats.source_fr2++;
            else stats.source_fr1++;

            volatile uint8_t *ring_base;
            uint16_t ring_mask;
            uint16_t prev_end;
            uint8_t source;

            if (info.is_fr2) {
                ring_base = fr2_ring_buffer;
                ring_mask = FR2_RING_MASK;
                prev_end = last_end_idx_fr2;
                source = FROM_FR2;
            } else {
                ring_base = fr1_ring_buffer;
                ring_mask = FR1_RING_MASK;
                prev_end = last_end_idx_fr1;
                source = FROM_FR1;
            }

            uint16_t len = (uint16_t)((info.end_idx - prev_end) & ring_mask);

            if (len == 0 || len > MAX_FRAME_BUF_SIZE_BYTES)
            {
                if (info.is_fr2) last_end_idx_fr2 = info.end_idx;
                else last_end_idx_fr1 = info.end_idx;
                if (len == 0) stats.zero_len++;
                else stats.overflow_len++;
                continue;
            }

            uint16_t start = (uint16_t)((info.end_idx - len) & ring_mask);
            uint16_t first = (uint16_t)((len <= (ring_mask + 1 - start)) ? len : (ring_mask + 1 - start));
            memcpy(temp_buffer, (const void *)(ring_base + start), first);
            if (first < len)
            {
                memcpy(temp_buffer + first, (const void *)ring_base, (size_t)(len - first));
            }

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
                    break;
                }

                stats.len_ok++;

                flexray_frame_t frame;
                if (!parse_frame_from_slice(header, expected_len, source, &frame))
                {
                    stats.parse_fail++;
                    pos = (uint16_t)(pos + 1);
                    continue;
                }
                else if (is_valid_frame(&frame, header))
                {
                    stats.valid++;

                    static uint8_t prev_cycle_for_decay = 0xFF;
                    if ((frame.cycle_count >> 4) != (prev_cycle_for_decay >> 4))
                        decay_frame_source_counts();
                    prev_cycle_for_decay = frame.cycle_count;

                    uint8_t demuxed = lookup_frame_source(frame.frame_id);
                    if (demuxed != FROM_UNKNOWN) {
                        frame.source |= demuxed;
                        if (demuxed & FROM_FR3) stats.source_fr3++;
                        if (demuxed & FROM_FR4) stats.source_fr4++;
                    }
                    try_cache_last_target_frame(frame.frame_id, frame.cycle_count, expected_len, header);
                    diag_track_frame(&stats, &frame);
                    if (!panda_flexray_fifo_push(&frame)) {
                        stats.panda_push_fail++;
                    }
                }

                pos = (uint16_t)(pos + expected_len);
            }

            if (info.is_fr2) last_end_idx_fr2 = info.end_idx;
            else last_end_idx_fr1 = info.end_idx;
        } while (notify_queue_pop(&encoded));
    }

    return 0;
}
