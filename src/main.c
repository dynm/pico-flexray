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

#include "board_config.h"
#if FLEXRAY_FRAME_GEN
#include "flexray_frame_gen.h"
#endif
#include "flexray_frame.h"
#include "panda_usb.h"
#include "flexray_bss_streamer.h"
#include "flexray_forwarder_with_injector.h"
#include "usb_netif.h"
#include "udp_server.h"
#include "dhserver.h"

#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "lwip/ip4_addr.h"
#include "lwip/def.h"

#define SRAM __attribute__((section(".data")))
#define FLASH __attribute__((section(".rodata")))

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
#define BGE_PIN 2
#define STBN_PIN 3

#define LED_PIN 20
#define RELAY_FR_1_2 17
#define RELAY_FR_3_4 18

// Forward declaration for the Core 1 counter
extern volatile uint32_t core1_sent_frame_count;


void print_pin_assignments(void)
{
    printf("BGE Pin: %02d\n", BGE_PIN);
    printf("STBN Pin: %02d\n", STBN_PIN);
    printf("FR1 Transceiver Pins: RXD=%02d, TXD=%02d, TXEN=%02d\n", RXD_FR_1_PIN, TXD_FR_1_PIN, TXEN_FR_1_PIN);
    printf("FR2 Transceiver Pins: RXD=%02d, TXD=%02d, TXEN=%02d\n", RXD_FR_2_PIN, TXD_FR_2_PIN, TXEN_FR_2_PIN);
#if !FLEXRAY_FRAME_GEN
    printf("FR3 Transceiver Pins: RXD=%02d, TXD=%02d, TXEN=%02d\n", RXD_FR_3_PIN, TXD_FR_3_PIN, TXEN_FR_3_PIN);
    printf("FR4 Transceiver Pins: RXD=%02d, TXD=%02d, TXEN=%02d\n", RXD_FR_4_PIN, TXD_FR_4_PIN, TXEN_FR_4_PIN);
#endif
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
} stream_stats_t;

static stream_stats_t stats;

typedef struct {
    uint32_t flush_attempts;
    uint32_t flush_ok;
    uint32_t flush_failures;
    uint32_t frames_pushed;
    uint32_t frames_rejected;
    uint32_t frames_dropped;
    uint32_t bytes_attempted;
    uint32_t bytes_dropped;
} udp_batch_stats_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t uptime_ms;
    uint32_t total_notif;
    uint32_t seq_gap;
    uint32_t len_ok;
    uint32_t len_mismatch;
    uint32_t overflow_len;
    uint32_t zero_len;
    uint32_t parse_fail;
    uint32_t valid;
    uint32_t notify_dropped;
    udp_batch_stats_t batch;
    udp_server_stats_t udp;
    usb_netif_stats_t ncm;
} profile_stats_wire_t;

#define PROFILE_STATS_MAGIC 0x59524650u
#define PROFILE_STATS_VERSION 1u
#define PROFILE_STATS_QUERY "PFRY_STATS"

uint8_t FRAME_CACHE[262][10];

// --- USB NCM / lwIP / UDP ---
static struct netif ncm_netif;
static ip4_addr_t ncm_ip;
static ip4_addr_t ncm_netmask;
static ip4_addr_t ncm_gw;

// Minimal DHCP server: hand out IPs on 192.168.7.0/24 with no router/gateway.
#define DHCP_NUM_ENTRIES 3
#define DHCP_INIT_IP4(a, b, c, d) { PP_HTONL(LWIP_MAKEU32(a, b, c, d)) }
static dhcp_entry_t dhcp_entries[DHCP_NUM_ENTRIES] = {
    {{0}, DHCP_INIT_IP4(192, 168, 7, 2), 24 * 60 * 60},
    {{0}, DHCP_INIT_IP4(192, 168, 7, 3), 24 * 60 * 60},
    {{0}, DHCP_INIT_IP4(192, 168, 7, 4), 24 * 60 * 60},
};
static const dhcp_config_t dhcp_config = {
    .router = {0},       // no default route
    .port = 67,
    .dns = {0},          // no DNS server
    .domain = NULL,
    .num_entry = DHCP_NUM_ENTRIES,
    .entries = dhcp_entries,
};

static void ncm_udp_recv_handler(const uint8_t *data, uint16_t len,
                                 const ip_addr_t *addr, uint16_t port);

// Low-latency micro-batching for the high-rate FlexRay stream. Keep UDP below
// the Ethernet MTU, wait briefly for fuller datagrams, and queue completed
// batches so transient USB NCM backpressure does not discard whole batches.
#define UDP_BATCH_BUF_SIZE 1400u
#define UDP_BATCH_FLUSH_MS 4u
#define UDP_PENDING_BATCH_SLOTS 16u
#define UDP_PENDING_SEND_BUDGET 2u
#define NOTIFY_PROCESS_BUDGET 32u

typedef struct {
    uint16_t len;
    uint16_t frame_count;
    uint8_t data[UDP_BATCH_BUF_SIZE];
} udp_pending_batch_t;

static uint8_t udp_batch_buf[UDP_BATCH_BUF_SIZE];
static uint16_t udp_batch_len;
static uint16_t udp_batch_frame_count;
static absolute_time_t udp_batch_deadline;
static udp_batch_stats_t udp_batch_stats;
static udp_pending_batch_t udp_pending_batches[UDP_PENDING_BATCH_SLOTS];
static uint8_t udp_pending_head;
static uint8_t udp_pending_tail;
static uint8_t udp_pending_count;

static void udp_batch_cancel_all(void)
{
    // Switching to an actively consumed Panda Bulk-IN stream is intentional
    // routing, not congestion loss. Discard data that has not yet been handed
    // to NCM so the two transports do not compete for Full-Speed USB.
    udp_batch_len = 0;
    udp_batch_frame_count = 0;
    udp_pending_head = 0;
    udp_pending_tail = 0;
    udp_pending_count = 0;
}

static void udp_batch_commit(void)
{
    if (udp_batch_len == 0)
    {
        return;
    }

    if (udp_pending_count < UDP_PENDING_BATCH_SLOTS)
    {
        udp_pending_batch_t *pending = &udp_pending_batches[udp_pending_tail];
        pending->len = udp_batch_len;
        pending->frame_count = udp_batch_frame_count;
        memcpy(pending->data, udp_batch_buf, udp_batch_len);
        udp_pending_tail = (uint8_t)((udp_pending_tail + 1u) % UDP_PENDING_BATCH_SLOTS);
        udp_pending_count++;
    }
    else
    {
        // Preserve the older queued stream in order; only a sustained USB
        // outage long enough to fill the bounded queue drops a new batch.
        udp_batch_stats.frames_dropped += udp_batch_frame_count;
        udp_batch_stats.bytes_dropped += udp_batch_len;
    }
    udp_batch_len = 0;
    udp_batch_frame_count = 0;
}

static void udp_batch_service_pending(void)
{
    uint32_t budget = UDP_PENDING_SEND_BUDGET;

    while (udp_pending_count > 0 && budget-- > 0)
    {
        udp_pending_batch_t *pending = &udp_pending_batches[udp_pending_head];
        udp_batch_stats.flush_attempts++;
        udp_batch_stats.bytes_attempted += pending->len;
        if (!udp_server_send(pending->data, pending->len))
        {
            // NCM has no free IN NTB right now. Keep this exact batch at the
            // head and retry after TinyUSB has serviced the next USB event.
            udp_batch_stats.flush_failures++;
            break;
        }

        udp_batch_stats.flush_ok++;
        udp_pending_head = (uint8_t)((udp_pending_head + 1u) % UDP_PENDING_BATCH_SLOTS);
        udp_pending_count--;
    }
}

// Bitmask-to-decimal source byte, matching panda_usb.c source_decimal
// (FR1=bit3, FR2=bit2, FR3=bit1, FR4=bit0). This is the combined source ID used by
// the USB pandad flexray path (13 = FR1+FR3 EPS, 14 = FR1+FR4 common,
// 24 = FR2+FR4 vehicle).
static uint8_t source_decimal_lookup(uint8_t mask)
{
    static const uint8_t source_decimal[16] = {
        0, 4, 3, 34, 2, 24, 23, 234, 1, 14, 13, 134, 12, 124, 123, 0
    };
    return source_decimal[mask & 0x0F];
}

// Append one USB-pandad FlexRay record:
// [u8 source][5B header][payload][3B CRC].
static void udp_batch_push_frame(const uint8_t *header, uint16_t frame_len, uint8_t source_mask)
{
    uint16_t record_len = (uint16_t)(frame_len + 1u);
    if (frame_len > FRAME_BUF_SIZE_BYTES || record_len > UDP_BATCH_BUF_SIZE)
    {
        udp_batch_stats.frames_rejected++;
        return;
    }
    if (udp_batch_len == 0)
    {
        udp_batch_deadline = make_timeout_time_ms(UDP_BATCH_FLUSH_MS);
    }
    if ((uint16_t)(udp_batch_len + record_len) > UDP_BATCH_BUF_SIZE)
    {
        udp_batch_commit();
        udp_batch_deadline = make_timeout_time_ms(UDP_BATCH_FLUSH_MS);
    }
    udp_batch_buf[udp_batch_len++] = source_decimal_lookup(source_mask);
    memcpy(udp_batch_buf + udp_batch_len, header, frame_len);
    udp_batch_len = (uint16_t)(udp_batch_len + frame_len);
    udp_batch_frame_count++;
    udp_batch_stats.frames_pushed++;
}

static void ncm_udp_recv_handler(const uint8_t *data, uint16_t len,
                                 const ip_addr_t *addr, uint16_t port)
{
    static const char query[] = PROFILE_STATS_QUERY;

    if (len == sizeof(query) - 1u && memcmp(data, query, sizeof(query) - 1u) == 0)
    {
        udp_server_stats_t udp_stats;
        usb_netif_stats_t ncm_stats;
        profile_stats_wire_t snapshot = {
            .magic = PROFILE_STATS_MAGIC,
            .version = PROFILE_STATS_VERSION,
            .size = sizeof(profile_stats_wire_t),
            .uptime_ms = to_ms_since_boot(get_absolute_time()),
            .total_notif = stats.total_notif,
            .seq_gap = stats.seq_gap,
            .len_ok = stats.len_ok,
            .len_mismatch = stats.len_mismatch,
            .overflow_len = stats.overflow_len,
            .zero_len = stats.zero_len,
            .parse_fail = stats.parse_fail,
            .valid = stats.valid,
            .notify_dropped = notify_queue_dropped(),
            .batch = udp_batch_stats,
        };
        udp_server_get_stats(&udp_stats);
        usb_netif_get_stats(&ncm_stats);
        snapshot.udp = udp_stats;
        snapshot.ncm = ncm_stats;
        (void)udp_server_sendto(addr, port, &snapshot, sizeof(snapshot));
        return;
    }

    printf("UDP: recv %u bytes from %s:%u\n",
           (unsigned)len, ip4addr_ntoa(ip_2_ip4(addr)), (unsigned)port);
}

void core1_entry(void)
{
#if !FLEXRAY_FRAME_GEN
    setup_stream_fr34(pio0,
                      RXD_FR_3_PIN, TXEN_FR_4_PIN,
                      RXD_FR_4_PIN, TXEN_FR_3_PIN);
#endif

    setup_stream(pio1,
                 RXD_FR_1_PIN, TXEN_FR_2_PIN,
                 RXD_FR_2_PIN, TXEN_FR_1_PIN);
#if FLEXRAY_FRAME_GEN
    flexray_frame_gen_init();
#endif
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
#if FLEXRAY_FRAME_GEN
    // GPIO16 carries the inducer; keep both unused transceivers disabled.
    const uint unused_txen_pins[] = {TXEN_FR_3_PIN, TXEN_FR_4_PIN};
    for (uint i = 0; i < count_of(unused_txen_pins); i++) {
        gpio_init(unused_txen_pins[i]);
        gpio_put(unused_txen_pins[i], 1);
        gpio_set_dir(unused_txen_pins[i], GPIO_OUT);
    }
#else
    gpio_pull_up(TXEN_FR_3_PIN);
    gpio_pull_up(TXEN_FR_4_PIN);
#endif

    gpio_init(RXD_FR_1_PIN);
    gpio_set_dir(RXD_FR_1_PIN, GPIO_IN);
    gpio_init(RXD_FR_2_PIN);
    gpio_set_dir(RXD_FR_2_PIN, GPIO_IN);

#if !FLEXRAY_FRAME_GEN
    gpio_init(RXD_FR_3_PIN);
    gpio_set_dir(RXD_FR_3_PIN, GPIO_IN);
    gpio_init(RXD_FR_4_PIN);
    gpio_set_dir(RXD_FR_4_PIN, GPIO_IN);
#endif

    gpio_pull_up(RXD_FR_1_PIN);
    gpio_pull_up(RXD_FR_2_PIN);
#if !FLEXRAY_FRAME_GEN
    gpio_pull_up(RXD_FR_3_PIN);
    gpio_pull_up(RXD_FR_4_PIN);
#endif

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

    bool clock_configured = set_sys_clock_khz(150000, true);
    stdio_init_all();
    printf("static_used=%lu B\n", (unsigned long)((uintptr_t)&__end__ - (uintptr_t)SRAM_BASE));
    print_ram_usage();
    // Initialize Panda USB interface
    panda_usb_init();
    // Initialize cross-core notification queue before starting streams
    notify_queue_init();

    // Initialize lwIP + the USB NCM netif + the UDP server
    lwip_init();
    IP4_ADDR(&ncm_ip, 192, 168, 7, 1);
    IP4_ADDR(&ncm_netmask, 255, 255, 255, 0);
    IP4_ADDR(&ncm_gw, 0, 0, 0, 0);
    usb_netif_init(&ncm_netif, &ncm_ip, &ncm_netmask, &ncm_gw);
    udp_server_init();
    udp_server_set_recv_handler(ncm_udp_recv_handler);
    udp_inject_server_init();
    if (dhserv_init(&dhcp_config) != ERR_OK)
    {
        printf("Warning: DHCP server init failed\n");
    }
    printf("USB NCM netif up, DHCP + UDP server on port %d + inject on %d\n",
           UDP_SERVER_PORT, UDP_INJECT_PORT);

    // --- Keep the RP2350 system clock at 150 MHz ---
    // The FlexRay PIO programs run with clkdiv = 1 and use 15 cycles per bit.
    if (!clock_configured)
    {
        printf("Warning: Failed to set system clock, using default\n");
    }
    else
    {
        printf("System clock set to 150MHz\n");
    }

    print_pin_assignments();

    printf("Actual system clock: %lu Hz\n", clock_get_hz(clk_sys));
    printf("\n--- FlexRay Continuous Streaming Bridge (Forwarder Mode) ---\n");

#if !FLEXRAY_FRAME_GEN
    multicore_launch_core1(core1_entry);
    sleep_ms(500);
#endif


    setup_forwarder_with_injector(pio2,
                                  RXD_FR_1_PIN, TXD_FR_2_PIN,
                                  RXD_FR_2_PIN, TXD_FR_1_PIN,
                                  RXD_FR_3_PIN, TXD_FR_4_PIN,
                                  RXD_FR_4_PIN, TXD_FR_3_PIN);
#if FLEXRAY_FRAME_GEN
    // Build on core1 requires the forwarder program and DMA to be ready.
    multicore_launch_core1(core1_entry);
    sleep_ms(500);
#endif

    uint8_t temp_buffer[MAX_FRAME_BUF_SIZE_BYTES];

	absolute_time_t next_led_toggle_time = make_timeout_time_ms(500);
	bool led_on = false;

    while (true)
    {
        panda_usb_task();
        usb_netif_poll();
        sys_check_timeouts();
        bool vendor_stream_active = panda_usb_vendor_stream_active();
        if (vendor_stream_active &&
            (udp_batch_len > 0 || udp_pending_count > 0))
        {
            udp_batch_cancel_all();
        }
        if (udp_batch_len > 0 && time_reached(udp_batch_deadline))
        {
            udp_batch_commit();
        }
		udp_batch_service_pending();
		if (time_reached(next_led_toggle_time))
		{
			next_led_toggle_time = make_timeout_time_ms(500);
			led_on = !led_on;
			gpio_put(LED_PIN, led_on);
		}
        // Consume frame-end notifications from core1 (FR1/FR2 only).
        static uint16_t last_end_idx_fr1 = 0;
        static uint16_t last_end_idx_fr2 = 0;
        static uint32_t last_seq = 0;

        uint32_t encoded;
        if (!notify_queue_pop(&encoded))
        {
            // Do not sleep while a USB batch is waiting for a free NCM NTB;
            // the next pass services TinyUSB and retries it immediately.
            if (udp_pending_count > 0)
            {
                continue;
            }
            panda_usb_task();
            // Service host transports regularly while FlexRay is quiet.
            (void)best_effort_wfe_or_timeout(make_timeout_time_us(1000));
            continue;
        }
        uint32_t notify_budget = NOTIFY_PROCESS_BUDGET;
        do {
            notify_info_t info; notify_decode(encoded, &info);

            stats.total_notif++;
            if (stats.total_notif > 1 && ((info.seq - last_seq) & 0x3FFF) != 1) stats.seq_gap++;
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

#if !FLEXRAY_FRAME_GEN
                    uint8_t demuxed = info.fr34_source;
                    if (demuxed != FROM_UNKNOWN) {
                        frame.source |= demuxed;
                        if (demuxed & FROM_FR3) stats.source_fr3++;
                        if (demuxed & FROM_FR4) stats.source_fr4++;
                    }
#endif
                    try_cache_last_target_frame(frame.frame_id, frame.cycle_count, expected_len, header);
                    panda_flexray_fifo_push(&frame);

                    // Micro-batch raw FlexRay frames for NCM efficiency. A
                    // bounded retry queue absorbs transient USB backpressure.
                    if (udp_server_has_peer() && !vendor_stream_active)
                    {
                        udp_batch_push_frame(header, expected_len, frame.source);
                    }
                }

                pos = (uint16_t)(pos + expected_len);
            }

            if (info.is_fr2) last_end_idx_fr2 = info.end_idx;
            else last_end_idx_fr1 = info.end_idx;
        } while (--notify_budget > 0 && notify_queue_pop(&encoded));
    }

    return 0;
}
