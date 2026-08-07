#ifndef FLEXRAY_BSS_STREAMER_H
#define FLEXRAY_BSS_STREAMER_H

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "flexray_frame.h"


// --- Global State ---
extern uint dma_data_from_fr1_chan;
extern uint dma_data_from_fr2_chan;

// Ring-ready buffers for FR1/FR2 stream (defined in flexray_bss_streamer.c)
extern volatile uint8_t fr1_ring_buffer[];
extern volatile uint8_t fr2_ring_buffer[];

// Ring-ready buffers for FR3/FR4 stream (defined in flexray_bss_streamer.c)
extern volatile uint8_t fr3_ring_buffer[];
extern volatile uint8_t fr4_ring_buffer[];

// Ring sizes for FR1/FR2
#define FR1_RING_SIZE_BYTES   (1u << 12)
#define FR2_RING_SIZE_BYTES   (1u << 12)
#define FR1_RING_MASK         (FR1_RING_SIZE_BYTES - 1)
#define FR2_RING_MASK         (FR2_RING_SIZE_BYTES - 1)

// Ring sizes for FR3/FR4
#define FR3_RING_SIZE_BYTES   (1u << 12)
#define FR4_RING_SIZE_BYTES   (1u << 12)
#define FR3_RING_MASK         (FR3_RING_SIZE_BYTES - 1)
#define FR4_RING_MASK         (FR4_RING_SIZE_BYTES - 1)

// Address table for automatic buffer switching
extern volatile void *buffer_addresses[2];

// --- Function Prototypes ---
void streamer_header_irq_handler(void);
void streamer_frame_end_irq_handler(void);
void streamer_fr34_header_irq_handler(void);
void streamer_fr34_frame_end_irq_handler(void);

// Setup primary stream (FR1/FR2)
void setup_stream(PIO pio,
                  uint rx_pin_from_fr1, uint tx_en_pin_to_fr2,
                  uint rx_pin_from_fr2, uint tx_en_pin_to_fr1);

// Setup secondary stream (FR3/FR4) for source identification
void setup_stream_fr34(PIO pio,
                       uint rx_pin_from_fr3, uint tx_en_pin_to_fr4,
                       uint rx_pin_from_fr4, uint tx_en_pin_to_fr3);

// --- Cross-core notification ring (single producer on core1 ISR, single consumer on core0) ---
// Encoded format: [31]=is_fr2, [30:29]=fr34_source, [28:12]=seq(17 bits), [11:0]=ring index
bool notify_queue_pop(uint32_t *encoded);
void notify_queue_init(void);
uint32_t notify_queue_dropped(void);

// Decoded notification info
typedef struct {
    bool is_fr2;         // true if FR2 side, false if FR1 side
    uint8_t fr34_source; // FROM_UNKNOWN, FROM_FR3, or FROM_FR4 for this frame
    uint32_t seq;        // 17-bit sequence
    uint16_t end_idx;    // 12-bit ring index (end position)
} notify_info_t;

// Decode encoded notification into structured fields
static inline void notify_decode(uint32_t encoded, notify_info_t *out)
{
    out->is_fr2 = (encoded >> 31) & 0x1;
    out->fr34_source = (uint8_t)((encoded >> 29) & 0x3);
    out->seq = (encoded >> 12) & 0x1FFFF;
    out->end_idx = (uint16_t)(encoded & 0x0FFF);
}

static inline uint32_t notify_encode(bool is_fr2, uint8_t fr34_source, uint32_t seq, uint16_t end_idx)
{
    return ((uint32_t)is_fr2 << 31) |
           ((uint32_t)(fr34_source & 0x3u) << 29) |
           ((seq & 0x1FFFFu) << 12) |
           (end_idx & 0x0FFF);
}

#endif // FLEXRAY_BSS_STREAMER_H
