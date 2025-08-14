#ifndef FLEXRAY_BSS_STREAMER_H
#define FLEXRAY_BSS_STREAMER_H

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "flexray_frame.h"

#define STREAMER_SM_ECU 0
#define STREAMER_SM_VEHICLE 1

// --- Global State ---
extern uint dma_data_from_ecu_chan;
extern uint dma_data_from_vehicle_chan;

// Ring-ready buffers (defined in flexray_bss_streamer.c)
extern volatile uint8_t ecu_ring_buffer[];
extern volatile uint8_t vehicle_ring_buffer[];

// Ring sizes for consumers (must match definitions in .c)
#define ECU_RING_SIZE_BYTES   (1u << 12)
#define VEH_RING_SIZE_BYTES   (1u << 12)
#define ECU_RING_MASK         (ECU_RING_SIZE_BYTES - 1)
#define VEH_RING_MASK         (VEH_RING_SIZE_BYTES - 1)

// Address table for automatic buffer switching
extern volatile void *buffer_addresses[2];

// Debug counter removed; using multicore FIFO notifications

// --- Function Prototypes ---
void streamer_irq0_handler(void);
void setup_stream(PIO pio,
                  uint rx_pin_from_ecu, uint tx_en_pin_to_vehicle,
                  uint rx_pin_from_vehicle, uint tx_en_pin_to_ecu);

// --- Cross-core notification ring (single producer on core1 ISR, single consumer on core0) ---
// Encoded format: [31]=source(1=VEH), [30:12]=seq(19 bits), [11:0]=ring index
bool notify_queue_pop(uint32_t *encoded);
void notify_queue_init(void);
uint32_t notify_queue_dropped(void);

// Decoded notification info
typedef struct {
    bool is_vehicle;    // true if vehicle source, false if ECU
    uint32_t seq;       // 19-bit sequence
    uint16_t end_idx;   // 12-bit ring index (end position)
} notify_info_t;

// Decode encoded notification into structured fields
static inline void notify_decode(uint32_t encoded, notify_info_t *out)
{
    out->is_vehicle = (encoded >> 31) & 0x1;
    out->seq = (encoded >> 12) & 0x7FFFF;
    out->end_idx = (uint16_t)(encoded & 0x0FFF);
}

static inline uint32_t notify_encode(bool is_vehicle, uint32_t seq, uint16_t end_idx)
{
    return ((uint32_t)is_vehicle << 31) | (seq << 12) | (end_idx & 0x0FFF);
}

#endif // FLEXRAY_BSS_STREAMER_H 