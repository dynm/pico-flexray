#ifndef FLEXRAY_BSS_STREAMER_H
#define FLEXRAY_BSS_STREAMER_H

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "flexray_frame.h" // For FRAME_BUF_SIZE_WORDS etc.

// --- Global State ---
extern uint dma_data_from_ecu_chan;
extern uint dma_data_from_vehicle_chan;

// Ping-pong buffers
// These are defined in flexray_bss_streamer.c
extern volatile uint32_t capture_buffer_a[];
extern volatile uint32_t capture_buffer_b[];

// Address table for automatic buffer switching
extern volatile void *buffer_addresses[2];

// Current buffer index (for CPU to know which buffer was just filled)
extern volatile uint32_t current_buffer_index;

// Flag to indicate that a new buffer is ready for processing
extern volatile bool g_new_data_available;

// Debug counter for interrupt handler
extern volatile uint32_t irq_handler_call_count;

// --- Function Prototypes ---
void streamer_irq0_handler(void);
void setup_stream(PIO pio,
                  uint rx_pin_from_ecu, uint tx_en_pin_to_vehicle,
                  uint rx_pin_from_vehicle, uint tx_en_pin_to_ecu);

#endif // FLEXRAY_BSS_STREAMER_H 