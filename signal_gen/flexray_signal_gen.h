#ifndef FLEXRAY_SIGNAL_GEN_H
#define FLEXRAY_SIGNAL_GEN_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/pio.h"

#define SIGNAL_GEN_MAX_CHANNELS 4
#define SIGNAL_GEN_MAX_SLOTS    4
#define FLEXRAY_CYCLE_PERIOD_US 5000  // 200 Hz = 5 ms

typedef struct {
    uint tx_pin;
    uint txen_pin;
} fr_channel_pins_t;

bool signal_gen_init(PIO pio, const fr_channel_pins_t *channels, uint num_channels);

// Configure a shared frame slot with a channel output mask.
// channel_mask: bitmask of channels to output on (bit 0 = ch0, bit 1 = ch1, ...).
// The frame is built once per cycle then DMA-kicked on all masked channels
// simultaneously, ensuring perfect alignment across channels.
bool signal_gen_set_slot(uint slot, uint8_t channel_mask, uint16_t frame_id,
                         uint8_t indicators, const uint8_t *payload,
                         uint16_t payload_len);

void signal_gen_clear_slot(uint slot);

bool signal_gen_update_slot_payload(uint slot,
                                    const uint8_t *payload, uint16_t payload_len);

// Global 200 Hz start / stop. Cycle count auto-rolls 0–63.
void signal_gen_start(void);
void signal_gen_stop(void);
bool signal_gen_is_running(void);

// Call from main loop — drives the 200 Hz periodic transmissions.
// Returns bitmask of channels that transmitted (bit 0 = ch0, etc.).
uint8_t signal_gen_tick(void);

#endif // FLEXRAY_SIGNAL_GEN_H
