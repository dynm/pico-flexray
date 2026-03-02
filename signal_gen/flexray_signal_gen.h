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

// Configure a frame slot. The slot becomes active immediately.
// If transmission is already running, the slot joins the next cycle.
bool signal_gen_set_slot(uint channel, uint slot, uint16_t frame_id,
                         uint8_t indicators, const uint8_t *payload,
                         uint16_t payload_len);

void signal_gen_clear_slot(uint channel, uint slot);

// Update payload for an active slot without clearing it.
bool signal_gen_update_slot_payload(uint channel, uint slot,
                                    const uint8_t *payload, uint16_t payload_len);

// Global 200 Hz start / stop. Cycle count auto-rolls 0–63.
void signal_gen_start(void);
void signal_gen_stop(void);
bool signal_gen_is_running(void);

// Call from main loop — drives the 200 Hz periodic transmissions.
void signal_gen_tick(void);

#endif // FLEXRAY_SIGNAL_GEN_H
