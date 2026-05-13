#ifndef FLEXRAY_SIGNAL_GEN_H
#define FLEXRAY_SIGNAL_GEN_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/pio.h"

#define SIGNAL_GEN_MAX_CHANNELS 4
#define SIGNAL_GEN_MAX_SLOTS    64
#define FLEXRAY_CYCLE_PERIOD_US 5000  // 200 Hz = 5 ms

typedef struct {
    uint tx_pin;
    uint txen_pin;
} fr_channel_pins_t;

typedef struct {
    uint32_t txstall_count;
    uint32_t late_buffer_count;
    uint32_t completed_cycles;
    uint32_t handled_cycles;
    uint32_t last_render_us;
    uint32_t max_render_us;
} signal_gen_diag_t;

bool signal_gen_init(PIO pio, const fr_channel_pins_t *channels, uint num_channels);
bool signal_gen_set_channel_pins(uint channel, uint tx_pin, uint txen_pin);
bool signal_gen_set_channel_pin_map(const fr_channel_pins_t *pins, uint num_channels);
bool signal_gen_pio_test(uint channel, bool enabled);
uint32_t signal_gen_stall_count(void);
void signal_gen_diag(signal_gen_diag_t *diag);

// Configure a shared frame slot with a channel output mask.
// channel_mask: bitmask of channels to output on (bit 0 = ch0, bit 1 = ch1, ...).
// Future cycles are rendered into per-channel ping/pong buffers. Unrouted
// slots are filled with recessive bits, and a chained control DMA rearms the
// data DMA at each 5 ms cycle boundary.
bool signal_gen_set_slot(uint slot, uint8_t channel_mask, uint16_t frame_id,
                         uint8_t indicators, const uint8_t *payload,
                         uint16_t payload_len);

void signal_gen_clear_slot(uint slot);
void signal_gen_clear_all_slots(void);

bool signal_gen_update_slot_payload(uint slot,
                                    const uint8_t *payload, uint16_t payload_len);

// Global 200 Hz start / stop. Cycle count auto-rolls 0–63.
bool signal_gen_can_start(void);
bool signal_gen_start(void);
void signal_gen_stop(void);
bool signal_gen_is_running(void);

// Call from main loop — drives the 200 Hz periodic transmissions.
// Returns bitmask of channels that transmitted (bit 0 = ch0, etc.).
uint8_t signal_gen_tick(void);

#endif // FLEXRAY_SIGNAL_GEN_H
