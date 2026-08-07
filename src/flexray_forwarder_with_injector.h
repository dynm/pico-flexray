#ifndef FLEXRAY_FORWARDER_WITH_INJECTOR_H
#define FLEXRAY_FORWARDER_WITH_INJECTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/pio.h"

// Cache a frame's raw bytes (header+payload+CRC) when rules match
void try_cache_last_target_frame(uint16_t frame_id, uint8_t cycle_count, uint16_t frame_length, uint8_t *captured_bytes);

// Prepare a matching injection while the configured previous frame is still on the bus.
bool prepare_inject_frame(uint16_t frame_id, uint8_t cycle_count);

// Drop any stale preparation before processing a new header.
void discard_prepared_injection(void);

// Start the prepared injection when the trigger frame ends.
void inject_prepared_frame(void);

void setup_forwarder_with_injector(PIO pio,
    uint rx_pin_from_fr1, uint tx_pin_to_fr2,
    uint rx_pin_from_fr2, uint tx_pin_to_fr1,
    uint rx_pin_from_fr3, uint tx_pin_to_fr4,
    uint rx_pin_from_fr4, uint tx_pin_to_fr3);

// Submit a host-provided replacement slice to be used on next matching injection
// bytes must contain only the replacement payload slice; length must equal rule->replace_len
// The override applies when id matches a rule's target_id and (cycle_count & rule->cycle_mask) == rule->cycle_base
bool injector_submit_override(uint16_t id, uint8_t base, uint16_t len, const uint8_t *bytes);

// Enable/disable injection at runtime
void injector_set_enabled(bool enabled);
bool injector_is_enabled(void);


#endif // FLEXRAY_FORWARDER_WITH_INJECTOR_H
