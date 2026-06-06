#ifndef FLEXRAY_INJECTOR_H
#define FLEXRAY_INJECTOR_H

#include <stdint.h>
#include <stdbool.h>

// Cache a frame's raw bytes (header+payload+CRC) when rules match.
void try_cache_last_target_frame(uint16_t frame_id, uint8_t cycle_count, uint16_t frame_length, uint8_t *captured_bytes);

// On receiving the previous frame header, prepare a pending injection.
// Returns true only when a frame was actually prepared for injection.
bool prepare_inject_frame(uint16_t frame_id, uint8_t cycle_count);

// On previous frame end, start the prepared injection if one is pending.
void inject_prepared_frame(void);

// Clear protection for the frame that just ended.
void injector_note_frame_end(void);

// Blocker arbitration: returns block direction bits that must not be suppressed
// because the injector is actively driving this frame.
uint8_t injector_block_exclusion_mask_for_frame(uint16_t frame_id, uint8_t cycle_count);

// Submit a host-provided replacement slice to be used on next matching injection.
bool injector_submit_override(uint16_t id, uint8_t base, uint16_t len, const uint8_t *bytes);

void injector_set_enabled(bool enabled);
bool injector_is_enabled(void);

#endif // FLEXRAY_INJECTOR_H
