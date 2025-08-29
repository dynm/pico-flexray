#ifndef FLEXRAY_INJECTOR_H
#define FLEXRAY_INJECTOR_H

#include <stdint.h>

// Cache a frame's raw bytes (header+payload+CRC) when rules match
void try_cache_last_target_frame(uint16_t frame_id, uint8_t cycle_count, uint16_t frame_length, uint8_t *captured_bytes);

// On receiving a frame, check triggers; if matched, mutate template and request injection
void try_to_inject_frame(uint16_t frame_id, uint8_t cycle_count);

#endif // FLEXRAY_INJECTOR_H


