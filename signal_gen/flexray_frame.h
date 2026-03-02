#ifndef FLEXRAY_FRAME_H
#define FLEXRAY_FRAME_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define MAX_FRAME_PAYLOAD_BYTES 254
#define FRAME_BUF_SIZE_BYTES 8 + MAX_FRAME_PAYLOAD_BYTES
#define MAX_FRAME_BUF_SIZE_BYTES 264

uint32_t calculate_flexray_frame_crc(const uint8_t *restrict p, const uint16_t len16);
uint16_t calculate_flexray_header_crc(const uint8_t *raw_buffer);

static inline void fix_flexray_frame_crc(uint8_t *restrict frame_bytes, const uint16_t total_len_bytes)
{
    uint32_t new_crc = calculate_flexray_frame_crc(frame_bytes, (uint16_t)(total_len_bytes - 3));
    frame_bytes[total_len_bytes - 3] = (uint8_t)(new_crc >> 16);
    frame_bytes[total_len_bytes - 2] = (uint8_t)(new_crc >> 8);
    frame_bytes[total_len_bytes - 1] = (uint8_t)(new_crc);
}

#endif // FLEXRAY_FRAME_H
