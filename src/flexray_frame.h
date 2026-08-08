#ifndef FLEXRAY_FRAME_H
#define FLEXRAY_FRAME_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define FLEXRAY_FIFO_SIZE 256

#define MAX_FRAME_PAYLOAD_BYTES 254
#define FRAME_BUF_SIZE_BYTES 8 + MAX_FRAME_PAYLOAD_BYTES
#define MAX_FRAME_BUF_SIZE_BYTES 264

// 4-bit bitmask: each bit indicates the frame was seen on that source.
// e.g. FR1 + FR3 = 0b1010
#define FROM_FR1 (1u << 3)  // 0b1000
#define FROM_FR2 (1u << 2)  // 0b0100
#define FROM_FR3 (1u << 1)  // 0b0010
#define FROM_FR4 (1u << 0)  // 0b0001
#define FROM_UNKNOWN 0

// FlexRay frame structure definition based on specification
typedef struct
{
    uint32_t frame_crc; // 24 bits

    uint16_t frame_id;                  // 11 bits
    uint16_t header_crc;                // 11 bits
    
    uint8_t indicators;                 // 5 bit
    uint8_t payload_length_words;       // 7 bits (number of 16-bit words)
    uint8_t cycle_count;                // 6 bits
    uint8_t source; // from ecu or vehicle
    uint8_t payload[MAX_FRAME_PAYLOAD_BYTES];
} flexray_frame_t;

bool parse_frame(const uint8_t *raw_buffer, flexray_frame_t *parsed_frame);
// Fast-path parse from a contiguous slice without sentinel; caller supplies source and total length
bool parse_frame_from_slice(const uint8_t *raw_buffer, uint16_t slice_len, uint8_t source, flexray_frame_t *parsed_frame);
void print_frame(flexray_frame_t *frame);
bool is_valid_frame(flexray_frame_t *frame, const uint8_t *raw_buffer);
uint16_t calculate_flexray_header_crc(const uint8_t *raw_buffer);
uint32_t calculate_flexray_frame_crc(const uint8_t *restrict p, const uint16_t len16);
uint8_t calculate_autosar_e2e_crc8(const uint8_t *restrict p, const uint8_t init_value, const uint8_t len);
uint8_t calculate_autosar_e2e_crc8_0x2f(const uint8_t *restrict p, const uint8_t init_value, const uint8_t len);

// In-place update of the 24-bit payload CRC at the end of a frame slice.
// total_len_bytes is the length of header+payload+CRC (i.e., includes 3 CRC bytes).
static inline void fix_flexray_frame_crc(uint8_t *restrict frame_bytes, const uint16_t total_len_bytes)
{
    uint32_t new_crc = calculate_flexray_frame_crc(frame_bytes, (uint16_t)(total_len_bytes - 3));
    frame_bytes[total_len_bytes - 3] = (uint8_t)(new_crc >> 16);
    frame_bytes[total_len_bytes - 2] = (uint8_t)(new_crc >> 8);
    frame_bytes[total_len_bytes - 1] = (uint8_t)(new_crc);
}
#endif // FLEXRAY_FRAME_H
