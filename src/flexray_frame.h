#ifndef FLEXRAY_FRAME_H
#define FLEXRAY_FRAME_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define FLEXRAY_FIFO_SIZE 256

#define MAX_FRAME_PAYLOAD_BYTES 254
#define FRAME_BUF_SIZE_BYTES 8 + MAX_FRAME_PAYLOAD_BYTES
#define MAX_FRAME_BUF_SIZE_BYTES 264

#define FROM_ECU 0
#define FROM_VEHICLE 1
#define FROM_UNKNOWN 0xff

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

#endif // FLEXRAY_FRAME_H
