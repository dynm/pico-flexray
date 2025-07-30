#ifndef FLEXRAY_FRAME_H
#define FLEXRAY_FRAME_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define FLEXRAY_FIFO_SIZE 32

#define MAX_FRAME_PAYLOAD_BYTES 64
#define FRAME_BUF_SIZE_BYTES 8+MAX_FRAME_PAYLOAD_BYTES

#define FROM_ECU 0
#define FROM_VEHICLE 1
#define FROM_UNKNOWN 0xff

// FlexRay frame structure definition based on specification
typedef struct
{
    // Header - 40 bits total
    bool reserved_bit;                  // 1 bit
    bool payload_preamble_indicator;    // 1 bit
    bool null_frame_indicator;          // 1 bit
    bool sync_frame_indicator;          // 1 bit
    bool startup_frame_indicator;       // 1 bit
    uint16_t frame_id;                  // 11 bits
    uint8_t payload_length_words;       // 7 bits (number of 16-bit words)
    uint16_t header_crc;                // 11 bits
    uint8_t cycle_count;                // 6 bits

    // Payload - 0 to MAX_FRAME_PAYLOAD_BYTES bytes
    uint8_t payload[MAX_FRAME_PAYLOAD_BYTES];

    // Trailer - 24 bits
    uint32_t payload_crc; // 24 bits
    uint8_t source; // from ecu or vehicle
} flexray_frame_t;

bool parse_frame(const uint8_t *raw_buffer, flexray_frame_t *parsed_frame);
void print_frame(flexray_frame_t *frame);
bool is_valid_frame(flexray_frame_t *frame, const uint8_t *raw_buffer);

#endif // FLEXRAY_FRAME_H
