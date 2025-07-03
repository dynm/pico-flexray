#ifndef FLEXRAY_FRAME_H
#define FLEXRAY_FRAME_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// This was in pico_flexray.c, moved here for module encapsulation.
#define FRAME_BITS_TO_CAPTURE 2048
#define FRAME_BUF_SIZE_WORDS (FRAME_BITS_TO_CAPTURE / 32)
#define TOTAL_FRAMES 512

// FlexRay frame structure definition based on specification
typedef struct {
    // Header - 40 bits total
    bool     startup_frame_indicator;     // 1 bit
    bool     sync_frame_indicator;        // 1 bit
    bool     null_frame_indicator;        // 1 bit
    bool     payload_preamble_indicator;  // 1 bit
    bool     reserved_bit;                // 1 bit
    uint16_t frame_id;                    // 11 bits
    uint8_t  payload_length_words;        // 7 bits (number of 16-bit words)
    uint16_t header_crc;                  // 11 bits
    uint8_t  cycle_count;                 // 6 bits

    // Payload - 0 to 254 bytes
    uint8_t  payload[254];

    // Trailer - 24 bits
    uint32_t payload_crc;                 // 24 bits
} flexray_frame_t;

void parse_frame(const uint32_t *raw_buffer, flexray_frame_t *parsed_frame);
void print_frame(flexray_frame_t *frame);
bool is_valid_frame(flexray_frame_t *frame, const uint32_t *raw_buffer);

#endif // FLEXRAY_FRAME_H
