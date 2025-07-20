#ifndef FLEXRAY_FRAME_H
#define FLEXRAY_FRAME_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define FLEXRAY_FIFO_SIZE 32

// #define FRAME_BITS_TO_CAPTURE 2112
#define FRAME_BITS_TO_CAPTURE 2048
#define FRAME_BUF_SIZE_WORDS (FRAME_BITS_TO_CAPTURE / 32) * 2

#define FROM_ECU 0
#define FROM_VEHICLE 1
#define FROM_UNKNOWN 0xff

// Bitfield union for direct raw buffer parsing
// Note: This matches MSB-first bit ordering used by get_bits_msb function
typedef union {
    uint32_t raw_words[FRAME_BUF_SIZE_WORDS];
    struct {
        // First word (MSB-first: bit 0 = MSB, bit 31 = LSB)
        uint32_t header_crc_low : 9;             // bits 23-31 (MSB 0-8)
        uint32_t payload_length_words : 7;       // bits 16-22 (MSB 9-15)
        uint32_t frame_id : 11;                  // bits 5-15 (MSB 16-26)
        uint32_t startup_frame_indicator : 1;    // bit 4 (MSB 27)
        uint32_t sync_frame_indicator : 1;       // bit 3 (MSB 28)
        uint32_t null_frame_indicator : 1;       // bit 2 (MSB 29)
        uint32_t payload_preamble_indicator : 1; // bit 1 (MSB 30)
        uint32_t reserved_bit : 1;               // bit 0 (MSB 31, LSB)
        
        // Second word (bits 32-63 in MSB ordering)
        uint32_t payload_start : 24;             // bits 40-63 (MSB 0-23)
        uint32_t cycle_count : 6;                // bits 34-39 (MSB 24-29)
        uint32_t header_crc_high : 2;            // bits 32-33 (MSB 30-31)
        
        // Remaining words for payload and CRC
        uint32_t remaining_data[FRAME_BUF_SIZE_WORDS - 2];
    } __attribute__((packed)) fields;
} flexray_raw_frame_t;

// FlexRay frame structure definition based on specification
typedef struct
{
    // Header - 40 bits total
    bool startup_frame_indicator;    // 1 bit
    bool sync_frame_indicator;       // 1 bit
    bool null_frame_indicator;       // 1 bit
    bool payload_preamble_indicator; // 1 bit
    bool reserved_bit;               // 1 bit
    uint16_t frame_id;               // 11 bits
    uint8_t payload_length_words;    // 7 bits (number of 16-bit words)
    uint16_t header_crc;             // 11 bits
    uint8_t cycle_count;             // 6 bits

    // Payload - 0 to 254 bytes
    uint8_t payload[254];

    // Trailer - 24 bits
    uint32_t payload_crc; // 24 bits
    uint8_t source; // 1 bit
} flexray_frame_t;

void parse_frame(const uint32_t *raw_buffer, flexray_frame_t *parsed_frame);
void parse_frame_fast(const uint32_t *raw_buffer, flexray_frame_t *parsed_frame);
void extract_payload_optimized(const uint32_t *raw_buffer, uint8_t *payload, size_t payload_bytes);
void test_bitfield_parsing(void);
void print_frame(flexray_frame_t *frame);
bool is_valid_frame(flexray_frame_t *frame, const uint32_t *raw_buffer);

#endif // FLEXRAY_FRAME_H
