#include "flexray_frame.h"
#include "flexray_crc_table.h"
#include <stdio.h>
#include <string.h>

static inline uint32_t get_bits_msb(const uint32_t *buffer, int start_bit_idx, int num_bits)
{
    if (num_bits == 0)
        return 0;
    if (num_bits > 32)
        return 0;

    int word_idx = start_bit_idx / 32;
    int bit_offset = start_bit_idx % 32;

    if (bit_offset + num_bits <= 32)
    {
        return (buffer[word_idx] << bit_offset) >> (32 - num_bits);
    }
    else
    {
        uint64_t two_words = ((uint64_t)buffer[word_idx] << 32) | buffer[word_idx + 1];
        return (uint32_t)((two_words << bit_offset) >> (64 - num_bits));
    }
}

static bool get_bit_from_byte_array(const uint8_t *buffer, int bit_index)
{
    int byte_index = bit_index / 8;
    int bit_offset = 7 - (bit_index % 8);
    return (buffer[byte_index] >> bit_offset) & 1;
}

static uint16_t calculate_flexray_header_crc(const uint8_t *raw_buffer, const flexray_frame_t *frame)
{
    uint32_t data_word = 0;
    data_word = (uint32_t)(raw_buffer[0] & 0b11111) << 16;
    data_word |= (uint32_t)raw_buffer[1] << 8;
    data_word |= (uint32_t)raw_buffer[2] << 0;
    data_word >>= 1;

    uint16_t crc = 0x1A;
    const uint16_t poly = 0x385;

    uint8_t byte0 = (data_word >> 12) & 0xFF; // bit19-12
    uint8_t index = ((crc >> 3) & 0xFF) ^ byte0;
    crc = ((crc << 8) & 0x7FF) ^ flexray_crc11_table[index];

    uint8_t byte1 = (data_word >> 4) & 0xFF; // bit11-4
    index = ((crc >> 3) & 0xFF) ^ byte1;
    crc = ((crc << 8) & 0x7FF) ^ flexray_crc11_table[index];

    // last nibble
    uint8_t last_bits = data_word & 0xF;
    uint8_t tbl_idx = ((crc >> 7) & 0xF) ^ last_bits;
    crc = ((crc << 4) & 0x7FF) ^ flexray_crc11_4bit_table[tbl_idx];

    // for (int i = 19; i >= 0; --i)
    // {
    //     bool data_bit = (data_word >> i) & 1;
    //     bool crc_msb = (crc >> 10) & 1;

    //     crc <<= 1;
    //     if (data_bit ^ crc_msb)
    //     {
    //         crc ^= poly;
    //     }
    // }
    return crc & 0x7FF;
}

static uint32_t calculate_flexray_frame_crc(const uint8_t *raw_buffer, const flexray_frame_t *frame)
{
    uint32_t crc = 0xFEDCBA;
    const uint32_t poly = 0x5D6DCB;

    const uint8_t *data_ptr = raw_buffer;
    size_t len = 5 + (frame->payload_length_words * 2);

    for (size_t i = 0; i < len; ++i)
    {
        uint8_t tbl_idx = ((crc >> 16) & 0xFF) ^ data_ptr[i];
        crc = (crc << 8) ^ flexray_crc24_table[tbl_idx];
    }
    return crc & 0xFFFFFF;
}

static bool check_header_crc(flexray_frame_t *frame, const uint8_t *raw_buffer)
{
    uint16_t calculated_crc = calculate_flexray_header_crc(raw_buffer, frame);
    return calculated_crc == frame->header_crc;
}

static bool check_frame_crc(flexray_frame_t *frame, const uint8_t *raw_buffer)
{
    if (frame->payload_length_words == 0)
    {
        return frame->frame_crc == 0;
    }
    uint32_t calculated_crc = calculate_flexray_frame_crc(raw_buffer, frame);
    return calculated_crc == frame->frame_crc;
}

bool parse_frame(const uint8_t *raw_buffer, flexray_frame_t *parsed_frame)
{
    if (!raw_buffer || !parsed_frame)
    {
        return false;
    }

    // --- Header Parsing (5 bytes) ---
    const uint8_t *header = raw_buffer;

    // Byte 0: Indicators and Frame ID MSBs
    parsed_frame->indicators = header[0] >> 3;

    // Byte 0 & 1: Assemble the 11-bit Frame ID (Big-Endian)
    // Frame ID High 3 bits from Byte 0, Low 8 bits are all of Byte 1
    parsed_frame->frame_id = ((uint16_t)(header[0] & 0x07) << 8) | header[1];
    parsed_frame->source = raw_buffer[FRAME_BUF_SIZE_BYTES - 1];

    // Byte 2: Payload Length and Header CRC MSB
    parsed_frame->payload_length_words = (header[2] >> 1) & 0x7F; // 7 bits
    if (parsed_frame->payload_length_words * 2 > MAX_FRAME_PAYLOAD_BYTES)
    {
        return false;
    }

    // Byte 2, 3, 4: Assemble the 11-bit Header CRC (Big-Endian)
    uint16_t crc_part1 = (uint16_t)(header[2] & 0x01) << 10; // Bit 10
    uint16_t crc_part2 = (uint16_t)header[3] << 2;           // Bits 9-2
    uint16_t crc_part3 = (header[4] >> 6) & 0x03;            // Bits 1-0
    parsed_frame->header_crc = crc_part1 | crc_part2 | crc_part3;

    // Byte 4: Assemble the 6-bit Cycle Count
    parsed_frame->cycle_count = header[4] & 0x3F;

    // --- Payload and CRC ---
    // Point to the payload data, which starts after the 5-byte header.
    memcpy(parsed_frame->payload, &raw_buffer[5], parsed_frame->payload_length_words * 2);

    // The 24-bit payload CRC is at the end of the payload.
    // Its position depends on the payload length.
    // Note: FlexRay payload length is in words (2 bytes).
    uint16_t payload_len_bytes = parsed_frame->payload_length_words * 2;
    if (5 + payload_len_bytes + 3 <= FRAME_BUF_SIZE_BYTES)
    {
        const uint8_t *crc_ptr = &raw_buffer[5 + payload_len_bytes];
        // Assemble 24-bit CRC (Big-Endian)
        parsed_frame->frame_crc = ((uint32_t)crc_ptr[0] << 16) |
                                    ((uint32_t)crc_ptr[1] << 8) |
                                    (uint32_t)crc_ptr[2];
    }
    return true;
}

void print_frame(flexray_frame_t *frame)
{
    printf("%d,%d,%02X,%d,", frame->frame_id, frame->payload_length_words, frame->header_crc, frame->cycle_count);
    for (int i = 0; i < frame->payload_length_words * 2; i++)
    {
        printf("%02X", frame->payload[i]);
    }
    printf(",%02X,%s\n", frame->frame_crc, frame->source == FROM_ECU ? "ECU" : frame->source == FROM_VEHICLE ? "VEHICLE"
                                                                                                               : "UNKNOWN");
}

bool is_valid_frame(flexray_frame_t *frame, const uint8_t *raw_buffer)
{
    if (!frame)
        return false;
    if (frame->frame_id >= 2048)
        return false;

    if (frame->payload_length_words > 127)
        return false;

    if (!check_header_crc(frame, raw_buffer))
    {
        return false;
    }

    return check_frame_crc(frame, raw_buffer);
}
