#include "flexray_frame.h"
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

static uint16_t calculate_flexray_header_crc(const flexray_frame_t *frame)
{
    uint32_t data_word = 0;
    data_word |= (uint32_t)frame->sync_frame_indicator << 19;
    data_word |= (uint32_t)frame->startup_frame_indicator << 18;
    data_word |= (uint32_t)frame->frame_id << 7;
    data_word |= (uint32_t)frame->payload_length_words;

    uint16_t crc = 0x1A;
    const uint16_t poly = 0x385;

    for (int i = 19; i >= 0; --i)
    {
        bool data_bit = (data_word >> i) & 1;
        bool crc_msb = (crc >> 10) & 1;

        crc <<= 1;
        if (data_bit ^ crc_msb)
        {
            crc ^= poly;
        }
    }
    return crc & 0x7FF;
}

static uint32_t calculate_flexray_payload_crc_bitwise(const uint32_t *raw_buffer, const flexray_frame_t *frame)
{
    size_t payload_bits = frame->payload_length_words * 16;
    size_t total_bits_to_crc = 40 + payload_bits;

    uint32_t crc = 0xFEDCBA;
    const uint32_t poly = 0x5D6DCB;

    for (int i = 0; i < total_bits_to_crc; ++i)
    {
        bool data_bit = get_bits_msb(raw_buffer, i, 1);

        bool crc_msb = (crc >> 23) & 1;
        crc <<= 1;
        if (data_bit ^ crc_msb)
        {
            crc ^= poly;
        }
    }
    return crc & 0xFFFFFF;
}

static bool check_header_crc(flexray_frame_t *frame)
{
    uint16_t calculated_crc = calculate_flexray_header_crc(frame);
    return calculated_crc == frame->header_crc;
}

static bool check_payload_crc(flexray_frame_t *frame, const uint32_t *raw_buffer)
{
    if (frame->payload_length_words == 0)
    {
        return frame->payload_crc == 0;
    }
    uint32_t calculated_crc = calculate_flexray_payload_crc_bitwise(raw_buffer, frame);
    return calculated_crc == frame->payload_crc;
}

uint32_t payload_crc_precompute(uint32_t *override_payload, uint32_t *bit_length)
{
    // 
}

void parse_frame(const uint32_t *raw_buffer, flexray_frame_t *parsed_frame)
{
    int current_bit = 0;

    parsed_frame->reserved_bit = get_bits_msb(raw_buffer, current_bit, 1);
    current_bit += 1;
    parsed_frame->payload_preamble_indicator = get_bits_msb(raw_buffer, current_bit, 1);
    current_bit += 1;
    parsed_frame->null_frame_indicator = get_bits_msb(raw_buffer, current_bit, 1);
    current_bit += 1;
    parsed_frame->sync_frame_indicator = get_bits_msb(raw_buffer, current_bit, 1);
    current_bit += 1;
    parsed_frame->startup_frame_indicator = get_bits_msb(raw_buffer, current_bit, 1);
    current_bit += 1;
    parsed_frame->frame_id = get_bits_msb(raw_buffer, current_bit, 11);
    current_bit += 11;
    parsed_frame->payload_length_words = get_bits_msb(raw_buffer, current_bit, 7);
    current_bit += 7;
    parsed_frame->header_crc = get_bits_msb(raw_buffer, current_bit, 11);
    current_bit += 11;
    parsed_frame->cycle_count = get_bits_msb(raw_buffer, current_bit, 6);
    current_bit += 6;

    size_t payload_size_bytes = parsed_frame->payload_length_words * 2;
    if (payload_size_bytes > sizeof(parsed_frame->payload))
    {
        payload_size_bytes = sizeof(parsed_frame->payload);
    }
    for (size_t i = 0; i < payload_size_bytes; ++i)
    {
        parsed_frame->payload[i] = get_bits_msb(raw_buffer, current_bit, 8);
        current_bit += 8;
    }

    if ((current_bit + 24) <= (FRAME_BUF_SIZE_WORDS * 32))
    {
        parsed_frame->payload_crc = get_bits_msb(raw_buffer, current_bit, 24);
    }
    else
    {
        parsed_frame->payload_crc = 0;
    }
    if (raw_buffer[FRAME_BUF_SIZE_WORDS - 1] == 0x55555555)
    {
        parsed_frame->source = FROM_ECU;
    }
    else if (raw_buffer[FRAME_BUF_SIZE_WORDS - 1] == 0xAAAAAAAA)
    {
        parsed_frame->source = FROM_VEHICLE;
    }
    else
    {
        parsed_frame->source = FROM_UNKNOWN;
    }
}

void parse_frame_fast(const uint32_t *raw_buffer, flexray_frame_t *parsed_frame)
{
    // Cast raw buffer to bitfield union for direct access
    const flexray_raw_frame_t *raw_frame = (const flexray_raw_frame_t *)raw_buffer;
    
    // Extract header fields directly from bitfields
    parsed_frame->reserved_bit = raw_frame->fields.reserved_bit;
    parsed_frame->payload_preamble_indicator = raw_frame->fields.payload_preamble_indicator;
    parsed_frame->null_frame_indicator = raw_frame->fields.null_frame_indicator;
    parsed_frame->sync_frame_indicator = raw_frame->fields.sync_frame_indicator;
    parsed_frame->startup_frame_indicator = raw_frame->fields.startup_frame_indicator;
    parsed_frame->frame_id = raw_frame->fields.frame_id;
    parsed_frame->payload_length_words = raw_frame->fields.payload_length_words;
    
    // Reconstruct 11-bit header CRC from split fields
    // header_crc_low contains bits 23-31 (9 bits, the first part)
    // header_crc_high contains bits 32-33 (2 bits, the second part)
    // Correct reconstruction: (first_part << 2) | second_part
    parsed_frame->header_crc = (raw_frame->fields.header_crc_low << 2) | 
                              raw_frame->fields.header_crc_high;
    
    parsed_frame->cycle_count = raw_frame->fields.cycle_count;
    
    // Extract payload
    size_t payload_size_bytes = parsed_frame->payload_length_words * 2;
    if (payload_size_bytes > sizeof(parsed_frame->payload))
    {
        payload_size_bytes = sizeof(parsed_frame->payload);
    }

    memset(parsed_frame->payload, 0x00, sizeof(parsed_frame->payload));
    uint8_t *payload_ptr = parsed_frame->payload;
    const uint8_t *buffer_bytes = (const uint8_t *)raw_buffer;
    size_t bytes_copied = 0;

    // The header is 40 bits (5 bytes). The payload starts at the 6th byte of the raw buffer.
    // The first 3 payload bytes come from raw_buffer[1].
    // raw_buffer[1] bytes (LE): [b0, b1, b2, b3] -> MSB is b3 (header)
    // payload[0] = b2, payload[1] = b1, payload[2] = b0
    if (bytes_copied < payload_size_bytes) {
        payload_ptr[bytes_copied++] = buffer_bytes[6]; // byte 2 of raw_buffer[1]
        payload_ptr[bytes_copied++] = buffer_bytes[5]; // byte 1 of raw_buffer[1]
    }

    if (bytes_copied < payload_size_bytes) {
        payload_ptr[bytes_copied++] = buffer_bytes[4]; // byte 0 of raw_buffer[1]
    }

    // Remaining payload comes from raw_buffer[2] onwards.
    // Each 32-bit word needs to be byte-swapped.
    for (size_t i = 2; bytes_copied < payload_size_bytes; i++) {
        uint32_t bswapped_word = __builtin_bswap32(raw_buffer[i]);
        size_t bytes_to_copy = 4;
        if (bytes_copied + bytes_to_copy > payload_size_bytes) {
            bytes_to_copy = payload_size_bytes - bytes_copied;
        }
        memcpy(payload_ptr + bytes_copied, &bswapped_word, bytes_to_copy);
        bytes_copied += bytes_to_copy;
    }

    // Extract payload CRC (starts after payload)
    int payload_crc_bit = 40 + (parsed_frame->payload_length_words * 2 * 8);
    if ((payload_crc_bit + 24) <= (FRAME_BUF_SIZE_WORDS * 32))
    {
        parsed_frame->payload_crc = get_bits_msb(raw_buffer, payload_crc_bit, 24);
    }
    else
    {
        parsed_frame->payload_crc = 0;
    }
    
    // Determine source
    if (raw_buffer[FRAME_BUF_SIZE_WORDS - 1] == 0x55555555)
    {
        parsed_frame->source = FROM_ECU;
    }
    else if (raw_buffer[FRAME_BUF_SIZE_WORDS - 1] == 0xAAAAAAAA)
    {
        parsed_frame->source = FROM_VEHICLE;
    }
    else
    {
        parsed_frame->source = FROM_UNKNOWN;
    }
}

void print_frame(flexray_frame_t *frame)
{
    printf("%d,%d,%02X,%d,", frame->frame_id, frame->payload_length_words, frame->header_crc, frame->cycle_count);
    for (int i = 0; i < frame->payload_length_words * 2; i++)
    {
        printf("%02X", frame->payload[i]);
    }
    printf(",%02X,%s\n", frame->payload_crc, frame->source == FROM_ECU ? "ECU" : frame->source == FROM_VEHICLE ? "VEHICLE" : "UNKNOWN");
}

bool is_valid_frame(flexray_frame_t *frame, const uint32_t *raw_buffer)
{
    if (!frame)
        return false;
    if (frame->frame_id >= 2048)
        return false;

    if (frame->payload_length_words > 127)
        return false;

    if (!check_header_crc(frame))
    {
        return false;
    }

    if (frame->null_frame_indicator)
    {
        return true;
    }

    return check_payload_crc(frame, raw_buffer);
}
