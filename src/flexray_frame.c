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
}

void print_frame(flexray_frame_t *frame)
{
    printf("%d,%d,%02X,%d,", frame->frame_id, frame->payload_length_words, frame->header_crc, frame->cycle_count);
    for (int i = 0; i < frame->payload_length_words * 2; i++)
    {
        printf("%02X", frame->payload[i]);
    }
    printf(",%02X\n", frame->payload_crc);
}

bool is_valid_frame(flexray_frame_t *frame, const uint32_t *raw_buffer)
{
    if (!frame)
        return false;
    if (frame->frame_id >= TOTAL_FRAMES)
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
