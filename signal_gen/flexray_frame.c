#include "flexray_frame.h"
#include "flexray_crc_table.h"

uint16_t calculate_flexray_header_crc(const uint8_t *raw_buffer)
{
    uint32_t data_word = 0;
    data_word = (uint32_t)(raw_buffer[0] & 0b11111) << 16;
    data_word |= (uint32_t)raw_buffer[1] << 8;
    data_word |= (uint32_t)raw_buffer[2] << 0;
    data_word >>= 1;

    uint16_t crc = 0x1A;

    uint8_t byte0 = (data_word >> 12) & 0xFF;
    uint8_t index = ((crc >> 3) & 0xFF) ^ byte0;
    crc = ((crc << 8) & 0x7FF) ^ flexray_crc11_table[index];

    uint8_t byte1 = (data_word >> 4) & 0xFF;
    index = ((crc >> 3) & 0xFF) ^ byte1;
    crc = ((crc << 8) & 0x7FF) ^ flexray_crc11_table[index];

    uint8_t last_bits = data_word & 0xF;
    uint8_t tbl_idx = ((crc >> 7) & 0xF) ^ last_bits;
    crc = ((crc << 4) & 0x7FF) ^ flexray_crc11_4bit_table[tbl_idx];

    return crc & 0x7FF;
}

uint32_t calculate_flexray_frame_crc(const uint8_t *restrict p, const uint16_t len16)
{
    uint32_t crc = 0xFEDCBA;

    uint32_t n = (uint32_t)len16;
    while (n >= 4) {
        uint8_t i0 = (uint8_t)((crc >> 16) ^ *p++); crc = (crc << 8) ^ flexray_crc24_table[i0];
        uint8_t i1 = (uint8_t)((crc >> 16) ^ *p++); crc = (crc << 8) ^ flexray_crc24_table[i1];
        uint8_t i2 = (uint8_t)((crc >> 16) ^ *p++); crc = (crc << 8) ^ flexray_crc24_table[i2];
        uint8_t i3 = (uint8_t)((crc >> 16) ^ *p++); crc = (crc << 8) ^ flexray_crc24_table[i3];
        n -= 4;
    }
    while (n--) {
        uint8_t idx = (uint8_t)((crc >> 16) ^ *p++);
        crc = (crc << 8) ^ flexray_crc24_table[idx];
    }
    return crc & 0xFFFFFF;
}
