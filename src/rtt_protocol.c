#include "rtt_protocol.h"

#include <stddef.h>

static uint32_t read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0]
         | ((uint32_t)data[1] << 8)
         | ((uint32_t)data[2] << 16)
         | ((uint32_t)data[3] << 24);
}

static uint64_t read_u64_le(const uint8_t *data)
{
    uint64_t value = 0;
    for (uint32_t index = 0; index < 8u; index++) {
        value |= (uint64_t)data[index] << (index * 8u);
    }
    return value;
}

static void write_u32_le(uint8_t *data, uint32_t value)
{
    for (uint32_t index = 0; index < 4u; index++) {
        data[index] = (uint8_t)(value >> (index * 8u));
    }
}

static void write_u64_le(uint8_t *data, uint64_t value)
{
    for (uint32_t index = 0; index < 8u; index++) {
        data[index] = (uint8_t)(value >> (index * 8u));
    }
}

bool rtt_decode_request(const uint8_t *data, uint16_t length,
                        uint32_t *sequence, uint64_t *host_send_time_us)
{
    if (data == NULL || length != RTT_REQUEST_LENGTH ||
        data[0] != RTT_REQUEST_OP || data[1] != RTT_VERSION) {
        return false;
    }

    if (sequence != NULL) {
        *sequence = read_u32_le(&data[2]);
    }
    if (host_send_time_us != NULL) {
        *host_send_time_us = read_u64_le(&data[6]);
    }
    return true;
}

void rtt_build_response(uint8_t response[RTT_RESPONSE_LENGTH], uint32_t sequence,
                        uint64_t host_send_time_us,
                        uint64_t device_receive_time_us,
                        uint64_t device_send_time_us)
{
    response[0] = RTT_RESPONSE_OP;
    response[1] = RTT_VERSION;
    write_u32_le(&response[2], sequence);
    write_u64_le(&response[6], host_send_time_us);
    write_u64_le(&response[14], device_receive_time_us);
    write_u64_le(&response[22], device_send_time_us);
}
