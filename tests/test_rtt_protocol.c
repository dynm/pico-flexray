#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "rtt_protocol.h"

int main(void)
{
    static const uint8_t request[RTT_REQUEST_LENGTH] = {
        RTT_REQUEST_OP, RTT_VERSION,
        0x78u, 0x56u, 0x34u, 0x12u,
        0x08u, 0x07u, 0x06u, 0x05u, 0x04u, 0x03u, 0x02u, 0x01u,
    };
    uint32_t sequence = 0u;
    uint64_t host_send_time_us = 0u;
    assert(rtt_decode_request(request, sizeof(request),
                              &sequence, &host_send_time_us));
    assert(sequence == 0x12345678u);
    assert(host_send_time_us == UINT64_C(0x0102030405060708));
    assert(!rtt_decode_request(request, sizeof(request) - 1u, NULL, NULL));

    uint8_t invalid[RTT_REQUEST_LENGTH];
    memcpy(invalid, request, sizeof(invalid));
    invalid[1]++;
    assert(!rtt_decode_request(invalid, sizeof(invalid), NULL, NULL));

    uint8_t response[RTT_RESPONSE_LENGTH];
    rtt_build_response(response, sequence, host_send_time_us,
                       UINT64_C(0x1112131415161718),
                       UINT64_C(0x2122232425262728));
    static const uint8_t expected[RTT_RESPONSE_LENGTH] = {
        RTT_RESPONSE_OP, RTT_VERSION,
        0x78u, 0x56u, 0x34u, 0x12u,
        0x08u, 0x07u, 0x06u, 0x05u, 0x04u, 0x03u, 0x02u, 0x01u,
        0x18u, 0x17u, 0x16u, 0x15u, 0x14u, 0x13u, 0x12u, 0x11u,
        0x28u, 0x27u, 0x26u, 0x25u, 0x24u, 0x23u, 0x22u, 0x21u,
    };
    assert(memcmp(response, expected, sizeof(expected)) == 0);

    puts("RTT protocol tests passed");
    return 0;
}
