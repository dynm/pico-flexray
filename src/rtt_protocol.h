#ifndef RTT_PROTOCOL_H
#define RTT_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define RTT_REQUEST_OP 0x92u
#define RTT_RESPONSE_OP 0x93u
#define RTT_VERSION 1u

// UDP and USB Vendor Bulk use the same wire format:
// request  [op][version][u32 sequence LE][u64 host_send_time_us LE]
// response [op][version][u32 sequence LE][u64 host_send_time_us LE]
//          [u64 device_receive_time_us LE][u64 device_send_time_us LE]
#define RTT_REQUEST_LENGTH 14u
#define RTT_RESPONSE_LENGTH 30u

bool rtt_decode_request(const uint8_t *data, uint16_t length,
                        uint32_t *sequence, uint64_t *host_send_time_us);
void rtt_build_response(uint8_t response[RTT_RESPONSE_LENGTH], uint32_t sequence,
                        uint64_t host_send_time_us,
                        uint64_t device_receive_time_us,
                        uint64_t device_send_time_us);

#endif // RTT_PROTOCOL_H
