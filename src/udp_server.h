#ifndef UDP_SERVER_H
#define UDP_SERVER_H

#include <stdint.h>
#include <stdbool.h>

#include "lwip/ip_addr.h"

#define UDP_SERVER_PORT 5500
#define UDP_INJECT_PORT 5501

typedef void (*udp_recv_handler_t)(const uint8_t *data, uint16_t len,
                                   const ip_addr_t *addr, uint16_t port);

typedef struct {
    uint32_t send_attempts;
    uint32_t invalid_args;
    uint32_t pbuf_alloc_failures;
    uint32_t send_errors;
    uint32_t send_ok;
} udp_server_stats_t;

// Create and bind the UDP server PCB. Must be called after lwip_init().
void udp_server_init(void);

// Create and bind the UDP injection channel (UDP_INJECT_PORT). Processes
// the same 0x90/0x91 override/enable commands as the Panda USB protocol.
void udp_inject_server_init(void);

// Optional handler invoked for every received datagram (in addition to the
// built-in echo). Runs in lwIP context: do not block.
void udp_server_set_recv_handler(udp_recv_handler_t handler);

// True once a remote host has sent us a datagram (peer address known).
bool udp_server_has_peer(void);

// Forget the current peer (call on USB unmount / host change).
void udp_server_reset_peer(void);

// Send to the last host that sent us a datagram.
bool udp_server_send(const void *data, uint16_t len);

// Send to an explicit host:port.
bool udp_server_sendto(const ip_addr_t *addr, uint16_t port,
                       const void *data, uint16_t len);

// Read-only cumulative counters for profiling the UDP/lwIP send path.
void udp_server_get_stats(udp_server_stats_t *stats);

#endif /* UDP_SERVER_H */
