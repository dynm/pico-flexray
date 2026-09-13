#ifndef USB_NETIF_H
#define USB_NETIF_H

#include "lwip/netif.h"
#include "lwip/ip4_addr.h"

typedef struct {
    uint32_t linkoutput_calls;
    uint32_t not_ready;
    uint32_t xmit_busy;
    uint32_t xmit_ok;
} usb_netif_stats_t;

// Initialise an lwIP netif backed by the TinyUSB NCM driver.
// Must be called after lwip_init().
void usb_netif_init(struct netif *netif, const ip4_addr_t *ip,
                    const ip4_addr_t *netmask, const ip4_addr_t *gw);

// Service the NCM receive path. Call from the main loop (never from an IRQ).
// Drains frames buffered by tud_network_recv_cb() into the lwIP stack and
// re-arms the TinyUSB receiver.
void usb_netif_poll(void);

// Bring the netif link up/down (e.g. on USB mount/unmount). Clearing the link
// also drops stale ARP entries so a re-plugged host is resolved freshly.
void usb_netif_set_link(bool up);

// Read-only cumulative counters for profiling TinyUSB NCM backpressure.
void usb_netif_get_stats(usb_netif_stats_t *stats);

#endif /* USB_NETIF_H */
