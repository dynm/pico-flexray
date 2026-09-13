#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

// Single-threaded (NO_SYS) lwIP configuration tuned for a USB NCM
// device running a small UDP server. TCP and sockets are disabled to
// keep memory usage low.

#define NO_SYS                      1
#define LWIP_SOCKET                 0
#define LWIP_NETCONN                0

#define MEM_LIBC_MALLOC             0
#define MEM_ALIGNMENT               4
#define MEM_SIZE                    16384

#define MEMP_NUM_UDP_PCB            8
#define MEMP_NUM_TCP_PCB            0
#define MEMP_NUM_TCP_SEG            0
#define MEMP_NUM_ARP_QUEUE          10

#define PBUF_POOL_SIZE              24
#define PBUF_POOL_BUFSIZE           1600

#define LWIP_ARP                    1
#define LWIP_ETHERNET               1
#define LWIP_ICMP                   1
#define LWIP_RAW                    0

#define LWIP_IPV4                   1
#define LWIP_IPV6                   0
#define LWIP_UDP                    1
#define LWIP_TCP                    0
#define LWIP_DHCP                   0
#define LWIP_AUTOIP                 0

// The device runs a DHCP *server* (not the lwIP DHCP client). Defining this
// enables lwIP's IP_ACCEPT_LINK_LAYER_ADDRESSING, which allows packets with a
// 0.0.0.0 source (the client's DISCOVER) through the IPv4 source check instead
// of dropping them as "old skool broadcast".
#define LWIP_IP_ACCEPT_UDP_PORT(port) ((port) == PP_NTOHS(67))

#define LWIP_NETIF_STATUS_CALLBACK  1
#define LWIP_NETIF_LINK_CALLBACK    1
#define LWIP_NETIF_HOSTNAME         1
#define LWIP_CHECKSUM_CTRL_PER_NETIF 1

#define TCP_MSS                     1460
#define TCP_WND                     (8 * TCP_MSS)
#define TCP_SND_BUF                 (8 * TCP_MSS)

#define LWIP_DEBUG                  0
#define UDP_DEBUG                   0
#define ETHARP_DEBUG                0
#define PBUF_DEBUG                  0
#define NETIF_DEBUG                 0
#define SYS_DEBUG                   0

#endif /* _LWIPOPTS_H */
