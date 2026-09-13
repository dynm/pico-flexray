#include "usb_netif.h"

#include "tusb.h"
#include "class/net/net_device.h"

#include "pico/time.h"

#include "lwip/etharp.h"
#include "netif/ethernet.h"
#include "lwip/pbuf.h"

#include <string.h>

// Locally administered unicast MAC address. The host derives its own interface
// MAC from this value (via the iMacAddr string), so the device's lwIP MAC must
// differ (see netif_init_cb). Changing this makes the host treat the device as
// a new interface, which forces a fresh DHCP request.
uint8_t tud_network_mac_address[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x04};

static struct netif *g_netif;
static usb_netif_stats_t netif_stats;

// --- Receive buffering -------------------------------------------------------
// tud_network_recv_cb() runs in USB IRQ context on the Pico, so lwIP must not
// be touched there. Incoming frames are copied into a small ring and handed to
// the lwIP stack from usb_netif_poll() in the main loop instead.
#define USB_NETIF_RX_SLOTS 4
#define USB_NETIF_RX_BUF   CFG_TUD_NET_MTU

typedef struct {
    uint16_t len;
    uint8_t data[USB_NETIF_RX_BUF];
} usb_netif_rx_slot_t;

static usb_netif_rx_slot_t rx_slots[USB_NETIF_RX_SLOTS];
static volatile uint8_t rx_head;
static volatile uint8_t rx_tail;

// --- lwIP netif callbacks ----------------------------------------------------

static err_t linkoutput_fn(struct netif *netif, struct pbuf *p)
{
    (void)netif;
    netif_stats.linkoutput_calls++;

    // This path must never wait for the host. If all NCM IN NTBs are busy,
    // report backpressure to lwIP and let this packet be dropped/retried by
    // the caller. FlexRay forwarding continues independently in PIO.
    if (!tud_ready())
    {
        netif_stats.not_ready++;
        return ERR_USE;
    }
    if (!tud_network_can_xmit(p->tot_len))
    {
        netif_stats.xmit_busy++;
        return ERR_MEM;
    }

    tud_network_xmit(p, 0);
    netif_stats.xmit_ok++;
    return ERR_OK;
}

static err_t ip4_output_fn(struct netif *netif, struct pbuf *p, const ip4_addr_t *addr)
{
    return etharp_output(netif, p, addr);
}

static err_t netif_init_cb(struct netif *netif)
{
    LWIP_ASSERT("netif != NULL", (netif != NULL));

    netif->mtu = CFG_TUD_NET_MTU;
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP |
                   NETIF_FLAG_LINK_UP | NETIF_FLAG_UP;
    netif->state = NULL;
    netif->name[0] = 'E';
    netif->name[1] = 'N';
    netif->linkoutput = linkoutput_fn;
    netif->output = ip4_output_fn;
    netif->hwaddr_len = sizeof(tud_network_mac_address);
    memcpy(netif->hwaddr, tud_network_mac_address, sizeof(tud_network_mac_address));
    // The host derives its interface MAC from tud_network_mac_address, so the
    // device must use a different one or L2 (ARP/DHCP/UDP) breaks. Toggle the
    // LSbit of the last byte, as the TinyUSB net example does.
    netif->hwaddr[5] ^= 0x01;

    return ERR_OK;
}

// --- TinyUSB network callbacks ----------------------------------------------

// Called from USB IRQ context. Copy the frame and let the main loop consume it.
bool tud_network_recv_cb(const uint8_t *src, uint16_t size)
{
    uint8_t next;

    if (size > USB_NETIF_RX_BUF)
    {
        return false;
    }

    next = (uint8_t)((rx_head + 1) % USB_NETIF_RX_SLOTS);
    if (next == rx_tail)
    {
        return false; // ring full; the frame will be retried on the next renew
    }

    memcpy(rx_slots[rx_head].data, src, size);
    rx_slots[rx_head].len = size;
    rx_head = next;

    return true;
}

// Called from USB IRQ context. Copy the pending lwIP pbuf into the TinyUSB NTB.
uint16_t tud_network_xmit_cb(uint8_t *dst, void *ref, uint16_t arg)
{
    struct pbuf *p = (struct pbuf *)ref;

    (void)arg;

    return pbuf_copy_partial(p, dst, p->tot_len, 0);
}

// --- Public API --------------------------------------------------------------

void usb_netif_init(struct netif *netif, const ip4_addr_t *ip,
                    const ip4_addr_t *netmask, const ip4_addr_t *gw)
{
    g_netif = netif;

    netif_add(netif, ip, netmask, gw, NULL, netif_init_cb, ethernet_input);
    netif_set_default(netif);
}

void usb_netif_poll(void)
{
    if (g_netif == NULL)
    {
        return;
    }

    while (rx_tail != rx_head)
    {
        usb_netif_rx_slot_t *slot = &rx_slots[rx_tail];
        struct pbuf *p = pbuf_alloc(PBUF_RAW, slot->len, PBUF_POOL);

        if (p == NULL)
        {
            break; // out of buffers; retry on the next poll
        }

        pbuf_take(p, slot->data, slot->len);
        rx_tail = (uint8_t)((rx_tail + 1) % USB_NETIF_RX_SLOTS);

        if (g_netif->input(p, g_netif) != ERR_OK)
        {
            pbuf_free(p);
        }
    }

    if (tud_ready())
    {
        tud_network_recv_renew();
    }
}

void usb_netif_set_link(bool up)
{
    if (g_netif == NULL)
    {
        return;
    }

    if (up)
    {
        netif_set_link_up(g_netif);
    }
    else
    {
        netif_set_link_down(g_netif);
        etharp_cleanup_netif(g_netif);
    }
}

void usb_netif_get_stats(usb_netif_stats_t *stats)
{
    if (stats != NULL)
    {
        *stats = netif_stats;
    }
}

// --- lwIP NO_SYS platform glue ----------------------------------------------

sys_prot_t sys_arch_protect(void)
{
    return 0;
}

void sys_arch_unprotect(sys_prot_t pval)
{
    (void)pval;
}

uint32_t sys_now(void)
{
    return to_ms_since_boot(get_absolute_time());
}

uint32_t sys_jiffies(void)
{
    return time_us_32();
}
