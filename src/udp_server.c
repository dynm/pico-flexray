#include "udp_server.h"

#include "lwip/udp.h"
#include "lwip/pbuf.h"

#include "flexray_forwarder_with_injector.h"
#include "flexray_frame.h"
#include "board_config.h"
#if FLEXRAY_FRAME_GEN
#include "flexray_frame_gen.h"
#endif
#include "pico/time.h"
#include "rtt_protocol.h"

#include <string.h>

static struct udp_pcb *udp_pcb;
static ip_addr_t peer_addr;
static uint16_t peer_port;
static udp_recv_handler_t recv_handler;
static udp_server_stats_t send_stats;

static void recv_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                    const ip_addr_t *addr, u16_t port)
{
    (void)arg;
    (void)pcb;

    if (p == NULL)
    {
        return;
    }

    ip_addr_copy(peer_addr, *addr);
    peer_port = port;

    if (recv_handler != NULL)
    {
        recv_handler((const uint8_t *)p->payload, (uint16_t)p->len, addr, port);
    }

    // Echo the received datagram back so both directions are exercised.
    struct pbuf *reply = pbuf_alloc(PBUF_TRANSPORT, p->len, PBUF_RAM);
    if (reply != NULL)
    {
        memcpy(reply->payload, p->payload, p->len);
        udp_sendto(pcb, reply, addr, port);
        pbuf_free(reply);
    }

    pbuf_free(p);
}

void udp_server_init(void)
{
    ip_addr_t any;

    udp_pcb = udp_new();
    if (udp_pcb == NULL)
    {
        return;
    }

    ip_addr_set_any(0, &any);
    udp_bind(udp_pcb, &any, UDP_SERVER_PORT);
    udp_recv(udp_pcb, recv_cb, NULL);
}

void udp_server_set_recv_handler(udp_recv_handler_t handler)
{
    recv_handler = handler;
}

bool udp_server_has_peer(void)
{
    return peer_port != 0;
}

void udp_server_reset_peer(void)
{
    peer_port = 0;
    ip_addr_set_any(0, &peer_addr);
}

bool udp_server_send(const void *data, uint16_t len)
{
    return udp_server_sendto(&peer_addr, peer_port, data, len);
}

bool udp_server_sendto(const ip_addr_t *addr, uint16_t port,
                       const void *data, uint16_t len)
{
    struct pbuf *p;
    err_t err;

    send_stats.send_attempts++;

    if (udp_pcb == NULL || addr == NULL || port == 0 || len == 0)
    {
        send_stats.invalid_args++;
        return false;
    }

    p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    if (p == NULL)
    {
        send_stats.pbuf_alloc_failures++;
        return false;
    }

    memcpy(p->payload, data, len);
    err = udp_sendto(udp_pcb, p, addr, port);
    pbuf_free(p);

    if (err == ERR_OK)
    {
        send_stats.send_ok++;
    }
    else
    {
        send_stats.send_errors++;
    }

    return err == ERR_OK;
}

void udp_server_get_stats(udp_server_stats_t *stats)
{
    if (stats != NULL)
    {
        *stats = send_stats;
    }
}

// ---------------------------------------------------------------------------
// UDP injection channel (mirrors the Panda USB vendor injection protocol):
//   op 0x90: [0x90][u16 id LE][u8 base][u16 len LE][len bytes payload]
//            -> injector_submit_override(id, base, len, payload)
//   op 0x91: [0x91][u8 enabled]
//            -> injector_set_enabled(enabled)
// ---------------------------------------------------------------------------
static struct udp_pcb *inject_pcb;

static void send_rtt_response(struct udp_pcb *pcb, const ip_addr_t *addr,
                              u16_t port, const uint8_t *request,
                              uint64_t receive_time_us)
{
    struct pbuf *reply = pbuf_alloc(PBUF_TRANSPORT, RTT_RESPONSE_LENGTH, PBUF_RAM);
    if (reply == NULL)
    {
        return;
    }

    uint32_t sequence;
    uint64_t host_send_time_us;
    if (!rtt_decode_request(request, RTT_REQUEST_LENGTH,
                            &sequence, &host_send_time_us)) {
        pbuf_free(reply);
        return;
    }

    uint8_t response[RTT_RESPONSE_LENGTH];
    rtt_build_response(response, sequence, host_send_time_us,
                       receive_time_us, time_us_64());

    if (pbuf_take(reply, response, RTT_RESPONSE_LENGTH) == ERR_OK)
    {
        (void)udp_sendto(pcb, reply, addr, port);
    }
    pbuf_free(reply);
}

static void inject_recv_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                           const ip_addr_t *addr, u16_t port)
{
    uint8_t packet[1u + MAX_FRAME_PAYLOAD_BYTES + 8u];
    const uint8_t *data = packet;
    uint16_t len;
    uint32_t off = 0;
    const uint64_t receive_time_us = time_us_64();

    (void)arg;

    if (p == NULL)
    {
        return;
    }

    len = (uint16_t)p->tot_len;
    if (len > sizeof(packet) || pbuf_copy_partial(p, packet, len, 0) != len)
    {
        pbuf_free(p);
        return;
    }

    if (rtt_decode_request(data, len, NULL, NULL))
    {
        send_rtt_response(pcb, addr, port, data, receive_time_us);
        pbuf_free(p);
        return;
    }

#if FLEXRAY_FRAME_GEN
    if (len > 0u && data[0] >= FLEXRAY_FRAME_GEN_OP_CONFIG &&
        data[0] <= FLEXRAY_FRAME_GEN_OP_DIAGNOSTICS) {
        uint8_t response[4u + sizeof(flexray_frame_gen_status_t)];
        size_t response_len = flexray_frame_gen_command(data, len, response, sizeof(response));
        struct pbuf *reply = pbuf_alloc(PBUF_TRANSPORT, (u16_t)response_len, PBUF_RAM);
        if (reply != NULL) {
            if (pbuf_take(reply, response, (u16_t)response_len) == ERR_OK) {
                (void)udp_sendto(pcb, reply, addr, port);
            }
            pbuf_free(reply);
        }
        pbuf_free(p);
        return;
    }
#endif

    while ((uint16_t)(len - off) >= 1)
    {
        uint8_t op = data[off++];
        if (op == 0x90)
        {
            uint16_t id;
            uint8_t base;
            uint16_t flen;

            if ((uint16_t)(len - off) < 5)
            {
                break;
            }
            id = (uint16_t)(data[off] | ((uint16_t)data[off + 1] << 8));
            base = data[off + 2];
            flen = (uint16_t)(data[off + 3] | ((uint16_t)data[off + 4] << 8));
            off += 5;
            if ((uint16_t)(len - off) < flen)
            {
                break;
            }
            injector_submit_override_from(id, base, flen, &data[off],
                                          INJECT_TRANSPORT_UDP);
            off += flen;
        }
        else if (op == 0x91)
        {
            if ((uint16_t)(len - off) < 1)
            {
                break;
            }
            injector_set_enabled(data[off++] != 0);
        }
#if FLEXRAY_FRAME_GEN
        else if (op == FLEXRAY_FRAME_GEN_OP_PAYLOAD || op == FLEXRAY_FRAME_GEN_OP_SWITCH)
        {
            size_t used = flexray_frame_gen_action(data + off - 1u, len - off + 1u);
            if (!used) break;
            off += (uint16_t)used - 1u;
        }
#endif
        else
        {
            break;
        }
    }

    pbuf_free(p);
}

void udp_inject_server_init(void)
{
    ip_addr_t any;

    inject_pcb = udp_new();
    if (inject_pcb == NULL)
    {
        return;
    }

    ip_addr_set_any(0, &any);
    udp_bind(inject_pcb, &any, UDP_INJECT_PORT);
    udp_recv(inject_pcb, inject_recv_cb, NULL);
}
