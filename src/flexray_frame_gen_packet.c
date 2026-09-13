#include "flexray_frame_gen_packet.h"
#include "flexray_frame.h"
#include "pico/platform/sections.h"
#include <string.h>

// Adjacent static slots call this from frame-end; avoid XIP latency in that deadline.
const flexray_frame_gen_packet_t *__no_inline_not_in_flash_func(flexray_frame_gen_template_prepare)(flexray_frame_gen_template_t *tpl,
                                                              uint16_t len, uint8_t cycle)
{
    uint8_t *raw = (uint8_t *)(tpl->packet.words + 1);
    raw[4] = (uint8_t)((raw[4] & 0xc0u) | cycle);
    memcpy(raw + len - 3u, tpl->cycle_crc[cycle], 3u);
    return &tpl->packet;
}

void flexray_frame_gen_template_generate(flexray_frame_gen_template_t *tpl, const uint8_t *frame,
                                   uint16_t len, bool null_frame)
{
    memset(tpl, 0, sizeof(*tpl));
    tpl->packet.words[0] = __builtin_bswap32((uint32_t)len - 1u);
    uint8_t *raw = (uint8_t *)(tpl->packet.words + 1);
    memcpy(raw, frame, len);
    if (null_frame) {
        raw[0] &= (uint8_t)~0x60u; // NFI=0, PPI=0; preserve header/static length.
        memset(raw + 5, 0, len - 8u);
    }
    for (uint8_t cycle = 0; cycle < 64u; ++cycle) {
        raw[4] = (uint8_t)((raw[4] & 0xc0u) | cycle);
        uint32_t crc = calculate_flexray_frame_crc(raw, len - 3u);
        tpl->cycle_crc[cycle][0] = (uint8_t)(crc >> 16);
        tpl->cycle_crc[cycle][1] = (uint8_t)(crc >> 8);
        tpl->cycle_crc[cycle][2] = (uint8_t)crc;
    }
    (void)flexray_frame_gen_template_prepare(tpl, len, 0u);
}

static uint32_t pulse(bool high, uint32_t clocks)
{
    return ((clocks - 5u) << 1u) | (uint32_t)high;
}

uint32_t flexray_frame_gen_inducer_generate(uint32_t *words, uint16_t len, uint8_t tss)
{
    if (len < 8u || len > 262u || tss < 6u || tss > 15u) return 0;
    uint32_t n = 0;
    words[n++] = pulse(false, (uint32_t)tss * 15u);
    words[n++] = pulse(true, 30u); // FSS + first BSS HIGH
    for (uint16_t byte = 0; byte < len; ++byte) {
        // BSS LOW + 8 zero bits. Last byte includes FES LOW as well.
        words[n++] = pulse(false, byte + 1u == len ? 150u : 135u);
        if (byte + 1u < len) words[n++] = pulse(true, 15u);
    }
    words[n++] = pulse(true, 165u); // Full 11-bit post-FES idle while owning TXEN.
    words[n++] = 1u; // End ownership only after the wire has finished.
    return n;
}
