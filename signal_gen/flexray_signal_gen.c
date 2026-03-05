#include "flexray_signal_gen.h"
#include "flexray_frame.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/structs/pio.h"
#include "pico/time.h"
#include <string.h>
#include <stdio.h>

#include "flexray_signal_gen.pio.h"

typedef struct {
    bool active;
    uint8_t channel_mask;
    uint16_t frame_id;
    uint8_t indicators;
    uint16_t payload_len;
    uint8_t payload[MAX_FRAME_PAYLOAD_BYTES];
    uint8_t frame_buf[MAX_FRAME_BUF_SIZE_BYTES] __attribute__((aligned(4)));
} frame_slot_t;

typedef struct {
    bool hw_ready;
    uint sm;
    int dma_chan;
    dma_channel_config dma_cfg;
} channel_state_t;

static PIO gen_pio;
static channel_state_t chans[SIGNAL_GEN_MAX_CHANNELS];
static frame_slot_t slots[SIGNAL_GEN_MAX_SLOTS];
static uint num_hw_channels;

static bool running;
static uint8_t cycle_count;
static absolute_time_t next_tx_time;
static uint32_t tx_count;
static uint32_t slot_duration_us;

bool signal_gen_init(PIO pio, const fr_channel_pins_t *pins, uint num_channels)
{
    if (num_channels > SIGNAL_GEN_MAX_CHANNELS) return false;
    gen_pio = pio;
    num_hw_channels = num_channels;
    running = false;
    cycle_count = 0;
    tx_count = 0;
    slot_duration_us = 0;
    memset(slots, 0, sizeof(slots));

    uint offset = pio_add_program(pio, &flexray_signal_gen_program);

    for (uint i = 0; i < num_channels; i++) {
        channel_state_t *ch = &chans[i];
        memset(ch, 0, sizeof(*ch));

        ch->sm = pio_claim_unused_sm(pio, true);
        flexray_signal_gen_program_init(pio, ch->sm, offset,
                                        pins[i].tx_pin, pins[i].txen_pin);

        ch->dma_chan = (int)dma_claim_unused_channel(true);
        ch->dma_cfg = dma_channel_get_default_config((uint)ch->dma_chan);
        channel_config_set_transfer_data_size(&ch->dma_cfg, DMA_SIZE_32);
        channel_config_set_bswap(&ch->dma_cfg, true);
        channel_config_set_read_increment(&ch->dma_cfg, true);
        channel_config_set_write_increment(&ch->dma_cfg, false);
        channel_config_set_dreq(&ch->dma_cfg, pio_get_dreq(pio, ch->sm, true));
        dma_channel_set_config((uint)ch->dma_chan, &ch->dma_cfg, false);
        dma_channel_set_write_addr((uint)ch->dma_chan,
                                   (void *)&pio->txf[ch->sm], false);
        ch->hw_ready = true;
        printf("Signal gen ch%u: sm=%u dma=%d tx=%u txen=%u\n",
               i, ch->sm, ch->dma_chan, pins[i].tx_pin, pins[i].txen_pin);
    }
    return true;
}

static uint16_t build_header(uint8_t *buf, uint16_t frame_id, uint8_t indicators,
                              uint8_t payload_length_words, uint8_t cyc)
{
    buf[0] = (uint8_t)((indicators << 3) | ((frame_id >> 8) & 0x07));
    buf[1] = (uint8_t)(frame_id & 0xFF);
    buf[2] = (uint8_t)(payload_length_words << 1);
    uint16_t hcrc = calculate_flexray_header_crc(buf);
    buf[2] = (uint8_t)((payload_length_words << 1) | ((hcrc >> 10) & 0x01));
    buf[3] = (uint8_t)((hcrc >> 2) & 0xFF);
    buf[4] = (uint8_t)(((hcrc & 0x03) << 6) | (cyc & 0x3F));
    return hcrc;
}

static void wait_for_pio_idle(channel_state_t *ch)
{
    dma_channel_wait_for_finish_blocking((uint)ch->dma_chan);

    uint32_t mask = 1u << (PIO_FDEBUG_TXSTALL_LSB + ch->sm);
    while (!(gen_pio->fdebug & mask))
        tight_loop_contents();
}

// Build frame into slot's frame_buf. Returns total_len (padded to 4B)
// and sets *out_byte_count to the wire byte count.
static uint16_t build_frame(frame_slot_t *sl, uint8_t cyc, uint32_t *out_byte_count)
{
    uint8_t plw = (uint8_t)(sl->payload_len / 2);
    build_header(sl->frame_buf, sl->frame_id, sl->indicators, plw, cyc);

    if (sl->payload_len > 0)
        memcpy(sl->frame_buf + 5, sl->payload, sl->payload_len);

    uint16_t before_crc = (uint16_t)(5 + sl->payload_len);
    uint32_t crc = calculate_flexray_frame_crc(sl->frame_buf, before_crc);
    sl->frame_buf[before_crc + 0] = (uint8_t)(crc >> 16);
    sl->frame_buf[before_crc + 1] = (uint8_t)(crc >> 8);
    sl->frame_buf[before_crc + 2] = (uint8_t)(crc);

    uint16_t total_len = (uint16_t)(before_crc + 3);
    *out_byte_count = (uint32_t)total_len;

    while (total_len & 3)
        sl->frame_buf[total_len++] = 0xFF;

    return total_len;
}

static uint32_t calc_slot_duration(void)
{
    uint16_t max_payload = 0;
    for (uint s = 0; s < SIGNAL_GEN_MAX_SLOTS; s++)
        if (slots[s].active && slots[s].payload_len > max_payload)
            max_payload = slots[s].payload_len;

    uint32_t frame_bytes = max_payload + 8;
    uint32_t wire_us = frame_bytes + 3;
    uint32_t dur = wire_us + 10;
    dur = ((dur + 4) / 5) * 5;
    if (dur < 25) dur = 25;
    return dur;
}

bool signal_gen_set_slot(uint slot, uint8_t channel_mask, uint16_t frame_id,
                         uint8_t indicators, const uint8_t *payload,
                         uint16_t payload_len)
{
    if (slot >= SIGNAL_GEN_MAX_SLOTS) return false;
    if (frame_id > 2047 || payload_len > MAX_FRAME_PAYLOAD_BYTES) return false;
    if (payload_len & 1) return false;
    uint8_t hw_mask = (uint8_t)((1u << num_hw_channels) - 1u);
    if (channel_mask == 0 || (channel_mask & ~hw_mask)) return false;

    frame_slot_t *sl = &slots[slot];
    sl->frame_id = frame_id;
    sl->indicators = indicators;
    sl->payload_len = payload_len;
    sl->channel_mask = channel_mask;
    if (payload_len > 0 && payload != NULL)
        memcpy(sl->payload, payload, payload_len);
    sl->active = true;

    printf("SET slot%u: mask=0x%02x id=0x%03x ind=0x%02x len=%u\n",
           slot, channel_mask, frame_id, indicators, payload_len);
    return true;
}

void signal_gen_clear_slot(uint slot)
{
    if (slot >= SIGNAL_GEN_MAX_SLOTS) return;
    slots[slot].active = false;
    slots[slot].channel_mask = 0;
    printf("CLEAR slot%u\n", slot);
}

bool signal_gen_update_slot_payload(uint slot,
                                    const uint8_t *payload, uint16_t payload_len)
{
    if (slot >= SIGNAL_GEN_MAX_SLOTS) return false;
    frame_slot_t *sl = &slots[slot];
    if (!sl->active) return false;
    if (payload_len > MAX_FRAME_PAYLOAD_BYTES || (payload_len & 1)) return false;

    sl->payload_len = payload_len;
    if (payload_len > 0 && payload != NULL)
        memcpy(sl->payload, payload, payload_len);
    return true;
}

void signal_gen_start(void)
{
    slot_duration_us = calc_slot_duration();
    cycle_count = 0;
    tx_count = 0;
    next_tx_time = get_absolute_time();
    running = true;
    printf("START @200Hz  gdStaticSlot=%lu us\n", (unsigned long)slot_duration_us);
}

void signal_gen_stop(void)
{
    if (running) {
        running = false;
        printf("STOP after %lu cycles\n", (unsigned long)tx_count);
    }
}

bool signal_gen_is_running(void)
{
    return running;
}

uint8_t signal_gen_tick(void)
{
    if (!running) return 0;

    absolute_time_t now = get_absolute_time();
    if (!time_reached(next_tx_time)) return 0;

    uint8_t tx_mask = 0;
    absolute_time_t tick_start = get_absolute_time();

    for (uint s = 0; s < SIGNAL_GEN_MAX_SLOTS; s++) {
        frame_slot_t *sl = &slots[s];
        if (!sl->active || sl->channel_mask == 0) continue;

        absolute_time_t slot_start = delayed_by_us(tick_start,
                                                    s * slot_duration_us);
        busy_wait_until(slot_start);

        // Wait for all target channels to be idle
        for (uint c = 0; c < num_hw_channels; c++) {
            if (!(sl->channel_mask & (1u << c))) continue;
            if (!chans[c].hw_ready) continue;
            wait_for_pio_idle(&chans[c]);
        }

        // Build frame once into shared buffer
        uint32_t byte_count;
        uint16_t total_len = build_frame(sl, cycle_count, &byte_count);
        uint32_t dma_words = total_len / 4;

        // Phase 1: clear stall flags and push byte_count to all SMs
        // (each SM immediately begins TSS/FSS preamble after receiving this)
        for (uint c = 0; c < num_hw_channels; c++) {
            if (!(sl->channel_mask & (1u << c))) continue;
            if (!chans[c].hw_ready) continue;
            channel_state_t *ch = &chans[c];
            gen_pio->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + ch->sm);
            pio_sm_put(gen_pio, ch->sm, byte_count - 1);
        }

        // Phase 2: start DMA for all channels — SMs are in preamble,
        // ~40 PIO cycles before first data byte is needed
        for (uint c = 0; c < num_hw_channels; c++) {
            if (!(sl->channel_mask & (1u << c))) continue;
            if (!chans[c].hw_ready) continue;
            channel_state_t *ch = &chans[c];
            dma_channel_set_read_addr((uint)ch->dma_chan, sl->frame_buf, false);
            dma_channel_set_trans_count((uint)ch->dma_chan, dma_words, true);
            tx_mask |= (1u << c);
        }
    }

    for (uint c = 0; c < num_hw_channels; c++)
        if (chans[c].hw_ready)
            wait_for_pio_idle(&chans[c]);

    cycle_count = (cycle_count + 1) & 0x3F;
    tx_count++;
    next_tx_time = delayed_by_us(next_tx_time, FLEXRAY_CYCLE_PERIOD_US);

    if (absolute_time_diff_us(next_tx_time, now) > FLEXRAY_CYCLE_PERIOD_US)
        next_tx_time = delayed_by_us(now, FLEXRAY_CYCLE_PERIOD_US);

    return tx_mask;
}
