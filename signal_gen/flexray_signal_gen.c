#include "flexray_signal_gen.h"
#include "flexray_frame.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/structs/pio.h"
#include "hardware/regs/dma.h"
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
    uint tx_pin;
    uint txen_pin;
    int dma_chan;
    dma_channel_config dma_cfg;
} channel_state_t;

static PIO gen_pio;
static uint gen_program_offset;
static channel_state_t chans[SIGNAL_GEN_MAX_CHANNELS];
static frame_slot_t slots[SIGNAL_GEN_MAX_SLOTS];
static uint num_hw_channels;

static bool running;
static uint8_t cycle_count;
static uint32_t tx_count;
static uint32_t slot_duration_us;

#define FLEXRAY_BITS_PER_US 10u
#define CYCLE_BITS (FLEXRAY_CYCLE_PERIOD_US * FLEXRAY_BITS_PER_US)
#define INTERLEAVED_BITS_PER_WIRE_BIT 2u
#define CYCLE_STREAM_BITS (CYCLE_BITS * INTERLEAVED_BITS_PER_WIRE_BIT)
#define CYCLE_BYTES (CYCLE_STREAM_BITS / 8u)
#define MAX_SLOT_DURATION_US \
    (((MAX_FRAME_PAYLOAD_BYTES + 8u + 3u + 10u + 4u) / 5u) * 5u)
#define MAX_CYCLE_BUF_BYTES (((CYCLE_STREAM_BITS + 31u) / 32u) * 4u)
#define RING_SIZE_BITS 15u
#define RING_BYTES (1u << RING_SIZE_BITS)
#define RING_WORDS (RING_BYTES / 4u)
#define TX_FIFO_PREFILL_WORDS 4u
#define TX_FIFO_PREFILL_BYTES (TX_FIFO_PREFILL_WORDS * 4u)
#define RING_PREFILL_CYCLES 2u
#define RING_LOW_WATER_CYCLES 1u

static uint8_t cycle_bufs[SIGNAL_GEN_MAX_CHANNELS][MAX_CYCLE_BUF_BYTES]
    __attribute__((aligned(4)));
static uint8_t stream_rings[SIGNAL_GEN_MAX_CHANNELS][RING_BYTES]
    __attribute__((aligned(RING_BYTES)));

static uint64_t ring_write_abs;
static uint32_t last_read_offset;
static uint64_t read_wraps;
static uint8_t render_cycle_count;
static uint8_t current_tx_mask;

typedef struct {
    uint8_t *buf;
    uint32_t bit_pos;
    uint32_t bit_limit;
} bit_writer_t;

bool signal_gen_init(PIO pio, const fr_channel_pins_t *pins, uint num_channels)
{
    if (num_channels > SIGNAL_GEN_MAX_CHANNELS) return false;
    gen_pio = pio;
    num_hw_channels = num_channels;
    running = false;
    cycle_count = 0;
    tx_count = 0;
    slot_duration_us = 0;
    ring_write_abs = 0;
    last_read_offset = 0;
    read_wraps = 0;
    render_cycle_count = 0;
    current_tx_mask = 0;
    memset(slots, 0, sizeof(slots));

    gen_program_offset = pio_add_program(pio, &flexray_signal_gen_program);

    for (uint i = 0; i < num_channels; i++) {
        channel_state_t *ch = &chans[i];
        memset(ch, 0, sizeof(*ch));

        ch->sm = pio_claim_unused_sm(pio, true);
        ch->tx_pin = pins[i].tx_pin;
        ch->txen_pin = pins[i].txen_pin;
        flexray_signal_gen_program_init(pio, ch->sm, gen_program_offset,
                                        pins[i].tx_pin, pins[i].txen_pin);

        ch->dma_chan = (int)dma_claim_unused_channel(true);
        ch->dma_cfg = dma_channel_get_default_config((uint)ch->dma_chan);
        channel_config_set_transfer_data_size(&ch->dma_cfg, DMA_SIZE_32);
        channel_config_set_bswap(&ch->dma_cfg, true);
        channel_config_set_read_increment(&ch->dma_cfg, true);
        channel_config_set_write_increment(&ch->dma_cfg, false);
        channel_config_set_ring(&ch->dma_cfg, false, RING_SIZE_BITS);
        channel_config_set_dreq(&ch->dma_cfg, pio_get_dreq(pio, ch->sm, true));
#if PICO_RP2040
        // RP2040 cannot self-trigger; we keep chain_to = self so completion is
        // harmless, and use a huge transfer count in signal_gen_start().
        channel_config_set_chain_to(&ch->dma_cfg, (uint)ch->dma_chan);
#endif
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

static void stop_dma_chan(int dma_chan)
{
    uint chan = (uint)dma_chan;
    dma_channel_hw_addr(chan)->ctrl_trig &= ~DMA_CH0_CTRL_TRIG_EN_BITS;
    dma_channel_abort(chan);
}

static void stop_channel_dma(channel_state_t *ch)
{
    stop_dma_chan(ch->dma_chan);
}

static void park_channel(channel_state_t *ch)
{
    pio_sm_set_enabled(gen_pio, ch->sm, false);
    pio_sm_set_pins_with_mask(gen_pio, ch->sm,
                              (1u << ch->tx_pin) | (1u << ch->txen_pin),
                              (1u << ch->tx_pin) | (1u << ch->txen_pin));
}

// Build frame into slot's frame_buf and sets *out_byte_count to the real
// frame byte count. Padding is kept only for aligned scratch storage.
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

static uint32_t calc_payload_slot_duration(uint16_t payload_len)
{
    uint32_t frame_bytes = payload_len + 8u;
    uint32_t wire_us = frame_bytes + 3u;
    uint32_t dur = wire_us + 10u;
    dur = ((dur + 4u) / 5u) * 5u;
    if (dur < 25u) dur = 25u;
    return dur;
}

static uint32_t calc_slot_duration(void)
{
    uint16_t max_payload = 0;
    for (uint s = 0; s < SIGNAL_GEN_MAX_SLOTS; s++)
        if (slots[s].active && slots[s].payload_len > max_payload)
            max_payload = slots[s].payload_len;

    return calc_payload_slot_duration(max_payload);
}

static uint32_t active_slot_extent(void)
{
    uint32_t extent = 0;
    for (uint s = 0; s < SIGNAL_GEN_MAX_SLOTS; s++)
        if (slots[s].active)
            extent = s + 1u;
    return extent;
}

static inline void bw_append_bit(bit_writer_t *bw, bool bit)
{
    if (bw->bit_pos >= bw->bit_limit) return;
    if (!bit)
        bw->buf[bw->bit_pos >> 3] &= (uint8_t)~(0x80u >> (bw->bit_pos & 7u));
    bw->bit_pos++;
}

static inline void bw_append_wire_bit(bit_writer_t *bw, bool txd, bool txen)
{
    bw_append_bit(bw, txen);
    bw_append_bit(bw, txd);
}

static void bw_append_byte(bit_writer_t *bw, uint8_t byte)
{
    for (int bit = 7; bit >= 0; bit--)
        bw_append_wire_bit(bw, (byte >> bit) & 1u, false);
}

static void bw_skip_to(bit_writer_t *bw, uint32_t bit_pos)
{
    if (bit_pos > bw->bit_limit) bit_pos = bw->bit_limit;
    if (bw->bit_pos < bit_pos)
        bw->bit_pos = bit_pos;
}

static uint32_t render_slot_frame_bits(frame_slot_t *sl, uint8_t cyc,
                                       bit_writer_t *bw, uint32_t slot_end_bit)
{
    uint32_t byte_count;
    build_frame(sl, cyc, &byte_count);
    uint32_t frame_start_bit = bw->bit_pos;

    bw_append_wire_bit(bw, false, false); // TSS
    bw_append_wire_bit(bw, false, false);
    bw_append_wire_bit(bw, true, false);  // FSS

    for (uint32_t i = 0; i < byte_count; i++) {
        bw_append_wire_bit(bw, true, false);  // BSS0
        bw_append_wire_bit(bw, false, false); // BSS1
        bw_append_byte(bw, sl->frame_buf[i]);
    }

    bw_append_wire_bit(bw, false, false); // FES
    for (uint i = 0; i < 11; i++)
        bw_append_wire_bit(bw, true, false); // CID

    uint32_t frame_end_bit = bw->bit_pos;
    bw_skip_to(bw, slot_end_bit);
    return frame_end_bit > frame_start_bit ? frame_end_bit : frame_start_bit;
}

static uint8_t render_cycle_buffers(uint8_t cyc)
{
    uint32_t slot_bits = slot_duration_us * FLEXRAY_BITS_PER_US *
                         INTERLEAVED_BITS_PER_WIRE_BIT;
    uint32_t bit_count = CYCLE_STREAM_BITS;
    uint32_t byte_count = (bit_count + 7u) / 8u;
    uint32_t padded_bytes = (byte_count + 3u) & ~3u;
    uint8_t tx_mask = 0;

    for (uint c = 0; c < num_hw_channels; c++)
        memset(cycle_bufs[c], 0xFF, padded_bytes);

    for (uint c = 0; c < num_hw_channels; c++) {
        bit_writer_t bw = {
            .buf = cycle_bufs[c],
            .bit_pos = 0,
            .bit_limit = bit_count,
        };

        for (uint s = 0; s < SIGNAL_GEN_MAX_SLOTS; s++) {
            uint32_t slot_end_bit = (s + 1u) * slot_bits;
            frame_slot_t *sl = &slots[s];
            if (sl->active && (sl->channel_mask & (1u << c))) {
                render_slot_frame_bits(sl, cyc, &bw, slot_end_bit);
                tx_mask |= (1u << c);
            } else {
                bw_skip_to(&bw, slot_end_bit);
            }
        }
    }

    return tx_mask;
}

static void ring_copy(uint8_t *ring, uint32_t offset, const uint8_t *src,
                      uint32_t len)
{
    uint32_t first = RING_BYTES - offset;
    if (first > len) first = len;
    memcpy(&ring[offset], src, first);
    if (first < len)
        memcpy(ring, src + first, len - first);
}

static uint8_t render_next_cycle_to_ring(void)
{
    uint8_t tx_mask = render_cycle_buffers(render_cycle_count);
    uint32_t offset = (uint32_t)ring_write_abs & (RING_BYTES - 1u);

    for (uint c = 0; c < num_hw_channels; c++)
        ring_copy(stream_rings[c], offset, cycle_bufs[c], CYCLE_BYTES);

    ring_write_abs += CYCLE_BYTES;
    render_cycle_count = (render_cycle_count + 1u) & 0x3Fu;
    return tx_mask;
}

static uint64_t dma_read_abs(void)
{
    if (num_hw_channels == 0 || !chans[0].hw_ready) return 0;

    channel_state_t *ch = &chans[0];
    uintptr_t base = (uintptr_t)stream_rings[0];
    uintptr_t addr = dma_channel_hw_addr((uint)ch->dma_chan)->read_addr;
    uint32_t offset = (uint32_t)(addr - base) & (RING_BYTES - 1u);

    if (offset < last_read_offset &&
        (last_read_offset - offset) > (RING_BYTES / 2u)) {
        read_wraps += RING_BYTES;
    }
    last_read_offset = offset;
    return read_wraps + offset;
}

static void prefill_sm_fifo(channel_state_t *ch, const uint8_t *stream)
{
    for (uint i = 0; i < TX_FIFO_PREFILL_WORDS; i++) {
        if (pio_sm_is_tx_fifo_full(gen_pio, ch->sm))
            break;
        uint32_t word;
        memcpy(&word, stream + (i * sizeof(word)), sizeof(word));
        word = __builtin_bswap32(word);
        pio_sm_put(gen_pio, ch->sm, word);
    }
}

static bool is_valid_gpio_pair(uint tx_pin, uint txen_pin)
{
    if (tx_pin == txen_pin) return false;
    if (tx_pin >= NUM_BANK0_GPIOS || txen_pin >= NUM_BANK0_GPIOS) return false;
    return true;
}

static bool pins_collide_with_other_channel(uint channel, uint tx_pin, uint txen_pin)
{
    for (uint c = 0; c < num_hw_channels; c++) {
        if (c == channel || !chans[c].hw_ready) continue;
        if (tx_pin == chans[c].tx_pin || tx_pin == chans[c].txen_pin ||
            txen_pin == chans[c].tx_pin || txen_pin == chans[c].txen_pin) {
            return true;
        }
    }
    return false;
}

static void apply_channel_pins(uint channel, uint tx_pin, uint txen_pin)
{
    channel_state_t *ch = &chans[channel];
    if (ch->tx_pin == tx_pin && ch->txen_pin == txen_pin) return;

    stop_channel_dma(ch);
    park_channel(ch);
    pio_sm_clear_fifos(gen_pio, ch->sm);

    uint old_tx = ch->tx_pin;
    uint old_txen = ch->txen_pin;

    gpio_init(old_tx);
    gpio_init(old_txen);
    gpio_pull_up(old_tx);
    gpio_pull_up(old_txen);

    ch->tx_pin = tx_pin;
    ch->txen_pin = txen_pin;
    flexray_signal_gen_program_init(gen_pio, ch->sm, gen_program_offset,
                                    tx_pin, txen_pin);
    dma_channel_set_write_addr((uint)ch->dma_chan,
                               (void *)&gen_pio->txf[ch->sm], false);

    printf("PINS ch%u: tx=%u txen=%u\n", channel, tx_pin, txen_pin);
}

bool signal_gen_set_channel_pins(uint channel, uint tx_pin, uint txen_pin)
{
    if (channel >= num_hw_channels) return false;
    if (running) return false;
    if (!is_valid_gpio_pair(tx_pin, txen_pin)) return false;
    if (pins_collide_with_other_channel(channel, tx_pin, txen_pin)) return false;
    if (!chans[channel].hw_ready) return false;

    apply_channel_pins(channel, tx_pin, txen_pin);
    return true;
}

bool signal_gen_set_channel_pin_map(const fr_channel_pins_t *pins, uint num_channels)
{
    if (pins == NULL || num_channels != num_hw_channels) return false;
    if (running) return false;

    for (uint c = 0; c < num_channels; c++) {
        if (!chans[c].hw_ready) return false;
        if (!is_valid_gpio_pair(pins[c].tx_pin, pins[c].txen_pin)) return false;
        for (uint other = c + 1u; other < num_channels; other++) {
            if (pins[c].tx_pin == pins[other].tx_pin ||
                pins[c].tx_pin == pins[other].txen_pin ||
                pins[c].txen_pin == pins[other].tx_pin ||
                pins[c].txen_pin == pins[other].txen_pin) {
                return false;
            }
        }
    }

    for (uint c = 0; c < num_channels; c++) {
        channel_state_t *ch = &chans[c];
        if (ch->tx_pin == pins[c].tx_pin && ch->txen_pin == pins[c].txen_pin)
            continue;
        stop_channel_dma(ch);
        park_channel(ch);
        pio_sm_clear_fifos(gen_pio, ch->sm);
    }

    for (uint c = 0; c < num_channels; c++) {
        bool old_tx_reused = false;
        bool old_txen_reused = false;
        for (uint target = 0; target < num_channels; target++) {
            old_tx_reused |= chans[c].tx_pin == pins[target].tx_pin ||
                             chans[c].tx_pin == pins[target].txen_pin;
            old_txen_reused |= chans[c].txen_pin == pins[target].tx_pin ||
                               chans[c].txen_pin == pins[target].txen_pin;
        }
        if (!old_tx_reused) {
            gpio_init(chans[c].tx_pin);
            gpio_pull_up(chans[c].tx_pin);
        }
        if (!old_txen_reused) {
            gpio_init(chans[c].txen_pin);
            gpio_pull_up(chans[c].txen_pin);
        }
    }

    for (uint c = 0; c < num_channels; c++) {
        channel_state_t *ch = &chans[c];
        ch->tx_pin = pins[c].tx_pin;
        ch->txen_pin = pins[c].txen_pin;
        flexray_signal_gen_program_init(gen_pio, ch->sm, gen_program_offset,
                                        ch->tx_pin, ch->txen_pin);
        dma_channel_set_write_addr((uint)ch->dma_chan,
                                   (void *)&gen_pio->txf[ch->sm], false);
        printf("PINS ch%u: tx=%u txen=%u\n", c, ch->tx_pin, ch->txen_pin);
    }

    return true;
}

bool signal_gen_pio_test(uint channel, bool enabled)
{
    if (channel >= num_hw_channels || !chans[channel].hw_ready) return false;
    channel_state_t *ch = &chans[channel];

    signal_gen_stop();
    stop_channel_dma(ch);
    park_channel(ch);
    pio_sm_clear_fifos(gen_pio, ch->sm);
    flexray_signal_gen_program_init(gen_pio, ch->sm, gen_program_offset,
                                    ch->tx_pin, ch->txen_pin);
    if (!enabled) return true;

    pio_sm_restart(gen_pio, ch->sm);
    pio_sm_exec(gen_pio, ch->sm, pio_encode_jmp(gen_program_offset));

    // TXEN/TXD bit pairs, MSB first. This creates an obvious alternating
    // pattern on TX while TXEN remains active low.
    for (uint i = 0; i < 8; i++) {
        pio_sm_put(gen_pio, ch->sm, 0x33333333u);
    }
    pio_sm_set_enabled(gen_pio, ch->sm, true);
    return true;
}

static uint8_t maintain_ring_fill(void)
{
    uint64_t read_abs = dma_read_abs();
    int64_t ahead = (int64_t)(ring_write_abs - read_abs);
    if (ahead < 0) ahead = 0;
    uint8_t tx_mask = 0;

    if ((uint64_t)ahead >= (uint64_t)RING_LOW_WATER_CYCLES * CYCLE_BYTES)
        return 0;

    while ((uint64_t)ahead < (uint64_t)RING_PREFILL_CYCLES * CYCLE_BYTES) {
        if ((uint64_t)ahead >
            (uint64_t)(RING_BYTES - CYCLE_BYTES)) {
            break;
        }
        tx_mask |= render_next_cycle_to_ring();
        ahead = (int64_t)(ring_write_abs - read_abs);
        if (ahead < 0) ahead = 0;
    }
    return tx_mask;
}

bool signal_gen_set_slot(uint slot, uint8_t channel_mask, uint16_t frame_id,
                         uint8_t indicators, const uint8_t *payload,
                         uint16_t payload_len)
{
    if (slot >= SIGNAL_GEN_MAX_SLOTS) return false;
    if (frame_id > 2047 || payload_len > MAX_FRAME_PAYLOAD_BYTES) return false;
    if (payload_len & 1) return false;
    if (running && calc_payload_slot_duration(payload_len) > slot_duration_us)
        return false;
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

void signal_gen_clear_all_slots(void)
{
    for (uint slot = 0; slot < SIGNAL_GEN_MAX_SLOTS; slot++) {
        slots[slot].active = false;
        slots[slot].channel_mask = 0;
    }
    printf("CLEAR all slots\n");
}

bool signal_gen_update_slot_payload(uint slot,
                                    const uint8_t *payload, uint16_t payload_len)
{
    if (slot >= SIGNAL_GEN_MAX_SLOTS) return false;
    frame_slot_t *sl = &slots[slot];
    if (!sl->active) return false;
    if (payload_len > MAX_FRAME_PAYLOAD_BYTES || (payload_len & 1)) return false;
    if (running && calc_payload_slot_duration(payload_len) > slot_duration_us)
        return false;

    sl->payload_len = payload_len;
    if (payload_len > 0 && payload != NULL)
        memcpy(sl->payload, payload, payload_len);
    return true;
}

bool signal_gen_can_start(void)
{
    uint32_t slot_extent = active_slot_extent();
    uint32_t next_slot_duration_us = calc_slot_duration();
    if (slot_extent == 0 ||
        (slot_extent * next_slot_duration_us) > FLEXRAY_CYCLE_PERIOD_US) {
        printf("START rejected: slot_extent=%lu gdStaticSlot=%lu us\n",
               (unsigned long)slot_extent,
               (unsigned long)next_slot_duration_us);
        return false;
    }
    return true;
}

bool signal_gen_start(void)
{
    uint32_t slot_extent = active_slot_extent();
    uint32_t next_slot_duration_us = calc_slot_duration();
    if (!signal_gen_can_start()) return false;

    for (uint c = 0; c < num_hw_channels; c++) {
        if (!chans[c].hw_ready) continue;
        stop_channel_dma(&chans[c]);
        park_channel(&chans[c]);
        flexray_signal_gen_program_init(gen_pio, chans[c].sm, gen_program_offset,
                                        chans[c].tx_pin, chans[c].txen_pin);
        pio_sm_clear_fifos(gen_pio, chans[c].sm);
    }

    slot_duration_us = next_slot_duration_us;
    cycle_count = 0;
    tx_count = 0;
    ring_write_abs = 0;
    last_read_offset = 0;
    read_wraps = 0;
    render_cycle_count = 0;
    current_tx_mask = 0;
    memset(stream_rings, 0xFF, sizeof(stream_rings));

    for (uint i = 0; i < RING_PREFILL_CYCLES; i++)
        current_tx_mask |= render_next_cycle_to_ring();

    uint32_t dma_mask = 0;
    uint32_t sm_mask = 0;
    for (uint c = 0; c < num_hw_channels; c++) {
        if (!chans[c].hw_ready) continue;
        channel_state_t *ch = &chans[c];
        uint dma_chan = (uint)ch->dma_chan;

        pio_sm_clear_fifos(gen_pio, ch->sm);
        pio_sm_restart(gen_pio, ch->sm);
        pio_sm_exec(gen_pio, ch->sm, pio_encode_jmp(gen_program_offset));
        gen_pio->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + ch->sm);

        dma_channel_set_config(dma_chan, &ch->dma_cfg, false);
        dma_channel_set_write_addr(dma_chan, (void *)&gen_pio->txf[ch->sm], false);
#if PICO_RP2040
        // RP2040 has no self-trigger. Use a huge count so the DMA never stops;
        // ring wrap keeps the address within the buffer. Start from ring base
        // because RP2040 wraps to the aligned base, not the initial offset.
        dma_channel_set_read_addr(dma_chan, stream_rings[c], false);
        dma_channel_set_transfer_count(dma_chan, 0xFFFFFFFFu, false);
#else
        prefill_sm_fifo(ch, stream_rings[c]);
        dma_channel_set_read_addr(dma_chan, stream_rings[c] + TX_FIFO_PREFILL_BYTES, false);
        dma_channel_set_transfer_count(
            dma_chan,
            dma_encode_transfer_count_with_self_trigger(RING_WORDS),
            false);
#endif

        dma_mask |= 1u << dma_chan;
        sm_mask |= 1u << ch->sm;
    }

    if (dma_mask)
        dma_start_channel_mask(dma_mask);
    if (sm_mask)
        pio_enable_sm_mask_in_sync(gen_pio, sm_mask);

    running = true;
    printf("START @200Hz  slot_extent=%lu gdStaticSlot=%lu us\n",
           (unsigned long)slot_extent, (unsigned long)slot_duration_us);
    return true;
}

void signal_gen_stop(void)
{
    if (running) {
        running = false;
        for (uint c = 0; c < num_hw_channels; c++) {
            if (!chans[c].hw_ready) continue;
            stop_channel_dma(&chans[c]);
            park_channel(&chans[c]);
            pio_sm_clear_fifos(gen_pio, chans[c].sm);
        }
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

    uint64_t read_abs = dma_read_abs();
    uint32_t cycles_sent = (uint32_t)(read_abs / CYCLE_BYTES);
    if (cycles_sent > tx_count) {
        uint32_t delta = cycles_sent - tx_count;
        tx_count = cycles_sent;
        cycle_count = (cycle_count + delta) & 0x3Fu;
    }

    current_tx_mask |= maintain_ring_fill();
    return current_tx_mask;
}
