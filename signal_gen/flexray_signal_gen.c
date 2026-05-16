#include "flexray_signal_gen.h"
#include "flexray_frame.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/structs/pio.h"
#include "hardware/regs/dma.h"
#include "pico/time.h"
#include <string.h>
#include <stdio.h>

#include "flexray_signal_gen.pio.h"

#define CYCLE_BUFFER_COUNT 2u

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
    int dma_chan[CYCLE_BUFFER_COUNT];
    dma_channel_config dma_cfg[CYCLE_BUFFER_COUNT];
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
#define TX_FIFO_FILL_TIMEOUT_US 1000u
#define CYCLE_WORDS (CYCLE_BYTES / 4u)

static uint8_t stream_cycle_bufs[SIGNAL_GEN_MAX_CHANNELS][CYCLE_BUFFER_COUNT][MAX_CYCLE_BUF_BYTES]
    __attribute__((aligned(4)));

static volatile uint32_t dma_completed_cycles;
static uint32_t handled_completed_cycles;
static uint8_t current_tx_mask;
static uint32_t txstall_count;
static uint32_t late_buffer_count;
static uint32_t last_render_us;
static uint32_t max_render_us;
static uint full_render_banks_remaining;

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
    dma_completed_cycles = 0;
    handled_completed_cycles = 0;
    current_tx_mask = 0;
    txstall_count = 0;
    late_buffer_count = 0;
    last_render_us = 0;
    max_render_us = 0;
    full_render_banks_remaining = 0;
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

        for (uint bank = 0; bank < CYCLE_BUFFER_COUNT; bank++) {
            ch->dma_chan[bank] = (int)dma_claim_unused_channel(true);
            ch->dma_cfg[bank] = dma_channel_get_default_config((uint)ch->dma_chan[bank]);
            channel_config_set_transfer_data_size(&ch->dma_cfg[bank], DMA_SIZE_32);
            channel_config_set_bswap(&ch->dma_cfg[bank], true);
            channel_config_set_read_increment(&ch->dma_cfg[bank], true);
            channel_config_set_write_increment(&ch->dma_cfg[bank], false);
            channel_config_set_dreq(&ch->dma_cfg[bank], pio_get_dreq(pio, ch->sm, true));
            channel_config_set_high_priority(&ch->dma_cfg[bank], true);
        }
        channel_config_set_chain_to(&ch->dma_cfg[0], (uint)ch->dma_chan[1]);
        channel_config_set_chain_to(&ch->dma_cfg[1], (uint)ch->dma_chan[0]);

        for (uint bank = 0; bank < CYCLE_BUFFER_COUNT; bank++) {
            dma_channel_set_config((uint)ch->dma_chan[bank], &ch->dma_cfg[bank], false);
            dma_channel_set_write_addr((uint)ch->dma_chan[bank], (void *)&pio->txf[ch->sm], false);
        }
        ch->hw_ready = true;
        printf("Signal gen ch%u: sm=%u dma0=%d dma1=%d tx=%u txen=%u\n",
               i, ch->sm, ch->dma_chan[0], ch->dma_chan[1], pins[i].tx_pin, pins[i].txen_pin);
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
    dma_irqn_set_channel_enabled(0, chan, false);
    dma_irqn_acknowledge_channel(0, chan);
    dma_channel_hw_addr(chan)->ctrl_trig &= ~DMA_CH0_CTRL_TRIG_EN_BITS;
    dma_channel_abort(chan);
    dma_irqn_acknowledge_channel(0, chan);
}

static void stop_channel_dma(channel_state_t *ch)
{
    stop_dma_chan(ch->dma_chan[0]);
    stop_dma_chan(ch->dma_chan[1]);
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

static uint32_t render_built_slot_frame_bits(frame_slot_t *sl, uint32_t byte_count,
                                             bit_writer_t *bw, uint32_t slot_end_bit)
{
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

static uint8_t render_cycle_buffers(uint8_t cyc, uint buffer_index)
{
    uint32_t slot_bits = slot_duration_us * FLEXRAY_BITS_PER_US *
                         INTERLEAVED_BITS_PER_WIRE_BIT;
    uint32_t bit_count = CYCLE_STREAM_BITS;
    uint32_t byte_count = (bit_count + 7u) / 8u;
    uint32_t padded_bytes = (byte_count + 3u) & ~3u;
    bit_writer_t bws[SIGNAL_GEN_MAX_CHANNELS];
    uint8_t tx_mask = 0;

    for (uint c = 0; c < num_hw_channels; c++) {
        memset(stream_cycle_bufs[c][buffer_index], 0xFF, padded_bytes);
        bws[c] = (bit_writer_t) {
            .buf = stream_cycle_bufs[c][buffer_index],
            .bit_pos = 0,
            .bit_limit = bit_count,
        };
    }

    for (uint s = 0; s < SIGNAL_GEN_MAX_SLOTS; s++) {
        uint32_t slot_end_bit = (s + 1u) * slot_bits;
        frame_slot_t *sl = &slots[s];
        uint32_t frame_byte_count = 0;
        uint8_t active_mask = 0;

        if (sl->active) {
            active_mask = (uint8_t)(sl->channel_mask & ((1u << num_hw_channels) - 1u));
            if (active_mask)
                build_frame(sl, cyc, &frame_byte_count);
        }

        for (uint c = 0; c < num_hw_channels; c++) {
            if (active_mask & (1u << c)) {
                render_built_slot_frame_bits(sl, frame_byte_count, &bws[c], slot_end_bit);
                tx_mask |= (1u << c);
            } else {
                bw_skip_to(&bws[c], slot_end_bit);
            }
        }
    }

    return tx_mask;
}

static uint8_t render_cycle_to_buffer(uint8_t cyc, uint buffer_index)
{
    absolute_time_t start = get_absolute_time();
    uint8_t tx_mask = render_cycle_buffers(cyc, buffer_index);
    last_render_us = (uint32_t)absolute_time_diff_us(start, get_absolute_time());
    if (last_render_us > max_render_us)
        max_render_us = last_render_us;
    return tx_mask;
}

static inline void patch_interleaved_bit(uint8_t *buf, uint32_t bit_pos, bool bit)
{
    uint8_t mask = (uint8_t)(0x80u >> (bit_pos & 7u));
    if (bit)
        buf[bit_pos >> 3] |= mask;
    else
        buf[bit_pos >> 3] &= (uint8_t)~mask;
}

static void patch_encoded_byte(uint8_t *buf, uint32_t frame_start_bit,
                               uint32_t byte_index, uint8_t value)
{
    uint32_t data_bit = frame_start_bit + 10u + (byte_index * 20u) + 1u;
    for (int bit = 7; bit >= 0; bit--, data_bit += 2u)
        patch_interleaved_bit(buf, data_bit, (value >> bit) & 1u);
}

static uint8_t patch_cycle_to_buffer(uint8_t cyc, uint buffer_index, uint source_index)
{
    absolute_time_t start = get_absolute_time();
    uint32_t slot_bits = slot_duration_us * FLEXRAY_BITS_PER_US *
                         INTERLEAVED_BITS_PER_WIRE_BIT;
    uint8_t tx_mask = 0;

    for (uint c = 0; c < num_hw_channels; c++)
        memcpy(stream_cycle_bufs[c][buffer_index],
               stream_cycle_bufs[c][source_index],
               CYCLE_BYTES);

    for (uint s = 0; s < SIGNAL_GEN_MAX_SLOTS; s++) {
        frame_slot_t *sl = &slots[s];
        uint8_t active_mask = sl->active
                              ? (uint8_t)(sl->channel_mask & ((1u << num_hw_channels) - 1u))
                              : 0;
        if (!active_mask) continue;

        uint32_t frame_byte_count;
        build_frame(sl, cyc, &frame_byte_count);
        uint32_t frame_start_bit = s * slot_bits;
        uint32_t crc_index = 5u + sl->payload_len;
        uint32_t patch_indices[8] = {
            0u, 1u, 2u, 3u, 4u,
            crc_index, crc_index + 1u, crc_index + 2u,
        };
        (void)frame_byte_count;

        for (uint c = 0; c < num_hw_channels; c++) {
            if (!(active_mask & (1u << c))) continue;
            for (uint i = 0; i < 8; i++)
                patch_encoded_byte(stream_cycle_bufs[c][buffer_index],
                                   frame_start_bit,
                                   patch_indices[i],
                                   sl->frame_buf[patch_indices[i]]);
            tx_mask |= (1u << c);
        }
    }

    last_render_us = (uint32_t)absolute_time_diff_us(start, get_absolute_time());
    if (last_render_us > max_render_us)
        max_render_us = last_render_us;
    return tx_mask;
}

static void configure_dma_bank(uint bank)
{
    for (uint c = 0; c < num_hw_channels; c++) {
        if (!chans[c].hw_ready) continue;
        uint dma_chan = (uint)chans[c].dma_chan[bank];
        dma_channel_set_config(dma_chan, &chans[c].dma_cfg[bank], false);
        dma_channel_set_write_addr(dma_chan, (void *)&gen_pio->txf[chans[c].sm], false);
        dma_channel_set_read_addr(dma_chan, stream_cycle_bufs[c][bank], false);
        dma_channel_set_transfer_count(dma_chan, CYCLE_WORDS, false);
    }
}

static void dma_cycle_irq_handler(void)
{
    if (num_hw_channels == 0 || !chans[0].hw_ready) return;
    for (uint bank = 0; bank < CYCLE_BUFFER_COUNT; bank++) {
        uint dma_chan = (uint)chans[0].dma_chan[bank];
        if (dma_irqn_get_channel_status(0, dma_chan)) {
            dma_irqn_acknowledge_channel(0, dma_chan);
            dma_completed_cycles++;
        }
    }
}

static void ensure_dma_cycle_irq(void)
{
    static bool installed;
    if (installed) return;
    irq_set_exclusive_handler(DMA_IRQ_NUM(0), dma_cycle_irq_handler);
    irq_set_priority(DMA_IRQ_NUM(0), 0);
    irq_set_enabled(DMA_IRQ_NUM(0), true);
    installed = true;
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
    for (uint bank = 0; bank < CYCLE_BUFFER_COUNT; bank++)
        dma_channel_set_write_addr((uint)ch->dma_chan[bank],
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
        for (uint bank = 0; bank < CYCLE_BUFFER_COUNT; bank++)
            dma_channel_set_write_addr((uint)ch->dma_chan[bank],
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

uint32_t signal_gen_stall_count(void)
{
    return txstall_count;
}

void signal_gen_diag(signal_gen_diag_t *diag)
{
    if (diag == NULL) return;
    diag->txstall_count = txstall_count;
    diag->late_buffer_count = late_buffer_count;
    diag->completed_cycles = dma_completed_cycles;
    diag->handled_cycles = handled_completed_cycles;
    diag->last_render_us = last_render_us;
    diag->max_render_us = max_render_us;
}

static uint8_t maintain_cycle_buffers(void)
{
    uint8_t tx_mask = 0;
    uint32_t completed = dma_completed_cycles;
    uint32_t lag = completed - handled_completed_cycles;

    if (lag == 0)
        return 0;

    if (lag > 1u)
        late_buffer_count += lag - 1u;

    while (handled_completed_cycles < completed) {
        uint completed_bank = handled_completed_cycles & 1u;
        uint8_t render_cycle = (uint8_t)((handled_completed_cycles + 2u) & 0x3Fu);
        if (full_render_banks_remaining > 0) {
            tx_mask |= render_cycle_to_buffer(render_cycle, completed_bank);
            full_render_banks_remaining--;
        } else {
            tx_mask |= patch_cycle_to_buffer(render_cycle, completed_bank, completed_bank ^ 1u);
        }
        configure_dma_bank(completed_bank);
        handled_completed_cycles++;
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
    if (running)
        full_render_banks_remaining = CYCLE_BUFFER_COUNT;

    printf("SET slot%u: mask=0x%02x id=0x%03x ind=0x%02x len=%u\n",
           slot, channel_mask, frame_id, indicators, payload_len);
    return true;
}

void signal_gen_clear_slot(uint slot)
{
    if (slot >= SIGNAL_GEN_MAX_SLOTS) return;
    slots[slot].active = false;
    slots[slot].channel_mask = 0;
    if (running)
        full_render_banks_remaining = CYCLE_BUFFER_COUNT;
    printf("CLEAR slot%u\n", slot);
}

void signal_gen_clear_all_slots(void)
{
    for (uint slot = 0; slot < SIGNAL_GEN_MAX_SLOTS; slot++) {
        slots[slot].active = false;
        slots[slot].channel_mask = 0;
    }
    if (running)
        full_render_banks_remaining = CYCLE_BUFFER_COUNT;
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
    if (running)
        full_render_banks_remaining = CYCLE_BUFFER_COUNT;
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
    dma_completed_cycles = 0;
    handled_completed_cycles = 0;
    current_tx_mask = 0;
    txstall_count = 0;
    late_buffer_count = 0;
    last_render_us = 0;
    max_render_us = 0;
    memset(stream_cycle_bufs, 0xFF, sizeof(stream_cycle_bufs));

    current_tx_mask |= render_cycle_to_buffer(0, 0);
    current_tx_mask |= render_cycle_to_buffer(1, 1);
    configure_dma_bank(0);
    configure_dma_bank(1);
    ensure_dma_cycle_irq();

    uint32_t dma_mask = 0;
    uint32_t sm_mask = 0;
    for (uint c = 0; c < num_hw_channels; c++) {
        if (!chans[c].hw_ready) continue;
        channel_state_t *ch = &chans[c];

        pio_sm_clear_fifos(gen_pio, ch->sm);
        pio_sm_restart(gen_pio, ch->sm);
        pio_sm_exec(gen_pio, ch->sm, pio_encode_jmp(gen_program_offset));
        gen_pio->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + ch->sm);

        if (c == 0) {
            for (uint bank = 0; bank < CYCLE_BUFFER_COUNT; bank++) {
                uint dma_chan = (uint)ch->dma_chan[bank];
                dma_irqn_acknowledge_channel(0, dma_chan);
                dma_irqn_set_channel_enabled(0, dma_chan, true);
            }
        }

        dma_mask |= 1u << (uint)ch->dma_chan[0];
        sm_mask |= 1u << ch->sm;
    }

    if (dma_mask)
        dma_start_channel_mask(dma_mask);
    absolute_time_t fifo_deadline = make_timeout_time_us(TX_FIFO_FILL_TIMEOUT_US);
    bool fifo_ready;
    do {
        fifo_ready = true;
        for (uint c = 0; c < num_hw_channels; c++) {
            if (chans[c].hw_ready)
                fifo_ready &= pio_sm_is_tx_fifo_full(gen_pio, chans[c].sm);
        }
    } while (!fifo_ready && absolute_time_diff_us(get_absolute_time(), fifo_deadline) > 0);
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

    uint32_t completed = dma_completed_cycles;
    if (completed > tx_count) {
        uint32_t delta = completed - tx_count;
        tx_count = completed;
        cycle_count = (cycle_count + delta) & 0x3Fu;
    }

    current_tx_mask |= maintain_cycle_buffers();
    uint32_t stall_mask = 0;
    for (uint c = 0; c < num_hw_channels; c++) {
        if (chans[c].hw_ready)
            stall_mask |= 1u << (PIO_FDEBUG_TXSTALL_LSB + chans[c].sm);
    }
    uint32_t stalls = gen_pio->fdebug & stall_mask;
    if (stalls) {
        txstall_count += __builtin_popcount(stalls);
        gen_pio->fdebug = stalls;
    }
    return current_tx_mask;
}
