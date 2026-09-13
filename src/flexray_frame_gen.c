#include "flexray_frame_gen.h"
#include "flexray_frame_gen_rules.h"
#include "board_config.h"
#include "flexray_frame.h"
#include "flexray_slot_schedule.h"
#include "flexray_frame_gen_packet.h"
#include "flexray_fss_timing.pio.h"
#include "flexray_frame_gen_output.pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "pico/time.h"
#include <string.h>

#define CAPTURE_SM 3u
#define PACE_SM 1u
#define PHASE_SM 3u
#define INDUCER_SM 0u
#define LOCAL_SM 2u
#define CLOCK_HZ 150000000u
#define MEASURE_BITS 12u
#define MEASURE_WORDS (1u << (MEASURE_BITS - 2u))
#define MEASURE_TRANSFERS 0x0fffffffu
#define MEASURE_MARGIN 8u
#define PACE_OVERHEAD 9u
#define TAIL_GUARDS 64u
#define PERIOD_WORDS (FLEXRAY_SLOT_MAX_SLOTS + TAIL_GUARDS + 2u)
#define PULSE_WORDS FLEXRAY_FRAME_GEN_INDUCER_WORDS

static const flexray_frame_gen_config_t default_config = {
    .target_id = FLEXRAY_FRAME_GEN_FIRST_ID, .second_target_id = FLEXRAY_FRAME_GEN_SECOND_ID,
    .static_max_id = FLEXRAY_FRAME_GEN_STATIC_MAX_ID,
    .payload_bytes = FLEXRAY_FRAME_GEN_PAYLOAD_BYTES, .learn_samples = 12,
    .cycle_mask = FLEXRAY_FRAME_GEN_CYCLE_REP - 1u, .cycle_base = FLEXRAY_FRAME_GEN_CYCLE_BASE,
    .tss_bits = 8, .tolerance_cycles = 32,
};
static flexray_frame_gen_config_t config;
static flexray_slot_schedule_t schedule;
static volatile flexray_frame_gen_status_t stats;
static volatile flexray_frame_gen_diagnostics_t diagnostics;
static volatile bool ready, command_pending;
static bool runtime, seeded, pacing, enabled, phase_armed;
static volatile bool reacquire_pending, recovering;
static uint32_t last_sync_us;
static volatile uint32_t sync_generation;
static uint16_t reference_id;
static uint8_t base_cycle, phase_cycle;
static uint32_t last_beat;
static uint64_t total_beats;
static bool have_beat, pace_fresh;
static uint capture_offset, timing_offset, phase_offset, inducer_offset;
// One sampling DMA: acquisition on PACE_SM, then phase bursts on PHASE_SM.
static int sample_dma = -1;
static uint32_t phase_words;
static uint pace_dma, pace_reload_dma, pace_reset_dma, pulse_dma, packet_dma;
static uint32_t pace_base_addr;
static volatile uint32_t measure_ring[MEASURE_WORDS] __attribute__((aligned(1u << MEASURE_BITS)));
static uint32_t previous_measure_stamp;
static bool have_measure_stamp;
static volatile uint32_t periods[PERIOD_WORDS];
static uint gap_index;
static uint32_t nominal_tail;
static uint32_t pulses[PULSE_WORDS];
typedef struct {
    flexray_frame_gen_template_t null_template, data_templates[2];
    uint16_t template_len; // Pending single-use host frame; zero selects null.
    uint8_t active_bank;
} reservation_t;
static reservation_t reservations[FLEXRAY_FRAME_GEN_TARGETS];
// Core0 builds an inactive bank then publishes one mailbox per target. Core1
// applies only outside prepared DMA ownership, and clears pending last. Thus a
// single bulk/datagram may submit both C and D without a shared mailbox clash.
static volatile bool payload_pending[FLEXRAY_FRAME_GEN_TARGETS];
static uint8_t payload_bank[FLEXRAY_FRAME_GEN_TARGETS];
static uint32_t payload_generation[FLEXRAY_FRAME_GEN_TARGETS];
// Selected in the preceding slot and immutable through frame end. Commands
// use the existing mailbox/slot ownership; no lock is needed for this pointer.
static const flexray_frame_gen_packet_t *prepared_packet;
static uint32_t prepared_beat;

typedef struct {
    uint8_t op, slot, bank[FLEXRAY_FRAME_GEN_TARGETS];
    bool enable;
    flexray_frame_gen_config_t config;
    uint16_t frame_len;
    uint32_t generation;
} command_t;
static command_t mailbox;

static bool local_active(void)
{
    return prepared_packet && (!gpio_get(FLEXRAY_FRAME_GEN_OWNERSHIP_PIN) ||
        pio_interrupt_get(pio0, flexray_frame_gen_inducer_DONE_IRQ));
}

static uint32_t minimum_slot(const flexray_frame_gen_config_t *c)
{
    // Original RX timeout plus its original 11-bit idle requalification.
    // This is a fit check only; neither learning nor pace uses frame length.
    return ((uint32_t)c->tss_bits + 1u + (c->payload_bytes + 8u) * 10u + 2u) * 15u + 270u;
}

static uint32_t local_tss_cycles(uint8_t bits)
{
    // RP2350 bench: the unchanged WAIT PIN / two-clock JMP PIN path
    // shortens odd TSS by one clock. Keep its FSS deadline independent
    // of that parity; see docs/frame-gen-design.md.
    return (uint32_t)bits * 15u - (bits & 1u);
}

static uint16_t target_id(const flexray_frame_gen_config_t *c, uint slot)
{
    return slot ? c->second_target_id : c->target_id;
}

static bool config_valid(const flexray_frame_gen_config_t *c)
{
    int64_t expected = (int64_t)local_tss_cycles(c->tss_bits) - 19 - c->phase_cycles;
    if (c->target_id < 2u || c->target_id > c->static_max_id ||
        (c->second_target_id && (c->second_target_id < 2u ||
         c->second_target_id > c->static_max_id || c->second_target_id == c->target_id)) ||
        c->static_max_id < 2 || c->static_max_id > FLEXRAY_SLOT_MAX_SLOTS ||
        c->payload_bytes > 254 || (c->payload_bytes & 1u) || c->rx_channel > 1 ||
        c->cycle_mask > 63 || (c->cycle_base & ~c->cycle_mask) ||
        c->tss_bits < 6 || c->tss_bits > 15 || c->learn_samples < 8 || c->learn_samples > 32 ||
        c->reserved || !c->tolerance_cycles || c->tolerance_cycles > 32 ||
        expected <= (int)c->tolerance_cycles || expected + 15 + (int)c->tolerance_cycles >= 256 ||
        (c->slot_cycles && (c->slot_cycles < minimum_slot(c) || c->slot_cycles > 1500000u)) ||
        c->cycle_cycles > 0x0fffffffu ||
        (c->cycle_cycles && c->cycle_cycles < (uint64_t)minimum_slot(c) * c->static_max_id + FLEXRAY_SLOT_TAIL_OVERHEAD) ||
        (c->slot_cycles && c->cycle_cycles &&
         c->cycle_cycles < (uint64_t)c->slot_cycles * c->static_max_id + FLEXRAY_SLOT_TAIL_OVERHEAD)) return false;
    return true;
}

static void init_schedule(void)
{
    flexray_slot_schedule_config_t c = {
        .static_max_id = config.static_max_id, .learn_samples = config.learn_samples,
        .min_slot_cycles = minimum_slot(&config), .max_slot_cycles = 1500000u,
        .tolerance_cycles = config.tolerance_cycles,
        .slot_cycles = config.slot_cycles, .cycle_cycles = config.cycle_cycles,
    };
    (void)flexray_slot_schedule_init(&schedule, &c);
}

static void generate_static_header(uint8_t *raw, const flexray_frame_gen_config_t *c, uint16_t id)
{
    raw[0] = (uint8_t)(id >> 8); raw[1] = (uint8_t)id;
    raw[2] = (uint8_t)c->payload_bytes;
    uint16_t crc = calculate_flexray_header_crc(raw);
    raw[2] |= (uint8_t)(crc >> 10); raw[3] = (uint8_t)(crc >> 2); raw[4] = (uint8_t)(crc << 6);
}

static void generate_template(flexray_frame_gen_template_t *tpl, const flexray_frame_gen_config_t *c,
                           uint16_t id, const uint8_t *frame)
{
    uint8_t raw[262] = {0};
    uint16_t len = c->payload_bytes + 8u;
    if (!frame) generate_static_header(raw, c, id);
    flexray_frame_gen_template_generate(tpl, frame ? frame : raw, len, !frame);
}

// Called by every real-header and pace IRQ. Flash cache misses here can
// delay frame-end service into the next header and expire the phase window.
static void __no_inline_not_in_flash_func(publish)(void)
{
    stats.flags = FLEXRAY_FRAME_GEN_CONFIGURED | (reservations[0].template_len ? FLEXRAY_FRAME_GEN_TEMPLATE_VALID : 0u) |
        (reservations[1].template_len ? FLEXRAY_FRAME_GEN_SECOND_TEMPLATE_VALID : 0u) |
        (enabled ? FLEXRAY_FRAME_GEN_ENABLED : 0u) | (schedule.locked ? FLEXRAY_FRAME_GEN_LOCKED : 0u) |
        (prepared_packet ? FLEXRAY_FRAME_GEN_TX_PENDING : 0u) | (pacing ? FLEXRAY_FRAME_GEN_PACING : 0u) |
        ((recovering || reacquire_pending) ? FLEXRAY_FRAME_GEN_RESYNCING : 0u);
    stats.slot_cycles = schedule.slot_cycles; stats.cycle_cycles = schedule.cycle_cycles;
    stats.samples = schedule.samples; stats.cycle_samples = schedule.cycle_samples_seen;
    stats.rejected = schedule.rejected;
    stats.template_len = reservations[0].template_len;
    stats.second_template_len = reservations[1].template_len;
    stats.reference_id = reference_id; stats.cycle_shape_cycles = schedule.cycle_shape;
}

static void load_register(PIO pio, uint sm, uint reg, uint32_t value)
{
    pio_sm_put(pio, sm, value);
    pio_sm_exec(pio, sm, pio_encode_pull(false, true));
    pio_sm_exec(pio, sm, pio_encode_out(reg, 32));
}

static void stop_dma(uint32_t channels)
{
    // RP2350-E5: disable every member before ABORT can fire CHAIN_TO.
    for (uint n = 0; n < NUM_DMA_CHANNELS; ++n)
        if (channels & (1u << n)) hw_clear_bits(&dma_channel_hw_addr(n)->ctrl_trig, DMA_CH0_CTRL_TRIG_EN_BITS);
    for (uint n = 0; n < NUM_DMA_CHANNELS; ++n)
        if (channels & (1u << n)) dma_channel_abort(n);
}

static dma_channel_config pio_dma_config(uint channel, PIO pio, uint sm, bool tx)
{
    dma_channel_config d = dma_channel_get_default_config(channel);
    channel_config_set_transfer_data_size(&d, DMA_SIZE_32);
    channel_config_set_read_increment(&d, tx);
    channel_config_set_write_increment(&d, !tx);
    channel_config_set_dreq(&d, pio_get_dreq(pio, sm, tx));
    channel_config_set_high_priority(&d, true);
    return d;
}

static void start_pace_dma(uint count)
{
    // Replay the period table; reset the consumed correction before reloading.
    // READ_ADDR_TRIG reloads TRANS_COUNT (RP2350 DMA TRANS_COUNT semantics).
    pace_base_addr = (uint32_t)(uintptr_t)periods;
    dma_channel_config d = pio_dma_config(pace_dma, pio0, PACE_SM, true);
    channel_config_set_chain_to(&d, pace_reset_dma);
    dma_channel_configure(pace_dma, &d, &pio0->txf[PACE_SM], periods, count, false);

    d = dma_channel_get_default_config(pace_reset_dma);
    channel_config_set_transfer_data_size(&d, DMA_SIZE_32);
    channel_config_set_read_increment(&d, false);
    channel_config_set_write_increment(&d, false);
    channel_config_set_chain_to(&d, pace_reload_dma);
    dma_channel_configure(pace_reset_dma, &d, &periods[gap_index], &nominal_tail, 1u, false);

    d = dma_channel_get_default_config(pace_reload_dma);
    channel_config_set_transfer_data_size(&d, DMA_SIZE_32);
    channel_config_set_read_increment(&d, false);
    channel_config_set_write_increment(&d, false);
    dma_channel_configure(pace_reload_dma, &d, &dma_channel_hw_addr(pace_dma)->al3_read_addr_trig,
                          &pace_base_addr, 1u, false);
    dma_channel_start(pace_dma);
}

static void reset_capture(void)
{
    pio_sm_set_enabled(pio2, CAPTURE_SM, false);
    pio_sm_config c = flexray_fss_capture_program_get_default_config(capture_offset);
    sm_config_set_in_pins(&c, config.rx_channel ? TXEN_FR_2_PIN : TXEN_FR_1_PIN);
    sm_config_set_jmp_pin(&c, config.rx_channel ? RXD_FR_2_PIN : RXD_FR_1_PIN);
    sm_config_set_sideset_pins(&c, FLEXRAY_FSS_MARKER_PIN);
    sm_config_set_clkdiv(&c, 1.0f);
    pio_sm_init(pio2, CAPTURE_SM, capture_offset, &c);
    pio2->fdebug = 1u << (PIO_FDEBUG_RXSTALL_LSB + CAPTURE_SM);
    pio_sm_set_enabled(pio2, CAPTURE_SM, true);
}

static void arm_phase(void)
{
    pio_sm_set_enabled(pio0, PHASE_SM, false);
    stop_dma(1u << (uint)sample_dma);
    pio_interrupt_clear(pio0, flexray_fss_phase_sampler_PHASE_IRQ);
    pio_sm_config c = flexray_fss_phase_sampler_program_get_default_config(phase_offset);
    sm_config_set_in_pins(&c, FLEXRAY_FSS_MARKER_PIN);
    sm_config_set_clkdiv(&c, 1.0f);
    pio_sm_init(pio0, PHASE_SM, phase_offset, &c);
    // A finite burst both captures the first 256 clocks and bounds the
    // lifetime of this cycle's phase result. It ends BEFORE the tail shaping
    // word can enter the pace FIFO. Late headers cannot affect a later cycle.
    phase_words = (schedule.slot_cycles * 3u / 4u - 96u) / 32u;
    dma_channel_config d = pio_dma_config((uint)sample_dma, pio0, PHASE_SM, false);
    channel_config_set_ring(&d, true, MEASURE_BITS);
    dma_channel_configure((uint)sample_dma, &d, measure_ring, &pio0->rxf[PHASE_SM], phase_words, true);
    pio0->fdebug = 1u << (PIO_FDEBUG_RXSTALL_LSB + PHASE_SM);
    pio_sm_set_enabled(pio0, PHASE_SM, true);
    phase_armed = true;
}

static void reset_output(void)
{
    pio_sm_set_enabled(pio0, INDUCER_SM, false);
    if (!gpio_get(FLEXRAY_FRAME_GEN_OWNERSHIP_PIN)) {
        pio_interrupt_clear(pio1, 7u);
        pio_sm_set_pins_with_mask(pio0, INDUCER_SM, 1u << FLEXRAY_FRAME_GEN_OWNERSHIP_PIN,
                                  1u << FLEXRAY_FRAME_GEN_OWNERSHIP_PIN);
    }
    stop_dma((1u << pulse_dma) | (1u << packet_dma));
    flexray_frame_gen_forwarder_local_config(false);
    pio_sm_clear_fifos(pio2, LOCAL_SM);
    pio_interrupt_clear(pio0, flexray_frame_gen_inducer_DONE_IRQ);
    pio_interrupt_clear(pio0, flexray_frame_gen_inducer_TRIGGER_IRQ);
    // Always consume pace with an empty FIFO, even while disabled. Preparing
    // a waveform in slot N cannot consume a stale IRQ from slot N itself.
    pio_sm_config c = flexray_frame_gen_inducer_program_get_default_config(inducer_offset);
    sm_config_set_out_pins(&c, FLEXRAY_INTERNAL_PIN, 1u);
    sm_config_set_sideset_pins(&c, FLEXRAY_FRAME_GEN_OWNERSHIP_PIN);
    sm_config_set_clkdiv(&c, 1.0f);
    pio_sm_init(pio0, INDUCER_SM, inducer_offset, &c);
    pio_sm_set_enabled(pio0, INDUCER_SM, true);
    // Configure two finite, unchained DMAs once. Preparation only writes their
    // READ_ADDR_TRIG registers, which reload the configured transfer counts.
    uint32_t pulse_count = flexray_frame_gen_inducer_generate(pulses, config.payload_bytes + 8u, config.tss_bits);
    dma_channel_config d = pio_dma_config(packet_dma, pio2, LOCAL_SM, true);
    channel_config_set_bswap(&d, true);
    dma_channel_configure(packet_dma, &d, &pio2->txf[LOCAL_SM], &reservations[0].null_template.packet,
                          1u + (config.payload_bytes + 11u) / 4u, false);
    d = pio_dma_config(pulse_dma, pio0, INDUCER_SM, true);
    dma_channel_configure(pulse_dma, &d, &pio0->txf[INDUCER_SM], pulses, pulse_count, false);
    prepared_packet = NULL;
    // These original programs already return to idle after each frame. Start
    // them once when enabling frame generation; do not reset them between adjacent slots.
    if (enabled) {
        flexray_frame_gen_forwarder_local_config(true);
    }
}

static void reset_timing(void)
{
    reset_output();
    pio_sm_set_enabled(pio2, CAPTURE_SM, false);
    pio_sm_set_enabled(pio0, PACE_SM, false);
    pio_sm_set_enabled(pio0, PHASE_SM, false);
    stop_dma((1u << pace_dma) | (1u << pace_reload_dma) | (1u << pace_reset_dma));
    if (sample_dma >= 0) {
        stop_dma(1u << (uint)sample_dma);
        dma_channel_unclaim((uint)sample_dma);
        if (runtime) pio_remove_program(pio0, &flexray_fss_phase_sampler_program, phase_offset);
        pio_remove_program(pio0, runtime ? &flexray_slot_pace_program : &flexray_fss_sampler_program,
                           timing_offset);
    }
    sample_dma = (int)dma_claim_unused_channel(true);
    runtime = schedule.locked;
    seeded = pacing = have_beat = phase_armed = pace_fresh = false;
    pio0->fdebug = 1u << (PIO_FDEBUG_RXSTALL_LSB + PACE_SM);
    reference_id = 0; total_beats = 0;
    pio_interrupt_clear(pio0, flexray_fss_capture_FSS_IRQ);
    pio_interrupt_clear(pio0, flexray_slot_pace_PACE_IRQ);
    if (runtime) {
        timing_offset = pio_add_program(pio0, &flexray_slot_pace_program);
        phase_offset = pio_add_program(pio0, &flexray_fss_phase_sampler_program);
        pio_sm_config c = flexray_slot_pace_program_get_default_config(timing_offset);
        sm_config_set_clkdiv(&c, 1.0f);
        pio_sm_init(pio0, PACE_SM, timing_offset, &c);
        // First beat is one CYCLE after hardware FSS, leaving its header time
        // to label/rotate the table without starting or shifting this clock.
        uint32_t x = (uint32_t)((int64_t)schedule.cycle_cycles - local_tss_cycles(config.tss_bits) -
                                15u + config.phase_cycles);
        load_register(pio0, PACE_SM, pio_x, x);
        load_register(pio0, PACE_SM, pio_y, UINT32_MAX);
        pio_sm_set_enabled(pio0, PACE_SM, true);
        arm_phase();
    } else {
        timing_offset = pio_add_program(pio0, &flexray_fss_sampler_program);
        pio_sm_config c = flexray_fss_sampler_program_get_default_config(timing_offset);
        sm_config_set_in_pins(&c, FLEXRAY_FSS_MARKER_PIN);
        sm_config_set_clkdiv(&c, 1.0f);
        pio_sm_init(pio0, PACE_SM, timing_offset, &c);
        dma_channel_config d = pio_dma_config((uint)sample_dma, pio0, PACE_SM, false);
        channel_config_set_ring(&d, true, MEASURE_BITS);
        dma_channel_configure((uint)sample_dma, &d, measure_ring, &pio0->rxf[PACE_SM], MEASURE_TRANSFERS, true);
        have_measure_stamp = false;
        pio0->fdebug = 1u << (PIO_FDEBUG_RXSTALL_LSB + PACE_SM);
        pio_sm_set_enabled(pio0, PACE_SM, true);
    }
    reset_capture();
}

// Before a reference header is bound, rejecting a non-candidate must only
// restart the FSS wait. Reallocating PIO programs/DMA in every such header can
// overrun intervening real header/frame-end callbacks and break their pairing.
static void restart_bootstrap(void)
{
    pio_sm_set_enabled(pio2, CAPTURE_SM, false);
    pio_sm_set_enabled(pio0, PACE_SM, false);
    pio_interrupt_clear(pio0, flexray_fss_capture_FSS_IRQ);
    pio_interrupt_clear(pio0, flexray_slot_pace_PACE_IRQ);
    pio_sm_config c = flexray_slot_pace_program_get_default_config(timing_offset);
    sm_config_set_clkdiv(&c, 1.0f);
    pio_sm_init(pio0, PACE_SM, timing_offset, &c);
    uint32_t x = (uint32_t)((int64_t)schedule.cycle_cycles - local_tss_cycles(config.tss_bits) -
                            15u + config.phase_cycles);
    load_register(pio0, PACE_SM, pio_x, x);
    load_register(pio0, PACE_SM, pio_y, UINT32_MAX);
    have_beat = pace_fresh = pacing = false;
    pio_sm_set_enabled(pio0, PACE_SM, true);
    arm_phase();
    reset_capture();
}

static bool service_reacquisition(void)
{
    if (!reacquire_pending) return false;
    // Finish an already active local frame with its existing TXEN lifecycle.
    // No later reservation may prepare while this flag is set.
    if (local_active()) return true;
    init_schedule();
    // Even diagnostic fixed-period configurations must observe the new bus
    // after loss. Never attach a restarted source to the old learned periods.
    schedule.slot_locked = schedule.cycle_locked = schedule.locked = false;
    schedule.slot_cycles = schedule.cycle_cycles = 0;
    reset_timing();
    reacquire_pending = false;
    publish();
    return true;
}

static void lose_sync(uint32_t reason)
{
    if (!reacquire_pending) {
        recovering = reacquire_pending = true;
        __dmb();
        ++sync_generation;
        __dmb();
        ++stats.sync_losses; stats.sync_loss_reason = reason;
        for (uint i = 0; i < FLEXRAY_FRAME_GEN_TARGETS; ++i) reservations[i].template_len = 0;
        // Stop future hardware beats before inspecting ACTIVE. A beat already
        // crossing from PIO0 to PIO2 may still be in WAIT/status/branch/ACTIVE;
        // let that short pipeline settle without pausing the framing SM. This
        // fault-only wait cannot move an active frame's FSS or truncate TSS.
        pio_sm_set_enabled(pio0, PACE_SM, false);
        pio_interrupt_clear(pio0, flexray_frame_gen_inducer_TRIGGER_IRQ);
        busy_wait_us_32(1u);
        pacing = false;
        publish(); // report loss even while waiting for an already active frame
    }
    (void)service_reacquisition();
}

static bool sync_timeout(void)
{
    // CPU time is only a watchdog; it never schedules the transmitted FSS.
    // Allow short missing references, then release the bus and reacquire.
    uint32_t timeout_us = (schedule.cycle_cycles * 3u + 149u) / 150u;
    if (seeded && pacing && (uint32_t)(time_us_32() - last_sync_us) >= timeout_us) {
        lose_sync(1u); return true;
    }
    return service_reacquisition();
}

static bool measured_stamp(uint32_t *stamp)
{
    uint32_t remaining = dma_channel_hw_addr((uint)sample_dma)->transfer_count;
    if ((pio0->fdebug & (1u << (PIO_FDEBUG_RXSTALL_LSB + PACE_SM))) || !remaining) {
        schedule.have_previous = false;
        reset_timing();
        return false;
    }
    uint32_t done = MEASURE_TRANSFERS - remaining;
    __dmb();
    if (done <= MEASURE_MARGIN) return false;
    uint32_t end = done - MEASURE_MARGIN;
    uint32_t span = end < MEASURE_WORDS - MEASURE_MARGIN - 2u ? end : MEASURE_WORDS - MEASURE_MARGIN - 2u;
    for (uint32_t back = 0; back < span; ++back) {
        uint32_t index = end - 1u - back;
        uint32_t bits = measure_ring[index & (MEASURE_WORDS - 1u)];
        if (!bits) continue;
        uint32_t value = 0u - (index * 32u + 31u - (uint32_t)__builtin_ctz(bits));
        __dmb();
        uint32_t now = MEASURE_TRANSFERS - dma_channel_hw_addr((uint)sample_dma)->transfer_count;
        if ((pio0->fdebug & (1u << (PIO_FDEBUG_RXSTALL_LSB + PACE_SM))) ||
            now - index >= MEASURE_WORDS || (have_measure_stamp && value == previous_measure_stamp)) return false;
        previous_measure_stamp = value; have_measure_stamp = true; *stamp = value;
        return true;
    }
    return false;
}

static bool take_fss(uint32_t *stamp)
{
    uint32_t stall = 1u << (PIO_FDEBUG_RXSTALL_LSB + CAPTURE_SM);
    bool lost = (pio2->fdebug & stall) != 0;
    uint count = pio_sm_get_rx_fifo_level(pio2, CAPTURE_SM);
    for (uint i = 0; i < count; ++i) (void)pio_sm_get(pio2, CAPTURE_SM);
    if (lost) pio2->fdebug = stall;
    if (lost || count != 1u || (!runtime && !measured_stamp(stamp))) {
        ++stats.lost_stamps; schedule.have_previous = false; return false;
    }
    ++stats.captured;
    return true;
}

static void __time_critical_func(consume_pace)(void)
{
    uint32_t stall = 1u << (PIO_FDEBUG_RXSTALL_LSB + PACE_SM);
    bool dropped = (pio0->fdebug & stall) != 0;
    bool received = false;
    while (!pio_sm_is_rx_fifo_empty(pio0, PACE_SM)) {
        uint32_t beat = ~pio_sm_get(pio0, PACE_SM);
        received = true;
        if (!have_beat) { total_beats = beat; have_beat = true; }
        else {
            uint32_t delta = beat - last_beat;
            if (delta > 1u) stats.pace_skipped += delta - 1u;
            total_beats += delta;
        }
        last_beat = beat;
    }
    if (dropped || (pio0->fdebug & stall)) {
        pio0->fdebug = stall;
        pace_fresh = false; // resume CPU work on the next fresh hardware beat
    } else if (received) pace_fresh = true;
    if (!received || !seeded) return;
    uint64_t position = reference_id - 1u + total_beats;
    stats.current_slot = (uint32_t)(position % config.static_max_id) + 1u;
    stats.current_cycle = (uint32_t)(base_cycle + position / config.static_max_id) & 63u;
    stats.pace_ticks = (uint32_t)total_beats + 1u;
    pacing = true;
}

// Called only after a fault has already been detected, before cleanup destroys
// the evidence. Normal prepare/launch has no diagnostic timestamp or sampling.
static void __time_critical_func(record_miss)(uint reason)
{
    ++diagnostics.sequence; __dmb();
    ++diagnostics.counts[reason - 1u];
    diagnostics.reason = reason;
    const uint8_t *raw = prepared_packet ? (const uint8_t *)(prepared_packet->words + 1) : NULL;
    diagnostics.fid = raw ? ((raw[0] & 7u) << 8) | raw[1] : 0u;
    diagnostics.cycle = raw ? raw[4] & 63u : stats.current_cycle;
    diagnostics.slot = stats.current_slot;
    diagnostics.prepared_beat = prepared_beat; diagnostics.current_beat = last_beat;
    diagnostics.packet_remaining = dma_channel_hw_addr(packet_dma)->transfer_count;
    diagnostics.pulse_remaining = dma_channel_hw_addr(pulse_dma)->transfer_count;
    diagnostics.inducer_pc = pio_sm_get_pc(pio0, INDUCER_SM);
    diagnostics.local_pc = pio_sm_get_pc(pio2, LOCAL_SM);
    diagnostics.active = local_active();
    ++stats.missed;
    __dmb(); ++diagnostics.sequence;
}

static void __time_critical_func(prepare_output)(void)
{
    if (!enabled || !pacing || !pace_fresh || prepared_packet || recovering || reacquire_pending) return;
    // A reservation is prepared inside its preceding logical slot. Adjacent
    // local slots reuse this path after the first frame has finished.
    uint cycle = stats.current_cycle;
    uint slot = 0;
    while (slot < FLEXRAY_FRAME_GEN_TARGETS &&
           stats.current_slot + 1u != target_id(&config, slot)) ++slot;
    if (slot == FLEXRAY_FRAME_GEN_TARGETS) return;
    reservation_t *r = &reservations[slot];
    prepared_beat = last_beat;
    const flexray_frame_gen_rule_t *rule = &FLEXRAY_FRAME_GEN_RULES[slot];
    bool eligible = (cycle & (rule->cycle_rep - 1u)) == rule->cycle_base;
    flexray_frame_gen_template_t *tpl = eligible && r->template_len ?
        &r->data_templates[r->active_bank] : &r->null_template;
    // Consume at preparation, like a MITM override. A missed transmission must
    // not replay this payload in a later slot. Only core1 owns this length;
    // prepared_packet still protects the active bank until frame end/reset.
    if (eligible) r->template_len = 0;
    prepared_packet = flexray_frame_gen_template_prepare(tpl, config.payload_bytes + 8u, (uint8_t)cycle);
    // Publish the cycle/CRC bytes before preloading the original injector FIFO
    // in the preceding logical slot. No CRC calculation or frame copy here.
    __dmb();
    dma_channel_set_read_addr(packet_dma, prepared_packet, true);
    consume_pace();
    if (!pace_fresh || last_beat != prepared_beat) {
        // The preparation slot ended before publication. Do not send this
        // command in a later slot. Payload lateness instead selects NULL above.
        record_miss(1u); reset_output(); return;
    }
    __dmb();
    dma_channel_set_read_addr(pulse_dma, pulses, true);
    consume_pace();
    if ((!pace_fresh || last_beat != prepared_beat) && !local_active()) {
        // Publication crossed the next beat but missed its hardware FIFO
        // check. Discard the unused one shot before another beat can see it.
        record_miss(2u); reset_output();
    }
}

// C frame-end has only the remainder of one static slot to prepare D.
// Keep this mailbox fast path in SRAM; XIP misses must not consume that budget.
static bool __no_inline_not_in_flash_func(apply_command)(void)
{
    if (prepared_packet) return false;
    for (uint i = 0; i < FLEXRAY_FRAME_GEN_TARGETS; ++i) {
        if (!payload_pending[i]) continue;
        __dmb();
        if (payload_generation[i] == sync_generation && !recovering && !reacquire_pending) {
            reservations[i].active_bank = payload_bank[i];
            reservations[i].template_len = FLEXRAY_FRAME_GEN_PAYLOAD_BYTES + 8u;
        }
        __dmb(); payload_pending[i] = false;
    }
    if (!command_pending) return false;
    __dmb();
    bool changed = false;
    if (mailbox.op == FLEXRAY_FRAME_GEN_OP_CONFIG) {
        // Stop the old reservation before replacing any packet it could read.
        reset_output();
        config = mailbox.config; enabled = false;
        // Core0 built each null packet/CRC table in its inactive bank.
        // Configuration is applied outside local TX; no CRC work enters this ISR.
        for (uint i = 0; i < FLEXRAY_FRAME_GEN_TARGETS; ++i) {
            reservation_t *r = &reservations[i];
            r->template_len = 0;
            if (target_id(&config, i)) r->null_template = r->data_templates[mailbox.bank[i]];
        }
        init_schedule(); reset_timing(); changed = true;
    } else if (mailbox.op == FLEXRAY_FRAME_GEN_OP_TEMPLATE) {
        for (uint i = 0; i < FLEXRAY_FRAME_GEN_TARGETS; ++i) {
            if (mailbox.slot != i && mailbox.slot != FLEXRAY_FRAME_GEN_TARGETS) continue;
            bool fresh = mailbox.generation == sync_generation && !recovering && !reacquire_pending;
            reservations[i].template_len = fresh ? mailbox.frame_len : 0u;
            if (fresh && mailbox.frame_len) reservations[i].active_bank = mailbox.bank[i];
        }
    } else {
        if (enabled != mailbox.enable) {
            enabled = mailbox.enable;
            reset_output();
        }
        if (!enabled)
            for (uint i = 0; i < FLEXRAY_FRAME_GEN_TARGETS; ++i) reservations[i].template_len = 0;
    }
    stats.last_error = FLEXRAY_FRAME_GEN_OK;
    __dmb(); command_pending = false; publish();
    return changed;
}

static void __time_critical_func(pace_irq)(void)
{
    pio_interrupt_clear(pio0, flexray_slot_pace_PACE_IRQ);
    consume_pace();
    if (!pacing) return;
    if (sync_timeout()) return;
    if (apply_command()) return;
    uint previous = reference_id == 1u ? config.static_max_id : reference_id - 1u;
    if (pace_fresh && stats.current_slot == previous) {
        if (phase_armed) ++stats.phase_missed;
        arm_phase();
        phase_cycle = (uint8_t)((stats.current_cycle + (reference_id == 1u)) & 63u);
    }
    if (prepared_packet && last_beat != prepared_beat && !local_active()) {
        record_miss(3u); reset_output();
    }
    prepare_output();
    publish();
}

static void __time_critical_func(frame_gen_done_irq)(void)
{
    pio_interrupt_clear(pio0, flexray_frame_gen_inducer_DONE_IRQ);
    if (!prepared_packet) return;
    if (!dma_channel_is_busy(packet_dma) && dma_channel_hw_addr(packet_dma)->transfer_count == 0u) {
        const uint8_t *raw = (const uint8_t *)(prepared_packet->words + 1);
        ++stats.sent;
        if (!(raw[0] & 0x20u)) ++stats.null_sent;
    } else record_miss(4u);
    prepared_packet = NULL;
    if (service_reacquisition()) return;
    // The inducer has completed post-FES idle and released ownership/IRQ7.
    // C's frame end may prepare D in the remaining part of logical slot C;
    // only the NEXT hardware pace may start D, never this callback's time.
    consume_pace();
    if (!apply_command()) prepare_output();
    publish();
    return;
}

void __time_critical_func(flexray_frame_gen_on_rx_header)(bool is_fr2, const uint8_t header[5], bool valid)
{
    if (!ready) return;
    if (service_reacquisition()) return;
    if (apply_command()) return;
    if ((uint8_t)is_fr2 != config.rx_channel) return;
    uint32_t stamp = 0;
    if (!take_fss(&stamp) || !valid) {
        schedule.have_previous = false;
        // Bootstrap X started on the first FSS, even if its header is bad.
        // Do not attach the next frame's ID/cycle to that earlier edge.
        if (runtime && !seeded) restart_bootstrap();
        publish(); return;
    }
    uint16_t id = (uint16_t)(((header[0] & 7u) << 8) | header[1]);
    uint8_t cycle = header[4] & 63u;
    if (runtime && seeded && pacing && id && id <= config.static_max_id) {
        consume_pace();
        // The IRQ may be serviced in a later static slot. Its arrival is NOT
        // the FSS timestamp: phase loss is established by the hardware burst
        // watchdog, never by comparing this CPU-time slot against the FID.
        if (pace_fresh && id != stats.current_slot) ++stats.header_slot_mismatches;
        if (pace_fresh && cycle != stats.current_cycle) {
            lose_sync(2u); return;
        }
    }
    if (!runtime) {
        if (flexray_slot_schedule_observe(&schedule, id, cycle, stamp)) reset_timing();
    } else if (!seeded) {
        if (!flexray_slot_schedule_reference_allowed(&schedule, id) ||
            pio_sm_get_pc(pio0, PACE_SM) != timing_offset + flexray_slot_pace_offset_countdown) {
            restart_bootstrap(); return;
        }
        reference_id = id; base_cycle = (uint8_t)((cycle + 1u) & 63u);
        last_sync_us = time_us_32();
        phase_cycle = base_cycle;
        uint period_count = 0;
        nominal_tail = schedule.cycle_cycles - config.static_max_id * schedule.slot_cycles
                       - FLEXRAY_SLOT_TAIL_OVERHEAD;
        for (uint slot = id; slot < config.static_max_id; ++slot)
            periods[period_count++] = schedule.slot_cycles - PACE_OVERHEAD;
        periods[period_count++] = 0u; // static envelope ends
        for (uint g = 0; g < TAIL_GUARDS; ++g) {
            uint32_t span = (g + 1u) * schedule.slot_cycles / TAIL_GUARDS -
                            g * schedule.slot_cycles / TAIL_GUARDS;
            periods[period_count++] = span - 4u;
        }
        periods[period_count++] = 0u; // end guards, load cycle shaping word
        gap_index = period_count;
        periods[period_count++] = nominal_tail;
        for (uint slot = 1; slot < id; ++slot)
            periods[period_count++] = schedule.slot_cycles - PACE_OVERHEAD;
        start_pace_dma(period_count);
        seeded = true;
    } else if (pacing && id == reference_id && phase_armed && cycle == phase_cycle &&
               id == stats.current_slot && cycle == stats.current_cycle) {
        phase_armed = false;
        uint32_t sample[8];
        uint32_t remaining = dma_channel_hw_addr((uint)sample_dma)->transfer_count;
        uint32_t done = phase_words - remaining;
        bool window_open = remaining && done >= 8u && done < MEASURE_WORDS;
        __dmb();
        for (uint i = 0; i < 8u; ++i) sample[i] = measure_ring[i];
        int expected = (int)local_tss_cycles(config.tss_bits) - 19 - config.phase_cycles;
        int32_t error;
        bool valid_phase = window_open &&
            flexray_fss_phase_error(sample, expected, config.tolerance_cycles, &error);
        int32_t correction = valid_phase ? flexray_slot_shape_cycle(&schedule, error) : 0;
        uint32_t shaped_tail = (uint32_t)((int64_t)nominal_tail + correction);
        // Keep the sampling clock running until AFTER this check; stopping it
        // first would freeze the deadline while the CPU calculates the error.
        __dmb();
        remaining = dma_channel_hw_addr((uint)sample_dma)->transfer_count;
        if (valid_phase && remaining && phase_words - remaining < MEASURE_WORDS &&
            !(pio0->fdebug & (1u << (PIO_FDEBUG_RXSTALL_LSB + PHASE_SM)))) {
            periods[gap_index] = shaped_tail;
            schedule.cycle_shape = correction;
            stats.phase_error_cycles = error;
            ++stats.phase_updates;
            last_sync_us = time_us_32();
            recovering = false;
        } else ++stats.phase_missed;
        pio_sm_set_enabled(pio0, PHASE_SM, false);
        stop_dma(1u << (uint)sample_dma);
    }
    publish();
}

void flexray_frame_gen_init(void)
{
    if (clock_get_hz(clk_sys) != CLOCK_HZ) return;
    config = default_config;
    stats.magic = FLEXRAY_FRAME_GEN_STATUS_MAGIC; stats.version = FLEXRAY_FRAME_GEN_VERSION; stats.size = sizeof(stats);
    for (uint i = 0; i < FLEXRAY_FRAME_GEN_TARGETS; ++i)
        generate_template(&reservations[i].null_template, &config, target_id(&config, i), NULL);
    init_schedule();
    pio_sm_claim(pio2, CAPTURE_SM); pio_sm_claim(pio0, PACE_SM); pio_sm_claim(pio0, PHASE_SM);
    pio_sm_claim(pio0, INDUCER_SM);
    capture_offset = pio_add_program(pio2, &flexray_fss_capture_program);
    inducer_offset = pio_add_program(pio0, &flexray_frame_gen_inducer_program);
    pio_sm_set_pins_with_mask(pio2, CAPTURE_SM, 0u, 1u << FLEXRAY_FSS_MARKER_PIN);
    pio_sm_set_consecutive_pindirs(pio2, CAPTURE_SM, FLEXRAY_FSS_MARKER_PIN, 1u, true);
    pio_gpio_init(pio2, FLEXRAY_FSS_MARKER_PIN);
    pio_sm_set_pins_with_mask(pio0, INDUCER_SM, 1u << FLEXRAY_INTERNAL_PIN, 1u << FLEXRAY_INTERNAL_PIN);
    pio_sm_set_consecutive_pindirs(pio0, INDUCER_SM, FLEXRAY_INTERNAL_PIN, 1u, true);
    pio_gpio_init(pio0, FLEXRAY_INTERNAL_PIN);
    pace_dma = dma_claim_unused_channel(true); pace_reload_dma = dma_claim_unused_channel(true);
    pace_reset_dma = dma_claim_unused_channel(true);
    pulse_dma = dma_claim_unused_channel(true); packet_dma = dma_claim_unused_channel(true);
    pio_sm_set_pins_with_mask(pio0, INDUCER_SM, 1u << FLEXRAY_FRAME_GEN_OWNERSHIP_PIN,
                              1u << FLEXRAY_FRAME_GEN_OWNERSHIP_PIN);
    pio_sm_set_consecutive_pindirs(pio0, INDUCER_SM, FLEXRAY_FRAME_GEN_OWNERSHIP_PIN, 1u, true);
    pio_gpio_init(pio0, FLEXRAY_FRAME_GEN_OWNERSHIP_PIN);
    pio_set_irq1_source_enabled(pio0, pis_interrupt2, true);
    irq_set_exclusive_handler(pio_get_irq_num(pio0, 1), frame_gen_done_irq);
    irq_set_enabled(pio_get_irq_num(pio0, 1), true);
    reset_timing();
    pio_set_irq0_source_enabled(pio0, pis_interrupt5, true);
    irq_set_exclusive_handler(pio_get_irq_num(pio0, 0), pace_irq);
    irq_set_enabled(pio_get_irq_num(pio0, 0), true);
    publish(); __dmb(); ready = true;
}

static uint16_t submit(const uint8_t *request, size_t length)
{
    uint32_t generation = sync_generation;
    __dmb();
    if (!ready) return FLEXRAY_FRAME_GEN_UNAVAILABLE;
    if (command_pending) return FLEXRAY_FRAME_GEN_BUSY;
    if (request[0] != FLEXRAY_FRAME_GEN_OP_ENABLE)
        for (uint i = 0; i < FLEXRAY_FRAME_GEN_TARGETS; ++i)
            if (payload_pending[i]) return FLEXRAY_FRAME_GEN_BUSY;
    // Core1 publishes config before clearing command_pending. This mailbox
    // handshake makes the applied config safe to read here on core0.
    __dmb();
    for (uint i = 0; i < FLEXRAY_FRAME_GEN_TARGETS; ++i)
        mailbox.bank[i] = (uint8_t)(reservations[i].active_bank ^ 1u);
    if (request[0] == FLEXRAY_FRAME_GEN_OP_CONFIG) {
        if (length != sizeof(config) + 1u) return FLEXRAY_FRAME_GEN_BAD_LENGTH;
        flexray_frame_gen_config_t next;
        memcpy(&next, request + 1, sizeof(next));
        if (!config_valid(&next)) return FLEXRAY_FRAME_GEN_BAD_CONFIG;
        // The legacy configuration action may tune timing, never reservations.
        if (next.target_id != FLEXRAY_FRAME_GEN_RULES[0].id ||
            next.second_target_id != FLEXRAY_FRAME_GEN_RULES[1].id ||
            next.payload_bytes != FLEXRAY_FRAME_GEN_PAYLOAD_BYTES ||
            next.static_max_id != FLEXRAY_FRAME_GEN_STATIC_MAX_ID ||
            next.cycle_mask != FLEXRAY_FRAME_GEN_RULES[0].cycle_rep - 1u ||
            next.cycle_base != FLEXRAY_FRAME_GEN_RULES[0].cycle_base)
            return FLEXRAY_FRAME_GEN_BAD_CONFIG;
        for (uint i = 0; i < FLEXRAY_FRAME_GEN_TARGETS; ++i)
            if (target_id(&next, i))
                generate_template(&reservations[i].data_templates[mailbox.bank[i]],
                               &next, target_id(&next, i), NULL);
        mailbox.config = next;
    } else if (request[0] == FLEXRAY_FRAME_GEN_OP_TEMPLATE) {
        uint16_t len = (uint16_t)(length - 1u);
        const uint8_t *raw = request + 1;
        if (len && len != 2u && len != config.payload_bytes + 8u) return FLEXRAY_FRAME_GEN_BAD_LENGTH;
        // Empty clears both templates; a two-byte FID clears only that slot.
        // Full frames select their reservation by the real header's FID.
        uint16_t id = len == 2u ? (uint16_t)(raw[0] | ((uint16_t)raw[1] << 8)) :
            len ? (uint16_t)(((raw[0] & 7u) << 8) | raw[1]) : 0u;
        uint slot = 0;
        while (slot < FLEXRAY_FRAME_GEN_TARGETS && (!id || id != target_id(&config, slot))) ++slot;
        if (len && slot == FLEXRAY_FRAME_GEN_TARGETS) return FLEXRAY_FRAME_GEN_BAD_FRAME;
        if (len > 2u) {
            if (!pacing || recovering || reacquire_pending) return FLEXRAY_FRAME_GEN_NOT_SYNCED;
            mailbox.generation = generation;
            __dmb();
            uint16_t hcrc = (uint16_t)(((raw[2] & 1u) << 10) | ((uint16_t)raw[3] << 2) | (raw[4] >> 6));
            uint32_t crc = ((uint32_t)raw[len - 3u] << 16) | ((uint32_t)raw[len - 2u] << 8) | raw[len - 1u];
            if ((raw[2] >> 1) * 2u != config.payload_bytes ||
                calculate_flexray_header_crc(raw) != hcrc || calculate_flexray_frame_crc(raw, len - 3u) != crc)
                return FLEXRAY_FRAME_GEN_BAD_FRAME;
            generate_template(&reservations[slot].data_templates[mailbox.bank[slot]], &config, id, raw);
        }
        mailbox.slot = (uint8_t)slot;
        mailbox.frame_len = len > 2u ? len : 0u;
    } else if (request[0] == FLEXRAY_FRAME_GEN_OP_ENABLE) {
        if (length != 2u || request[1] > 1u) return FLEXRAY_FRAME_GEN_BAD_LENGTH;
        mailbox.enable = request[1] != 0;
    } else return FLEXRAY_FRAME_GEN_BAD_LENGTH;
    mailbox.op = request[0]; __dmb(); command_pending = true;
    return FLEXRAY_FRAME_GEN_OK;
}

size_t flexray_frame_gen_action(const uint8_t *data, size_t length)
{
    uint32_t generation = sync_generation;
    __dmb();
    if (!data || !length) return 0;
    if (data[0] == FLEXRAY_FRAME_GEN_OP_SWITCH) {
        if (length < 2u) { stats.last_error = FLEXRAY_FRAME_GEN_BAD_LENGTH; return 0; }
        uint8_t request[2] = {FLEXRAY_FRAME_GEN_OP_ENABLE, data[1]};
        stats.last_error = submit(request, sizeof(request));
        return 2u;
    }
    if (data[0] != FLEXRAY_FRAME_GEN_OP_PAYLOAD) return 0;
    if (length < 6u) { stats.last_error = FLEXRAY_FRAME_GEN_BAD_LENGTH; return 0; }
    uint16_t id = (uint16_t)(data[1] | ((uint16_t)data[2] << 8));
    uint16_t len = (uint16_t)(data[4] | ((uint16_t)data[5] << 8));
    if (length - 6u < len) { stats.last_error = FLEXRAY_FRAME_GEN_BAD_LENGTH; return 0; }
    uint16_t result = FLEXRAY_FRAME_GEN_OK;
    uint slot = 0;
    while (slot < FLEXRAY_FRAME_GEN_TARGETS && FLEXRAY_FRAME_GEN_RULES[slot].id != id) ++slot;
    if (!ready) result = FLEXRAY_FRAME_GEN_UNAVAILABLE;
    else if (len != FLEXRAY_FRAME_GEN_PAYLOAD_BYTES) result = FLEXRAY_FRAME_GEN_BAD_LENGTH;
    else if (slot == FLEXRAY_FRAME_GEN_TARGETS || data[3] != FLEXRAY_FRAME_GEN_RULES[slot].cycle_base)
        result = FLEXRAY_FRAME_GEN_BAD_CONFIG;
    else if (command_pending || payload_pending[slot]) result = FLEXRAY_FRAME_GEN_BUSY;
    else if (!pacing || recovering || reacquire_pending) result = FLEXRAY_FRAME_GEN_NOT_SYNCED;
    else {
        __dmb();
        uint8_t bank = (uint8_t)(reservations[slot].active_bank ^ 1u);
        // Private header: do not read a null packet while core1 changes its
        // cycle/CRC for DMA. Host supplies payload only.
        uint8_t raw[FLEXRAY_FRAME_GEN_PAYLOAD_BYTES + 8u] = {0};
        generate_static_header(raw, &config, id);
        raw[0] |= 0x20u; // NFI=1, PPI remains clear; no startup/sync flags.
        memcpy(raw + 5, data + 6, len);
        generate_template(&reservations[slot].data_templates[bank], &config, id, raw);
        if (generation != sync_generation || recovering || reacquire_pending) {
            stats.last_error = FLEXRAY_FRAME_GEN_NOT_SYNCED;
            return 6u + len;
        }
        payload_bank[slot] = bank;
        payload_generation[slot] = generation;
        __dmb(); payload_pending[slot] = true;
    }
    stats.last_error = result;
    return 6u + len;
}

size_t flexray_frame_gen_command(const uint8_t *request, size_t length, uint8_t *reply, size_t capacity)
{
    if (!request || !length || !reply || capacity < 4u) return 0;
    uint16_t result;
    size_t size = 4u;
    if (request[0] == FLEXRAY_FRAME_GEN_OP_DIAGNOSTICS) {
        if (!ready) result = FLEXRAY_FRAME_GEN_UNAVAILABLE;
        else if (length != 1u || capacity < 4u + sizeof(diagnostics)) result = FLEXRAY_FRAME_GEN_BAD_LENGTH;
        else {
            uint32_t sequence = diagnostics.sequence; __dmb();
            flexray_frame_gen_diagnostics_t snapshot = diagnostics;
            __dmb();
            if ((sequence & 1u) || sequence != diagnostics.sequence) result = FLEXRAY_FRAME_GEN_BUSY;
            else { memcpy(reply + 4, &snapshot, sizeof(snapshot)); size += sizeof(snapshot); result = FLEXRAY_FRAME_GEN_OK; }
        }
    } else if (request[0] == FLEXRAY_FRAME_GEN_OP_STATUS) {
        if (!ready) result = FLEXRAY_FRAME_GEN_UNAVAILABLE;
        else if (length != 1u || capacity < 4u + sizeof(stats)) result = FLEXRAY_FRAME_GEN_BAD_LENGTH;
        else {
            flexray_frame_gen_status_t snapshot = stats;
            if (command_pending || payload_pending[0] || payload_pending[1])
                snapshot.flags |= FLEXRAY_FRAME_GEN_COMMAND_PENDING;
            memcpy(reply + 4, &snapshot, sizeof(snapshot)); size += sizeof(snapshot); result = FLEXRAY_FRAME_GEN_OK;
        }
    } else { result = submit(request, length); if (result) stats.last_error = result; }
    reply[0] = request[0]; reply[1] = FLEXRAY_FRAME_GEN_VERSION;
    reply[2] = (uint8_t)result; reply[3] = (uint8_t)(result >> 8);
    return size;
}
