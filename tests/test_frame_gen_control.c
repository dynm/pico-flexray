#include "frame_gen_control_fakes.h"
static uint32_t fake_now_us;
static inline uint32_t time_us_32(void) { return fake_now_us; }
#include "../src/flexray_frame_gen.c"
#include <stdio.h>
enum { INJECT_TRANSPORT_VENDOR, INJECT_TRANSPORT_UDP };
static uint mitm_submissions, mitm_switches;
static bool injector_submit_override_from(uint16_t id, uint8_t base, uint16_t len,
                                         const uint8_t *data, uint8_t transport)
{
    assert(id == 8 && base == 2 && len == 1 && data[0] == 0xaa);
    assert(transport <= INJECT_TRANSPORT_UDP);
    ++mitm_submissions; return true;
}
static void injector_set_enabled(bool on) { assert(on); ++mitm_switches; }
#include "frame_gen_ingress.h"

static uint16_t command(uint8_t op, const void *body, uint len)
{
    uint8_t request[300] = {op}, reply[128];
    if (len) memcpy(request + 1, body, len);
    assert(flexray_frame_gen_command(request, len + 1, reply, sizeof(reply)) == 4);
    assert(reply[0] == op && reply[1] == FLEXRAY_FRAME_GEN_VERSION);
    return reply[2] | (reply[3] << 8);
}
static void header(uint id, uint cycle)
{
    uint8_t h[5] = {(uint8_t)(id >> 8), (uint8_t)id, 0, 0, (uint8_t)cycle};
    pio2->fdebug = 0;
    pio2->rx_count[CAPTURE_SM] = 1; pio2->rx[CAPTURE_SM][0] = 0;
    pio0->pc[PACE_SM] = timing_offset + flexray_slot_pace_offset_countdown;
    flexray_frame_gen_on_rx_header(config.rx_channel, h, true);
}
static void beat(uint n) { pio0->fdebug = 0; fake_beat(n); pace_irq(); }
static void configure(void)
{
    flexray_frame_gen_config_t c = default_config;
    c.slot_cycles = 6000; c.cycle_cycles = 750000;
    assert(command(FLEXRAY_FRAME_GEN_OP_CONFIG, &c, sizeof(c)) == 0);
    header(6, 62); header(6, 63);
    assert(seeded && reference_id == 6 && base_cycle == 0 && !enabled);
    assert(runtime && schedule.locked && fake_dma_claimed == 10);
    assert(pio0->used_words == 30 && pio2->used_words == 32);
    assert(fake_dmas[sample_dma].source == &pio0->rxf[PHASE_SM]);
}
static void action_switch(bool on)
{
    uint8_t action[2] = {0x95, on};
    assert(flexray_frame_gen_action(action, sizeof(action)) == sizeof(action));
    assert(stats.last_error == 0);
}
static void payload(uint id, uint8_t value)
{
    uint8_t action[24] = {0x94, (uint8_t)id, 0, 3, 18, 0};
    memset(action + 6, value, 18);
    assert(flexray_frame_gen_action(action, sizeof(action)) == sizeof(action));
    assert(stats.last_error == 0);
}
static void check_frame(const flexray_frame_gen_packet_t *p, uint id, uint cycle, int value)
{
    const uint8_t *raw = (const uint8_t *)(p->words + 1);
    assert(raw[1] == id && (raw[4] & 63u) == cycle && (raw[2] >> 1) == 9);
    assert((raw[0] & 0x60u) == (value < 0 ? 0u : 0x20u));
    for (uint i = 0; i < 18; ++i) assert(raw[5 + i] == (value < 0 ? 0 : value));
    uint16_t hcrc = ((raw[2] & 1u) << 10) | (raw[3] << 2) | (raw[4] >> 6);
    assert(calculate_flexray_header_crc(raw) == hcrc);
    uint32_t crc = ((uint32_t)raw[23] << 16) | ((uint32_t)raw[24] << 8) | raw[25];
    assert(calculate_flexray_frame_crc(raw, 23) == crc);
}
static void finish(const flexray_frame_gen_packet_t *p)
{
    assert(local_active() && prepared_packet == p);
    fake_dmas[packet_dma].busy = false; fake_dmas[packet_dma].transfer_count = 0;
    fake_dmas[pulse_dma].busy = false; fake_dmas[pulse_dma].transfer_count = 0;
    pio0->pins |= 1u << FLEXRAY_FRAME_GEN_OWNERSHIP_PIN;
    frame_gen_done_irq();
    assert(!local_active());
}
// Process every hardware beat, including adjacent D prepared at C frame end.
static void step(uint n, int expected)
{
    const flexray_frame_gen_packet_t *p = prepared_packet;
    if (p) pio0->pins &= ~(1u << FLEXRAY_FRAME_GEN_OWNERSHIP_PIN);
    beat(n);
    if (p) {
        assert(stats.current_slot == 0xc || stats.current_slot == 0xd);
        check_frame(p, stats.current_slot, stats.current_cycle, expected);
        finish(p);
    }
}
static void test_static_rules_and_actions(void)
{
    assert(default_config.target_id == 0xc && default_config.second_target_id == 0xd);
    assert(default_config.cycle_mask == 3 && default_config.cycle_base == 3);
    for (uint i = 0; i < 7; ++i) {
        flexray_frame_gen_config_t bad = default_config;
        switch (i) {
        case 0: bad.target_id = 8; break;
        case 1: bad.second_target_id = 0; break;
        case 2: bad.payload_bytes = 20; break;
        case 3: bad.cycle_mask = 1; break;
        case 4: bad.cycle_base = 2; break;
        case 5: bad.static_max_id = 0x11; break;
        default: bad.phase_cycles = INT32_MIN; break;
        }
        assert(command(FLEXRAY_FRAME_GEN_OP_CONFIG, &bad, sizeof(bad)) == FLEXRAY_FRAME_GEN_BAD_CONFIG);
    }
    configure();
    uint8_t invalid[24] = {0x94, 0xc, 0, 3, 18, 0};
    for (uint len = 0; len < sizeof(invalid); ++len)
        assert(flexray_frame_gen_action(invalid, len) == 0);
    invalid[1] = 0xe;
    assert(flexray_frame_gen_action(invalid, 24) == 24 && stats.last_error == FLEXRAY_FRAME_GEN_BAD_CONFIG);
    invalid[1] = 0xc; invalid[3] = 2;
    assert(flexray_frame_gen_action(invalid, 24) == 24 && stats.last_error == FLEXRAY_FRAME_GEN_BAD_CONFIG);
    invalid[3] = 3; invalid[4] = 17;
    assert(flexray_frame_gen_action(invalid, 24) == 23 && stats.last_error == FLEXRAY_FRAME_GEN_BAD_LENGTH);
    uint8_t bad_switch[2] = {0x95, 2};
    assert(flexray_frame_gen_action(bad_switch, 2) == 2 && stats.last_error == FLEXRAY_FRAME_GEN_BAD_LENGTH);
    action_switch(true); step(0, -1);
    uint sent = stats.sent, nulls = stats.null_sent, missed = stats.missed;
    uint setups = fake_local_setups;
    for (uint n = 1; n <= 1032; ++n) {
        if (n == 32) {
            // Same transfer can carry C and D before core1 applies either.
            payload(0xc, 0xc1); payload(0xd, 0xd1);
            assert(payload_pending[0] && payload_pending[1]);
            uint8_t duplicate[24] = {0x94, 0xc, 0, 3, 18, 0};
            assert(flexray_frame_gen_action(duplicate, 24) == 24 && stats.last_error == FLEXRAY_FRAME_GEN_BUSY);
        }
        if (n == 54) {
            assert(prepared_packet == &reservations[0].data_templates[reservations[0].active_bank].packet);
            flexray_frame_gen_packet_t copy = *prepared_packet;
            payload(0xc, 0xc2); // too late for cycle 3, retained for cycle 7
            assert(memcmp(prepared_packet, &copy, sizeof(copy)) == 0);
        }
        int expected = n == 54 ? 0xc1 : n == 55 ? 0xd1 : n == 118 ? 0xc2 : -1;
        step(n, expected);
        if (n == 55) assert(reservations[0].template_len == 26 && reservations[1].template_len == 0);
        if (n == 118) assert(reservations[0].template_len == 0);
    }
    // 65 complete C/D pairs, crossing cycle label 63 -> 0, three data frames.
    assert(stats.sent == sent + 130 && stats.null_sent == nulls + 127 && stats.missed == missed);
    assert(fake_local_setups == setups && stats.current_cycle == 0);
    payload(0xc, 0xaa); payload(0xd, 0xbb); action_switch(false); beat(1033);
    assert(!enabled && !prepared_packet && !payload_pending[0] && !payload_pending[1]);
    assert(reservations[0].template_len == 0 && reservations[1].template_len == 0);
    action_switch(true); beat(1034);
    for (uint n = 1035; n <= 1080; ++n) step(n, -1);
    action_switch(false); beat(1081);
}
static void test_both_ingresses(void)
{
    void (*inputs[2])(const uint8_t *, uint16_t) = {vendor_actions, ncm_actions};
    for (uint transport = 0; transport < 2; ++transport) {
        configure();
        beat(0);
        uint8_t actions[57] = {0};
        // Two build submissions and existing MITM actions in one transfer.
        actions[0] = actions[24] = 0x94;
        actions[1] = 0xc; actions[25] = 0xd;
        actions[3] = actions[27] = 3;
        actions[4] = actions[28] = 18;
        memset(actions + 6, 0x11 + transport, 18);
        memset(actions + 30, 0x22 + transport, 18);
        uint8_t mitm[9] = {0x90, 8, 0, 2, 1, 0, 0xaa, 0x91, 1};
        memcpy(actions + 48, mitm, 9);
        inputs[transport](actions, sizeof(actions));
        assert(payload_pending[0] && payload_pending[1]);
        assert(mitm_submissions == transport + 1 && mitm_switches == transport + 1);
        uint8_t on[2] = {0x95, 1}; inputs[transport](on, 2);
        for (uint n = 1; n <= 55; ++n)
            step(n, n == 54 ? (int)(0x11 + transport) : n == 55 ? (int)(0x22 + transport) : -1);
        uint8_t off[2] = {0x95, 0}; inputs[transport](off, 2); beat(56);
        assert(!enabled && !prepared_packet);
        for (uint len = 1; len < 24; ++len) inputs[transport](actions, len);
        assert(!payload_pending[0] && !payload_pending[1]);
    }
}
static void test_deadline_and_phase(void)
{
    configure(); beat(0);
    // Enable immediately before cycle 3; setup failure consumes payload once.
    for (uint n = 1; n <= 51; ++n) beat(n);
    payload(0xc, 0x55); action_switch(true); beat(52);
    fake_beat_on_packet = 54; beat(53);
    assert(stats.current_slot == 0xc && !prepared_packet && reservations[0].template_len == 0);
    assert(diagnostics.reason == 1 && diagnostics.fid == 0xc && diagnostics.cycle == 3);
    assert(diagnostics.current_beat != diagnostics.prepared_beat && diagnostics.counts[0] > 0);
    uint8_t diagnostic_op = FLEXRAY_FRAME_GEN_OP_DIAGNOSTICS, diagnostic_reply[128];
    assert(flexray_frame_gen_command(&diagnostic_op, 1, diagnostic_reply, sizeof(diagnostic_reply)) == 68);
    flexray_frame_gen_diagnostics_t snapshot;
    memcpy(&snapshot, diagnostic_reply + 4, sizeof(snapshot));
    assert(snapshot.sequence == diagnostics.sequence && !(snapshot.sequence & 1u));
    ++diagnostics.sequence;
    assert(flexray_frame_gen_command(&diagnostic_op, 1, diagnostic_reply, sizeof(diagnostic_reply)) == 4);
    assert(diagnostic_reply[2] == FLEXRAY_FRAME_GEN_BUSY);
    ++diagnostics.sequence;
    uint missed = stats.missed;
    for (uint n = 55; n <= 116; ++n) step(n, -1);
    step(117, -1); step(118, -1); step(119, -1);
    assert(stats.missed == missed);
    action_switch(false); beat(120);
    // Finite phase correction is unchanged by payload eligibility/null policy.
    uint next = 121;
    while (((reference_id - 1 + next) % config.static_max_id) + 1 != 5) ++next;
    beat(next); assert(phase_armed); beat(next + 1);
    uint expected = local_tss_cycles(config.tss_bits) - 19 - config.phase_cycles;
    memset((void *)measure_ring, 0, 8u * sizeof(uint32_t));
    for (uint i = expected + 16; i < expected + 31; ++i) measure_ring[i / 32] |= 1u << (31 - i % 32);
    fake_dmas[sample_dma].transfer_count = phase_words - 8u;
    uint updates = stats.phase_updates; header(6, stats.current_cycle);
    assert(stats.phase_updates == updates + 1 && stats.phase_error_cycles == 16);
    assert(periods[gap_index] == 750000 - 16 * 6000 - 13 + 16);
    assert(periods[0] == 5991 && periods[1] == 5991);
    beat(next + 16); beat(next + 17); fake_dmas[sample_dma].transfer_count = 0;
    uint32_t tail = periods[gap_index]; updates = stats.phase_updates;
    header(6, stats.current_cycle);
    assert(stats.phase_updates == updates && periods[gap_index] == tail);
    uint8_t op = FLEXRAY_FRAME_GEN_OP_STATUS, reply[128];
    assert(flexray_frame_gen_command(&op, 1, reply, sizeof(reply)) == 112);
    // Timing-only reconfiguration returns the same DMA/SMs to acquisition.
    assert(command(FLEXRAY_FRAME_GEN_OP_CONFIG, &default_config, sizeof(default_config)) == 0);
    header(6, 10);
    assert(!runtime && !seeded && fake_dma_claimed == 10 && pio0->used_words == 13);
    assert(fake_dmas[sample_dma].source == &pio0->rxf[PACE_SM]);
}

static void test_sleep_and_reacquisition(void)
{
    fake_now_us = UINT32_MAX - 8000u; // always-powered DUT: watchdog timer wrap
    configure(); action_switch(true); beat(0);
    uint losses = stats.sync_losses;
    uint generation = sync_generation;
    fake_now_us += 10000; beat(1);
    assert(pacing && enabled && stats.sync_losses == losses); // short holdover
    payload(0xc, 0xaa); payload(0xd, 0xbb); // queued before sleep detection
    fake_now_us += 6000; beat(2);
    assert(enabled && recovering && !runtime && !seeded && !pacing && !prepared_packet);
    assert(stats.sync_losses == losses + 1 && stats.sync_loss_reason == 1);
    assert(sync_generation == generation + 1);
    assert(reservations[0].template_len == 0 && reservations[1].template_len == 0);
    apply_command(); // stale published banks cannot cross the sleep epoch
    assert(!payload_pending[0] && !payload_pending[1]);
    assert(reservations[0].template_len == 0 && reservations[1].template_len == 0);
    uint8_t action[24] = {0x94, 0xc, 0, 3, 18, 0};
    assert(flexray_frame_gen_action(action, sizeof(action)) == sizeof(action));
    assert(stats.last_error == FLEXRAY_FRAME_GEN_NOT_SYNCED);

    // Feed actual acquisition timestamps from hardware marker/DMA fakes.
    // The waking source starts at an unrelated cycle label and new periods.
    uint cycle = 20;
    for (; cycle < 50 && !runtime; ++cycle) {
        for (uint id = 4; id <= 6 && !runtime; ++id) {
            uint32_t tick = (cycle - 20) * 749794u + (id - 1) * 5998u;
            uint32_t index = tick / 32u;
            memset((void *)measure_ring, 0, sizeof(measure_ring));
            measure_ring[index & (MEASURE_WORDS - 1)] = 1u << (31u - tick % 32u);
            fake_dmas[sample_dma].transfer_count = MEASURE_TRANSFERS - index - MEASURE_MARGIN - 2;
            header(id, cycle);
        }
    }
    assert(runtime && !seeded && recovering && !prepared_packet);
    assert(schedule.slot_cycles == 5998 && schedule.cycle_cycles == 749794);
    header(6, cycle); beat(0);
    assert(seeded && pacing && recovering && !prepared_packet);
    // A fresh hardware phase observation confirms the reacquired cycle.
    int expected = (int)local_tss_cycles(config.tss_bits) - 19 - config.phase_cycles;
    memset((void *)measure_ring, 0, sizeof(measure_ring));
    for (int i = expected; i < expected + 15; ++i) measure_ring[i / 32] |= 1u << (31 - i % 32);
    fake_dmas[sample_dma].transfer_count = phase_words - 8;
    header(6, stats.current_cycle);
    assert(!recovering && enabled && !(stats.flags & FLEXRAY_FRAME_GEN_RESYNCING));
    uint slot_mismatches = stats.header_slot_mismatches;
    header(2, stats.current_cycle); // delayed header IRQ, same actual cycle
    assert(!recovering && stats.sync_losses == losses + 1);
    assert(stats.header_slot_mismatches == slot_mismatches + 1);
    for (uint n = 1; n <= 7; ++n) step(n, -1); // resumes null, never old data
    for (uint n = 8; n <= 21; ++n) step(n, -1);
    const flexray_frame_gen_packet_t *p = prepared_packet;
    assert(p);
    flexray_frame_gen_packet_t in_flight = *p;
    pio0->pins &= ~(1u << FLEXRAY_FRAME_GEN_OWNERSHIP_PIN);
    beat(22); assert(local_active());
    // A real static header with a new cycle/slot invalidates the grid at once.
    header(1, (stats.current_cycle + 17u) & 63u);
    assert(recovering && reacquire_pending && local_active());
    assert(memcmp(p, &in_flight, sizeof(in_flight)) == 0);
    finish(p); // complete C's TXEN lifecycle, then reacquire; do not preload D
    assert(recovering && !pacing && stats.sync_losses == losses + 2 && stats.sync_loss_reason == 2);
}
static void test_high_id_bootstrap(void)
{
    flexray_frame_gen_config_t c = default_config;
    c.slot_cycles = 6000; c.cycle_cycles = 750000;
    assert(command(FLEXRAY_FRAME_GEN_OP_CONFIG, &c, sizeof(c)) == 0);
    header(15, 62); // apply config
    // Model the frozen acquisition candidates; reference 15 exceeds old limit.
    schedule.learn_id_count = 2;
    schedule.learn_ids[0] = 14; schedule.learn_ids[1] = 15;
    uint loads = fake_program_loads, allocations = fake_dma_allocations;
    // Dense non-candidate headers must not repeatedly reallocate the timing
    // programs/DMA and delay the real bridge's frame-end callbacks.
    for (uint id = 9; id <= 13; ++id) header(id, 63);
    assert(!seeded && fake_program_loads == loads && fake_dma_allocations == allocations);
    assert(schedule.learn_id_count == 2 && phase_armed);
    assert(pio0->start[PACE_SM] == timing_offset); // hardware WAIT FSS entry
    header(15, 63);
    assert(seeded && reference_id == 15 && base_cycle == 0);
    beat(0);
    assert(pacing && stats.current_slot == 15 && stats.current_cycle == 0);
}

int main(void)
{
    pio0->used_words = 0; pio1->used_words = 28; pio2->used_words = 22;
    for (uint i = 0; i < 4; ++i) (void)dma_claim_unused_channel(true);
    flexray_frame_gen_init();
    assert(ready && !runtime && !enabled && fake_dma_claimed == 10);
    check_frame(&reservations[0].null_template.packet, 0xc, 0, -1);
    check_frame(&reservations[1].null_template.packet, 0xd, 0, -1);
    test_static_rules_and_actions();
    test_both_ingresses();
    test_deadline_and_phase();
    test_sleep_and_reacquisition();
    test_high_id_bootstrap();
    puts("frame generation C control tests passed");
}
