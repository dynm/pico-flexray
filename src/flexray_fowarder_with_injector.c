#include <stdint.h>
#include <string.h>
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "pico/time.h"
#include "flexray_frame.h"
#include "hardware/pio.h"
#include "flexray_forwarder_with_injector.pio.h"
#include "flexray_forwarder_with_injector.h"
#include "flexray_injector_rules.h"
#if FLEXRAY_FRAME_GEN
#include "flexray_frame_gen.h"
static uint frame_gen_forwarder_offset;
#endif

static PIO pio_forwarder_with_injector;
static uint sm_forwarder_with_injector_to_fr1;
static uint sm_forwarder_with_injector_to_fr2;
#if !FLEXRAY_FRAME_GEN
static uint sm_forwarder_with_injector_to_fr3;
static uint sm_forwarder_with_injector_to_fr4;
#endif

extern volatile int dma_inject_chan_to_fr1;
extern volatile int dma_inject_chan_to_fr2;
#if !FLEXRAY_FRAME_GEN
extern volatile int dma_inject_chan_to_fr3;
extern volatile int dma_inject_chan_to_fr4;
#endif
static dma_channel_config injector_to_fr1_dc;
static dma_channel_config injector_to_fr2_dc;
#if !FLEXRAY_FRAME_GEN
static dma_channel_config injector_to_fr3_dc;
static dma_channel_config injector_to_fr4_dc;
#endif

// rules now come from flexray_injector_rules.h

#define INJECT_FRAME_BYTES (MAX_FRAME_PAYLOAD_BYTES + 8)
#define INJECT_FRAME_PADDED_BYTES ((INJECT_FRAME_BYTES + 3) & ~3)

_Static_assert((INJECT_FRAME_PADDED_BYTES % sizeof(uint32_t)) == 0,
               "DMA frame storage must be padded to 32-bit words");

typedef struct {
    // The DMA channel byte-swaps every 32-bit transfer. Store the PIO byte
    // count pre-swapped so the first word received by the PIO is len - 1.
    uint32_t dma_count_bswap;
    uint8_t data[INJECT_FRAME_PADDED_BYTES] __attribute__((aligned(4)));
    uint16_t len;    // header + payload bytes + 3 CRC bytes (max 262)
    uint8_t valid;  // 1 if data[] is valid
} frame_template_t;

_Static_assert(__builtin_offsetof(frame_template_t, data) == sizeof(uint32_t),
               "DMA count and frame data must be contiguous");

typedef struct {
    uint8_t pending;
    uint8_t direction;
    uint8_t command_source;
    uint32_t override_timestamp_us;
    const frame_template_t *template;
} prepared_injection_t;

static frame_template_t TEMPLATES[NUM_TRIGGER_RULES];
static prepared_injection_t prepared_injection;

// Host override storage: small ring to avoid malloc; single-consumer in ISR context
typedef struct {
    uint8_t valid;
    uint16_t id;
    uint8_t mask;
    uint8_t base;
    uint16_t len;
    uint8_t command_source;
    uint32_t timestamp_us;
    uint8_t data[INJECT_REPLACE_MASK_MAX];
} host_override_t;

#define HOST_OVERRIDE_CAP 4
#define HOST_OVERRIDE_TIMEOUT_US 100000u
enum {
    INJECTOR_SOURCE_NONE = 0,
    INJECTOR_SOURCE_HOST = 1,
};
static uint32_t host_override_head = 0;
static uint32_t host_override_tail = 0;
static host_override_t host_overrides[HOST_OVERRIDE_CAP];
static spin_lock_t *host_override_lock;
static volatile uint8_t active_command_source = INJECTOR_SOURCE_NONE;
static volatile bool host_commands_allowed = true;
static volatile bool source_submission_seen = false;
static volatile uint32_t source_last_submission_us = 0u;
static volatile bool forwarder_ready = false;
static injector_stats_t injector_stats = {
    .magic = INJECTOR_STATS_MAGIC,
    .version = INJECTOR_STATS_VERSION,
    .size = sizeof(injector_stats_t),
};

static bool claim_command_source(uint8_t source, uint32_t now_us);

static inline uint32_t host_override_count(void)
{
    uint32_t save = spin_lock_blocking(host_override_lock);
    uint32_t count = host_override_head >= host_override_tail ?
        host_override_head - host_override_tail :
        HOST_OVERRIDE_CAP - (host_override_tail - host_override_head);
    spin_unlock(host_override_lock, save);
    return count;
}

static inline bool host_override_push(uint16_t id, uint8_t mask, uint8_t base,
                                      uint16_t len, const uint8_t *bytes,
                                      uint8_t command_source,
                                      uint32_t timestamp_us)
{
    uint32_t save = spin_lock_blocking(host_override_lock);
    uint32_t head = host_override_head;
    uint32_t tail = host_override_tail;
    uint32_t next_head = (head + 1u) % HOST_OVERRIDE_CAP;
    if (len > sizeof(host_overrides[0].data)) {
        spin_unlock(host_override_lock, save);
        return false;
    }
    if (next_head == tail) {
        // Real-time value queue: a fresh value intentionally supersedes the
        // oldest one when the next eligible FlexRay slot has not arrived yet.
        host_overrides[tail].valid = 0;
        host_override_tail = (tail + 1u) % HOST_OVERRIDE_CAP;
        tail = host_override_tail;
        injector_stats.superseded++;
    }
    host_override_t *slot = &host_overrides[head];
    slot->id = id;
    slot->mask = mask;
    slot->base = base;
    slot->len = len;
    slot->command_source = command_source;
    slot->timestamp_us = timestamp_us;
    memcpy(slot->data, bytes, len);
    slot->valid = 1;
    host_override_head = next_head;

    uint32_t depth = next_head >= tail ? next_head - tail :
                     HOST_OVERRIDE_CAP - (tail - next_head);
    if (depth > injector_stats.queue_high_water) {
        injector_stats.queue_high_water = depth;
    }
    spin_unlock(host_override_lock, save);
    return true;
}

static inline bool host_override_try_pop_for(uint16_t id, uint8_t cycle_count,
                                             uint8_t command_source,
                                             uint32_t now_us, uint8_t *out,
                                             uint32_t *timestamp_us)
{
    uint32_t save = spin_lock_blocking(host_override_lock);
    uint32_t tail = host_override_tail;
    uint32_t head = host_override_head;
    if (tail == head) {
        spin_unlock(host_override_lock, save);
        return false;
    }

    uint32_t cursor = tail;
    uint32_t skipped = 0;
    while (cursor != head) {
        host_override_t *slot = &host_overrides[cursor];
        bool fresh = slot->valid &&
            (uint32_t)(now_us - slot->timestamp_us) <= HOST_OVERRIDE_TIMEOUT_US;
        if (!fresh) {
            slot->valid = 0;
            skipped++;
            cursor = (cursor + 1u) % HOST_OVERRIDE_CAP;
            continue;
        }
        if (slot->command_source == command_source && slot->id == id &&
            (uint8_t)(cycle_count & slot->mask) == slot->base) {
            memcpy(out, slot->data, slot->len);
            *timestamp_us = slot->timestamp_us;
            slot->valid = 0;
            host_override_tail = (cursor + 1u) % HOST_OVERRIDE_CAP;
            injector_stats.stale_discarded += skipped;
            injector_stats.consumed++;
            spin_unlock(host_override_lock, save);
            return true;
        }
        cursor = (cursor + 1u) % HOST_OVERRIDE_CAP;
        skipped++;
    }

    spin_unlock(host_override_lock, save);
    return false;
}

static inline int find_cache_slot_for_id(uint16_t id, uint8_t cycle_count) {
    for (int i = 0; i < (int)NUM_TRIGGER_RULES; i++) {
        if (INJECT_TRIGGERS[i].target_id == id && (uint8_t)(cycle_count & INJECT_TRIGGERS[i].cycle_mask) == INJECT_TRIGGERS[i].cycle_base) return i;
    }
    return -1;
}

void try_cache_last_target_frame(uint16_t frame_id, uint8_t cycle_count, uint16_t frame_len, uint8_t *captured_bytes)
{
    int slot = find_cache_slot_for_id(frame_id, cycle_count);
    if (slot < 0){
        return;
    }

    const trigger_rule_t *rule = &INJECT_TRIGGERS[slot];
    if ((uint8_t)(cycle_count & rule->cycle_mask) != rule->cycle_base){
        return;
    }

    if (frame_len > sizeof(TEMPLATES[slot].data)) {
        return;
    }
    // A complete FlexRay frame is 5 header bytes + payload + 3 CRC bytes.
    if (frame_len != (uint16_t)(rule->payload_length + 8u)) {
        return;
    }
    memcpy(TEMPLATES[slot].data, captured_bytes, frame_len);
    TEMPLATES[slot].len = (uint16_t)frame_len;
    TEMPLATES[slot].dma_count_bswap =
        __builtin_bswap32((uint32_t)frame_len - 1u);
    TEMPLATES[slot].valid = 1;
}

static void fix_cycle_count(uint8_t *full_frame, uint8_t cycle_count)
{
    // set full_frame[4] low 6 bits to cycle_count
    full_frame[4] = (full_frame[4] & 0b11000000) | (cycle_count & 0x3F);
}

static void inject_frame(const frame_template_t *tpl, uint8_t direction)
{
    if (tpl == NULL || tpl->len < 8u || tpl->len > INJECT_FRAME_BYTES) {
        return;
    }

    int dma_chan;

    switch (direction) {
    case INJECT_DIRECTION_TO_FR1:
        dma_chan = dma_inject_chan_to_fr1;
        break;
    case INJECT_DIRECTION_TO_FR2:
        dma_chan = dma_inject_chan_to_fr2;
        break;
#if !FLEXRAY_FRAME_GEN
    case INJECT_DIRECTION_TO_FR3:
        dma_chan = dma_inject_chan_to_fr3;
        break;
    case INJECT_DIRECTION_TO_FR4:
        dma_chan = dma_inject_chan_to_fr4;
        break;
#endif
    default:
        return;
    }

    // Never alter a running channel. Under the normal TDMA schedule this is
    // not expected; dropping is the safe fallback if that invariant is ever
    // violated.
    if (dma_channel_is_busy((uint)dma_chan)) {
        return;
    }

    // Publish the count word and complete frame to DMA before arming it. The
    // trans_count write below is the only operation that makes this packet
    // visible to the PIO TX FIFO.
    __dmb();
    dma_channel_set_read_addr((uint)dma_chan, tpl, false);

    injector_stats.injected++;
    dma_channel_set_trans_count((uint)dma_chan,
                                1u + (tpl->len + 3u) / 4u,
                                true);
}

uint8_t replace_bytes[254];
bool __time_critical_func(prepare_inject_frame)(uint16_t frame_id, uint8_t cycle_count)
{
    prepared_injection.pending = 0;
    uint8_t command_source = active_command_source;
    uint32_t now_us = time_us_32();

    // Find any trigger where current frame is the configured previous ID.
    for (int i = 0; i < (int)NUM_TRIGGER_RULES; i++) {
        const trigger_rule_t *rule = &INJECT_TRIGGERS[i];
        if (rule->trigger_id != frame_id) {
            continue;
        }
        if ((uint8_t)(cycle_count & rule->cycle_mask) != rule->cycle_base) {
            continue;
        }
        if (!forwarder_ready || command_source == INJECTOR_SOURCE_NONE) {
            injector_stats.disabled_skips++;
            continue;
        }

        int target_slot = find_cache_slot_for_id(rule->target_id, cycle_count);
        if (target_slot < 0) {
            continue;
        }

        frame_template_t *tpl = &TEMPLATES[target_slot];
        uint8_t *tpl_payload = tpl->data + 5;
        if (!tpl->valid || tpl->len < 8) {
            continue;
        }

        uint32_t override_timestamp_us = 0u;
        bool has_data = host_override_try_pop_for(
            rule->target_id, cycle_count, command_source, now_us,
            replace_bytes, &override_timestamp_us);
        if (!has_data) {
            continue;
        }
        if (rule->replace_len > INJECT_REPLACE_MASK_MAX ||
            (uint16_t)rule->replace_offset + rule->replace_len > rule->payload_length) {
            continue;
        }

        for (uint8_t j = 0; j < rule->replace_len; j++) {
            uint8_t mask = rule->replace_mask[j];
            uint8_t *dst = &tpl_payload[rule->replace_offset + j];
            *dst = (uint8_t)((*dst & (uint8_t)~mask) | (replace_bytes[j] & mask));
        }

        fix_cycle_count(tpl->data, cycle_count);
        fix_flexray_frame_crc(tpl->data, tpl->len);

        prepared_injection.template = tpl;
        prepared_injection.direction = rule->direction;
        prepared_injection.command_source = command_source;
        prepared_injection.override_timestamp_us = override_timestamp_us;
        prepared_injection.pending = 1;
        injector_stats.prepared++;
        return true;
    }

    return false;
}

void __time_critical_func(discard_prepared_injection)(void)
{
    prepared_injection.pending = 0;
}

void __time_critical_func(inject_prepared_frame)(void)
{
    if (!prepared_injection.pending) {
        return;
    }

    prepared_injection.pending = 0;
    if (active_command_source == prepared_injection.command_source &&
        (uint32_t)(time_us_32() - prepared_injection.override_timestamp_us) <=
            HOST_OVERRIDE_TIMEOUT_US) {
        inject_frame(prepared_injection.template, prepared_injection.direction);
    }
}

static void setup_inject_dma_channel(volatile int *chan, dma_channel_config *dc, uint sm)
{
    *chan = (int)dma_claim_unused_channel(true);
    *dc = dma_channel_get_default_config((uint)*chan);
    channel_config_set_transfer_data_size(dc, DMA_SIZE_32);
    channel_config_set_bswap(dc, true);
    channel_config_set_read_increment(dc, true);
    channel_config_set_write_increment(dc, false);
    channel_config_set_dreq(dc, pio_get_dreq(pio_forwarder_with_injector, sm, true));
    dma_channel_set_config((uint)*chan, dc, false);
    dma_channel_set_write_addr((uint)*chan, (void *)&pio_forwarder_with_injector->txf[sm], false);
}

static void setup_dma(void){
    setup_inject_dma_channel(&dma_inject_chan_to_fr1, &injector_to_fr1_dc, sm_forwarder_with_injector_to_fr1);
    setup_inject_dma_channel(&dma_inject_chan_to_fr2, &injector_to_fr2_dc, sm_forwarder_with_injector_to_fr2);
#if !FLEXRAY_FRAME_GEN
    setup_inject_dma_channel(&dma_inject_chan_to_fr3, &injector_to_fr3_dc, sm_forwarder_with_injector_to_fr3);
    setup_inject_dma_channel(&dma_inject_chan_to_fr4, &injector_to_fr4_dc, sm_forwarder_with_injector_to_fr4);
#endif
}

bool injector_submit_override(uint16_t id, uint8_t base, uint16_t len, const uint8_t *bytes)
{
    return injector_submit_override_from(id, base, len, bytes,
                                         INJECT_TRANSPORT_UNKNOWN);
}

bool injector_submit_override_from(uint16_t id, uint8_t base, uint16_t len,
                                   const uint8_t *bytes,
                                   injector_transport_t transport)
{
    if (transport == INJECT_TRANSPORT_VENDOR) injector_stats.vendor_submitted++;
    else if (transport == INJECT_TRANSPORT_UDP) injector_stats.udp_submitted++;

    // The host sends one integrity CRC byte followed by the complete payload.
    // Only the configured replacement slice is copied into the cached frame.
    if (bytes == NULL) {
        injector_stats.rejected_length++;
        return false;
    }

    if (len < 1 || len > MAX_FRAME_PAYLOAD_BYTES+1) {
        injector_stats.rejected_length++;
        return false;
    }

    uint8_t crc = calculate_host_crc8(bytes+1, 0xf1, len-1);
    if (crc != bytes[0]) {
        injector_stats.rejected_crc++;
        return false;
    }

    const trigger_rule_t *matched_rule = NULL;
    for (int i = 0; i < (int)NUM_TRIGGER_RULES; i++) {
        if (INJECT_TRIGGERS[i].target_id == id && INJECT_TRIGGERS[i].cycle_base == base) {
            matched_rule = &INJECT_TRIGGERS[i];
            break;
        }
    }
    if (matched_rule == NULL) {
        injector_stats.rejected_rule++;
        return false;
    }

    if (len != (uint16_t)matched_rule->payload_length + 1u ||
        (uint16_t)matched_rule->replace_offset + matched_rule->replace_len > matched_rule->payload_length) {
        injector_stats.rejected_length++;
        return false;
    }

    uint32_t now_us = time_us_32();
    if (!claim_command_source(INJECTOR_SOURCE_HOST, now_us)) {
        injector_stats.rejected_rule++;
        return false;
    }

    // bytes+1 skips the host integrity CRC byte.
    if (!host_override_push(id, matched_rule->cycle_mask,
                            matched_rule->cycle_base,
                            matched_rule->replace_len,
                            bytes + 1 + matched_rule->replace_offset,
                            INJECTOR_SOURCE_HOST, now_us)) {
        injector_stats.rejected_length++;
        return false;
    }
    source_last_submission_us = now_us;
    source_submission_seen = true;
    injector_stats.accepted++;
    return true;
}

static bool command_source_is_allowed(uint8_t source)
{
    if (source == INJECTOR_SOURCE_HOST) {
        return host_commands_allowed;
    }
    return false;
}

static bool claim_command_source(uint8_t source, uint32_t now_us)
{
    if (!forwarder_ready || !command_source_is_allowed(source)) {
        return false;
    }
    uint8_t current = active_command_source;
    if (current == source) {
        return true;
    }
    bool owner_fresh = source_submission_seen &&
        (uint32_t)(now_us - source_last_submission_us) <= HOST_OVERRIDE_TIMEOUT_US;
    if (current != INJECTOR_SOURCE_NONE && owner_fresh) {
        return false;
    }
    injector_clear_overrides();
    active_command_source = source;
    return true;
}

void injector_clear_overrides(void)
{
    if (host_override_lock == NULL) {
        return;
    }
    uint32_t save = spin_lock_blocking(host_override_lock);
    host_override_head = 0u;
    host_override_tail = 0u;
    memset(host_overrides, 0, sizeof(host_overrides));
    prepared_injection.pending = 0u;
    source_submission_seen = false;
    source_last_submission_us = 0u;
    spin_unlock(host_override_lock, save);
}

void injector_set_enabled(bool enabled)
{
    host_commands_allowed = enabled;
    if (!enabled) {
        active_command_source = INJECTOR_SOURCE_NONE;
        injector_clear_overrides();
    }
}

bool injector_is_enabled(void)
{
    return host_commands_allowed;
}

void injector_get_stats(injector_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }
    *stats = injector_stats;
    stats->queue_depth = host_override_count();
}

void injector_reset_stats_and_queue(void)
{
    uint32_t save = spin_lock_blocking(host_override_lock);
    host_override_head = 0;
    host_override_tail = 0;
    memset(host_overrides, 0, sizeof(host_overrides));
    memset(&injector_stats, 0, sizeof(injector_stats));
    injector_stats.magic = INJECTOR_STATS_MAGIC;
    injector_stats.version = INJECTOR_STATS_VERSION;
    injector_stats.size = sizeof(injector_stats_t);
    prepared_injection.pending = 0;
    active_command_source = INJECTOR_SOURCE_NONE;
    source_submission_seen = false;
    source_last_submission_us = 0u;
    spin_unlock(host_override_lock, save);
}

void setup_forwarder_with_injector(PIO pio,
    uint rx_pin_from_fr1, uint tx_pin_to_fr2,
    uint rx_pin_from_fr2, uint tx_pin_to_fr1,
    uint rx_pin_from_fr3, uint tx_pin_to_fr4,
    uint rx_pin_from_fr4, uint tx_pin_to_fr3)
{
    forwarder_ready = false;
    pio_forwarder_with_injector = pio;
    host_override_lock = spin_lock_instance(spin_lock_claim_unused(true));
    uint offset = pio_add_program(pio, &flexray_forwarder_with_injector_program);
#if FLEXRAY_FRAME_GEN
    frame_gen_forwarder_offset = offset;
    pio_sm_claim(pio, 2u);
    (void)rx_pin_from_fr3;
    (void)tx_pin_to_fr4;
    (void)rx_pin_from_fr4;
    (void)tx_pin_to_fr3;
#endif
    sm_forwarder_with_injector_to_fr1 = pio_claim_unused_sm(pio, true);
    sm_forwarder_with_injector_to_fr2 = pio_claim_unused_sm(pio, true);
#if !FLEXRAY_FRAME_GEN
    sm_forwarder_with_injector_to_fr3 = pio_claim_unused_sm(pio, true);
    sm_forwarder_with_injector_to_fr4 = pio_claim_unused_sm(pio, true);
#endif

    flexray_forwarder_with_injector_program_init(pio, sm_forwarder_with_injector_to_fr2, offset, rx_pin_from_fr1, tx_pin_to_fr2);
    flexray_forwarder_with_injector_program_init(pio, sm_forwarder_with_injector_to_fr1, offset, rx_pin_from_fr2, tx_pin_to_fr1);
#if !FLEXRAY_FRAME_GEN
    flexray_forwarder_with_injector_program_init(pio, sm_forwarder_with_injector_to_fr4, offset, rx_pin_from_fr3, tx_pin_to_fr4);
    flexray_forwarder_with_injector_program_init(pio, sm_forwarder_with_injector_to_fr3, offset, rx_pin_from_fr4, tx_pin_to_fr3);
#endif
    setup_dma();
    forwarder_ready = true;
}

#if FLEXRAY_FRAME_GEN
void flexray_frame_gen_forwarder_local_config(bool enable)
{
    // Reuse the original injector instructions. GPIO16 induces TSS/FSS/BSS;
    // the packet DMA supplies count + actual frame bytes to its TX FIFO.
    pio_sm_set_enabled(pio_forwarder_with_injector, 2u, false);
    if (!enable) return;
    pio_sm_config c = flexray_forwarder_with_injector_program_get_default_config(frame_gen_forwarder_offset);
    sm_config_set_set_pins(&c, TXD_FR_2_PIN, 1u);
    sm_config_set_out_pins(&c, TXD_FR_2_PIN, 1u);
    sm_config_set_in_pins(&c, FLEXRAY_INTERNAL_PIN);
    sm_config_set_jmp_pin(&c, FLEXRAY_INTERNAL_PIN);
    sm_config_set_clkdiv(&c, 1.0f);
    // Skip the initial SET pins,1 to leave any primary forwarding untouched.
    // The preceding slot preloads the FIFO; TSS/FSS still start in hardware.
    pio_sm_init(pio_forwarder_with_injector, 2u,
                frame_gen_forwarder_offset + FLEXRAY_FRAME_GEN_FORWARDER_ENTRY, &c);
    pio_sm_set_enabled(pio_forwarder_with_injector, 2u, true);
}

#endif
