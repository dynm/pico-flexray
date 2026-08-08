#include <stdint.h>
#include <string.h>
#include "hardware/dma.h"
#include "flexray_frame.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"


#include "flexray_forwarder_with_injector.pio.h"
#include "flexray_forwarder_with_injector.h"
#include "flexray_injector_rules.h"

static PIO pio_forwarder_with_injector;
static uint sm_forwarder_with_injector_to_fr1;
static uint sm_forwarder_with_injector_to_fr2;
static uint sm_forwarder_with_injector_to_fr3;
static uint sm_forwarder_with_injector_to_fr4;

extern volatile int dma_inject_chan_to_fr1;
extern volatile int dma_inject_chan_to_fr2;
extern volatile int dma_inject_chan_to_fr3;
extern volatile int dma_inject_chan_to_fr4;
static dma_channel_config injector_to_fr1_dc;
static dma_channel_config injector_to_fr2_dc;
static dma_channel_config injector_to_fr3_dc;
static dma_channel_config injector_to_fr4_dc;

// rules now come from flexray_injector_rules.h

#define INJECT_FRAME_BYTES (MAX_FRAME_PAYLOAD_BYTES + 8)
#define INJECT_FRAME_PADDED_BYTES ((INJECT_FRAME_BYTES + 3) & ~3)

_Static_assert((INJECT_FRAME_PADDED_BYTES % sizeof(uint32_t)) == 0,
               "DMA frame storage must be padded to 32-bit words");

typedef struct {
    uint8_t valid;  // 1 if data[] is valid
    uint16_t len;    // header + payload bytes + 3 CRC bytes (max 262)
    uint8_t data[INJECT_FRAME_PADDED_BYTES] __attribute__((aligned(4)));
} frame_template_t;

typedef struct {
    uint8_t pending;
    uint8_t direction;
    uint16_t len;
    uint8_t *data;
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
    uint8_t data[MAX_FRAME_PAYLOAD_BYTES + 8];
} host_override_t;

#define HOST_OVERRIDE_CAP 4
static volatile uint32_t host_override_head = 0;
static volatile uint32_t host_override_tail = 0;
static host_override_t host_overrides[HOST_OVERRIDE_CAP];
static volatile bool injector_enabled = true;

static inline bool host_override_push(uint16_t id, uint8_t mask, uint8_t base, uint16_t len, const uint8_t *bytes)
{
    uint32_t next_head = (host_override_head + 1u) % HOST_OVERRIDE_CAP;
    if (next_head == host_override_tail) {
        // drop oldest on overflow
        host_override_tail = (host_override_tail + 1u) % HOST_OVERRIDE_CAP;
    }
    host_override_t *slot = &host_overrides[host_override_head];
    slot->id = id;
    slot->mask = mask;
    slot->base = base;
    slot->len = len;
    if (len > sizeof(slot->data)) len = sizeof(slot->data);
    memcpy(slot->data, bytes, len);
    __atomic_store_n(&slot->valid, 1, __ATOMIC_RELEASE);
    host_override_head = next_head;
    return true;
}

static inline bool host_override_try_pop_for(uint16_t id, uint8_t cycle_count, uint8_t *out)
{
    uint32_t t = host_override_tail;
    while (t != host_override_head) {
        host_override_t *slot = &host_overrides[t];
        uint8_t v = __atomic_load_n(&slot->valid, __ATOMIC_ACQUIRE);
        if (v && slot->id == id && (uint8_t)(cycle_count & slot->mask) == slot->base) {
            memcpy(out, slot->data, slot->len);
            // invalidate and advance tail to this+1
            slot->valid = 0;
            host_override_tail = (t + 1u) % HOST_OVERRIDE_CAP;
            return true;
        }
        t = (t + 1u) % HOST_OVERRIDE_CAP;
    }
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
    TEMPLATES[slot].valid = 1;
}

static void fix_cycle_count(uint8_t *full_frame, uint8_t cycle_count)
{
    // set full_frame[4] low 6 bits to cycle_count
    full_frame[4] = (full_frame[4] & 0b11000000) | (cycle_count & 0x3F);
}

static void fix_e2e_payload(uint8_t *e2e_start_offset, uint8_t init_value,
                            uint8_t len, const uint8_t data_id[16])
{
    // advance e2e alive counter lower nibble
    uint8_t nibble = (e2e_start_offset[1]+1) & 0x0F;
    uint8_t payload_to_crc8[8];
    memcpy(payload_to_crc8, e2e_start_offset+1, len);
    payload_to_crc8[len] = data_id[nibble];
    e2e_start_offset[1] = (e2e_start_offset[1] & 0xF0) | nibble;
    payload_to_crc8[0] = e2e_start_offset[1];
    e2e_start_offset[0] = calculate_autosar_e2e_crc8_0x2f(payload_to_crc8, init_value, (uint8_t)(len + 1u));
}

static void inject_frame(uint8_t *full_frame, uint16_t injector_payload_length, uint8_t direction)
{
    uint sm;
    int dma_chan;

    switch (direction) {
    case INJECT_DIRECTION_TO_FR1:
        sm = sm_forwarder_with_injector_to_fr1;
        dma_chan = dma_inject_chan_to_fr1;
        break;
    case INJECT_DIRECTION_TO_FR2:
        sm = sm_forwarder_with_injector_to_fr2;
        dma_chan = dma_inject_chan_to_fr2;
        break;
    case INJECT_DIRECTION_TO_FR3:
        sm = sm_forwarder_with_injector_to_fr3;
        dma_chan = dma_inject_chan_to_fr3;
        break;
    case INJECT_DIRECTION_TO_FR4:
        sm = sm_forwarder_with_injector_to_fr4;
        dma_chan = dma_inject_chan_to_fr4;
        break;
    default:
        return;
    }

    pio_sm_put(pio_forwarder_with_injector, sm, injector_payload_length - 1);
    dma_channel_set_read_addr((uint)dma_chan, (const void *)full_frame, false);
    dma_channel_set_trans_count((uint)dma_chan, (injector_payload_length + 3) / 4, true);
}

uint8_t replace_bytes[254];
bool __time_critical_func(prepare_inject_frame)(uint16_t frame_id, uint8_t cycle_count)
{
    prepared_injection.pending = 0;

    // Find any trigger where current frame is the configured previous ID.
    for (int i = 0; i < (int)NUM_TRIGGER_RULES; i++) {
        const trigger_rule_t *rule = &INJECT_TRIGGERS[i];
        if (rule->trigger_id != frame_id) {
            continue;
        }
        if ((uint8_t)(cycle_count & rule->cycle_mask) != rule->cycle_base) {
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

        bool has_data = host_override_try_pop_for(rule->target_id, cycle_count, replace_bytes);
        if (!has_data) {
            continue;
        }
        if (rule->replace_len > INJECT_REPLACE_MASK_MAX ||
            (uint16_t)rule->replace_offset + rule->replace_len > rule->payload_length ||
            (uint16_t)rule->e2e_len + 1u > 8u ||
            (uint16_t)rule->e2e_offset + 1u + rule->e2e_len > rule->payload_length) {
            continue;
        }

        for (uint8_t j = 0; j < rule->replace_len; j++) {
            uint8_t mask = rule->replace_mask[j];
            uint8_t *dst = &tpl_payload[rule->replace_offset + j];
            *dst = (uint8_t)((*dst & (uint8_t)~mask) | (replace_bytes[j] & mask));
        }

        fix_e2e_payload(tpl_payload + rule->e2e_offset,
                        rule->e2e_init_value,
                        rule->e2e_len,
                        rule->data_id);
        fix_cycle_count(tpl->data, cycle_count);
        fix_flexray_frame_crc(tpl->data, tpl->len);

        prepared_injection.data = tpl->data;
        prepared_injection.len = tpl->len;
        prepared_injection.direction = rule->direction;
        prepared_injection.pending = 1;
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
    inject_frame(prepared_injection.data,
                 prepared_injection.len,
                 prepared_injection.direction);
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
    setup_inject_dma_channel(&dma_inject_chan_to_fr3, &injector_to_fr3_dc, sm_forwarder_with_injector_to_fr3);
    setup_inject_dma_channel(&dma_inject_chan_to_fr4, &injector_to_fr4_dc, sm_forwarder_with_injector_to_fr4);
}

bool injector_submit_override(uint16_t id, uint8_t base, uint16_t len, const uint8_t *bytes)
{
    // The host sends one integrity CRC byte followed by the complete payload.
    // Only the configured replacement slice is copied into the cached frame.
    if (bytes == NULL) {
        return false;
    }

    if (len < 1 || len > MAX_FRAME_PAYLOAD_BYTES+1) {
        return false;
    }

    uint8_t crc = calculate_autosar_e2e_crc8(bytes+1, 0xf1, len-1);
    if (crc != bytes[0]) {
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
        return false;
    }

    if (len != (uint16_t)matched_rule->payload_length + 1u ||
        (uint16_t)matched_rule->replace_offset + matched_rule->replace_len > matched_rule->payload_length) {
        return false;
    }

    // bytes+1 skips the host integrity CRC byte.
    return host_override_push(id, matched_rule->cycle_mask, matched_rule->cycle_base, matched_rule->replace_len, bytes + 1 + matched_rule->replace_offset);
}

void injector_set_enabled(bool enabled)
{
    injector_enabled = enabled;
}

bool injector_is_enabled(void)
{
    return injector_enabled;
}

void setup_forwarder_with_injector(PIO pio,
    uint rx_pin_from_fr1, uint tx_pin_to_fr2,
    uint rx_pin_from_fr2, uint tx_pin_to_fr1,
    uint rx_pin_from_fr3, uint tx_pin_to_fr4,
    uint rx_pin_from_fr4, uint tx_pin_to_fr3)
{
    pio_forwarder_with_injector = pio;
    uint offset = pio_add_program(pio, &flexray_forwarder_with_injector_program);
    sm_forwarder_with_injector_to_fr1 = pio_claim_unused_sm(pio, true);
    sm_forwarder_with_injector_to_fr2 = pio_claim_unused_sm(pio, true);
    sm_forwarder_with_injector_to_fr3 = pio_claim_unused_sm(pio, true);
    sm_forwarder_with_injector_to_fr4 = pio_claim_unused_sm(pio, true);

    flexray_forwarder_with_injector_program_init(pio, sm_forwarder_with_injector_to_fr2, offset, rx_pin_from_fr1, tx_pin_to_fr2);
    flexray_forwarder_with_injector_program_init(pio, sm_forwarder_with_injector_to_fr1, offset, rx_pin_from_fr2, tx_pin_to_fr1);
    flexray_forwarder_with_injector_program_init(pio, sm_forwarder_with_injector_to_fr4, offset, rx_pin_from_fr3, tx_pin_to_fr4);
    flexray_forwarder_with_injector_program_init(pio, sm_forwarder_with_injector_to_fr3, offset, rx_pin_from_fr4, tx_pin_to_fr3);
    setup_dma();
}
