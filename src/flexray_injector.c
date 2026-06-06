#include <stdint.h>
#include <string.h>

#include "pico.h"

#include "flexray_blocker.h"
#include "flexray_forwarder.h"
#include "flexray_frame.h"
#include "flexray_injector.h"
#include "flexray_injector_rules.h"

#define INJECT_FRAME_BYTES (MAX_FRAME_PAYLOAD_BYTES + 8)
#define INJECT_FRAME_PADDED_BYTES ((INJECT_FRAME_BYTES + 3) & ~3)

_Static_assert((INJECT_FRAME_PADDED_BYTES % sizeof(uint32_t)) == 0,
               "DMA frame storage must be padded to 32-bit words");

typedef struct {
    uint8_t valid;
    uint16_t len;
    uint8_t data[INJECT_FRAME_PADDED_BYTES] __attribute__((aligned(4)));
} frame_template_t;

typedef struct {
    uint8_t pending;
    uint8_t active;
    uint8_t direction;
    uint8_t active_block_exclusion_mask;
    uint16_t len;
    uint16_t target_id;
    uint8_t cycle_count;
    uint16_t active_target_id;
    uint8_t active_cycle_count;
    uint8_t *data;
} prepared_injection_t;

typedef struct {
    uint8_t valid;
    uint16_t id;
    uint8_t mask;
    uint8_t base;
    uint16_t len;
    uint8_t data[MAX_FRAME_PAYLOAD_BYTES + 8];
} host_override_t;

#define HOST_OVERRIDE_CAP 4

static frame_template_t TEMPLATES[NUM_TRIGGER_RULES];
static prepared_injection_t prepared_injection;
static volatile uint32_t host_override_head = 0;
static volatile uint32_t host_override_tail = 0;
static host_override_t host_overrides[HOST_OVERRIDE_CAP];
static volatile bool injector_enabled = true;
static uint8_t replace_bytes[254];

static inline uint8_t block_exclusion_mask_for_inject_direction(uint8_t direction)
{
    switch (direction) {
    case INJECT_DIRECTION_TO_FR1:
        return FLEXRAY_FILTER_DIR_FR2_TO_FR1;
    case INJECT_DIRECTION_TO_FR2:
        return FLEXRAY_FILTER_DIR_FR1_TO_FR2;
    case INJECT_DIRECTION_TO_FR3:
        return FLEXRAY_FILTER_DIR_FR4_TO_FR3;
    case INJECT_DIRECTION_TO_FR4:
        return FLEXRAY_FILTER_DIR_FR3_TO_FR4;
    default:
        return 0;
    }
}

static inline bool host_override_push(uint16_t id, uint8_t mask, uint8_t base, uint16_t len, const uint8_t *bytes)
{
    uint32_t next_head = (host_override_head + 1u) % HOST_OVERRIDE_CAP;
    if (next_head == host_override_tail) {
        host_override_tail = (host_override_tail + 1u) % HOST_OVERRIDE_CAP;
    }

    host_override_t *slot = &host_overrides[host_override_head];
    slot->id = id;
    slot->mask = mask;
    slot->base = base;
    slot->len = len;
    if (len > sizeof(slot->data)) {
        len = sizeof(slot->data);
    }
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
        uint8_t valid = __atomic_load_n(&slot->valid, __ATOMIC_ACQUIRE);
        if (valid && slot->id == id && (uint8_t)(cycle_count & slot->mask) == slot->base) {
            memcpy(out, slot->data, slot->len);
            slot->valid = 0;
            host_override_tail = (t + 1u) % HOST_OVERRIDE_CAP;
            return true;
        }
        t = (t + 1u) % HOST_OVERRIDE_CAP;
    }
    return false;
}

static inline int find_cache_slot_for_id(uint16_t id, uint8_t cycle_count)
{
    for (int i = 0; i < (int)NUM_TRIGGER_RULES; i++) {
        if (INJECT_TRIGGERS[i].target_id == id &&
            (uint8_t)(cycle_count & INJECT_TRIGGERS[i].cycle_mask) == INJECT_TRIGGERS[i].cycle_base) {
            return i;
        }
    }
    return -1;
}

static void fix_cycle_count(uint8_t *full_frame, uint8_t cycle_count)
{
    full_frame[4] = (full_frame[4] & 0b11000000) | (cycle_count & 0x3F);
}

static void fix_e2e_payload(uint8_t *e2e_start_offset, uint8_t init_value, uint8_t len)
{
    uint8_t nibble = (e2e_start_offset[1] & 0x0F) + 1;
    if (nibble == 0x0F) {
        nibble = 0;
    }
    e2e_start_offset[1] = (e2e_start_offset[1] & 0xF0) | (nibble & 0x0F);
    e2e_start_offset[0] = calculate_autosar_e2e_crc8(e2e_start_offset + 1, init_value, len);
}

void try_cache_last_target_frame(uint16_t frame_id, uint8_t cycle_count, uint16_t frame_len, uint8_t *captured_bytes)
{
    int slot = find_cache_slot_for_id(frame_id, cycle_count);
    if (slot < 0) {
        return;
    }

    const trigger_rule_t *rule = &INJECT_TRIGGERS[slot];
    if ((uint8_t)(cycle_count & rule->cycle_mask) != rule->cycle_base) {
        return;
    }

    if (frame_len > sizeof(TEMPLATES[slot].data)) {
        return;
    }
    memcpy(TEMPLATES[slot].data, captured_bytes, frame_len);
    TEMPLATES[slot].len = frame_len;
    TEMPLATES[slot].valid = 1;
}

bool __time_critical_func(prepare_inject_frame)(uint16_t frame_id, uint8_t cycle_count)
{
    prepared_injection.pending = 0;

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
        if (!tpl->valid || tpl->len < 8) {
            continue;
        }

        if (!host_override_try_pop_for(rule->target_id, cycle_count, replace_bytes)) {
            continue;
        }

        uint8_t *tpl_payload = tpl->data + 5;
        memcpy(tpl_payload + rule->replace_offset, replace_bytes, rule->replace_len);
        fix_e2e_payload(tpl_payload + rule->e2e_offset, rule->e2e_init_value, rule->e2e_len);
        fix_cycle_count(tpl->data, cycle_count);
        fix_flexray_frame_crc(tpl->data, tpl->len);

        prepared_injection.data = tpl->data;
        prepared_injection.len = tpl->len;
        prepared_injection.direction = rule->direction;
        prepared_injection.target_id = rule->target_id;
        prepared_injection.cycle_count = cycle_count;
        prepared_injection.pending = 1;
        return true;
    }
    return false;
}

void __time_critical_func(inject_prepared_frame)(void)
{
    if (!prepared_injection.pending) {
        return;
    }

    prepared_injection.pending = 0;
    prepared_injection.active = 1;
    prepared_injection.active_target_id = prepared_injection.target_id;
    prepared_injection.active_cycle_count = prepared_injection.cycle_count;
    prepared_injection.active_block_exclusion_mask = block_exclusion_mask_for_inject_direction(prepared_injection.direction);
    flexray_forwarder_inject_frame(prepared_injection.data, prepared_injection.len, prepared_injection.direction);
}

void __time_critical_func(injector_note_frame_end)(void)
{
    prepared_injection.active = 0;
    prepared_injection.active_block_exclusion_mask = 0;
}

uint8_t __time_critical_func(injector_block_exclusion_mask_for_frame)(uint16_t frame_id, uint8_t cycle_count)
{
    if (!prepared_injection.active) {
        return 0;
    }
    if (prepared_injection.active_target_id != frame_id) {
        return 0;
    }
    if (prepared_injection.active_cycle_count != cycle_count) {
        return 0;
    }
    return prepared_injection.active_block_exclusion_mask;
}

bool injector_submit_override(uint16_t id, uint8_t base, uint16_t len, const uint8_t *bytes)
{
    if (bytes == NULL || len < 1 || len > MAX_FRAME_PAYLOAD_BYTES + 1) {
        return false;
    }

    uint8_t crc = calculate_autosar_e2e_crc8(bytes + 1, 0xf1, len - 1);
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

    len = len - 1 - matched_rule->replace_offset;
    if (len != matched_rule->replace_len) {
        return false;
    }

    return host_override_push(id,
                              matched_rule->cycle_mask,
                              matched_rule->cycle_base,
                              matched_rule->replace_len,
                              bytes + 1 + matched_rule->replace_offset);
}

void injector_set_enabled(bool enabled)
{
    injector_enabled = enabled;
}

bool injector_is_enabled(void)
{
    return injector_enabled;
}
