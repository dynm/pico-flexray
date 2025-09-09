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
static uint sm_forwarder_with_injector_to_vehicle;
static uint sm_forwarder_with_injector_to_ecu;

static int dma_inject_chan_to_vehicle = -1;
static int dma_inject_chan_to_ecu = -1;
static dma_channel_config injector_to_vehicle_dc;
static dma_channel_config injector_to_ecu_dc;

// rules now come from flexray_injector_rules.h

typedef struct {
    uint8_t valid;  // 1 if data[] is valid
    uint16_t len;    // header + payload bytes + 3 CRC bytes (max 262)
    uint8_t data[MAX_FRAME_PAYLOAD_BYTES + 8];
} frame_template_t;

static frame_template_t TEMPLATES[NUM_TRIGGER_RULES];

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
    memcpy(TEMPLATES[slot].data, captured_bytes, frame_len);
    TEMPLATES[slot].len = (uint16_t)frame_len;
    TEMPLATES[slot].valid = 1;
}

static void fix_cycle_count(uint8_t *full_frame, uint8_t cycle_count)
{
    // set full_frame[4] low 6 bits to cycle_count
    full_frame[4] = (full_frame[4] & 0b11000000) | (cycle_count & 0x3F);
}

static void fix_e2e_payload(uint8_t *full_frame, uint8_t init_value, uint8_t len)
{
    // advance e2e alive counter lower nibble
    uint8_t nibble = (full_frame[6] & 0x0F) + 1;
    if (nibble == 0x0F) {
        nibble = 0;
    }
    full_frame[6] = (full_frame[6] & 0xF0) | (nibble & 0x0F);
    full_frame[5] = calculate_autosar_e2e_crc8(full_frame+6, init_value, len);
}

static void inject_frame(uint8_t *full_frame, uint16_t injector_payload_length, uint8_t direction)
{
    // first word is length indicator, rest is payload
    // pio y-- need pre-sub 1 from length
    if (direction == INJECT_DIRECTION_TO_VEHICLE) {
    pio_sm_put(pio_forwarder_with_injector, sm_forwarder_with_injector_to_vehicle, injector_payload_length - 1);
    dma_channel_set_read_addr((uint)dma_inject_chan_to_vehicle, (const void *)full_frame, false);
    dma_channel_set_trans_count((uint)dma_inject_chan_to_vehicle, injector_payload_length / 4, true);
    } else if (direction == INJECT_DIRECTION_TO_ECU) {
    pio_sm_put(pio_forwarder_with_injector, sm_forwarder_with_injector_to_ecu, injector_payload_length - 1);
    dma_channel_set_read_addr((uint)dma_inject_chan_to_ecu, (const void *)full_frame, false);
    dma_channel_set_trans_count((uint)dma_inject_chan_to_ecu, injector_payload_length / 4, true);
    } else {
        return;
    }
}

// flexray_frame_t dummy_frame;
// always fetch cache before store new value
uint8_t replace_bytes[32];
void __time_critical_func(try_inject_frame)(uint16_t frame_id, uint8_t cycle_count)
{
    // Find any trigger where current frame is the "previous" id
    for (int i = 0; i < (int)NUM_TRIGGER_RULES; i++) {
        if (INJECT_TRIGGERS[i].prev_id != frame_id){
            continue;
        }
        if ((uint8_t)(cycle_count & INJECT_TRIGGERS[i].cycle_mask) != INJECT_TRIGGERS[i].cycle_base){
            continue;
        }

        int target_slot = find_cache_slot_for_id(INJECT_TRIGGERS[i].target_id, cycle_count);
        if (target_slot < 0){
            continue;
        }

        frame_template_t *tpl = &TEMPLATES[target_slot];
        if (!tpl->valid || tpl->len < 8){
            continue;
        }

        bool has_data = host_override_try_pop_for(INJECT_TRIGGERS[i].target_id, cycle_count, replace_bytes);
        if (!has_data) {
            continue;
        }

        memcpy(tpl->data+5+INJECT_TRIGGERS[i].replace_offset, replace_bytes, INJECT_TRIGGERS[i].replace_len);
    
        fix_e2e_payload(tpl->data, INJECT_TRIGGERS[i].e2e_init_value, tpl->len - 8 - 1);
        fix_cycle_count(tpl->data, cycle_count);
        fix_flexray_frame_crc(tpl->data, tpl->len);
        inject_frame(tpl->data, tpl->len, INJECT_TRIGGERS[i].direction);
        break; // fire once per triggering frame
        
    }
}

static void setup_dma(void){
    dma_inject_chan_to_vehicle = (int)dma_claim_unused_channel(true);
    dma_inject_chan_to_ecu = (int)dma_claim_unused_channel(true);
    
    injector_to_vehicle_dc = dma_channel_get_default_config((uint)dma_inject_chan_to_vehicle);
    channel_config_set_transfer_data_size(&injector_to_vehicle_dc, DMA_SIZE_32);
    channel_config_set_bswap(&injector_to_vehicle_dc, true);
    channel_config_set_read_increment(&injector_to_vehicle_dc, true);
    channel_config_set_write_increment(&injector_to_vehicle_dc, false);
    channel_config_set_dreq(&injector_to_vehicle_dc, pio_get_dreq(pio_forwarder_with_injector, sm_forwarder_with_injector_to_vehicle, true)); // TX pacing
    dma_channel_set_config((uint)dma_inject_chan_to_vehicle, &injector_to_vehicle_dc, false);        
    dma_channel_set_write_addr((uint)dma_inject_chan_to_vehicle, (void *)&pio2->txf[sm_forwarder_with_injector_to_vehicle], false);
    
    injector_to_ecu_dc = dma_channel_get_default_config((uint)dma_inject_chan_to_ecu);
    channel_config_set_transfer_data_size(&injector_to_ecu_dc, DMA_SIZE_32);
    channel_config_set_bswap(&injector_to_ecu_dc, true);
    channel_config_set_read_increment(&injector_to_ecu_dc, true);
    channel_config_set_write_increment(&injector_to_ecu_dc, false);
    channel_config_set_dreq(&injector_to_ecu_dc, pio_get_dreq(pio_forwarder_with_injector, sm_forwarder_with_injector_to_ecu, true)); // TX pacing
    dma_channel_set_config((uint)dma_inject_chan_to_ecu, &injector_to_ecu_dc, false);        
    dma_channel_set_write_addr((uint)dma_inject_chan_to_ecu, (void *)&pio2->txf[sm_forwarder_with_injector_to_ecu], false);

}

bool injector_submit_override(uint16_t id, uint8_t base, uint16_t len, const uint8_t *bytes)
{
    // Host should provide only the replacement slice, not a full frame.
    // Match the provided id/base against a trigger rule's target_id/cycle_base
    // and enforce len == rule->replace_len. We use the rule's cycle_mask/cycle_base.
    if (bytes == NULL) {
        return false;
    }

    if (len < 3 || len > MAX_FRAME_PAYLOAD_BYTES+3) {
        return false;
    }

    len -= 3;

    // check if bytes[0] == base
    if (bytes[0] != base) {
        return false;
    }

    // check if bytes[1]|bytes[2]<<8 == id
    uint16_t id16 = (bytes[1]) | ((bytes[2]&0b111) << 8);
    if (id16 != id) {
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
    if (len != matched_rule->replace_len) {
        return false;
    }
    return host_override_push(id, matched_rule->cycle_mask, matched_rule->cycle_base, len, bytes+3);
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
    uint rx_pin_from_ecu, uint tx_pin_to_vehicle,
    uint rx_pin_from_vehicle, uint tx_pin_to_ecu)
{
    pio_forwarder_with_injector = pio;
    uint offset = pio_add_program(pio, &flexray_forwarder_with_injector_program);
    sm_forwarder_with_injector_to_vehicle = pio_claim_unused_sm(pio, true);
    sm_forwarder_with_injector_to_ecu = pio_claim_unused_sm(pio, true);

    flexray_forwarder_with_injector_program_init(pio, sm_forwarder_with_injector_to_vehicle, offset, rx_pin_from_ecu, tx_pin_to_vehicle);
    flexray_forwarder_with_injector_program_init(pio, sm_forwarder_with_injector_to_ecu, offset, rx_pin_from_vehicle, tx_pin_to_ecu);
    setup_dma();
}
