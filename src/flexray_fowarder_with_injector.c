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

static frame_template_t TEMPLATES[NUM_CACHE_RULES];

static inline int find_cache_slot_for_id(uint16_t id, uint8_t cycle_count) {
    for (int i = 0; i < (int)NUM_CACHE_RULES; i++) {
        if (CACHE_RULES[i].id == id && (uint8_t)(cycle_count & CACHE_RULES[i].cycle_mask) == CACHE_RULES[i].cycle_base) return i;
    }
    return -1;
}

void try_cache_last_target_frame(uint16_t frame_id, uint8_t cycle_count, uint16_t frame_len, uint8_t *captured_bytes)
{
    int slot = find_cache_slot_for_id(frame_id, cycle_count);
    if (slot < 0){
        return;
    }

    const cache_rule_t *rule = &CACHE_RULES[slot];
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
    cycle_count = 55;
    full_frame[4] = (full_frame[4] & 0b11000000) | (cycle_count & 0x3F);
}

static void inject_frame(uint8_t *full_frame, uint16_t injector_payload_length, bool to_vehicle)
{
    // first word is length indicator, rest is payload
    // pio y-- need pre-sub 1 from length
    if (to_vehicle) {
    pio_sm_put(pio_forwarder_with_injector, sm_forwarder_with_injector_to_vehicle, injector_payload_length - 1);
    dma_channel_set_read_addr((uint)dma_inject_chan_to_vehicle, (const void *)full_frame, false);
    dma_channel_set_trans_count((uint)dma_inject_chan_to_vehicle, injector_payload_length / 4, true);
    } else {
    pio_sm_put(pio_forwarder_with_injector, sm_forwarder_with_injector_to_ecu, injector_payload_length - 1);
    dma_channel_set_read_addr((uint)dma_inject_chan_to_ecu, (const void *)full_frame, false);
    dma_channel_set_trans_count((uint)dma_inject_chan_to_ecu, injector_payload_length / 4, true);
    }
}

// flexray_frame_t dummy_frame;
// always fetch cache before store new value
void __time_critical_func(try_inject_frame)(uint16_t frame_id, uint8_t cycle_count, bool to_vehicle)
{
    // Find any trigger where current frame is the "previous" id
    for (int i = 0; i < (int)NUM_TRIGGER_RULES; i++) {
        if (INJECT_TRIGGERS[i].prev_id != frame_id){
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
        fix_cycle_count(tpl->data, cycle_count);
        // dummy_frame.payload_length_words = (tpl->len - 8) / 2;
        uint32_t new_crc = calculate_flexray_frame_crc(tpl->data, tpl->len - 3);
        tpl->data[tpl->len - 1] = (uint8_t)(new_crc);
        tpl->data[tpl->len - 2] = (uint8_t)(new_crc >> 8);
        tpl->data[tpl->len - 3] = (uint8_t)(new_crc >> 16);

        inject_frame(tpl->data, tpl->len, to_vehicle);
        break; // fire once per triggering frame
    }
}

static void setup_dma(){
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
