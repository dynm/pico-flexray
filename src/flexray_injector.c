#include <stdint.h>
#include <string.h>
#include "flexray_frame.h"

// The injection start function is provided in flexray_bss_streamer.c (for now)
void inject_frame(uint16_t frame_id, uint8_t cycle_count, uint8_t payload_length);

// ---- Cache/Trigger configuration ----
typedef struct {
    uint16_t id;
    uint8_t  cycle_mask;
    uint8_t  cycle_base;
} cache_rule_t;

typedef struct {
    uint16_t prev_id;    // when this id arrives...
    uint16_t target_id;  // ...inject using cached template of this id (if available)
} trigger_rule_t;

// Configure your rules here
static const cache_rule_t CACHE_RULES[] = {
    { 107, 0x03, 1 }, // cache 107 when cycle_count % 4 == 1
};
static const trigger_rule_t INJECT_TRIGGERS[] = {
    { 97, 107 },       // when 97 arrives, try to inject cached 107
};

#define NUM_CACHE_RULES   (sizeof(CACHE_RULES)/sizeof(CACHE_RULES[0]))
#define NUM_TRIGGER_RULES (sizeof(INJECT_TRIGGERS)/sizeof(INJECT_TRIGGERS[0]))

typedef struct {
    uint8_t valid;  // 1 if data[] is valid
    uint8_t len;    // payload bytes + 3 CRC bytes (max 257)
    uint8_t data[MAX_FRAME_PAYLOAD_BYTES + 3];
} frame_template_t;

static frame_template_t TEMPLATES[NUM_CACHE_RULES];

static inline int find_cache_slot_for_id(uint16_t id) {
    for (int i = 0; i < (int)NUM_CACHE_RULES; i++) {
        if (CACHE_RULES[i].id == id) return i;
    }
    return -1;
}

void try_cache_last_target_frame(uint16_t frame_id, uint8_t cycle_count, uint8_t payload_length, uint8_t *captured_bytes)
{
    int slot = find_cache_slot_for_id(frame_id);
    if (slot < 0) return;

    const cache_rule_t *rule = &CACHE_RULES[slot];
    if ((uint8_t)(cycle_count & rule->cycle_mask) != rule->cycle_base) return;

    // Store payload + 3 bytes CRC (if available)
    uint16_t to_copy = (uint16_t)payload_length + 3u;
    if (to_copy > sizeof(TEMPLATES[slot].data)) to_copy = sizeof(TEMPLATES[slot].data);
    memcpy(TEMPLATES[slot].data, captured_bytes, to_copy);
    TEMPLATES[slot].len = (uint8_t)to_copy;
    TEMPLATES[slot].valid = 1;
}

void try_to_inject_frame(uint16_t frame_id, uint8_t cycle_count, uint8_t payload_length)
{
    (void)payload_length;

    // Find any trigger where current frame is the "previous" id
    for (int i = 0; i < (int)NUM_TRIGGER_RULES; i++) {
        if (INJECT_TRIGGERS[i].prev_id != frame_id) continue;

        int target_slot = find_cache_slot_for_id(INJECT_TRIGGERS[i].target_id);
        if (target_slot < 0) continue;

        frame_template_t *tpl = &TEMPLATES[target_slot];
        if (!tpl->valid || tpl->len < 3) continue;

        // Modify the 3rd byte of the cached template. Example uses current cycle_count.
        tpl->data[2] = cycle_count;

        // Start injector DMA; currently this uses the static injector_payload in streamer.
        inject_frame(INJECT_TRIGGERS[i].target_id, cycle_count, tpl->len);
        break; // fire once per triggering frame
    }
}


