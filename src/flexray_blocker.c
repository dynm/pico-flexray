#include "flexray_blocker.h"

#include "flexray_forwarder.h"
#include "flexray_frame.h"
#include "pico/platform.h"

typedef struct {
    uint16_t frame_id;
    uint8_t direction_mask;
} flexray_filter_rule_t;

static volatile bool flexray_filter_enabled = true;
static volatile uint8_t flexray_filter_whitelist_direction_mask = 0;
static volatile uint8_t flexray_filter_rule_count = 0;
static volatile flexray_filter_rule_t flexray_filter_rules[FLEXRAY_FILTER_MAX_RULES];

bool flexray_filter_set(uint8_t count, const uint16_t *ids, const uint8_t *direction_masks)
{
    if (count > FLEXRAY_FILTER_MAX_RULES || ids == NULL || direction_masks == NULL) {
        return false;
    }

    flexray_filter_rule_count = 0;
    for (uint8_t i = 0; i < count; i++) {
        if (ids[i] >= 2048u) {
            return false;
        }
        flexray_filter_rules[i].frame_id = ids[i];
        flexray_filter_rules[i].direction_mask = direction_masks[i] &
            (FLEXRAY_FILTER_DIR_FR1_TO_FR2 |
             FLEXRAY_FILTER_DIR_FR2_TO_FR1 |
             FLEXRAY_FILTER_DIR_FR3_TO_FR4 |
             FLEXRAY_FILTER_DIR_FR4_TO_FR3);
    }
    flexray_filter_rule_count = count;
    return true;
}

void flexray_filter_clear(void)
{
    flexray_filter_rule_count = 0;
    flexray_filter_whitelist_direction_mask = 0;
}

void flexray_filter_set_enabled(bool enabled)
{
    flexray_filter_enabled = enabled;
}

bool flexray_filter_is_enabled(void)
{
    return flexray_filter_enabled;
}

bool flexray_filter_set_whitelist_defaults(uint8_t direction_mask)
{
    uint8_t valid_mask = FLEXRAY_FILTER_DIR_FR1_TO_FR2 |
                         FLEXRAY_FILTER_DIR_FR2_TO_FR1 |
                         FLEXRAY_FILTER_DIR_FR3_TO_FR4 |
                         FLEXRAY_FILTER_DIR_FR4_TO_FR3;
    flexray_filter_whitelist_direction_mask = direction_mask & valid_mask;
    return true;
}

uint8_t __time_critical_func(flexray_filter_block_mask_for_id)(uint16_t frame_id)
{
    if (!flexray_filter_enabled) {
        return 0;
    }

    uint8_t matched_mask = 0;
    uint8_t count = flexray_filter_rule_count;
    for (uint8_t i = 0; i < count; i++) {
        if (flexray_filter_rules[i].frame_id == frame_id) {
            matched_mask |= flexray_filter_rules[i].direction_mask;
        }
    }

    if (flexray_filter_whitelist_direction_mask != 0u) {
        return (uint8_t)(flexray_filter_whitelist_direction_mask & (uint8_t)~matched_mask);
    }

    return matched_mask;
}

void __time_critical_func(flexray_blocker_suppress_direction_mask)(uint8_t direction_mask)
{
    if ((direction_mask & FLEXRAY_FILTER_DIR_FR1_TO_FR2) != 0u) {
        flexray_forwarder_suppress_source(FROM_FR1);
    }
    if ((direction_mask & FLEXRAY_FILTER_DIR_FR2_TO_FR1) != 0u) {
        flexray_forwarder_suppress_source(FROM_FR2);
    }
    if ((direction_mask & FLEXRAY_FILTER_DIR_FR3_TO_FR4) != 0u) {
        flexray_forwarder_suppress_source(FROM_FR3);
    }
    if ((direction_mask & FLEXRAY_FILTER_DIR_FR4_TO_FR3) != 0u) {
        flexray_forwarder_suppress_source(FROM_FR4);
    }
}

void __time_critical_func(flexray_blocker_release_all)(void)
{
    flexray_forwarder_release_all_suppressed();
}
