#ifndef FLEXRAY_BLOCKER_H
#define FLEXRAY_BLOCKER_H

#include <stdint.h>
#include <stdbool.h>

#define FLEXRAY_FILTER_DIR_FR1_TO_FR2 (1u << 0)
#define FLEXRAY_FILTER_DIR_FR2_TO_FR1 (1u << 1)
#define FLEXRAY_FILTER_DIR_FR3_TO_FR4 (1u << 2)
#define FLEXRAY_FILTER_DIR_FR4_TO_FR3 (1u << 3)
#define FLEXRAY_FILTER_MAX_RULES 96

bool flexray_filter_set(uint8_t count, const uint16_t *ids, const uint8_t *direction_masks);
void flexray_filter_clear(void);
void flexray_filter_set_enabled(bool enabled);
bool flexray_filter_is_enabled(void);
bool flexray_filter_set_whitelist_defaults(uint8_t direction_mask);
uint8_t flexray_filter_block_mask_for_id(uint16_t frame_id);

void flexray_blocker_suppress_direction_mask(uint8_t direction_mask);
void flexray_blocker_release_all(void);

#endif // FLEXRAY_BLOCKER_H
