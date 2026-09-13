#ifndef FLEXRAY_INJECTOR_RULES_H
#define FLEXRAY_INJECTOR_RULES_H

#include <stdint.h>
#include "board_config.h"

#define INJECT_DIRECTION_TO_FR1 0
#define INJECT_DIRECTION_TO_FR2 1
#define INJECT_DIRECTION_TO_FR3 2
#define INJECT_DIRECTION_TO_FR4 3
#define INJECT_REPLACE_MASK_MAX 8
typedef struct {
	uint16_t trigger_id;    // when this id arrives...
	uint16_t target_id;  // ...inject using cached template of this id (if available)
	uint8_t cycle_mask;
	uint8_t cycle_base;
	uint8_t payload_length;
	uint8_t replace_offset;
	uint8_t replace_len;
	uint8_t replace_mask[INJECT_REPLACE_MASK_MAX];
	uint8_t direction;
} trigger_rule_t;

static const trigger_rule_t INJECT_TRIGGERS[] = {
	// Synthetic demo: replace the first four payload bytes in a reserved target slot.
	{
		.trigger_id = 0x06,
		.target_id = 0x08,
		.cycle_mask = 0x03,
		.cycle_base = 0x02,
		.payload_length = 18,
		.replace_offset = 0,
		.replace_len = 4,
		// Remaining payload bytes are copied unchanged from the observed template.
		.replace_mask = {0xFF, 0xFF, 0xFF, 0xFF},
		.direction = FLEXRAY_FRAME_GEN ? INJECT_DIRECTION_TO_FR1 : INJECT_DIRECTION_TO_FR3,
	},
};

#define NUM_TRIGGER_RULES (sizeof(INJECT_TRIGGERS)/sizeof(INJECT_TRIGGERS[0]))

#endif // FLEXRAY_INJECTOR_RULES_H
