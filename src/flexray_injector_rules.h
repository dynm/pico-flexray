#ifndef FLEXRAY_INJECTOR_RULES_H
#define FLEXRAY_INJECTOR_RULES_H

#include <stdint.h>

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
	uint8_t e2e_offset;
	uint8_t e2e_len;
	uint8_t e2e_init_value;
	uint8_t data_id[16];
	uint8_t replace_offset;
	uint8_t replace_len;
	uint8_t replace_mask[INJECT_REPLACE_MASK_MAX];
	uint8_t direction;
} trigger_rule_t;

static const trigger_rule_t INJECT_TRIGGERS[] = {
	// I connect the ECU side to the Domain Controller, so reverse the direction
	{
		.trigger_id = 0x06,
		.target_id = 0x08,
		.cycle_mask = 0x03,
		.cycle_base = 0x02,
		.payload_length = 18,
		.e2e_offset = 10,
		.e2e_len = 7,
		.e2e_init_value = 0xff,
		.data_id = {
			0x4B, 0x98, 0xE5, 0x37, 0x84, 0xD1, 0x23, 0x70,
			0xBD, 0x0F, 0x5C, 0xA9, 0xF6, 0x48, 0x95, 0xE2,
		},
		.replace_offset = 13,
		.replace_len = 4,
		// Preserve unrelated bits 104..106 and 132..135 around the DBC fields.
		.replace_mask = {0xF8, 0xFF, 0xFF, 0x0F},
		.direction = INJECT_DIRECTION_TO_FR3,
	},
};

#define NUM_TRIGGER_RULES (sizeof(INJECT_TRIGGERS)/sizeof(INJECT_TRIGGERS[0]))

#endif // FLEXRAY_INJECTOR_RULES_H
