#ifndef FLEXRAY_INJECTOR_RULES_H
#define FLEXRAY_INJECTOR_RULES_H

#include <stdint.h>

#define INJECT_DIRECTION_TO_VEHICLE 1
#define INJECT_DIRECTION_TO_ECU 0
typedef struct {
	uint16_t trigger_id;    // when this id arrives...
	uint16_t target_id;  // ...inject using cached template of this id (if available)
	uint8_t cycle_mask;
	uint8_t cycle_base;
	uint8_t e2e_offset;
	uint8_t e2e_len;
	uint8_t e2e_init_value;
	uint8_t replace_offset;
	uint8_t replace_len;
	uint8_t direction;
} trigger_rule_t;

static const trigger_rule_t INJECT_TRIGGERS[] = {
	// I connect the ECU side to the Domain Controller, so reverse the direction
	{
		.trigger_id = 0x47,
		.target_id = 0x48,
		.cycle_mask = 0b11,
		.cycle_base = 1,
		.e2e_offset = 0,
		.e2e_len = 15,
		.e2e_init_value = 0xd6,
		.replace_offset = 2,
		.replace_len = 14,
		.direction = INJECT_DIRECTION_TO_ECU,
	},
};

#define NUM_TRIGGER_RULES (sizeof(INJECT_TRIGGERS)/sizeof(INJECT_TRIGGERS[0]))

#endif // FLEXRAY_INJECTOR_RULES_H


