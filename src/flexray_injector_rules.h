#ifndef FLEXRAY_INJECTOR_RULES_H
#define FLEXRAY_INJECTOR_RULES_H

#include <stdint.h>

// ---- Cache/Trigger configuration ----
// typedef struct {
// 	uint16_t id;
// 	uint8_t  cycle_mask;
// 	uint8_t  cycle_base;
// } cache_rule_t;

typedef struct {
	uint16_t prev_id;    // when this id arrives...
	uint16_t target_id;  // ...inject using cached template of this id (if available)
	uint8_t cycle_mask;
	uint8_t cycle_base;
	uint8_t e2e_init_value;
	uint8_t replace_offset;
	uint8_t replace_len;
} trigger_rule_t;

static const trigger_rule_t INJECT_TRIGGERS[] = {
	// { 0x47, 0x48, 0b00, 1, 0xd6 },
	{ 97, 108, 0b00, 0, 0xd6, 2, 2},
};

#define NUM_TRIGGER_RULES (sizeof(INJECT_TRIGGERS)/sizeof(INJECT_TRIGGERS[0]))

#endif // FLEXRAY_INJECTOR_RULES_H


