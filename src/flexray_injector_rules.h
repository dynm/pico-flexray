#ifndef FLEXRAY_INJECTOR_RULES_H
#define FLEXRAY_INJECTOR_RULES_H

#include <stdint.h>

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

// rule sets
static const cache_rule_t CACHE_RULES[] = {
	{ 108, 0b00, 0 },
};

static const trigger_rule_t INJECT_TRIGGERS[] = {
	{ 97, 108 },
};

#define NUM_CACHE_RULES   (sizeof(CACHE_RULES)/sizeof(CACHE_RULES[0]))
#define NUM_TRIGGER_RULES (sizeof(INJECT_TRIGGERS)/sizeof(INJECT_TRIGGERS[0]))

#endif // FLEXRAY_INJECTOR_RULES_H


