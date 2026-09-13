#ifndef FLEXRAY_FRAME_GEN_RULES_H
#define FLEXRAY_FRAME_GEN_RULES_H
#include <stdint.h>

// Assigned FR2 slots. Null frames occupy these slots on EVERY cycle.
// Host payload is single-use and permitted only at cycle_base modulo cycle_rep.
#define FLEXRAY_FRAME_GEN_PAYLOAD_BYTES 18u
#define FLEXRAY_FRAME_GEN_STATIC_MAX_ID 0x10u
#define FLEXRAY_FRAME_GEN_FIRST_ID 0xcu
#define FLEXRAY_FRAME_GEN_SECOND_ID 0xdu
#define FLEXRAY_FRAME_GEN_CYCLE_REP 4u
#define FLEXRAY_FRAME_GEN_CYCLE_BASE 3u
typedef struct {
    uint16_t id;
    uint8_t cycle_rep, cycle_base;
} flexray_frame_gen_rule_t;
static const flexray_frame_gen_rule_t FLEXRAY_FRAME_GEN_RULES[2] = {
    {FLEXRAY_FRAME_GEN_FIRST_ID, FLEXRAY_FRAME_GEN_CYCLE_REP, FLEXRAY_FRAME_GEN_CYCLE_BASE},
    {FLEXRAY_FRAME_GEN_SECOND_ID, FLEXRAY_FRAME_GEN_CYCLE_REP, FLEXRAY_FRAME_GEN_CYCLE_BASE},
};
#endif
