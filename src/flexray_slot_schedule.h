#ifndef FLEXRAY_SLOT_SCHEDULE_H
#define FLEXRAY_SLOT_SCHEDULE_H
#include <stdbool.h>
#include <stdint.h>

#define FLEXRAY_SLOT_SAMPLE_CAPACITY 32u
#define FLEXRAY_SLOT_BIT_CYCLES 15u
#define FLEXRAY_SLOT_MAX_SLOTS 2047u
#define FLEXRAY_SLOT_LEARN_ID_COUNT 5u
#define FLEXRAY_FSS_PHASE_WORDS 8u
#define FLEXRAY_SLOT_TAIL_OVERHEAD 13u

typedef struct {
    uint16_t static_max_id;
    uint16_t learn_samples;
    uint32_t min_slot_cycles;
    uint32_t max_slot_cycles;
    uint32_t tolerance_cycles;
    uint32_t slot_cycles;
    uint32_t cycle_cycles;
} flexray_slot_schedule_config_t;

typedef struct {
    flexray_slot_schedule_config_t config;
    uint32_t slot_cycles, cycle_cycles;
    uint32_t slot_samples[32], cycle_samples[32];
    uint32_t samples, cycle_samples_seen, rejected;
    uint32_t previous_stamp;
    uint16_t previous_id;
    uint16_t learn_ids[FLEXRAY_SLOT_LEARN_ID_COUNT];
    uint8_t learn_id_count;
    uint8_t previous_cycle, slot_count, cycle_count, slot_next, cycle_next;
    bool have_previous, slot_locked, cycle_locked, locked;
    int32_t cycle_shape;
} flexray_slot_schedule_t;

bool flexray_slot_schedule_init(flexray_slot_schedule_t *s,
                                  const flexray_slot_schedule_config_t *cfg);
// Acquisition only. Timestamp decreases by one per system clock. Use only
// The lowest five distinct observed static IDs are retained during acquisition.
// ID gaps represent empty TDMA slots, not missing elapsed time.
bool flexray_slot_schedule_observe(flexray_slot_schedule_t *s,
                                     uint16_t id, uint8_t cycle, uint32_t stamp);
// Frozen acquisition candidates; fixed-period startup accepts any static ID.
bool flexray_slot_schedule_reference_allowed(const flexray_slot_schedule_t *s, uint16_t id);
// Runtime phase burst. Exactly one 15-clock marker must fit the whole window.
bool flexray_fss_phase_error(const uint32_t samples[8], int expected,
                                uint32_t tolerance, int32_t *error);
// One correction per cycle boundary. No integral or change of static spacing.
int32_t flexray_slot_shape_cycle(const flexray_slot_schedule_t *s, int32_t error);
#endif
