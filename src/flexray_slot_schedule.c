#include "flexray_slot_schedule.h"
#include "pico/platform/sections.h"
#include <stddef.h>
#include <string.h>

bool flexray_slot_schedule_init(flexray_slot_schedule_t *s,
                                  const flexray_slot_schedule_config_t *cfg)
{
    if (!s || !cfg || cfg->static_max_id < 2u || cfg->static_max_id > 2047u ||
        cfg->learn_samples < 8u || cfg->learn_samples > 32u ||
        !cfg->min_slot_cycles || cfg->max_slot_cycles < cfg->min_slot_cycles ||
        (cfg->slot_cycles && (cfg->slot_cycles < cfg->min_slot_cycles ||
                             cfg->slot_cycles > cfg->max_slot_cycles)) ||
        (cfg->slot_cycles && cfg->cycle_cycles &&
         cfg->cycle_cycles < (uint64_t)cfg->slot_cycles * cfg->static_max_id + FLEXRAY_SLOT_TAIL_OVERHEAD)) return false;
    flexray_slot_schedule_config_t saved = *cfg;
    memset(s, 0, sizeof(*s));
    s->config = saved;
    s->slot_cycles = saved.slot_cycles;
    s->cycle_cycles = saved.cycle_cycles;
    s->slot_locked = saved.slot_cycles != 0;
    s->cycle_locked = saved.cycle_cycles != 0;
    s->locked = s->slot_locked && s->cycle_locked;
    return true;
}

static bool fit(uint32_t *values, uint8_t *count, uint8_t *next,
                uint32_t value, uint32_t tolerance, uint16_t required, uint32_t *period)
{
    values[*next] = value;
    *next = (uint8_t)((*next + 1u) % 32u);
    if (*count < 32u) ++*count;
    if (*count < required) return false;
    uint32_t sorted[32];
    for (uint8_t i = 0; i < *count; ++i) {
        uint32_t v = values[i];
        uint8_t j = i;
        while (j && sorted[j - 1u] > v) { sorted[j] = sorted[j - 1u]; --j; }
        sorted[j] = v;
    }
    uint32_t median = sorted[*count / 2u];
    unsigned good = 0;
    for (uint8_t i = 0; i < *count; ++i) {
        uint32_t distance = sorted[i] > median ? sorted[i] - median : median - sorted[i];
        if (distance <= tolerance) ++good;
    }
    if (good < required) return false;
    *period = median;
    return true;
}

// Online sorted set: a newly observed lower ID displaces the largest candidate.
// Previously collected intervals remain physical measurements of the same grid.
static bool learn_id(flexray_slot_schedule_t *s, uint16_t id)
{
    if (!id || id > s->config.static_max_id) return false;
    uint8_t at = 0;
    while (at < s->learn_id_count && s->learn_ids[at] < id) ++at;
    if (at < s->learn_id_count && s->learn_ids[at] == id) return true;
    if (at == FLEXRAY_SLOT_LEARN_ID_COUNT) return false;
    if (s->learn_id_count == FLEXRAY_SLOT_LEARN_ID_COUNT) {
        if (s->have_previous && s->previous_id == s->learn_ids[s->learn_id_count - 1u])
            s->have_previous = false;
    } else ++s->learn_id_count;
    for (uint8_t i = s->learn_id_count - 1u; i > at; --i) s->learn_ids[i] = s->learn_ids[i - 1u];
    s->learn_ids[at] = id;
    return true;
}

bool flexray_slot_schedule_reference_allowed(const flexray_slot_schedule_t *s, uint16_t id)
{
    if (!id || id > s->config.static_max_id) return false;
    if (!s->learn_id_count) return true; // explicit fixed-period startup
    for (uint8_t i = 0; i < s->learn_id_count; ++i)
        if (s->learn_ids[i] == id) return true;
    return false;
}

bool flexray_slot_schedule_observe(flexray_slot_schedule_t *s,
                                     uint16_t id, uint8_t cycle, uint32_t stamp)
{
    if (s->locked || cycle > 63u || !learn_id(s, id)) return false;
    if (s->have_previous) {
        uint8_t cycles = (uint8_t)((cycle - s->previous_cycle) & 63u);
        int32_t ids = (int32_t)id - s->previous_id;
        uint32_t elapsed = s->previous_stamp - stamp;
        if (!cycles && ids <= 0) { ++s->rejected; return false; }
        if (!cycles && !s->slot_locked) {
            uint32_t sample = elapsed / (uint32_t)ids;
            if (sample < s->config.min_slot_cycles || sample > s->config.max_slot_cycles) {
                ++s->rejected;
            } else {
                ++s->samples;
                s->slot_locked = fit(s->slot_samples, &s->slot_count, &s->slot_next,
                    sample, s->config.tolerance_cycles, s->config.learn_samples, &s->slot_cycles);
            }
        } else if (cycles && s->slot_locked && !s->cycle_locked) {
            int64_t span = (int64_t)elapsed - (int64_t)ids * s->slot_cycles;
            int64_t sample = span / cycles;
            if (sample < (int64_t)s->slot_cycles * s->config.static_max_id + FLEXRAY_SLOT_TAIL_OVERHEAD || sample > 0x0fffffffu) {
                ++s->rejected;
            } else {
                ++s->cycle_samples_seen;
                s->cycle_locked = fit(s->cycle_samples, &s->cycle_count, &s->cycle_next,
                    (uint32_t)sample, s->config.tolerance_cycles,
                    s->config.learn_samples, &s->cycle_cycles);
            }
        }
    }
    s->have_previous = true;
    s->previous_id = id; s->previous_cycle = cycle; s->previous_stamp = stamp;
    s->locked = s->slot_locked && s->cycle_locked;
    if (s->locked && s->cycle_cycles < (uint64_t)s->slot_cycles * s->config.static_max_id + FLEXRAY_SLOT_TAIL_OVERHEAD) {
        s->cycle_locked = s->locked = false;
        ++s->rejected;
    }
    return s->locked;
}

// These runtime checks must finish before the phase DMA expires. Keep both
// in RAM so XIP cache misses cannot stretch the header ISR past that deadline.
bool __no_inline_not_in_flash_func(flexray_fss_phase_error)(const uint32_t samples[8], int expected,
                                uint32_t tolerance, int32_t *error)
{
    int first = -1;
    for (int w = 0; w < 8; ++w) {
        if (samples[w]) { first = w * 32 + __builtin_clz(samples[w]); break; }
    }
    if (first <= 0 || first + 15 >= 256) return false;
    for (int w = 0; w < 8; ++w) {
        int shift = first - w * 32;
        uint32_t marker = shift >= 32 || shift <= -15 ? 0u :
            shift < 0 ? 0xfffe0000u << -shift : 0xfffe0000u >> shift;
        if (samples[w] != marker) return false;
    }
    int32_t delta = first - expected;
    if (delta < -(int32_t)tolerance || delta > (int32_t)tolerance) return false;
    *error = delta;
    return true;
}

int32_t __no_inline_not_in_flash_func(flexray_slot_shape_cycle)(const flexray_slot_schedule_t *s, int32_t error)
{
    int32_t limit = (int32_t)s->config.tolerance_cycles;
    if (error > limit) error = limit;
    if (error < -limit) error = -limit;
    int64_t spare = (int64_t)s->cycle_cycles - (int64_t)s->slot_cycles * s->config.static_max_id
                    - FLEXRAY_SLOT_TAIL_OVERHEAD;
    if (error < -spare) error = (int32_t)-spare;
    return error;
}

