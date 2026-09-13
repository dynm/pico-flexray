#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "flexray_slot_schedule.h"

static flexray_slot_schedule_t initialized(uint32_t slot, uint32_t cycle)
{
    flexray_slot_schedule_config_t c = {
        .static_max_id=8, .learn_samples=12, .min_slot_cycles=1500,
        .max_slot_cycles=1500000, .tolerance_cycles=32,
        .slot_cycles=slot, .cycle_cycles=cycle,
    };
    flexray_slot_schedule_t s;
    assert(flexray_slot_schedule_init(&s, &c));
    return s;
}

static void test_acquisition_sparse_ids_and_wrap(void)
{
    flexray_slot_schedule_t s = initialized(0, 0);
    for (unsigned c=0; c<40 && !s.locked; ++c) {
        uint32_t t = c * 750000u;
        (void)flexray_slot_schedule_observe(&s, 4, (c+60)&63, 100u-t);
        // Absent IDs 5 and 7..8: infer by ID distance, never frame length.
        (void)flexray_slot_schedule_observe(&s, 6, (c+60)&63, 100u-t-12000);
        (void)flexray_slot_schedule_observe(&s, 10, (c+60)&63, 100u-t-36000);
    }
    assert(s.locked && s.slot_cycles==6000 && s.cycle_cycles==750000);
    assert(s.samples==12 && s.cycle_samples_seen==12);
    flexray_slot_schedule_t saved = s;
    assert(!flexray_slot_schedule_observe(&s, 1, 30, 0));
    assert(memcmp(&s, &saved, sizeof(s))==0); // measurement has stopped
}

static void test_lowest_five_observed_ids(void)
{
    flexray_slot_schedule_t s = initialized(0, 0);
    s.config.static_max_id = 120;
    // Arrival order is not rank. Fill five high candidates, then displace them.
    for (unsigned id = 110; id < 115; ++id)
        (void)flexray_slot_schedule_observe(&s, id, 0, 0u - id * 6000u);
    assert(s.learn_id_count == 5 && s.learn_ids[0] == 110);
    for (unsigned id = 20; id <= 28; id += 2)
        (void)flexray_slot_schedule_observe(&s, id, 1, 0u - 750000u - id * 6000u);
    assert(s.learn_id_count == 5);
    for (unsigned i = 0; i < 5; ++i) assert(s.learn_ids[i] == 20 + i * 2);
    flexray_slot_schedule_t saved = s;
    assert(!flexray_slot_schedule_observe(&s, 100, 1, 0));
    assert(memcmp(&s, &saved, sizeof(s)) == 0);
    assert(!flexray_slot_schedule_reference_allowed(&s, 110));
    assert(flexray_slot_schedule_reference_allowed(&s, 28));
    for (unsigned c = 2; c < 50 && !s.locked; ++c)
        for (unsigned id = 20; id <= 28 && !s.locked; id += 2)
            (void)flexray_slot_schedule_observe(&s, id, (c + 60) & 63,
                                                  0u - c * 750000u - id * 6000u);
    assert(s.locked && s.slot_cycles == 6000 && s.cycle_cycles == 750000);
    // Reacquisition discovers its own set, never retaining the previous bus.
    flexray_slot_schedule_config_t cfg = s.config;
    assert(flexray_slot_schedule_init(&s, &cfg));
    assert(s.learn_id_count == 0);
    for (unsigned c = 0; c < 50 && !s.locked; ++c) {
        (void)flexray_slot_schedule_observe(&s, 50, c & 63, 0u - c * 750000u);
        (void)flexray_slot_schedule_observe(&s, 57, c & 63, 0u - c * 750000u - 42000u);
    }
    assert(s.locked && s.learn_id_count == 2 && s.learn_ids[0] == 50);
    assert(s.slot_cycles == 6000 && s.cycle_cycles == 750000);
}

static void test_manual_and_only_one_id(void)
{
    flexray_slot_schedule_t s = initialized(0,0);
    for (unsigned c=0;c<40;++c) (void)flexray_slot_schedule_observe(&s,6,c&63,0u-c*750000);
    assert(!s.locked && s.samples==0); // one ID cannot infer static spacing
    s=initialized(6000,0);
    for (unsigned c=0;c<40 && !s.locked;++c) (void)flexray_slot_schedule_observe(&s,6,c&63,0u-c*750000);
    assert(s.locked && s.cycle_cycles==750000);
    s=initialized(6000,750000);
    assert(s.locked && s.samples==0 && s.cycle_samples_seen==0);
    s.config.cycle_cycles=47999;
    assert(!flexray_slot_schedule_init(&s,&s.config));
}

static void test_outliers_and_bad_headers(void)
{
    flexray_slot_schedule_t s=initialized(0,750000);
    for(unsigned c=0;c<28 && !s.locked;++c) {
        uint32_t slot = c<4 ? 10000+c*1000 : 6000;
        (void)flexray_slot_schedule_observe(&s,2,c,0u-c*750000);
        (void)flexray_slot_schedule_observe(&s,6,c,0u-c*750000-slot*4);
    }
    assert(s.locked && s.slot_cycles==6000);
    s=initialized(0,0);
    (void)flexray_slot_schedule_observe(&s,6,63,100);
    (void)flexray_slot_schedule_observe(&s,6,63,99);
    (void)flexray_slot_schedule_observe(&s,3,63,98);
    (void)flexray_slot_schedule_observe(&s,0,63,90);
    assert(s.previous_id==6 && s.previous_stamp==100 && s.rejected==2);
    (void)flexray_slot_schedule_observe(&s,8,63,100u-12000);
    assert(s.slot_samples[0]==6000);
}

static void test_phase_window_and_cycle_shaping(void)
{
    for(int first=70;first<135;++first) {
        uint32_t samples[8]={0};
        for(int i=first;i<first+15;++i) samples[i/32]|=1u<<(31-i%32);
        int32_t error;
        assert(flexray_fss_phase_error(samples,102,32,&error));
        assert(error==first-102);
        samples[7]|=1;
        assert(!flexray_fss_phase_error(samples,102,32,&error));
    }
    // A correction moves only the next envelope's origin. Once consumed,
    // the following cycle uses nominal C; missing references add no drift.
    flexray_slot_schedule_t s=initialized(6000,750000);
    assert(flexray_slot_shape_cycle(&s,20)==20);
    assert(flexray_slot_shape_cycle(&s,-12)==-12); // no integral from +20
    assert(flexray_slot_shape_cycle(&s,0)==0);
    double phase=0;
    for(int c=0;c<1000;++c) {
        phase += 0.25; // observed cycle is 0.25 clocks longer than nominal
        int correction=flexray_slot_shape_cycle(&s,(int)(phase+0.5));
        phase-=correction;
        assert(phase>=-0.5 && phase<=0.5);
    }

}

int main(void)
{
    test_acquisition_sparse_ids_and_wrap(); test_manual_and_only_one_id();
    test_outliers_and_bad_headers();
    test_lowest_five_observed_ids();
    test_phase_window_and_cycle_shaping();
    puts("flexray static schedule tests passed");
}
