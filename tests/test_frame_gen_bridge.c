// Link the real streamer IRQs, MITM injector and frame generation controller together.
#define STATIC_TX_REAL_BRIDGE
#include "frame_gen_control_fakes.h"
#include <stdio.h>
#include "demo_inject_action.h"

static uint32_t fake_now_us = 1000;
static inline uint32_t time_us_32(void) {return fake_now_us;}
static struct {uint32_t gpio_set,gpio_clr;} fake_sio;
#define sio_hw (&fake_sio)
#define __sev() ((void)0)
typedef struct {bool held;} spin_lock_t;
static spin_lock_t fake_lock;
static inline uint spin_lock_claim_unused(bool required) {(void)required;return 0;}
static inline spin_lock_t *spin_lock_instance(uint n) {(void)n;return &fake_lock;}
static inline uint32_t spin_lock_blocking(spin_lock_t *lock) {assert(!lock->held);lock->held=true;return 0;}
static inline void spin_unlock(spin_lock_t *lock,uint32_t save) {(void)save;assert(lock->held);lock->held=false;}

#include "../src/flexray_fowarder_with_injector.c"
// Mach-O ASLR need not preserve 32 KiB global alignment. The host supplies
// DMA write addresses explicitly, so relax ONLY these fake RX ring buffers;
// the firmware frame generation retains its full hardware ring alignment.
#define aligned(bytes) aligned(4)
#include "../src/flexray_bss_streamer.c"
#undef aligned
#if FLEXRAY_FRAME_GEN
#include "../src/flexray_frame_gen.c"
#define MITM_DMA dma_inject_chan_to_fr1
#define MITM_SM sm_forwarder_with_injector_to_fr1
#else
#define MITM_DMA dma_inject_chan_to_fr3
#define MITM_SM sm_forwarder_with_injector_to_fr3
#endif

static void real_header(bool fr2,uint id,uint cycle,bool valid)
{
    uint8_t h[5]={(uint8_t)(id>>8),(uint8_t)id,18,0,(uint8_t)cycle};
    uint16_t crc=calculate_flexray_header_crc(h);
    h[2]|=crc>>10;h[3]=crc>>2;h[4]|=(crc&3u)<<6;
    if(!valid) h[3]^=1;
    volatile uint8_t *ring=fr2?fr2_ring_buffer:fr1_ring_buffer;
    uint start=fr2?fr2_prev_write_idx:fr1_prev_write_idx;
    uint mask=fr2?FR2_RING_MASK:FR1_RING_MASK;
    uint dma=fr2?dma_data_from_fr2_chan:dma_data_from_fr1_chan;
    for(uint i=0;i<5;++i) ring[(start+i)&mask]=h[i];
    fake_dmas[dma].write_addr=(uintptr_t)ring+((start+5u)&mask);
    pio0->fdebug=0;
#if FLEXRAY_FRAME_GEN
    if(ready) {pio2->rx_count[CAPTURE_SM]=1;pio2->rx[CAPTURE_SM][0]=0;}
#endif
    streamer_header_irq_handler();
}
static void real_end(bool fr2)
{
    volatile uint8_t *ring=fr2?fr2_ring_buffer:fr1_ring_buffer;
    uint start=fr2?fr2_prev_write_idx:fr1_prev_write_idx;
    uint mask=fr2?FR2_RING_MASK:FR1_RING_MASK;
    uint dma=fr2?dma_data_from_fr2_chan:dma_data_from_fr1_chan;
    fake_dmas[dma].write_addr=(uintptr_t)ring+((start+26u)&mask);
    uint8_t secondary_source = 0;
#if !FLEXRAY_FRAME_GEN
    secondary_source = fr34_detected_source;
#endif
    streamer_frame_end_irq_handler();
    uint32_t notice;notify_info_t decoded;
    assert(notify_queue_pop(&notice));notify_decode(notice,&decoded);
    assert(decoded.is_fr2==fr2 && decoded.end_idx==((start+26u)&mask));
    assert(decoded.fr34_source==secondary_source);
    assert(!notify_queue_pop(&notice));
}
static void reset_mitm(void)
{
    fake_dmas[MITM_DMA].busy=false;
    injector_reset_stats_and_queue();
    injector_set_enabled(true);
}
static void host_command(injector_transport_t transport)
{
    const uint8_t *wire = demo_inject_action;
    assert(sizeof(demo_inject_action)==25 && wire[0]==0x90);
    uint16_t id=wire[1]|((uint16_t)wire[2]<<8);
    uint16_t len=wire[4]|((uint16_t)wire[5]<<8);
    assert(injector_submit_override_from(id,wire[3],len,wire+6,transport));
}
static void expect_mitm_dma(void)
{
    assert(!prepared_injection.pending && injector_stats.injected==1);
    uint channel=(uint)MITM_DMA;
    assert(fake_dmas[channel].busy && fake_dmas[channel].count==8);
    assert(fake_dmas[channel].destination==&pio2->txf[MITM_SM]);
    assert(fake_dmas[channel].config.bswap);
    const frame_template_t *p=(const frame_template_t *)fake_dmas[channel].source;
    assert(p->dma_count_bswap==__builtin_bswap32(25));
    assert((p->data[4]&63u)==2);
    assert(calculate_flexray_header_crc(p->data)==header_crc_from_header(p->data));
    uint32_t crc=calculate_flexray_frame_crc(p->data,23);
    assert(p->data[23]==(crc>>16) && p->data[24]==((crc>>8)&255u) && p->data[25]==(crc&255u));
}
static void test_mitm_failsafe(void)
{
    // Both original host transports reach DMA through real IRQ callbacks.
    for(uint t=INJECT_TRANSPORT_VENDOR;t<=INJECT_TRANSPORT_UDP;++t) {
        reset_mitm();host_command((injector_transport_t)t);
        uint8_t previous[18];memcpy(previous,TEMPLATES[0].data+5,18);
        real_header(t==INJECT_TRANSPORT_UDP,6,2,true);
        assert(prepared_injection.pending && injector_stats.consumed==1);
        assert(!fake_dmas[MITM_DMA].busy);
        for(uint i=0;i<4;++i) assert(TEMPLATES[0].data[5+i]==0x52);
        assert(memcmp(TEMPLATES[0].data+9,previous+4,14)==0);
        real_end(t==INJECT_TRANSPORT_UDP);expect_mitm_dma();
    }
    reset_mitm();host_command(INJECT_TRANSPORT_UDP);
    real_header(true,6,1,true); // excluded cycle does not consume a command
    assert(!prepared_injection.pending && host_override_count()==1);
    real_end(true);assert(injector_stats.injected==0);
    fake_now_us+=HOST_OVERRIDE_TIMEOUT_US+1;
    real_header(true,6,2,true);real_end(true);
    assert(injector_stats.consumed==0 && injector_stats.injected==0);

    reset_mitm();host_command(INJECT_TRANSPORT_UDP);
    real_header(true,6,2,true);assert(prepared_injection.pending);
    fake_now_us+=HOST_OVERRIDE_TIMEOUT_US+1; // expires between prepare and commit
    real_end(true);assert(injector_stats.injected==0);

    reset_mitm();host_command(INJECT_TRANSPORT_UDP);
    real_header(true,6,2,true);assert(prepared_injection.pending);
    injector_set_enabled(false);
    real_end(true);assert(injector_stats.injected==0 && host_override_count()==0);

    reset_mitm();host_command(INJECT_TRANSPORT_UDP);
    real_header(true,6,2,true);assert(prepared_injection.pending);
    real_header(true,6,2,false); // next, invalid header discards stale preparation
    assert(!prepared_injection.pending);
    real_end(true);assert(injector_stats.injected==0);

    reset_mitm();host_command(INJECT_TRANSPORT_UDP);
    real_header(true,6,2,true);
    fake_dmas[MITM_DMA].busy=true;
    const volatile void *old_source=fake_dmas[MITM_DMA].source;
    real_end(true);
    assert(injector_stats.injected==0 && fake_dmas[MITM_DMA].source==old_source);

    reset_mitm();
}
#if FLEXRAY_FRAME_GEN
static void build_command(uint8_t op,const void *body,uint size)
{
    uint8_t request[40]={op},reply[4];
    if(size) memcpy(request+1,body,size);
    assert(flexray_frame_gen_command(request,size+1,reply,sizeof(reply))==4);
    assert(reply[2]==FLEXRAY_FRAME_GEN_OK && reply[3]==0);
}
static void build_beat(uint n) {pio0->fdebug=0;fake_beat(n);pace_irq();}

#else
static void test_secondary_capture(void)
{
    for (uint fr4=0;fr4<2;++fr4) {
        uint dma=fr4?dma_data_from_fr4_chan:dma_data_from_fr3_chan;
        volatile uint8_t *ring=fr4?fr4_ring_buffer:fr3_ring_buffer;
        uint32_t start=fr4?fr4_prev_write_idx:fr3_prev_write_idx;
        uint8_t h[5]={0,0x29,18,0,1};
        uint16_t crc=calculate_flexray_header_crc(h);
        h[2]|=crc>>10;h[3]=crc>>2;h[4]|=(crc&3u)<<6;
        memcpy((void *)(ring+start),h,5);
        fake_dmas[dma].write_addr=(uintptr_t)ring+start+5u;
        streamer_fr34_header_irq_handler();
        uint8_t source=fr4?FROM_FR4:FROM_FR3;
        assert(fr34_detected_source==source);
        real_header(fr4,0x29,1,true);real_end(fr4);
        assert(fr34_detected_source==FROM_UNKNOWN);
        fake_dmas[dma].write_addr=(uintptr_t)ring+start+26u;
        streamer_fr34_frame_end_irq_handler();
        assert((fr4?fr4_prev_write_idx:fr3_prev_write_idx)==start+26u);
        // A bad secondary header must not retain the preceding source tag.
        fr34_detected_source=source;h[3]^=1;
        memcpy((void *)(ring+start+26u),h,5);
        fake_dmas[dma].write_addr=(uintptr_t)ring+start+31u;
        streamer_fr34_header_irq_handler();
        assert(fr34_detected_source==FROM_UNKNOWN);
        streamer_fr34_frame_end_irq_handler();
    }
}
#endif

int main(void)
{
#if !FLEXRAY_FRAME_GEN
    // Match the original four-channel startup: receivers precede forwarders.
    setup_stream_fr34(pio0,RXD_FR_3_PIN,TXEN_FR_4_PIN,RXD_FR_4_PIN,TXEN_FR_3_PIN);
    setup_stream(pio1,RXD_FR_1_PIN,TXEN_FR_2_PIN,RXD_FR_2_PIN,TXEN_FR_1_PIN);
    notify_queue_init();
    real_header(false,6,2,true);real_end(false);
    assert(!forwarder_ready && !prepared_injection.pending);
    assert(dma_inject_chan_to_fr1==-1 && dma_inject_chan_to_fr3==-1);
#endif
    setup_forwarder_with_injector(pio2,RXD_FR_1_PIN,TXD_FR_2_PIN,RXD_FR_2_PIN,TXD_FR_1_PIN,
                                  RXD_FR_3_PIN,TXD_FR_4_PIN,RXD_FR_4_PIN,TXD_FR_3_PIN);
#if FLEXRAY_FRAME_GEN
    setup_stream(pio1,RXD_FR_1_PIN,TXEN_FR_2_PIN,RXD_FR_2_PIN,TXEN_FR_1_PIN);
#else
    for(uint sm=0;sm<4;++sm) assert(pio2->claimed[sm] && pio2->enabled[sm]);
    assert(pio0->used_words==28 && pio1->used_words==28 && pio2->used_words==22);
#endif
    notify_queue_init();
    assert(dma_ring_write_idx(dma_data_from_fr1_chan,fr1_ring_buffer,FR1_RING_MASK)==(FLEXRAY_FRAME_GEN?0u:26u));
    assert(dma_ring_write_idx(dma_data_from_fr2_chan,fr2_ring_buffer,FR2_RING_MASK)==0);
    uint8_t raw[26]={0x20,8,18,0,2};memset(raw+5,0xa5,18);
    uint16_t hcrc=calculate_flexray_header_crc(raw);
    raw[2]|=hcrc>>10;raw[3]=hcrc>>2;raw[4]|=(hcrc&3u)<<6;
    fix_flexray_frame_crc(raw,sizeof(raw));
    try_cache_last_target_frame(8,2,sizeof(raw),raw);
#if FLEXRAY_FRAME_GEN
    assert(!ready);test_mitm_failsafe(); // frame generation need not be initialized
    flexray_frame_gen_init();assert(ready && !schedule.locked && fake_dma_claimed==10);
    test_mitm_failsafe(); // acquisition/missing FSS cannot gate MITM

    flexray_frame_gen_config_t c=default_config;
    c.slot_cycles=6000;c.cycle_cycles=750000;
    build_command(FLEXRAY_FRAME_GEN_OP_CONFIG,&c,sizeof(c));real_header(false,6,62,true);real_end(false);
    pio0->pc[PACE_SM]=timing_offset+flexray_slot_pace_offset_countdown;
    real_header(false,6,63,true);real_end(false);assert(seeded);
    uint8_t on=1;build_command(FLEXRAY_FRAME_GEN_OP_ENABLE,&on,1);
    host_command(INJECT_TRANSPORT_UDP);
    injector_stats_t before=injector_stats;
    for(uint i=0;i<=5;++i) build_beat(i); // logical B prepares static frame generation C
    assert(prepared_packet==&reservations[0].null_template.packet);
    assert(pio1->out_base[LOCAL_SM]==TXEN_FR_2_PIN && pio2->out_base[LOCAL_SM]==TXD_FR_2_PIN);
    assert(host_override_count()==1 && !prepared_injection.pending);
    assert(!fake_dmas[MITM_DMA].busy);
    assert(fake_dmas[packet_dma].destination==&pio2->txf[LOCAL_SM]);
    pio0->pins &= ~(1u<<FLEXRAY_FRAME_GEN_OWNERSHIP_PIN);build_beat(6);
    // Regression: a delayed REAL frame-end cannot release active C.
    const flexray_frame_gen_packet_t *active_c=prepared_packet;
    streamer_frame_end_irq_handler();
    assert(prepared_packet==active_c && stats.sent==0);
    fake_dmas[packet_dma].busy=false;fake_dmas[packet_dma].transfer_count=0;
    pio0->pins |= 1u<<FLEXRAY_FRAME_GEN_OWNERSHIP_PIN;
    frame_gen_done_irq();
    assert(stats.sent==1 && stats.null_sent==1 && prepared_packet==&reservations[1].null_template.packet);
    pio0->pins &= ~(1u<<FLEXRAY_FRAME_GEN_OWNERSHIP_PIN);build_beat(7);
    fake_dmas[packet_dma].busy=false;fake_dmas[packet_dma].transfer_count=0;
    pio0->pins |= 1u<<FLEXRAY_FRAME_GEN_OWNERSHIP_PIN;
    frame_gen_done_irq();
    assert(stats.sent==2 && stats.null_sent==2 && !prepared_packet);
    assert(memcmp(&before,&injector_stats,sizeof(before))==0 && host_override_count()==1);
    uint32_t notice;while(notify_queue_pop(&notice)) {}
    assert(pio1->enabled[0] && pio1->enabled[1] && pio2->enabled[0] && pio2->enabled[1]);
    for(uint i=8;i<=16;++i) build_beat(i);
    assert(stats.current_slot==6 && stats.current_cycle==1 && !prepared_packet);
    // frame generation disable, applied in this real-header callback, leaves the original
    // preparation intact. Only the subsequent real frame end commits MITM DMA.
    uint8_t off=0;build_command(FLEXRAY_FRAME_GEN_OP_ENABLE,&off,1);
    real_header(true,6,2,true);
    assert(!enabled && pacing && prepared_injection.pending);
    assert(!fake_dmas[MITM_DMA].busy);
    real_end(true);expect_mitm_dma();
    puts("independent MITM/frame generation integration tests passed");
#else
    assert(fake_dma_claimed==8);
    test_mitm_failsafe();
    test_secondary_capture();
    puts("four-channel MITM/secondary capture tests passed");
#endif
}
