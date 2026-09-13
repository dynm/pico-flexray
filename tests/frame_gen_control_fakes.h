#ifndef STATIC_TX_CONTROL_FAKES_H
#define STATIC_TX_CONTROL_FAKES_H
// CPU-visible register/FIFO state, not a simulation of DMA bus arbitration.
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "board_config.h"
typedef unsigned int uint;
static inline void busy_wait_us_32(uint32_t us) { (void)us; }
#define __time_critical_func(name) name
#define __no_inline_not_in_flash_func(name) name
#define __dmb() __asm__ volatile("" ::: "memory")
enum {clk_sys, DMA_SIZE_32, DMA_SIZE_8, pis_interrupt2, pis_interrupt3, pis_interrupt4, pis_interrupt5, pio_x, pio_y};
#define PIO_FDEBUG_RXSTALL_LSB 0
#define NUM_DMA_CHANNELS 16
#define DMA_CH0_CTRL_TRIG_EN_BITS 1u
typedef struct {uint start,in_base,jmp_pin,out_base,set_base,set_count,side_base;} pio_sm_config;
typedef struct {uint length;} pio_program_t;
typedef struct {
    uint32_t fdebug,irq_force,txf[4],rxf[4],rx[4][8],tx[4][8],osr[4],x[4],y[4],pins;
    uint pc[4],start[4],rx_count[4],tx_count[4],in_base[4],out_base[4],side_base[4],used_words;
    bool enabled[4],claimed[4];
} fake_pio_t;
typedef fake_pio_t *PIO;
static fake_pio_t fake_pios[3];
#define pio0 (&fake_pios[0])
#define pio1 (&fake_pios[1])
#define pio2 (&fake_pios[2])
typedef struct {uint chain_to,dreq,ring_bits;bool read_inc,write_inc,ring_write,bswap;} dma_channel_config;
static struct {
    bool busy,claimed;
    uint32_t transfer_count,al3_read_addr_trig,ctrl_trig;
    uintptr_t read_addr,write_addr;
    dma_channel_config config;
    const volatile void *source;
    volatile void *destination;
    uint count;
} fake_dmas[16];
static inline void hw_clear_bits(volatile uint32_t *p,uint32_t bits) {*p &= ~bits;}
static uint fake_dma_claimed;
static uint fake_program_loads, fake_dma_allocations;
static uint fake_beat_on_dma=UINT32_MAX, fake_beat_on_packet=UINT32_MAX;
static bool fake_active_on_dma;
static inline void fake_beat(uint n) {pio0->rx[1][pio0->rx_count[1]++]=~n;}
static inline uint32_t clock_get_hz(uint c) {(void)c;return 150000000;}
static inline void pio_sm_claim(PIO p,uint sm) {assert(!p->claimed[sm]);p->claimed[sm]=true;}
static inline uint pio_claim_unused_sm(PIO p,bool required) {
    (void)required;for(uint sm=0;sm<4;++sm) if(!p->claimed[sm]) {pio_sm_claim(p,sm);return sm;}
    assert(false);return 0;
}
static inline uint pio_add_program(PIO p,const pio_program_t *pr) {
    ++fake_program_loads;
    uint offset=p->used_words;p->used_words+=pr->length;assert(p->used_words<=32);return offset;
}
static inline void pio_remove_program(PIO p,const pio_program_t *pr,uint off) {(void)off;p->used_words-=pr->length;}
static inline void pio_sm_init(PIO p,uint sm,uint pc,const pio_sm_config *c) {
    p->pc[sm]=p->start[sm]=pc;p->rx_count[sm]=p->tx_count[sm]=0;
    p->in_base[sm]=c->in_base;p->out_base[sm]=c->set_count?c->set_base:c->out_base;p->side_base[sm]=c->side_base;
}
static inline void pio_sm_set_enabled(PIO p,uint sm,bool en) {
    p->enabled[sm]=en;
    if(en) p->fdebug=0; // acknowledge preceding firmware W1C writes
}
static inline void pio_sm_clear_fifos(PIO p,uint sm) {p->rx_count[sm]=p->tx_count[sm]=0;}
static inline void pio_interrupt_clear(PIO p,uint irq) {p->irq_force&=~(1u<<irq);}
static inline bool pio_interrupt_get(PIO p,uint irq) {return (p->irq_force&(1u<<irq))!=0;}
static inline void pio_sm_put(PIO p,uint sm,uint32_t v) {assert(p->tx_count[sm]<8);p->tx[sm][p->tx_count[sm]++]=v;}
static inline uint pio_encode_pull(bool a,bool b) {(void)a;(void)b;return 0x8000;}
static inline uint pio_encode_out(uint reg,uint count) {(void)count;return 0x6000|reg;}
static inline void pio_sm_exec(PIO p,uint sm,uint instruction) {
    if(instruction==0x8000) {
        assert(p->tx_count[sm]);p->osr[sm]=p->tx[sm][0];
        memmove(p->tx[sm],p->tx[sm]+1,(--p->tx_count[sm])*4);
    } else if(instruction==(0x6000|pio_x)) p->x[sm]=p->osr[sm];
    else if(instruction==(0x6000|pio_y)) p->y[sm]=p->osr[sm];
    else assert(false);
}
static inline uint pio_sm_get_pc(PIO p,uint sm) {return p->pc[sm];}
static inline uint pio_sm_get_rx_fifo_level(PIO p,uint sm) {return p->rx_count[sm];}
static inline bool pio_sm_is_rx_fifo_empty(PIO p,uint sm) {return p->rx_count[sm]==0;}
static inline uint32_t pio_sm_get(PIO p,uint sm) {
    assert(p->rx_count[sm]);uint32_t v=p->rx[sm][0];
    memmove(p->rx[sm],p->rx[sm]+1,(--p->rx_count[sm])*4);return v;
}
static inline void pio_sm_set_pins_with_mask(PIO p,uint sm,uint32_t pins,uint32_t mask) {(void)sm;p->pins=(p->pins&~mask)|pins;}
static inline void gpio_init(uint pin) {(void)pin;}
static inline bool gpio_get(uint pin) {return (pio0->pins & (1u << pin)) != 0;}
static inline void gpio_pull_up(uint pin) {(void)pin;}
static inline void pio_gpio_init(PIO p,uint pin) {(void)p;(void)pin;}
static inline void pio_sm_set_consecutive_pindirs(PIO p,uint sm,uint pin,uint n,bool out) {(void)p;(void)sm;(void)pin;(void)n;(void)out;}
static inline void sm_config_set_in_pins(pio_sm_config *c,uint n) {c->in_base=n;}
static inline void sm_config_set_jmp_pin(pio_sm_config *c,uint n) {c->jmp_pin=n;}
static inline void sm_config_set_clkdiv(pio_sm_config *c,float n) {(void)c;(void)n;}
static inline void sm_config_set_set_pins(pio_sm_config *c,uint p,uint n) {c->set_base=p;c->set_count=n;}
static inline void sm_config_set_sideset_pins(pio_sm_config *c,uint pin) {c->side_base=pin;}
static inline void sm_config_set_out_pins(pio_sm_config *c,uint p,uint n) {(void)n;c->out_base=p;}
static inline uint pio_get_irq_num(PIO p,uint n) {(void)p;return n;}
static inline void pio_set_irq0_source_enabled(PIO p,uint n,bool en) {(void)p;(void)n;(void)en;}
static inline void pio_set_irq1_source_enabled(PIO p,uint n,bool en) {(void)p;(void)n;(void)en;}
static inline void irq_set_exclusive_handler(uint irq,void (*handler)(void)) {(void)irq;(void)handler;}
static inline void irq_set_enabled(uint irq,bool en) {(void)irq;(void)en;}
static inline uint dma_claim_unused_channel(bool required) {
    (void)required;for(uint n=0;n<16;++n) if(!fake_dmas[n].claimed) {
        fake_dmas[n].claimed=true;++fake_dma_claimed;++fake_dma_allocations;return n;
    } assert(false);return 0;
}
static inline void dma_channel_unclaim(uint n) {assert(fake_dmas[n].claimed);fake_dmas[n].claimed=false;--fake_dma_claimed;}
#define dma_channel_hw_addr(n) (&fake_dmas[n])
static inline dma_channel_config dma_channel_get_default_config(uint n) {return (dma_channel_config){.chain_to=n,.dreq=63};}
static inline void channel_config_set_chain_to(dma_channel_config *c,uint n) {c->chain_to=n;}
static inline void channel_config_set_transfer_data_size(dma_channel_config *c,uint n) {(void)c;(void)n;}
static inline void channel_config_set_read_increment(dma_channel_config *c,bool b) {c->read_inc=b;}
static inline void channel_config_set_write_increment(dma_channel_config *c,bool b) {c->write_inc=b;}
static inline void channel_config_set_high_priority(dma_channel_config *c,bool b) {(void)c;(void)b;}
static inline void channel_config_set_bswap(dma_channel_config *c,bool b) {c->bswap=b;}
static inline void channel_config_set_dreq(dma_channel_config *c,uint n) {c->dreq=n;}
static inline void channel_config_set_ring(dma_channel_config *c,bool write,uint bits) {c->ring_write=write;c->ring_bits=bits;}
static inline uint pio_get_dreq(PIO p,uint sm,bool tx) {return (uint)(p-fake_pios)*8+sm+(tx?0:4);}
static inline uint dma_encode_transfer_count_with_self_trigger(uint n) {return n|0x10000000;}
static inline bool dma_channel_is_busy(uint n) {return fake_dmas[n].busy;}
static inline void dma_channel_abort(uint n) {fake_dmas[n].busy=false;}
static inline void dma_channel_start(uint n) {
    fake_dmas[n].busy=true;
    if(fake_dmas[n].destination==&pio2->txf[2] && fake_beat_on_packet!=UINT32_MAX) {
        fake_beat(fake_beat_on_packet);fake_beat_on_packet=UINT32_MAX;
    }
    if(fake_dmas[n].destination==&pio0->txf[0] && fake_beat_on_dma!=UINT32_MAX) {
        fake_beat(fake_beat_on_dma);fake_beat_on_dma=UINT32_MAX;
        if(fake_active_on_dma) {pio0->pins &= ~(1u<<17u);fake_active_on_dma=false;}
    }
}
static inline void dma_channel_set_read_addr(uint n,const volatile void *src,bool trigger) {
    fake_dmas[n].source=src;fake_dmas[n].read_addr=(uintptr_t)src;
    if(trigger) {fake_dmas[n].transfer_count=fake_dmas[n].count;dma_channel_start(n);}
}
static inline void dma_channel_set_write_addr(uint n,volatile void *dst,bool trigger) {
    fake_dmas[n].destination=dst;fake_dmas[n].write_addr=(uintptr_t)dst;
    if(trigger) dma_channel_start(n);
}
static inline void dma_channel_set_config(uint n,const dma_channel_config *c,bool trigger) {
    fake_dmas[n].config=*c;if(trigger) dma_channel_start(n);
}
static inline void dma_channel_set_trans_count(uint n,uint count,bool trigger) {
    fake_dmas[n].count=fake_dmas[n].transfer_count=count;if(trigger) dma_channel_start(n);
}
static inline void dma_channel_configure(uint n,const dma_channel_config *c,volatile void *dst,const volatile void *src,uint count,bool start) {
    fake_dmas[n].config=*c;fake_dmas[n].source=src;fake_dmas[n].read_addr=(uintptr_t)src;
    fake_dmas[n].destination=dst;fake_dmas[n].write_addr=(uintptr_t)dst;fake_dmas[n].count=fake_dmas[n].transfer_count=count;fake_dmas[n].busy=start;
}
#ifndef STATIC_TX_REAL_BRIDGE
static uint fake_local_setups;
void flexray_frame_gen_forwarder_local_config(bool en) {pio2->enabled[2]=en;if(en) {++fake_local_setups;pio2->out_base[2]=TXD_FR_2_PIN;}}
#endif
#endif
