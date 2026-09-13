#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "flexray_frame_gen_packet.h"
#include "flexray_frame.h"

static uint32_t independent_crc(const uint8_t *raw, unsigned len)
{
    uint32_t crc=0xfedcba;
    for(unsigned i=0;i<len;++i) {
        crc^=(uint32_t)raw[i]<<16;
        for(unsigned b=0;b<8;++b) crc=((crc<<1)^((crc&0x800000)?0x5d6dcb:0))&0xffffff;
    }
    return crc;
}

int main(void)
{
    assert(sizeof(flexray_frame_gen_template_t)==268u+64u*3u);
    for(unsigned len=8;len<=262;len+=2) {
        uint8_t raw[262]={0x60,8};
        raw[2]=(uint8_t)(len-8);
        uint16_t hcrc=calculate_flexray_header_crc(raw);
        raw[2]|=hcrc>>10;raw[3]=hcrc>>2;raw[4]=(uint8_t)(hcrc<<6);
        for(unsigned i=5;i<len-3;++i) raw[i]=(uint8_t)(i*73+29);
        for(unsigned null=0;null<2;++null) {
            struct {uint32_t before;flexray_frame_gen_template_t tpl;uint32_t after;} g={.before=123,.after=456};
            flexray_frame_gen_template_generate(&g.tpl,raw,len,null);
            uint8_t crc_table[64][3];memcpy(crc_table,g.tpl.cycle_crc,sizeof(crc_table));
            // Reuse ONE packet, including the 63 -> 0 cycle transition.
            for(unsigned iteration=0;iteration<=64;++iteration) {
                unsigned cycle=iteration&63u;
                const flexray_frame_gen_packet_t *packet=flexray_frame_gen_template_prepare(&g.tpl,len,cycle);
                assert(packet==&g.tpl.packet && g.before==123 && g.after==456);
                assert(__builtin_bswap32(packet->words[0])==len-1);
                const uint8_t *frame=(const uint8_t *)(packet->words+1);
                assert((frame[4]&63)==cycle && (frame[2]>>1)*2==len-8);
                assert(calculate_flexray_header_crc(frame)==hcrc);
                assert(frame[1]==raw[1] && frame[2]==raw[2] && frame[3]==raw[3]);
                for(unsigned i=5;i<len-3;++i) assert(frame[i]==(null?0:raw[i]));
                assert((frame[0]&0x60)==(null?0:0x60));
                uint32_t crc=((uint32_t)frame[len-3]<<16)|((uint32_t)frame[len-2]<<8)|frame[len-1];
                assert(crc==independent_crc(frame,len-3));
                assert(memcmp(g.tpl.cycle_crc,crc_table,sizeof(crc_table))==0);
                for(unsigned i=len;i<sizeof(packet->words)-4u;++i) assert(frame[i]==0);
            }
        }
        for(unsigned tss=6;tss<=15;++tss) {
            uint32_t words[FLEXRAY_FRAME_GEN_INDUCER_WORDS + 1u];
            memset(words,0xa5,sizeof(words));
            unsigned n=flexray_frame_gen_inducer_generate(words,len,tss);
            assert(n==2*len+3 && words[n]==0xa5a5a5a5);
            assert(words[0]==((tss*15-5)<<1));
            assert(words[1]==((30-5)<<1|1));
            unsigned time=0;
            for(unsigned i=0;i<n-1;++i) time+=(words[i]>>1)+5;
            assert(time==tss*15+30+len*150+165);
            assert(words[n-1]==1);
        }
    }
    uint32_t words[FLEXRAY_FRAME_GEN_INDUCER_WORDS + 1u];
    assert(!flexray_frame_gen_inducer_generate(words,7,6));
    assert(!flexray_frame_gen_inducer_generate(words,263,6));
    assert(!flexray_frame_gen_inducer_generate(words,8,5));
    assert(!flexray_frame_gen_inducer_generate(words,8,16));
    puts("flexray static wire tests passed");
}
