#ifndef FLEXRAY_FRAME_GEN_PACKET_H
#define FLEXRAY_FRAME_GEN_PACKET_H
#include <stdbool.h>
#include <stdint.h>
#define FLEXRAY_FRAME_GEN_PACKET_MIN_FRAME_BYTES 8u
#define FLEXRAY_FRAME_GEN_PACKET_MAX_FRAME_BYTES 262u
#define FLEXRAY_FRAME_GEN_PACKET_MIN_TSS_BITS 6u
#define FLEXRAY_FRAME_GEN_PACKET_MAX_TSS_BITS 15u
#define FLEXRAY_FRAME_GEN_PACKET_WORDS 67u
#define FLEXRAY_FRAME_GEN_INDUCER_WORDS (2u * FLEXRAY_FRAME_GEN_PACKET_MAX_FRAME_BYTES + 3u)

// Original injector format: byte-swapped count, native frame bytes; DMA
// byte-swaps each word for the established MSB-first OUT instructions.
typedef struct { uint32_t words[FLEXRAY_FRAME_GEN_PACKET_WORDS]; } flexray_frame_gen_packet_t;
// A template changes only its cycle byte and final CRC between cycles.
// The selected packet belongs to core1 through DMA/frame end; core0 builds
// replacements in the inactive bank.
typedef struct {
    flexray_frame_gen_packet_t packet;
    uint8_t cycle_crc[64][3];
} flexray_frame_gen_template_t;
void flexray_frame_gen_template_generate(flexray_frame_gen_template_t *tpl, const uint8_t *frame,
                                   uint16_t len, bool null_frame);
const flexray_frame_gen_packet_t *flexray_frame_gen_template_prepare(flexray_frame_gen_template_t *tpl,
                                                              uint16_t len, uint8_t cycle);
// TSS/FSS/BSS cadence, FES and post-FES idle; actual bytes use the injector FIFO.
uint32_t flexray_frame_gen_inducer_generate(uint32_t *words, uint16_t len, uint8_t tss);
#endif
