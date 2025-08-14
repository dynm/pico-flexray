#include <stdio.h>
#include <limits.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "flexray_replay_q8_frame.pio.h"
#include "replay_frame.h"

#define REAL_DATA 1

#ifdef REAL_DATA
// uint32_t replay_buffer[64] __attribute__((aligned(256))) = {
//     0xffffffff, 0xffffffff, 0xffffffff, 0x180fffff, 0x97113040, 0x01006216, 0x00401004, 0x40100401,
//     0x10040100, 0x04010040, 0x01004010, 0x00401004, 0x40100401, 0x10040100, 0x04010040, 0x01004010,
//     0xea40d834, 0xfffffffe, 0xffffffff, 0x01ffffff, 0xc46c1006, 0x4018a75d, 0x10040100, 0x04010040,
//     0x01004010, 0x00401004, 0x40100401, 0x10040100, 0x04010040, 0x01004010, 0x00401004, 0x40100401,
//     0x1e96c900, 0xffffffbb, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
//     0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
//     0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
//     0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff};

// bmw dynamic frame
uint32_t replay_buffer[64] __attribute__((aligned(256))) = {
    0xFFFFFFFF, 0x03104B90, 0x4D1314C8, 0x100C0100, 0x40100401, 0x0055F01D, 0xA4800000, 0x001FFFFF, 
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFF81882, 0x61822129, 0xA64CB66F, 0xDBFEE0BF, 
    0x67BBFE2F, 0x886E4400, 0x000000FF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFC0C4, 0x1364236E, 0x5D30DC25, 0x004FB004, 
    0xFB004FB6, 0x1445004F, 0xB004FB79, 0x443075C5, 0x63A00000, 0x3FFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};
#else
uint32_t replay_buffer[64] __attribute__((aligned(256))) = {
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
    0x55555555,
};
#endif

void setup_replay(PIO pio, uint replay_pin)
{
    uint offset = pio_add_program(pio, &flexray_replay_q8_frame_program);
    uint sm = pio_claim_unused_sm(pio, true);
    flexray_replay_q8_frame_program_init(pio, sm, offset, replay_pin);

    uint dma_chan = dma_claim_unused_channel(true);
    uint dma_replay_rearm_chan = dma_claim_unused_channel(true);
    dma_channel_config dma_c = dma_channel_get_default_config(dma_chan);

    channel_config_set_transfer_data_size(&dma_c, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_c, true);
    channel_config_set_write_increment(&dma_c, false);
    channel_config_set_dreq(&dma_c, pio_get_dreq(pio, sm, true));

    // Keep circular read ring on the replay buffer
    uint32_t buffer_words = (uint32_t)(sizeof(replay_buffer) / sizeof(uint32_t));
    uint32_t buffer_size_bytes = sizeof(replay_buffer);
    uint8_t ring_size_log2 = 0;
    if (buffer_size_bytes > 1) {
        ring_size_log2 = 32 - __builtin_clz(buffer_size_bytes - 1);
    }
    channel_config_set_ring(&dma_c, false, ring_size_log2); // false = wrap read address
    channel_config_set_chain_to(&dma_c, dma_replay_rearm_chan);

    dma_channel_configure(
        dma_chan,
        &dma_c,
        &pio->txf[sm],          // Write address: PIO TX FIFO
        replay_buffer,          // Read address: start of our data
        buffer_words,           // Transfer count: one full buffer
        true                    // Start immediately
    );

    // Rearm channel: write TRANS_COUNT_TRIG only (count+trigger).
    dma_channel_config rearm_c = dma_channel_get_default_config(dma_replay_rearm_chan);
    channel_config_set_transfer_data_size(&rearm_c, DMA_SIZE_32);
    channel_config_set_read_increment(&rearm_c, false);
    channel_config_set_write_increment(&rearm_c, false);
    channel_config_set_dreq(&rearm_c, DREQ_FORCE);

    static uint32_t refill_count;
    refill_count = buffer_words; // number of 32-bit words per block

    dma_channel_configure(
        dma_replay_rearm_chan,
        &rearm_c,
        (volatile void *)&dma_hw->ch[dma_chan].al1_transfer_count_trig, // dest: count+trigger of main channel
        &refill_count,                                                  // src: constant transfer count
        1,                                                              // one write per rearm
        false                                                           // start by chain
    );
}
