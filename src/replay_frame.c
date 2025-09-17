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
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
    0x00C01204, 0x2166C010, 0x04010040, 0x10040100, 0x40100401, 0x00401004, 0x01004014, 0x2CB1417F,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
    0x00C01224, 0x231C4011, 0x65ED685C, 0xF7F5FD00, 0x5FD3458D, 0x685CF6D5, 0x030F4016, 0xCD1739FF,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
};
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
    // uint dma_replay_rearm_chan = dma_claim_unused_channel(true);
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
    // channel_config_set_chain_to(&dma_c, dma_replay_rearm_chan);

    buffer_words = buffer_words | 0x10000000; // rp2350's self trigger
    dma_channel_configure(
        dma_chan,
        &dma_c,
        &pio->txf[sm],          // Write address: PIO TX FIFO
        replay_buffer,          // Read address: start of our data
        buffer_words,           // Transfer count: one full buffer
        true                    // Start immediately
    );
}
