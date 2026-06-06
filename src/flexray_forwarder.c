#include <stdint.h>
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"


#include "flexray_forwarder_with_injector.pio.h"
#include "flexray_forwarder.h"
#include "flexray_frame.h"
#include "flexray_injector_rules.h"
#include "flexray_pins.h"

static PIO pio_forwarder_with_injector;
static uint sm_forwarder_with_injector_to_fr1;
static uint sm_forwarder_with_injector_to_fr2;
static uint sm_forwarder_with_injector_to_fr3;
static uint sm_forwarder_with_injector_to_fr4;
static uint forwarder_program_offset;

extern volatile int dma_inject_chan_to_fr1;
extern volatile int dma_inject_chan_to_fr2;
extern volatile int dma_inject_chan_to_fr3;
extern volatile int dma_inject_chan_to_fr4;
static dma_channel_config injector_to_fr1_dc;
static dma_channel_config injector_to_fr2_dc;
static dma_channel_config injector_to_fr3_dc;
static dma_channel_config injector_to_fr4_dc;

static volatile uint8_t suppressed_source_mask = 0;

static void suppress_forwarder_sm(uint sm, uint tx_pin)
{
    if (pio_forwarder_with_injector == NULL) {
        return;
    }

    pio_sm_set_enabled(pio_forwarder_with_injector, sm, false);
    pio_sm_set_pins_with_mask(pio_forwarder_with_injector, sm, 1u << tx_pin, 1u << tx_pin);
}

void __time_critical_func(flexray_forwarder_suppress_source)(uint8_t source)
{
    if ((suppressed_source_mask & source) != 0u) {
        return;
    }

    switch (source) {
    case FROM_FR1:
        suppress_forwarder_sm(sm_forwarder_with_injector_to_fr2, TXD_FR_2_PIN);
        break;
    case FROM_FR2:
        suppress_forwarder_sm(sm_forwarder_with_injector_to_fr1, TXD_FR_1_PIN);
        break;
    case FROM_FR3:
        suppress_forwarder_sm(sm_forwarder_with_injector_to_fr4, TXD_FR_4_PIN);
        break;
    case FROM_FR4:
        suppress_forwarder_sm(sm_forwarder_with_injector_to_fr3, TXD_FR_3_PIN);
        break;
    default:
        return;
    }
    suppressed_source_mask |= source;
}

void __time_critical_func(flexray_forwarder_release_all_suppressed)(void)
{
    uint8_t suppressed = suppressed_source_mask;
    if (suppressed == 0u || pio_forwarder_with_injector == NULL) {
        return;
    }

    uint32_t sm_mask = 0;
    if ((suppressed & FROM_FR1) != 0u) sm_mask |= 1u << sm_forwarder_with_injector_to_fr2;
    if ((suppressed & FROM_FR2) != 0u) sm_mask |= 1u << sm_forwarder_with_injector_to_fr1;
    if ((suppressed & FROM_FR3) != 0u) sm_mask |= 1u << sm_forwarder_with_injector_to_fr4;
    if ((suppressed & FROM_FR4) != 0u) sm_mask |= 1u << sm_forwarder_with_injector_to_fr3;

    suppressed_source_mask = 0;
    pio_restart_sm_mask(pio_forwarder_with_injector, sm_mask);

    if ((suppressed & FROM_FR1) != 0u) {
        pio_sm_exec(pio_forwarder_with_injector, sm_forwarder_with_injector_to_fr2, pio_encode_jmp(forwarder_program_offset));
    }
    if ((suppressed & FROM_FR2) != 0u) {
        pio_sm_exec(pio_forwarder_with_injector, sm_forwarder_with_injector_to_fr1, pio_encode_jmp(forwarder_program_offset));
    }
    if ((suppressed & FROM_FR3) != 0u) {
        pio_sm_exec(pio_forwarder_with_injector, sm_forwarder_with_injector_to_fr4, pio_encode_jmp(forwarder_program_offset));
    }
    if ((suppressed & FROM_FR4) != 0u) {
        pio_sm_exec(pio_forwarder_with_injector, sm_forwarder_with_injector_to_fr3, pio_encode_jmp(forwarder_program_offset));
    }

    pio_set_sm_mask_enabled(pio_forwarder_with_injector, sm_mask, true);
}

void __time_critical_func(flexray_forwarder_inject_frame)(uint8_t *frame, uint16_t len, uint8_t direction)
{
    uint sm;
    int dma_chan;

    switch (direction) {
    case INJECT_DIRECTION_TO_FR1:
        sm = sm_forwarder_with_injector_to_fr1;
        dma_chan = dma_inject_chan_to_fr1;
        break;
    case INJECT_DIRECTION_TO_FR2:
        sm = sm_forwarder_with_injector_to_fr2;
        dma_chan = dma_inject_chan_to_fr2;
        break;
    case INJECT_DIRECTION_TO_FR3:
        sm = sm_forwarder_with_injector_to_fr3;
        dma_chan = dma_inject_chan_to_fr3;
        break;
    case INJECT_DIRECTION_TO_FR4:
        sm = sm_forwarder_with_injector_to_fr4;
        dma_chan = dma_inject_chan_to_fr4;
        break;
    default:
        return;
    }

    pio_sm_put(pio_forwarder_with_injector, sm, len - 1);
    dma_channel_set_read_addr((uint)dma_chan, (const void *)frame, false);
    dma_channel_set_trans_count((uint)dma_chan, (len + 3) / 4, true);
}

static void setup_inject_dma_channel(volatile int *chan, dma_channel_config *dc, uint sm)
{
    *chan = (int)dma_claim_unused_channel(true);
    *dc = dma_channel_get_default_config((uint)*chan);
    channel_config_set_transfer_data_size(dc, DMA_SIZE_32);
    channel_config_set_bswap(dc, true);
    channel_config_set_read_increment(dc, true);
    channel_config_set_write_increment(dc, false);
    channel_config_set_dreq(dc, pio_get_dreq(pio_forwarder_with_injector, sm, true));
    dma_channel_set_config((uint)*chan, dc, false);
    dma_channel_set_write_addr((uint)*chan, (void *)&pio_forwarder_with_injector->txf[sm], false);
}

static void setup_dma(void){
    setup_inject_dma_channel(&dma_inject_chan_to_fr1, &injector_to_fr1_dc, sm_forwarder_with_injector_to_fr1);
    setup_inject_dma_channel(&dma_inject_chan_to_fr2, &injector_to_fr2_dc, sm_forwarder_with_injector_to_fr2);
    setup_inject_dma_channel(&dma_inject_chan_to_fr3, &injector_to_fr3_dc, sm_forwarder_with_injector_to_fr3);
    setup_inject_dma_channel(&dma_inject_chan_to_fr4, &injector_to_fr4_dc, sm_forwarder_with_injector_to_fr4);
}

void setup_forwarder_with_injector(PIO pio)
{
    pio_forwarder_with_injector = pio;
    uint offset = pio_add_program(pio, &flexray_forwarder_with_injector_program);
    forwarder_program_offset = offset;
    sm_forwarder_with_injector_to_fr1 = pio_claim_unused_sm(pio, true);
    sm_forwarder_with_injector_to_fr2 = pio_claim_unused_sm(pio, true);
    sm_forwarder_with_injector_to_fr3 = pio_claim_unused_sm(pio, true);
    sm_forwarder_with_injector_to_fr4 = pio_claim_unused_sm(pio, true);

    flexray_forwarder_with_injector_program_init(pio, sm_forwarder_with_injector_to_fr2, offset, RXD_FR_1_PIN, TXD_FR_2_PIN);
    flexray_forwarder_with_injector_program_init(pio, sm_forwarder_with_injector_to_fr1, offset, RXD_FR_2_PIN, TXD_FR_1_PIN);
    flexray_forwarder_with_injector_program_init(pio, sm_forwarder_with_injector_to_fr4, offset, RXD_FR_3_PIN, TXD_FR_4_PIN);
    flexray_forwarder_with_injector_program_init(pio, sm_forwarder_with_injector_to_fr3, offset, RXD_FR_4_PIN, TXD_FR_3_PIN);
    pio_set_sm_mask_enabled(pio,
                            (1u << sm_forwarder_with_injector_to_fr1) |
                            (1u << sm_forwarder_with_injector_to_fr2) |
                            (1u << sm_forwarder_with_injector_to_fr3) |
                            (1u << sm_forwarder_with_injector_to_fr4),
                            true);
    setup_dma();
}
