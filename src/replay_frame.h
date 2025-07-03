#ifndef REPLAY_FRAME_H
#define REPLAY_FRAME_H

#include "pico/stdlib.h"
#include "hardware/pio.h"

extern uint32_t replay_buffer[64];

/**
 * @brief Sets up a PIO state machine and DMA channel to continuously replay a FlexRay frame.
 * 
 * This function configures a PIO program to act as a FlexRay frame transmitter.
 * It uses a DMA channel to feed data from a buffer to the PIO's TX FIFO,
 * creating a continuous stream of frames on the REPLAY_TX_PIN.
 * @param pio The PIO instance (pio0 or pio1) to use for the replay state machine.
 * @param replay_pin The GPIO pin to use for transmitting the replay data.
 */
uint setup_replay(PIO pio, uint replay_pin);

#endif // REPLAY_FRAME_H 