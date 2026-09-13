#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

// CMake selects one firmware mode; standalone host tests default to frame generation.
#ifndef FLEXRAY_FRAME_GEN
#define FLEXRAY_FRAME_GEN 0
#endif
#if FLEXRAY_FRAME_GEN != 0 && FLEXRAY_FRAME_GEN != 1
#error "FLEXRAY_FRAME_GEN must be 0 or 1"
#endif

// PIO1 owns primary TXEN; PIO2 owns TXD. Static TX reuses the secondary
// interface pins/SMs, so it is mutually exclusive with the FR3/FR4 bridge.
#define TXD_FR_1_PIN 28u
#define TXEN_FR_1_PIN 27u
#define RXD_FR_1_PIN 26u
#define TXD_FR_2_PIN 4u
#define TXEN_FR_2_PIN 5u
#define RXD_FR_2_PIN 6u
#define TXD_FR_3_PIN 10u
#define TXEN_FR_3_PIN 9u
#define RXD_FR_3_PIN 8u
#define TXD_FR_4_PIN 16u
#define TXEN_FR_4_PIN 22u
#define RXD_FR_4_PIN 21u
#if FLEXRAY_FRAME_GEN
#define FLEXRAY_INTERNAL_PIN 16u
#define FLEXRAY_FRAME_GEN_OWNERSHIP_PIN 17u
#define FLEXRAY_FSS_MARKER_PIN 10u
#endif

#endif // BOARD_CONFIG_H
