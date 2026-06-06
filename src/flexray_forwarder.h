#ifndef FLEXRAY_FORWARDER_H
#define FLEXRAY_FORWARDER_H

#include <stdint.h>
#include "hardware/pio.h"

void setup_forwarder_with_injector(PIO pio);

void flexray_forwarder_inject_frame(uint8_t *frame, uint16_t len, uint8_t direction);

void flexray_forwarder_suppress_source(uint8_t source);
void flexray_forwarder_release_all_suppressed(void);

#endif // FLEXRAY_FORWARDER_H
