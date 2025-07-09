#ifndef USB_COMMS_H
#define USB_COMMS_H

#include "flexray_frame.h"

void usb_comms_init(void);
void usb_comms_task(void);
void usb_comms_send_flexray_frame(flexray_frame_t *frame);

#endif // USB_COMMS_H 