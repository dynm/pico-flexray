#ifndef PANDA_USB_H
#define PANDA_USB_H

#include <stdint.h>
#include <stdbool.h>
#include "tusb.h"
#include "panda_can.h"
#include "flexray_frame.h"

// Panda Control Request Commands (from usb_comm.txt)
#define PANDA_GET_HW_TYPE           0xc1
#define PANDA_CAN_RESET_COMMS       0xc0
#define PANDA_SET_SAFETY_MODEL      0xdc
#define PANDA_SET_CAN_SPEED_KBPS    0xde
#define PANDA_SET_DATA_SPEED_KBPS   0xf9
#define PANDA_HEARTBEAT             0xf3

// Hardware Types
#define HW_TYPE_RED_PANDA           4

// Safety Models
#define SAFETY_SILENT               3

// Function prototypes
void panda_usb_init(void);
void panda_usb_task(void);
bool panda_handle_control_request(uint8_t rhport, tusb_control_request_t const* request);
void panda_send_can_data(const can_header* header, const uint8_t* data, uint8_t len);
uint8_t panda_calculate_checksum(const can_header* header, const uint8_t* data, uint8_t len);
void panda_send_flexray_as_can(const flexray_frame_t* fr_frame);

#endif // PANDA_USB_H 