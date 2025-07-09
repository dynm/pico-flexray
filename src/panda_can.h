#ifndef PANDA_CAN_H
#define PANDA_CAN_H

#include <stdint.h>

// Copied from usb_comm.txt
// In panda/board/can_definitions.h
typedef struct {
  uint8_t reserved : 1;
  uint8_t bus : 3;
  uint8_t data_len_code : 4; // DLC (数据长度码)
  uint8_t checksum : 1;
  uint8_t rejected : 1;
  uint8_t returned : 1;
  uint8_t extended : 1;
  uint32_t addr : 29;
} __attribute__((packed)) can_header;


#endif // PANDA_CAN_H 