#ifndef PANDA_USB_H_
#define PANDA_USB_H_

#include "tusb.h"
#include "flexray_frame.h"

// Panda USB control requests from a more complete reference
#define PANDA_GET_MICROSECOND_TIMER     0xa8
#define PANDA_SET_IR_POWER              0xb0
#define PANDA_SET_FAN_POWER             0xb1
#define PANDA_GET_FAN_RPM               0xb2
#define PANDA_RESET_CAN_COMMS           0xc0
#define PANDA_GET_HW_TYPE               0xc1
#define PANDA_GET_CAN_HEALTH_STATS      0xc2
#define PANDA_GET_MCU_UID               0xc3
#define PANDA_GET_INTERRUPT_CALL_RATE   0xc4
#define PANDA_DEBUG_DRIVE_RELAY         0xc5
#define PANDA_DEBUG_READ_SOM_GPIO       0xc6
#define PANDA_FETCH_SERIAL_NUMBER       0xd0
#define PANDA_ENTER_BOOTLOADER_MODE     0xd1
#define PANDA_GET_HEALTH_PACKET         0xd2
#define PANDA_GET_SIGNATURE_PART1       0xd3
#define PANDA_GET_SIGNATURE_PART2       0xd4
#define PANDA_GET_GIT_VERSION           0xd6
#define PANDA_SYSTEM_RESET              0xd8
#define PANDA_SET_OBD_CAN_MUX_MODE      0xdb
#define PANDA_SET_SAFETY_MODEL          0xdc
#define PANDA_GET_VERSIONS              0xdd
#define PANDA_SET_CAN_SPEED_KBPS        0xde
#define PANDA_SET_ALT_EXPERIENCE        0xdf
#define PANDA_UART_READ                 0xe0
#define PANDA_UART_SET_BAUD_RATE        0xe1
#define PANDA_UART_SET_PARITY           0xe2
#define PANDA_UART_SET_EXT_BAUD_RATE    0xe4
#define PANDA_SET_CAN_LOOPBACK          0xe5
#define PANDA_SET_CLOCK_SOURCE_PARAMS   0xe6
#define PANDA_SET_POWER_SAVE_STATE      0xe7
#define PANDA_SET_CAN_FD_AUTO_SWITCH    0xe8
#define PANDA_CAN_CLEAR_BUFFER          0xf1
#define PANDA_UART_CLEAR_BUFFER         0xf2
#define PANDA_HEARTBEAT                 0xf3
#define PANDA_SET_SIREN_ENABLED         0xf6
#define PANDA_SET_GREEN_LED_ENABLED     0xf7
#define PANDA_DISABLE_HEARTBEAT_CHECKS  0xf8
#define PANDA_SET_CAN_FD_DATA_BITRATE   0xf9
#define PANDA_SET_CAN_FD_NON_ISO_MODE   0xfc

// Hardware types
#define HW_TYPE_UNKNOWN             0
#define HW_TYPE_WHITE_PANDA         1
#define HW_TYPE_GREY_PANDA          2
#define HW_TYPE_BLACK_PANDA         3
#define HW_TYPE_RED_PANDA           7
#define HW_TYPE_RED_PANDA_V2        8

// Safety models
#define SAFETY_SILENT               0
#define SAFETY_HONDA_NIDEC          1
#define SAFETY_TOYOTA               2
#define SAFETY_ELM327               3
#define SAFETY_GM                   4
#define SAFETY_HONDA_BOSCH_GIRAFFE  5
#define SAFETY_FORD                 6
#define SAFETY_HYUNDAI              8
#define SAFETY_CHRYSLER             9
#define SAFETY_SUBARU               10
#define SAFETY_MAZDA                11

#define HEALTH_PACKET_VERSION 16
struct __attribute__((packed)) health_t {
    uint32_t uptime_pkt;
    uint32_t voltage_pkt;
    uint32_t current_pkt;
    uint32_t safety_tx_blocked_pkt;
    uint32_t safety_rx_invalid_pkt;
    uint32_t tx_buffer_overflow_pkt;
    uint32_t rx_buffer_overflow_pkt;
    uint32_t faults_pkt;
    uint8_t ignition_line_pkt;
    uint8_t ignition_can_pkt;
    uint8_t controls_allowed_pkt;
    uint8_t car_harness_status_pkt;
    uint8_t safety_mode_pkt;
    uint16_t safety_param_pkt;
    uint8_t fault_status_pkt;
    uint8_t power_save_enabled_pkt;
    uint8_t heartbeat_lost_pkt;
    uint16_t alternative_experience_pkt;
    float interrupt_load_pkt;
    uint8_t fan_power;
    uint8_t safety_rx_checks_invalid_pkt;
    uint16_t spi_error_count_pkt;
    uint16_t sbu1_voltage_mV;
    uint16_t sbu2_voltage_mV;
    uint8_t som_reset_triggered;
  };
  
#define CAN_HEALTH_PACKET_VERSION 5
struct __attribute__((packed)) can_health_t {
    uint8_t bus_off;
    uint32_t bus_off_cnt;
    uint8_t error_warning;
    uint8_t error_passive;
    uint8_t last_error; // real time LEC value
    uint8_t last_stored_error; // last LEC positive error code stored
    uint8_t last_data_error; // DLEC (for CANFD only)
    uint8_t last_data_stored_error; // last DLEC positive error code stored (for CANFD only)
    uint8_t receive_error_cnt; // Actual state of the receive error counter, values between 0 and 127. FDCAN_ECR.REC
    uint8_t transmit_error_cnt; // Actual state of the transmit error counter, values between 0 and 255. FDCAN_ECR.TEC
    uint32_t total_error_cnt; // How many times any error interrupt was invoked
    uint32_t total_tx_lost_cnt; // Tx event FIFO element lost
    uint32_t total_rx_lost_cnt; // Rx FIFO 0 message lost due to FIFO full condition
    uint32_t total_tx_cnt;
    uint32_t total_rx_cnt;
    uint32_t total_fwd_cnt; // Messages forwarded from one bus to another
    uint32_t total_tx_checksum_error_cnt;
    uint16_t can_speed;
    uint16_t can_data_speed;
    uint8_t canfd_enabled;
    uint8_t brs_enabled;
    uint8_t canfd_non_iso;
    uint32_t irq0_call_rate;
    uint32_t irq1_call_rate;
    uint32_t irq2_call_rate;
    uint32_t can_core_reset_cnt;
  };
  
void panda_usb_init(void);
void panda_usb_task(void);

// FIFO management - now exposed for external use (e.g., main.c)
bool panda_flexray_fifo_push(const flexray_frame_t *frame);

#endif /* PANDA_USB_H_ */