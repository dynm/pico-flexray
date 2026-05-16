#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#define CFG_TUD_ENABLED         (1)

#define CFG_TUSB_RHPORT0_MODE   (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#ifndef BOARD_TUD_RHPORT
#define BOARD_TUD_RHPORT        (0)
#endif

#ifndef CFG_TUSB_MCU
#define CFG_TUSB_MCU OPT_MCU_RP2040
#endif

#define CFG_TUSB_OS OPT_OS_PICO

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE  (64)
#endif

// Vendor class for WebUSB bulk transfers
#define CFG_TUD_VENDOR            1
#define CFG_TUD_VENDOR_RX_BUFSIZE 512
#define CFG_TUD_VENDOR_TX_BUFSIZE 512

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG            0
#endif

#define CFG_TUD_CDC               0
#define CFG_TUD_MSC               0
#define CFG_TUD_HID               0
#define CFG_TUD_MIDI              0
#define CFG_TUD_AUDIO             0
#define CFG_TUD_VIDEO             0
#define CFG_TUD_DFU_RUNTIME       0
#define CFG_TUD_DFU               0
#define CFG_TUD_ECM_RNDIS         0
#define CFG_TUD_NCM               0

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */
