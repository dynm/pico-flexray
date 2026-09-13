#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------
#define CFG_TUD_ENABLED         (1)

// Legacy RHPORT configuration
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

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

// Vendor class for Panda protocol
#define CFG_TUD_VENDOR            1
#define CFG_TUD_VENDOR_RX_BUFSIZE 4096
#define CFG_TUD_VENDOR_TX_BUFSIZE 8192
// Keep the USB descriptor's Full-Speed max packet at 64 bytes, but let TinyUSB
// submit several packets per software transfer. The RP2350 DCD splits it into
// 64-byte hardware packets; fifteen packets gave the best measured Panda
// throughput on the composite Vendor + NCM device.
#define CFG_TUD_VENDOR_EPSIZE     960

// Enable debug for troubleshooting (only if not already defined)
#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG            0
#endif

// Disable other classes
#define CFG_TUD_CDC               0
#define CFG_TUD_MSC               0
#define CFG_TUD_HID               0
#define CFG_TUD_MIDI              0
#define CFG_TUD_AUDIO             0
#define CFG_TUD_VIDEO             0
#define CFG_TUD_DFU_RUNTIME       0
#define CFG_TUD_DFU               0
#define CFG_TUD_ECM_RNDIS         0
#define CFG_TUD_NCM               1

// Network (CDC-NCM) configuration. Full-speed only on RP2350; keep the NTB
// buffers modest since we only stream UDP datagrams.
#define CFG_TUD_NCM_IN_NTB_MAX_SIZE   2048
#define CFG_TUD_NCM_OUT_NTB_MAX_SIZE  2048
#define CFG_TUD_NCM_IN_NTB_N          2
#define CFG_TUD_NCM_OUT_NTB_N         1

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */
