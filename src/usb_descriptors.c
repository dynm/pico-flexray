#include "tusb.h"
#include "pico/unique_id.h"
#include <string.h>

// Panda USB VID/PID
#define PANDA_VID 0xbbaa
#define PANDA_PID 0xddcc

#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN)

enum {
    ITF_NUM_VENDOR,
    ITF_NUM_TOTAL
};

enum {
    EPNUM_VENDOR_OUT = 0x03,  // Bulk OUT endpoint for CAN data from host to device
    EPNUM_VENDOR_IN = 0x81    // Bulk IN endpoint for CAN data from device to host
};

//--------------------------------------------------------------------+
// Device Descriptor
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = PANDA_VID,
    .idProduct = PANDA_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
uint8_t const desc_cfg[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, EP Out & In address, EP size
    TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, 4, EPNUM_VENDOR_OUT, EPNUM_VENDOR_IN, 64)
};

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+
enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_INTERFACE,
};

char const* string_desc_arr[] = {
    (char[]){ 0x09, 0x04 },  // 0: is supported language is English (0x0409)
    "comma.ai",              // 1: Manufacturer
    "panda",                 // 2: Product
    NULL,                    // 3: Serial, will be filled from board ID
    "Panda Interface"        // 4: Interface
};

static uint16_t _desc_str[32];
static char serial_str[17]; // 8 bytes * 2 hex chars + null terminator

//--------------------------------------------------------------------+
// TinyUSB Callbacks
//--------------------------------------------------------------------+
uint8_t const* tud_descriptor_device_cb(void) {
    return (uint8_t const*)&desc_device;
}

uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    (void)index; // for multiple configurations
    return desc_cfg;
}

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    
    uint8_t chr_count;

    if (index == 0) {
        memcpy(&_desc_str[0], string_desc_arr[0], 2);
        chr_count = 1;
    } else if (index == 3) {
        // Generate serial number from board unique ID
        if (serial_str[0] == '\0') {
            pico_unique_board_id_t board_id;
            pico_get_unique_board_id(&board_id);
            
            snprintf(serial_str, sizeof(serial_str), "%02x%02x%02x%02x%02x%02x%02x%02x",
                     board_id.id[0], board_id.id[1], board_id.id[2], board_id.id[3],
                     board_id.id[4], board_id.id[5], board_id.id[6], board_id.id[7]);
        }
        string_desc_arr[3] = serial_str;

        chr_count = strlen(string_desc_arr[index]);
        if (chr_count > 31) chr_count = 31;

        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = string_desc_arr[index][i];
        }
    } else if (index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) {
        const char* str = string_desc_arr[index];
        chr_count = strlen(str);
        if (chr_count > 31) chr_count = 31;

        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
    } else {
        return NULL;
    }

    // first byte is length (including header), second byte is string type
    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);

    return _desc_str;
} 