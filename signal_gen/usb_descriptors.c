#include "tusb.h"
#include "pico/unique_id.h"
#include <string.h>

// WebUSB vendor requests
#define VENDOR_REQUEST_WEBUSB    1
#define VENDOR_REQUEST_MICROSOFT 2

#define SIGNAL_GEN_VID 0xCAFE
#define SIGNAL_GEN_PID 0x4004

#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN)

enum {
    ITF_NUM_VENDOR,
    ITF_NUM_TOTAL
};

enum {
    EPNUM_VENDOR_OUT = 0x03,
    EPNUM_VENDOR_IN  = 0x81
};

// ---------- Device Descriptor (USB 2.1 for BOS) ----------
tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0210,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = SIGNAL_GEN_VID,
    .idProduct          = SIGNAL_GEN_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

// ---------- Configuration Descriptor ----------
uint8_t const desc_cfg[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN, 0x00, 100),
    TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, 4, EPNUM_VENDOR_OUT, EPNUM_VENDOR_IN, 64)
};

// ---------- BOS Descriptor (WebUSB + MS OS 2.0) ----------
#define BOS_TOTAL_LEN  (TUD_BOS_DESC_LEN + TUD_BOS_WEBUSB_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)

// MS OS 2.0 descriptor set length
#define MS_OS_20_DESC_LEN 0xB2

uint8_t const desc_bos[] = {
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 2),
    TUD_BOS_WEBUSB_DESCRIPTOR(VENDOR_REQUEST_WEBUSB, 1),
    TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, VENDOR_REQUEST_MICROSOFT),
};

// ---------- Microsoft OS 2.0 Descriptor Set ----------
static uint8_t const desc_ms_os_20[] = {
    // Set header
    0x0A, 0x00,
    0x00, 0x00,  // MS_OS_20_SET_HEADER_DESCRIPTOR
    0x00, 0x00, 0x03, 0x06,  // dwWindowsVersion 0x06030000
    (uint8_t)(MS_OS_20_DESC_LEN), (uint8_t)(MS_OS_20_DESC_LEN >> 8),

    // Configuration subset header
    0x08, 0x00,
    0x01, 0x00,  // MS_OS_20_SUBSET_HEADER_CONFIGURATION
    0x00, 0x00,
    (uint8_t)(MS_OS_20_DESC_LEN - 0x0A), (uint8_t)((MS_OS_20_DESC_LEN - 0x0A) >> 8),

    // Function subset header
    0x08, 0x00,
    0x02, 0x00,  // MS_OS_20_SUBSET_HEADER_FUNCTION
    ITF_NUM_VENDOR, 0x00,
    (uint8_t)(MS_OS_20_DESC_LEN - 0x0A - 0x08),
    (uint8_t)((MS_OS_20_DESC_LEN - 0x0A - 0x08) >> 8),

    // Compatible ID: WINUSB
    0x14, 0x00,
    0x03, 0x00,  // MS_OS_20_FEATURE_COMPATBLE_ID
    'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // Registry property: DeviceInterfaceGUIDs
    0x84, 0x00,
    0x04, 0x00,  // MS_OS_20_FEATURE_REG_PROPERTY
    0x07, 0x00,  // REG_MULTI_SZ
    0x2A, 0x00,  // wPropertyNameLength
    'D',0,'e',0,'v',0,'i',0,'c',0,'e',0,'I',0,'n',0,'t',0,'e',0,
    'r',0,'f',0,'a',0,'c',0,'e',0,'G',0,'U',0,'I',0,'D',0,'s',0,0,0,
    0x50, 0x00,  // wPropertyDataLength
    '{',0,'C',0,'D',0,'B',0,'3',0,'B',0,'5',0,'A',0,'D',0,'-',0,
    '2',0,'9',0,'3',0,'B',0,'-',0,'4',0,'6',0,'6',0,'3',0,'-',0,
    'A',0,'A',0,'3',0,'6',0,'-',0,'1',0,'A',0,'A',0,'E',0,'4',0,
    '6',0,'4',0,'6',0,'3',0,'7',0,'7',0,'6',0,'}',0,0,0,0,0,
};

_Static_assert(sizeof(desc_ms_os_20) == MS_OS_20_DESC_LEN,
               "MS OS 2.0 descriptor size mismatch");

// ---------- WebUSB URL Descriptor ----------
static uint8_t const desc_url[] = {
    3 + 26,                    // bLength (3 header + URL chars)
    3,                         // bDescriptorType (WEBUSB URL)
    1,                         // bScheme (https://)
    'e','x','a','m','p','l','e','.','c','o','m',
    '/','w','e','b','u','s','b','/','f','l','e','x','r','a','y',
};

// ---------- String Descriptors ----------
enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_INTERFACE,
};

char const *string_desc_arr[] = {
    (char[]){0x09, 0x04},       // 0: English
    "PicoFlexRay",              // 1: Manufacturer
    "FlexRay Signal Generator", // 2: Product
    NULL,                       // 3: Serial (filled at runtime)
    "FlexRay SignalGen"         // 4: Interface
};

static uint16_t _desc_str[32];
static char serial_str[25];

// ---------- TinyUSB Callbacks ----------
uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&desc_device;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_cfg;
}

uint8_t const *tud_descriptor_bos_cb(void) {
    return desc_bos;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    uint8_t chr_count;

    if (index == 0) {
        memcpy(&_desc_str[0], string_desc_arr[0], 2);
        chr_count = 1;
    } else if (index == 3) {
        pico_unique_board_id_t board_id;
        pico_get_unique_board_id(&board_id);
        static const char hex[] = "0123456789abcdef";
        memcpy(serial_str, "siggen", 6);
        uint8_t pos = 6;
        for (int i = 0; i < 8; i++) {
            serial_str[pos++] = hex[board_id.id[i] >> 4];
            serial_str[pos++] = hex[board_id.id[i] & 0x0F];
        }
        serial_str[pos] = '\0';
        string_desc_arr[3] = serial_str;
        chr_count = (uint8_t)strlen(serial_str);
        if (chr_count > 31) chr_count = 31;
        for (uint8_t i = 0; i < chr_count; i++)
            _desc_str[1 + i] = (uint16_t)serial_str[i];
    } else if (index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) {
        const char *str = string_desc_arr[index];
        chr_count = (uint8_t)strlen(str);
        if (chr_count > 31) chr_count = 31;
        for (uint8_t i = 0; i < chr_count; i++)
            _desc_str[1 + i] = (uint16_t)str[i];
    } else {
        return NULL;
    }
    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}

// ---------- Vendor control request handler (WebUSB + MS OS 2.0) ----------
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                 tusb_control_request_t const *request)
{
    if (stage != CONTROL_STAGE_SETUP) return true;

    switch (request->bmRequestType_bit.type) {
    case TUSB_REQ_TYPE_VENDOR:
        if (request->bRequest == VENDOR_REQUEST_WEBUSB) {
            return tud_control_xfer(rhport, request,
                                    (void *)(uintptr_t)desc_url, sizeof(desc_url));
        }
        if (request->bRequest == VENDOR_REQUEST_MICROSOFT &&
            request->wIndex == 7) {
            uint16_t total_len = sizeof(desc_ms_os_20);
            uint16_t xfer = (request->wLength < total_len) ? request->wLength : total_len;
            return tud_control_xfer(rhport, request,
                                    (void *)(uintptr_t)desc_ms_os_20, xfer);
        }
        break;

    case TUSB_REQ_TYPE_CLASS:
        if (request->bRequest == 0x22) {
            return tud_control_status(rhport, request);
        }
        break;

    default:
        break;
    }
    return false;
}
