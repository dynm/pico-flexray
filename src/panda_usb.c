#include "panda_usb.h"
#include "pico/stdlib.h"
#include "tusb.h"
#include "flexray_frame.h"
#include <string.h>

// Global state
static struct {
    uint8_t hw_type;
    uint8_t safety_model;
    uint16_t can_speed_kbps[3];  // Up to 3 CAN buses
    uint16_t can_fd_speed_kbps[3];
    bool initialized;
} panda_state;

// Internal functions
static bool handle_control_read(uint8_t rhport, tusb_control_request_t const* request);
static bool handle_control_write(uint8_t rhport, tusb_control_request_t const* request);

void panda_usb_init(void) {
    // Initialize TinyUSB
    tud_init(0);
    
    // Initialize panda state
    panda_state.hw_type = HW_TYPE_RED_PANDA;
    panda_state.safety_model = SAFETY_SILENT;
    memset(panda_state.can_speed_kbps, 0, sizeof(panda_state.can_speed_kbps));
    memset(panda_state.can_fd_speed_kbps, 0, sizeof(panda_state.can_fd_speed_kbps));
    panda_state.initialized = true;
    
    printf("Panda USB initialized - VID:0x%04x PID:0x%04x\n", 0xbbaa, 0xddcc);
}

void panda_usb_task(void) {
    tud_task();
}

bool panda_handle_control_request(uint8_t rhport, tusb_control_request_t const* request) {
    // Determine if this is a read (device-to-host) or write (host-to-device) request
    if (request->bmRequestType & TUSB_DIR_IN_MASK) {
        return handle_control_read(rhport, request);
    } else {
        return handle_control_write(rhport, request);
    }
}

static bool handle_control_read(uint8_t rhport, tusb_control_request_t const* request) {
    uint8_t response_data[64] = {0};
    uint16_t response_len = 0;
    
    switch (request->bRequest) {
        case PANDA_GET_HW_TYPE:
            // Return hardware type (Red Panda = 4 for CAN-FD support)
            response_data[0] = panda_state.hw_type;
            response_len = 1;
            printf("Control Read: GET_HW_TYPE -> %d\n", response_data[0]);
            break;
            
        default:
            printf("Control Read: Unknown request 0x%02x\n", request->bRequest);
            return false;
    }
    
    if (response_len > 0) {
        return tud_control_xfer(rhport, request, response_data, response_len);
    }
    
    return false;
}

static bool handle_control_write(uint8_t rhport, tusb_control_request_t const* request) {
    switch (request->bRequest) {
        case PANDA_CAN_RESET_COMMS:
            // Reset CAN communications - clear buffers, reset state
            printf("Control Write: CAN_RESET_COMMS\n");
            // TODO: Clear any CAN buffers here
            return tud_control_status(rhport, request);
            
        case PANDA_SET_SAFETY_MODEL:
            // Set safety model (wValue contains the safety model)
            panda_state.safety_model = request->wValue;
            printf("Control Write: SET_SAFETY_MODEL -> %d\n", panda_state.safety_model);
            return tud_control_status(rhport, request);
            
        case PANDA_SET_CAN_SPEED_KBPS:
            // Set CAN speed - wValue=bus_index, wIndex=speed_kbps*10
            if (request->wValue < 3) {
                panda_state.can_speed_kbps[request->wValue] = request->wIndex / 10;
                printf("Control Write: SET_CAN_SPEED_KBPS bus=%d speed=%d kbps\n", 
                       request->wValue, panda_state.can_speed_kbps[request->wValue]);
            }
            return tud_control_status(rhport, request);
            
        case PANDA_SET_DATA_SPEED_KBPS:
            // Set CAN-FD data speed - wValue=bus_index, wIndex=data_speed_kbps*10
            if (request->wValue < 3) {
                panda_state.can_fd_speed_kbps[request->wValue] = request->wIndex / 10;
                printf("Control Write: SET_DATA_SPEED_KBPS bus=%d speed=%d kbps\n", 
                       request->wValue, panda_state.can_fd_speed_kbps[request->wValue]);
            }
            return tud_control_status(rhport, request);
            
        case PANDA_HEARTBEAT:
            // Heartbeat from host - just acknowledge
            printf("Control Write: HEARTBEAT\n");
            return tud_control_status(rhport, request);
            
        default:
            printf("Control Write: Unknown request 0x%02x\n", request->bRequest);
            return false;
    }
}

uint8_t panda_calculate_checksum(const can_header* header, const uint8_t* data, uint8_t len) {
    uint8_t checksum = 0;
    
    // XOR all bytes of the header
    const uint8_t* header_bytes = (const uint8_t*)header;
    for (int i = 0; i < sizeof(can_header); i++) {
        checksum ^= header_bytes[i];
    }
    
    // XOR all bytes of the data
    for (int i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    
    return checksum;
}

void panda_send_can_data(const can_header* header, const uint8_t* data, uint8_t len) {
    // Create a copy of the header and set the checksum
    can_header header_with_checksum = *header;
    header_with_checksum.checksum = panda_calculate_checksum(header, data, len);
    
    // Prepare the packet: header + data
    uint8_t packet[sizeof(can_header) + 64]; // Max CAN-FD payload is 64 bytes
    memcpy(packet, &header_with_checksum, sizeof(can_header));
    memcpy(packet + sizeof(can_header), data, len);
    
    size_t total_len = sizeof(can_header) + len;
    
    // Send via Bulk IN endpoint (0x81)
    if (tud_vendor_mounted()) {
        uint32_t bytes_written = tud_vendor_write(packet, total_len);
        if (bytes_written > 0) {
            tud_vendor_flush();
            printf("Sent CAN packet: ID=0x%03x, bus=%d, len=%d, checksum=0x%02x\n", 
                   header->addr, header->bus, len, header_with_checksum.checksum);
        } else {
            printf("Failed to send CAN packet\n");
        }
    }
}

// Convert FlexRay frame to CAN format and send
void panda_send_flexray_as_can(const flexray_frame_t* fr_frame) {
    // Use a special CAN ID to indicate this is FlexRay data
    can_header header = {0};
    header.addr = 0x600 + fr_frame->frame_id; // FlexRay frames use 0x600-0x7FF range
    header.extended = 1;
    header.bus = 0;
    
    // Pack FlexRay header info into first 8 bytes of CAN data
    uint8_t can_data[64];
    can_data[0] = fr_frame->frame_id & 0xFF;
    can_data[1] = (fr_frame->frame_id >> 8) & 0x7;
    can_data[2] = fr_frame->payload_length_words;
    can_data[3] = fr_frame->cycle_count;
    can_data[4] = (fr_frame->startup_frame_indicator << 7) |
                  (fr_frame->sync_frame_indicator << 6) |
                  (fr_frame->null_frame_indicator << 5) |
                  (fr_frame->payload_preamble_indicator << 4);
    can_data[5] = fr_frame->header_crc & 0xFF;
    can_data[6] = (fr_frame->header_crc >> 8) & 0xFF;
    can_data[7] = (fr_frame->header_crc >> 16) & 0xFF;
    
    // Add some payload data if available (limited by CAN frame size)
    int payload_bytes = fr_frame->payload_length_words * 2;
    int can_payload_space = 64 - 8; // Remaining space after header
    int copy_bytes = (payload_bytes < can_payload_space) ? payload_bytes : can_payload_space;
    
    if (copy_bytes > 0) {
        memcpy(can_data + 8, fr_frame->payload, copy_bytes);
    }
    
    // Set DLC based on total data length
    int total_data_len = 8 + copy_bytes;
    if (total_data_len <= 8) {
        header.data_len_code = total_data_len;
    } else if (total_data_len <= 12) {
        header.data_len_code = 9;
    } else if (total_data_len <= 16) {
        header.data_len_code = 10;
    } else if (total_data_len <= 20) {
        header.data_len_code = 11;
    } else if (total_data_len <= 24) {
        header.data_len_code = 12;
    } else if (total_data_len <= 32) {
        header.data_len_code = 13;
    } else if (total_data_len <= 48) {
        header.data_len_code = 14;
    } else {
        header.data_len_code = 15; // 64 bytes
    }
    
    panda_send_can_data(&header, can_data, total_data_len);
}

//--------------------------------------------------------------------+
// TinyUSB Vendor Class Callbacks
//--------------------------------------------------------------------+

// Invoked when received data from host via OUT endpoint
void tud_vendor_rx_cb(uint8_t itf, uint8_t const* buffer, uint16_t bufsize) {
    if (bufsize > 0) {
        printf("Received %u bytes from host via Bulk OUT\n", bufsize);
        // TODO: Parse incoming CAN frames and forward to CAN bus
        // This would involve unpacking can_header + data from the buffer
        // and sending it to the physical CAN interface
    }
} 