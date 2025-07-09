#include "pico/stdlib.h"
#include "tusb.h"
#include "usb_comms.h"
#include "panda_can.h"
#include <string.h>
#include "pico/unique_id.h"

// --- CAN FD mapping ---
static const uint8_t dlc_to_len[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
static const uint8_t len_to_dlc[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15};

static inline uint8_t can_dlc_to_len(uint8_t dlc) {
    return dlc_to_len[dlc & 0xFU];
}

static inline uint8_t can_len_to_dlc(uint8_t len) {
    if (len > 64) len = 64;
    return len_to_dlc[len];
}

#define FLEXRAY_FRAME_ID 0x123

void usb_comms_send_flexray_frame(flexray_frame_t *frame) {
    // 1. Serialize the flexray_frame_t into a byte buffer
    // Max size: 5 bytes header + 254 payload + 3 trailer = 262
    uint8_t serialized_frame[300]; 
    size_t offset = 0;

    // Header (40 bits / 5 bytes)
    uint8_t header_buf[5] = {0};
    header_buf[0] = (frame->startup_frame_indicator << 7) |
                    (frame->sync_frame_indicator << 6) |
                    (frame->null_frame_indicator << 5) |
                    (frame->payload_preamble_indicator << 4) |
                    (frame->reserved_bit << 3) |
                    ((frame->frame_id >> 8) & 0x7);
    header_buf[1] = frame->frame_id & 0xFF;
    header_buf[2] = (frame->payload_length_words << 1) | ((frame->header_crc >> 10) & 0x1);
    header_buf[3] = (frame->header_crc >> 2) & 0xFF;
    header_buf[4] = ((frame->header_crc & 0x3) << 6) | (frame->cycle_count & 0x3F);
    
    memcpy(serialized_frame + offset, header_buf, sizeof(header_buf));
    offset += sizeof(header_buf);

    // Payload (up to 254 bytes)
    uint16_t payload_len_bytes = frame->payload_length_words * 2;
    memcpy(serialized_frame + offset, frame->payload, payload_len_bytes);
    offset += payload_len_bytes;
    
    // Trailer (3 bytes)
    memcpy(serialized_frame + offset, &frame->payload_crc, 3);
    offset += 3;

    // 2. Fragment and send the serialized data in CAN-FD frames
    const size_t max_payload = 64;
    size_t bytes_sent = 0;
    uint8_t seq = 0;

    while (bytes_sent < offset) {
        size_t chunk_size = (offset - bytes_sent > max_payload - 1) ? max_payload - 1 : offset - bytes_sent;
        
        uint8_t can_payload[max_payload];
        can_payload[0] = seq; // Use first byte for sequence number
        memcpy(can_payload + 1, serialized_frame + bytes_sent, chunk_size);
        
        can_header header = {0};
        header.addr = FLEXRAY_FRAME_ID;
        header.extended = 1;
        header.bus = 0;
        header.data_len_code = can_len_to_dlc(chunk_size + 1);

        // Send CAN frame via Panda USB (this is now handled in panda_usb.c)
        // send_can_frame(&header, can_payload);

        bytes_sent += chunk_size;
        seq++;
    }
}


// USB descriptors are now defined in usb_descriptors.c

// Function prototypes
static uint8_t calculate_checksum(const uint8_t *data, size_t len);

void usb_comms_init(void) {
    // This is handled by TinyUSB library, just need to be called in main
    tud_init(BOARD_TUD_RHPORT);
}

void usb_comms_task(void) {
    tud_task();
}

// ---------------------------------------------------------------------------
// TinyUSB callbacks are now implemented in usb_descriptors.c
// ---------------------------------------------------------------------------

// Note: TinyUSB vendor control callbacks are disabled because they are "poisoned"
// by TinyUSB when using the vendor class. Control requests will be handled elsewhere.

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

static uint8_t calculate_checksum(const uint8_t *data, size_t len) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

static void send_can_frame(const can_header *header, const uint8_t *data) {
    if (!tud_vendor_mounted()) {
        return;
    }

    uint8_t can_data_len = can_dlc_to_len(header->data_len_code);
    
    // The data sent to the host is a packed structure of header and data.
    uint8_t transfer_buf[sizeof(can_header) + 64]; // Max size for CAN FD

    // Calculate checksum over the header and data
    can_header final_header = *header;
    final_header.checksum = 0; // Checksum is calculated with its own field as 0
    
    uint8_t checksum = 0;
    uint8_t *header_bytes = (uint8_t*)&final_header;
    for (size_t i = 0; i < sizeof(can_header); i++) {
        checksum ^= header_bytes[i];
    }
    checksum ^= calculate_checksum(data, can_data_len);
    final_header.checksum = checksum;

    memcpy(transfer_buf, &final_header, sizeof(can_header));
    memcpy(transfer_buf + sizeof(can_header), data, can_data_len);

    tud_vendor_write(transfer_buf, sizeof(can_header) + can_data_len);
} 