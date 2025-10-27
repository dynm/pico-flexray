#include "panda_usb.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "pico/bootrom.h"
#include "hardware/watchdog.h"
#include "tusb.h"
#include "flexray_frame.h"
#include "flexray_fifo.h"
#include "flexray_forwarder_with_injector.h"
#include <string.h>

// Add near top after includes
static absolute_time_t last_usb_activity = 0;

// FlexRay FIFO
static flexray_fifo_t flexray_fifo;

// For delayed reset/bootloader
static bool pending_reset = false;
static bool pending_bootloader = false;

// Global state
static struct
{
    uint8_t hw_type;
    uint8_t safety_model;
    bool initialized;
    uint16_t alternative_experience;
} panda_state;

// Internal functions
static bool handle_control_read(uint8_t rhport, tusb_control_request_t const *request);
static bool handle_control_write(uint8_t rhport, tusb_control_request_t const *request);
static bool handle_control_data_stage(tusb_control_request_t const *request, uint8_t const *data, uint16_t len);
static bool try_send_from_fifo(const char *context);
// ------------------------------------------------------------
// Vendor OUT protocol (host -> device)
//  op 0x90: Push override replacement slice
//    [0x90][u16 id][u8 base][u16 len][len bytes slice]
//    - id must match a trigger_rule_t.target_id; base must match rule.cycle_base
//    - len must equal rule.replace_len
//  op 0x91: Set injector enable
//    [0x91][u8 enabled]
// ------------------------------------------------------------
static void handle_vendor_out_payload(const uint8_t *data, uint16_t len)
{
    uint32_t off = 0;
    while ((uint16_t)(len - off) >= 1) {
        uint8_t op = data[off++];
        if (op == 0x90) {
            if ((uint16_t)(len - off) < 5) {
                break;
            }
            uint16_t id = (uint16_t)(data[off] | ((uint16_t)data[off + 1] << 8));
            uint8_t base = data[off + 2];
            uint16_t flen = (uint16_t)(data[off + 3] | ((uint16_t)data[off + 4] << 8));
            off += 5;
            if ((uint16_t)(len - off) < flen) {
                break;
            }
            (void)injector_submit_override(id, base, flen, &data[off]);
            off += flen;
        } else if (op == 0x91) {
            if ((uint16_t)(len - off) < 1) {
                break;
            }
            bool en = data[off++] != 0;
            injector_set_enabled(en);
        } else if (op == 0x00) {
            continue;
        } else {
            // Unknown op: stop parsing this buffer
            break;
        }
    }
}


// Placeholder for git version
const char *GITLESS_REVISION = "dev";

void panda_usb_init(void)
{
    // Initialize TinyUSB
    tud_init(0);

    // Initialize FlexRay FIFO
    flexray_fifo_init(&flexray_fifo);

    // Initialize panda state
    panda_state.hw_type = HW_TYPE_RED_PANDA;
    panda_state.safety_model = SAFETY_SILENT;
    panda_state.initialized = true;
    panda_state.alternative_experience = 0;

    printf("Panda USB initialized - VID:0x%04x PID:0x%04x\n", 0x3801, 0xddcc);
    last_usb_activity = get_absolute_time();
}

void panda_usb_task(void)
{
    tud_task();
}

// TinyUSB vendor control transfer callback - this overrides the weak default implementation
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    static tusb_control_request_t current_request_local;
    static uint8_t control_buffer[CFG_TUD_ENDPOINT0_SIZE];

    switch (stage)
    {
    case CONTROL_STAGE_SETUP:
        // Store the request. We need it in the DATA stage to dispatch correctly.
        current_request_local = *request;
        pending_reset = false;
        pending_bootloader = false;

        if (request->bmRequestType & TUSB_DIR_IN_MASK)
        {
            // IN request
            return handle_control_read(rhport, request);
        }
        else
        {
            // OUT request
            if (request->wLength > 0)
            {
                // OUT with data: provide buffer for TinyUSB to receive data
                return tud_control_xfer(rhport, request, control_buffer, request->wLength);
            }
            else
            {
                // OUT with no data
                return handle_control_write(rhport, request);
            }
        }

    case CONTROL_STAGE_DATA:
        // Handle OUT data stage
        if (!(current_request_local.bmRequestType & TUSB_DIR_IN_MASK) && current_request_local.wLength > 0)
        {
            handle_control_data_stage(&current_request_local, control_buffer, current_request_local.wLength);
            // tud_control_status(rhport, request);
        }
        return true;

    case CONTROL_STAGE_ACK:
        // Execute delayed operations after status stage
        if (pending_reset)
        {
            watchdog_reboot(0, 0, 0);
        }
        else if (pending_bootloader)
        {
            reset_usb_boot(0, 0);
        }
        return true;

    default:
        // Let TinyUSB handle other stages like DATA
        return true;
    }
}

static bool handle_control_read(uint8_t rhport, tusb_control_request_t const *request)
{
    uint8_t response_data[64] = {0};
    uint16_t response_len = 0;

    switch (request->bRequest)
    {
    case PANDA_GET_HW_TYPE:
        // Return hardware type (Red Panda = 4 for CAN-FD support)
        response_data[0] = panda_state.hw_type;
        response_len = 1;
        // printf("Control Read: GET_HW_TYPE -> %d\n", response_data[0]);
        break;

    case PANDA_GET_MICROSECOND_TIMER:
        // Return microsecond timer value
        {
            uint32_t timer_val = time_us_32();
            memcpy(response_data, &timer_val, sizeof(timer_val));
            response_len = sizeof(timer_val);
            // printf("Control Read: GET_MICROSECOND_TIMER -> %lu\n", timer_val);
        }
        break;

    case PANDA_GET_FAN_RPM:
        // TODO: Implement fan RPM reading
        {
            uint16_t fan_rpm = 0; // Placeholder
            memcpy(response_data, &fan_rpm, sizeof(fan_rpm));
            response_len = sizeof(fan_rpm);
            // printf("Control Read: GET_FAN_RPM -> %d\n", fan_rpm);
        }
        break;

    case PANDA_GET_CAN_HEALTH_STATS:
        struct can_health_t * can_health = (struct can_health_t*)response_data;
        can_health->can_speed = 0;
        can_health->can_data_speed = 0;
        can_health->canfd_enabled = 0;
        can_health->brs_enabled = 0;
        can_health->canfd_non_iso = 0;
        response_len = sizeof(struct can_health_t);
        memcpy(response_data, can_health, response_len);
        // printf("Control Read: GET_CAN_HEALTH_STATS\n");
        break;

    // case PANDA_GET_MCU_UID:
    // {
    //     pico_unique_board_id_t uid;
    //     pico_get_unique_board_id(&uid);
    //     // On Pico, UID is 8 bytes. Panda expects 12. Pad with 0s.
    //     memset(response_data, 0, 12);
    //     memcpy(response_data, uid.id, PICO_UNIQUE_BOARD_ID_SIZE_BYTES);
    //     response_len = 12;
    //     // printf("Control Read: GET_MCU_UID\n");
    // }
    // break;

    case PANDA_GET_HEALTH_PACKET:
        struct health_t * health = (struct health_t*)response_data;
        health->uptime_pkt = 0;
        health->voltage_pkt = 0;
        health->current_pkt = 0;
        health->safety_tx_blocked_pkt = 0;
        health->safety_rx_invalid_pkt = 0;
        health->tx_buffer_overflow_pkt = 0;
        health->rx_buffer_overflow_pkt = 0;
        health->faults_pkt = 0;
        health->ignition_line_pkt = 1;
        health->ignition_can_pkt = 1;
        health->controls_allowed_pkt = 1;
        health->car_harness_status_pkt = 1;
        health->safety_mode_pkt = 17;
        health->safety_param_pkt = 0;
        health->fault_status_pkt = 0;
        health->power_save_enabled_pkt = 0;
        health->heartbeat_lost_pkt = 0;
        health->alternative_experience_pkt = panda_state.alternative_experience;
        health->interrupt_load_pkt = 0;
        health->fan_power = 0;
        health->safety_rx_checks_invalid_pkt = 0;
        health->spi_error_count_pkt = 0;
        health->sbu1_voltage_mV = 0;
        health->sbu2_voltage_mV = 0;
        health->som_reset_triggered = 0;
        response_len = sizeof(struct health_t);
        memcpy(response_data, health, response_len);
        // printf("Control Read: GET_HEALTH_PACKET\n");
        break;

    case PANDA_GET_SIGNATURE_PART1:
        response_len = 64;
        memset(response_data, 0, response_len);
        // printf("Control Read: PANDA_GET_SIGNATURE_PART1\n");
        break;

    case PANDA_GET_SIGNATURE_PART2:
        response_len = 64;
        memset(response_data, 0, response_len);
        // printf("Control Read: PANDA_GET_SIGNATURE_PART2\n");
        break;

    case PANDA_GET_GIT_VERSION:
        response_len = strlen(GITLESS_REVISION);
        memcpy(response_data, GITLESS_REVISION, response_len);
        // printf("Control Read: PANDA_GET_GIT_VERSION\n");
        break;


    case PANDA_GET_VERSIONS:
        response_data[0] = 17;
        response_data[1] = 4;
        response_data[2] = 5;
        response_len = 3;
        // printf("Control Read: PANDA_GET_VERSIONS\n");
        break;

    case PANDA_UART_READ:
        response_len = 0;
        memset(response_data, 'c', response_len);
        // printf("Control Read: PANDA_UART_READ\n");
        break;

    default:
        printf("Control Read: Unknown request 0x%02x\n", request->bRequest);
        return false;
    }

    // if (response_len >= 0)
    // {
        return tud_control_xfer(rhport, request, response_data, response_len);
    // }

    // return false;
}

static bool handle_control_write(uint8_t rhport, tusb_control_request_t const *request)
{
    bool handled = false;

    // Handle write requests without data - just process them
    switch (request->bRequest)
    {
    case PANDA_RESET_CAN_COMMS:
        // printf("Control Write: RESET_CAN_COMMS (request=0x%02x)\n", request->bRequest);
        flexray_fifo_init(&flexray_fifo);
        handled = true;
        break;

    case PANDA_SET_CAN_FD_AUTO_SWITCH:
        // printf("Control Write: SET_CAN_FD_AUTO_SWITCH -> %d\n", request->wValue);
        handled = true;
        break;

    case PANDA_SET_OBD_CAN_MUX_MODE:
        handled = true;
        break;

    case PANDA_SET_SAFETY_MODEL:
        panda_state.safety_model = request->wValue;
        // printf("Control Write: SET_SAFETY_MODEL -> %d, param %d\n", request->wValue, request->wIndex);
        handled = true;
        break;

    case PANDA_SET_ALT_EXPERIENCE:
        // Host writes two 16-bit values via wValue/wIndex. Save the first as alternative_experience.
        panda_state.alternative_experience = (uint16_t)request->wValue;
        handled = true;
        break;

    case PANDA_SET_CAN_SPEED_KBPS:
    case PANDA_SET_CAN_FD_DATA_BITRATE:
        // printf("Control Write: SET_CAN_SPEED_KBPS -> %d\n", request->wValue);
        handled = true;
        break;

    case PANDA_HEARTBEAT:
        // printf("Control Write: HEARTBEAT\n");
        handled = true;
        break;

    case PANDA_SET_IR_POWER:
        // printf("Control Write: SET_IR_POWER -> %d\n", request->wValue);
        handled = true;
        break;

    case PANDA_SET_FAN_POWER:
        // printf("Control Write: SET_FAN_POWER -> %d\n", request->wValue);
        handled = true;
        break;

    case PANDA_ENTER_BOOTLOADER_MODE:
        // printf("Control Write: ENTER_BOOTLOADER_MODE, mode=%d\n", request->wValue);
        if (request->wValue == 0)
        { // Bootloader
            pending_bootloader = true;
        }
        handled = true;
        break;

    case PANDA_SYSTEM_RESET:
        // printf("Control Write: SYSTEM_RESET\n");
        pending_reset = true;
        handled = true;
        break;

    case PANDA_SET_POWER_SAVE_STATE:
        // printf("Control Write: SET_POWER_SAVE_STATE -> %d\n", request->wValue);
        handled = true;
        break;

    case PANDA_DISABLE_HEARTBEAT_CHECKS:
        // printf("Control Write: DISABLE_HEARTBEAT_CHECKS\n");
        handled = true;
        break;

    default:
        printf("Control Write: Unknown request 0x%02x\n", request->bRequest);
        return false;
    }

    if (handled)
    {
        return tud_control_status(rhport, request);
    }

    return false;
}

static bool handle_control_data_stage(tusb_control_request_t const *request, uint8_t const *data, uint16_t len)
{
    // Process the received data for different commands
    switch (request->bRequest)
    {
    case PANDA_SET_CAN_FD_AUTO_SWITCH:
        printf("Control Data: SET_CAN_FD_AUTO_SWITCH -> %d\n", request->wValue);
        return true;

    case PANDA_SET_CAN_SPEED_KBPS:
        if (len >= 4) // Expect at least bus_id (2 bytes) + speed (2 bytes)
        {
            uint16_t bus_id = data[0] | (data[1] << 8);
            uint16_t speed_kbps = data[2] | (data[3] << 8);

            if (bus_id < 3) // We support up to 3 CAN buses
            {
                printf("Control Data: SET_CAN_SPEED_KBPS bus=%d speed=%d kbps\n", bus_id, speed_kbps);
            }
            else
            {
                printf("Control Data: SET_CAN_SPEED_KBPS invalid bus_id=%d\n", bus_id);
            }
        }
        else
        {
            printf("Control Data: SET_CAN_SPEED_KBPS insufficient data (got %d bytes)\n", len);
        }
        return true;

    case PANDA_SET_CAN_FD_DATA_BITRATE: // Renamed from SET_DATA_SPEED_KBPS
        if (len >= 4)                   // Expect at least bus_id (2 bytes) + speed (2 bytes)
        {
            uint16_t bus_id = data[0] | (data[1] << 8);
            uint16_t data_speed_kbps = data[2] | (data[3] << 8);

            if (bus_id < 3) // We support up to 3 CAN buses
            {
                printf("Control Data: SET_CAN_FD_DATA_BITRATE bus=%d data_speed=%d kbps\n", bus_id, data_speed_kbps);
            }
            else
            {
                printf("Control Data: SET_CAN_FD_DATA_BITRATE invalid bus_id=%d\n", bus_id);
            }
        }
        else
        {
            printf("Control Data: SET_CAN_FD_DATA_BITRATE insufficient data (got %d bytes)\n", len);
        }
        return true;

    default:
        printf("Control Data: Unexpected request 0x%02x with %d bytes\n", request->bRequest, len);
        return false;
    }
}

//--------------------------------------------------------------------+
// TinyUSB Device State Callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
    printf("USB Device mounted\n");
    last_usb_activity = get_absolute_time();
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
    printf("USB Device unmounted - clearing application state\n");

    // Reset control transfer state - not strictly needed with new design but good practice
    // Reset application state but keep device configuration
    // Don't reset panda_state entirely as it may contain valid configuration

    printf("USB unmount completed - ready for reconnection\n");
}

// Invoked when usb bus is suspended
void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;
    printf("USB Device suspended\n");
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
    printf("USB Device resumed\n");
}

//--------------------------------------------------------------------+
// TinyUSB Vendor Class Callbacks
//--------------------------------------------------------------------+

// Invoked when received data from host via OUT endpoint
void tud_vendor_rx_cb(uint8_t itf, uint8_t const *buffer, uint16_t bufsize)
{
    (void)itf;
    if (bufsize > 0)
    {
        handle_vendor_out_payload(buffer, bufsize);
    }
    // Drain any additional data queued by USB core
    while (tud_vendor_available()) {
        uint8_t tmp[256];
        uint32_t n = tud_vendor_read(tmp, sizeof(tmp));
        if (n == 0) break;
        handle_vendor_out_payload(tmp, (uint16_t)n);
    }
    last_usb_activity = get_absolute_time();
}

// Invoked when a transfer on Bulk IN endpoint is complete
void tud_vendor_tx_cb(uint8_t itf, uint32_t sent_bytes)
{
    (void)itf;
    (void)sent_bytes;

    // After a transfer is complete, try to send the next batch of data
    try_send_from_fifo("tx_cb trigger");
}

bool panda_flexray_fifo_push(const flexray_frame_t *frame)
{
    try_send_from_fifo("fifo_push");
    return flexray_fifo_push(&flexray_fifo, frame);
}

// Centralized function to trigger USB transmission from FIFO
static bool try_send_from_fifo(const char *context)
{
    (void)context;
    if (!tud_vendor_mounted() || flexray_fifo_is_empty(&flexray_fifo))
    {
        return false;
    }

    // Minimum record: 2-byte length + 1-byte source + 5-byte header + 0 payload + 3-byte CRC = 11 bytes
    const uint32_t MIN_RECORD_SIZE = 11u;

    uint32_t available_space = tud_vendor_write_available();
    if (available_space < MIN_RECORD_SIZE)
    {
        return false;
    }

    uint32_t total_sent = 0;
    bool sent_something = false;

    while (!flexray_fifo_is_empty(&flexray_fifo))
    {
        flexray_frame_t frame;
        // Peek first to preserve order in case we cannot send now
        if (!flexray_fifo_peek(&flexray_fifo, &frame))
        {
            break;
        }

        uint16_t payload_len_bytes = (uint16_t)(frame.payload_length_words * 2u);
        uint16_t body_len = (uint16_t)(1u /*source*/ + 5u /*header*/ + payload_len_bytes + 3u /*crc*/);
        uint16_t total_len = (uint16_t)(2u /*len field*/ + body_len);

        if (available_space < total_len)
        {
            // Not enough space for the head frame; stop and retry later
            break;
        }

        // Build record into a small stack buffer and write once
        uint8_t outbuf[2 + 1 + 5 + MAX_FRAME_PAYLOAD_BYTES + 3];
        outbuf[0] = (uint8_t)(body_len & 0xFF);
        outbuf[1] = (uint8_t)((body_len >> 8) & 0xFF);
        size_t w = 2;
        outbuf[w++] = frame.source;

        // Reconstruct 5-byte header
        uint8_t byte0 = (uint8_t)((frame.indicators << 3) | ((frame.frame_id >> 8) & 0x07));
        uint8_t byte1 = (uint8_t)(frame.frame_id & 0xFF);
        uint8_t byte2 = (uint8_t)((frame.payload_length_words << 1) | ((frame.header_crc >> 10) & 0x01));
        uint8_t byte3 = (uint8_t)((frame.header_crc >> 2) & 0xFF);
        uint8_t byte4 = (uint8_t)(((frame.header_crc & 0x03) << 6) | (frame.cycle_count & 0x3F));
        outbuf[w++] = byte0;
        outbuf[w++] = byte1;
        outbuf[w++] = byte2;
        outbuf[w++] = byte3;
        outbuf[w++] = byte4;

        // Payload (only used portion)
        if (payload_len_bytes > 0)
        {
            memcpy(&outbuf[w], frame.payload, payload_len_bytes);
            w += payload_len_bytes;
        }

        // 24-bit CRC big-endian
        outbuf[w++] = (uint8_t)((frame.frame_crc >> 16) & 0xFF);
        outbuf[w++] = (uint8_t)((frame.frame_crc >> 8) & 0xFF);
        outbuf[w++] = (uint8_t)(frame.frame_crc & 0xFF);

        // Write
        uint32_t written = tud_vendor_write(outbuf, (uint32_t)w);
        if (written != w)
        {
            // On partial write, stop loop; data will be retried next call
            break;
        }
        // Now we can safely pop the frame since it has been fully queued to USB
        (void)flexray_fifo_pop(&flexray_fifo, &frame);
        sent_something = true;
        total_sent += written;
        available_space = tud_vendor_write_available();

        // If buffer space drops low, flush early to free FIFO in USB core
        if (available_space < MIN_RECORD_SIZE)
        {
            break;
        }
    }

    if (sent_something)
    {
        tud_vendor_write_flush();
        return true;
    }
    return false;
}