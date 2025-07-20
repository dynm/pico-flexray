#include "panda_usb.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "pico/bootrom.h"
#include "hardware/watchdog.h"
#include "tusb.h"
#include "flexray_frame.h"
#include "flexray_fifo.h"
#include <string.h>

// Add near top after includes
static absolute_time_t last_usb_activity = 0;
static const int USB_ACTIVITY_TIMEOUT_US = 1000000; // 1 second

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
} panda_state;

// Internal functions
static bool handle_control_read(uint8_t rhport, tusb_control_request_t const *request);
static bool handle_control_write(uint8_t rhport, tusb_control_request_t const *request);
static bool handle_control_data_stage(tusb_control_request_t const *request, uint8_t const *data, uint16_t len);
static bool try_send_from_fifo(const char *context);

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
        printf("Control Read: GET_HW_TYPE -> %d\n", response_data[0]);
        break;

    case PANDA_GET_MICROSECOND_TIMER:
        // Return microsecond timer value
        {
            uint32_t timer_val = time_us_32();
            memcpy(response_data, &timer_val, sizeof(timer_val));
            response_len = sizeof(timer_val);
            printf("Control Read: GET_MICROSECOND_TIMER -> %u\n", timer_val);
        }
        break;

    case PANDA_GET_FAN_RPM:
        // TODO: Implement fan RPM reading
        {
            uint16_t fan_rpm = 0; // Placeholder
            memcpy(response_data, &fan_rpm, sizeof(fan_rpm));
            response_len = sizeof(fan_rpm);
            printf("Control Read: GET_FAN_RPM -> %d\n", fan_rpm);
        }
        break;

    case PANDA_GET_MCU_UID:
    {
        pico_unique_board_id_t uid;
        pico_get_unique_board_id(&uid);
        // On Pico, UID is 8 bytes. Panda expects 12. Pad with 0s.
        memset(response_data, 0, 12);
        memcpy(response_data, uid.id, PICO_UNIQUE_BOARD_ID_SIZE_BYTES);
        response_len = 12;
        printf("Control Read: GET_MCU_UID\n");
    }
    break;

    case PANDA_GET_GIT_VERSION:
        response_len = strlen(GITLESS_REVISION);
        memcpy(response_data, GITLESS_REVISION, response_len);
        printf("Control Read: PANDA_GET_GIT_VERSION\n");
        break;

    default:
        printf("Control Read: Unknown request 0x%02x\n", request->bRequest);
        return false;
    }

    if (response_len > 0)
    {
        return tud_control_xfer(rhport, request, response_data, response_len);
    }

    return false;
}

static bool handle_control_write(uint8_t rhport, tusb_control_request_t const *request)
{
    bool handled = false;

    // Handle write requests without data - just process them
    switch (request->bRequest)
    {
    case PANDA_RESET_CAN_COMMS:
        printf("Control Write: RESET_CAN_COMMS (request=0x%02x)\n", request->bRequest);
        flexray_fifo_init(&flexray_fifo);
        handled = true;
        break;

    case PANDA_SET_SAFETY_MODEL:
        panda_state.safety_model = request->wValue;
        printf("Control Write: SET_SAFETY_MODEL -> %d, param %d\n", request->wValue, request->wIndex);
        handled = true;
        break;

    case PANDA_SET_CAN_SPEED_KBPS:
    case PANDA_SET_CAN_FD_DATA_BITRATE:
        printf("Control Write: SET_CAN_SPEED_KBPS -> %d\n", request->wValue);
        handled = true;
        break;

    case PANDA_HEARTBEAT:
        printf("Control Write: HEARTBEAT\n");
        handled = true;
        break;

    case PANDA_SET_IR_POWER:
        printf("Control Write: SET_IR_POWER -> %d\n", request->wValue);
        handled = true;
        break;

    case PANDA_SET_FAN_POWER:
        printf("Control Write: SET_FAN_POWER -> %d\n", request->wValue);
        handled = true;
        break;

    case PANDA_ENTER_BOOTLOADER_MODE:
        printf("Control Write: ENTER_BOOTLOADER_MODE, mode=%d\n", request->wValue);
        if (request->wValue == 0)
        { // Bootloader
            pending_bootloader = true;
        }
        handled = true;
        break;

    case PANDA_SYSTEM_RESET:
        printf("Control Write: SYSTEM_RESET\n");
        pending_reset = true;
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
    if (bufsize > 0)
    {
        printf("Received %u bytes from host via Bulk OUT\n", bufsize);
        // TODO: Parse incoming CAN frames and forward to CAN bus
        // This would involve unpacking can_header + data from the buffer
        // and sending it to the physical CAN interface
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
    if (!tud_vendor_mounted() || flexray_fifo_is_empty(&flexray_fifo))
    {
        return false;
    }

    // Check if USB TX buffer has enough space for transmission
    uint32_t available_space = tud_vendor_write_available();
    if (available_space < sizeof(flexray_frame_t))
    {                 // Need at least one packet size
        return false; // Not enough space, don't send
    }

    // Send multiple packets if space allows (up to available buffer space)
    uint32_t total_sent = 0;
    bool sent_something = false;

    while (available_space >= sizeof(flexray_frame_t) && !flexray_fifo_is_empty(&flexray_fifo))
    {
        flexray_frame_t frame;
        if (flexray_fifo_pop(&flexray_fifo, &frame))
        {
            uint32_t bytes_written = tud_vendor_write(&frame, sizeof(flexray_frame_t));
            if (bytes_written > 0)
            {
                total_sent += bytes_written;
                sent_something = true;

                // Update available space
                available_space = tud_vendor_write_available();

                // If we sent less than a full packet, this indicates end of data
                if (bytes_written < sizeof(flexray_frame_t))
                {
                    break;
                }
            }
            else
            {
                // If write fails, re-queue the frame. This is a simplistic approach.
                // A more robust implementation might handle this differently.
                flexray_fifo_push(&flexray_fifo, &frame);
                break;
            }
        }
        else
        {
            break; // No more data to send
        }
    }

    if (sent_something)
    {
        uint32_t bytes_flushed = tud_vendor_write_flush();
        // printf("%s: sent %lu bytes total, flushed %lu bytes (%lu available space remaining)\n",
        //        context, total_sent, bytes_flushed, tud_vendor_write_available());
        return true;
    }

    return false;
}

void panda_print_fifo_stats(void)
{
    fifo_stats_t stats;
    flexray_fifo_get_stats(&flexray_fifo, &stats);
    uint32_t count = flexray_fifo_count(&flexray_fifo);

    printf("FlexRay FIFO Status:\n");
    printf("  Available packets: %lu / %d\n", count, FLEXRAY_FIFO_SIZE);
    printf("  FIFO utilization: %.1f%%\n",
           (float)count / (FLEXRAY_FIFO_SIZE - 1) * 100.0f);

    // Frame statistics
    printf("Frame Statistics:\n");
    printf("  Total received: %lu\n", stats.total_frames_received);
    printf("  Transmitted: %lu\n", stats.frames_transmitted);
    printf("  Dropped: %lu\n", stats.frames_dropped);
    printf("  Pending: %lu\n", count);

    if (stats.total_frames_received > 0)
    {
        printf("  Drop rate: %.2f%%\n",
               (float)stats.frames_dropped / stats.total_frames_received * 100.0f);
        printf("  Transmission rate: %.2f%%\n",
               (float)stats.frames_transmitted / stats.total_frames_received * 100.0f);
    }
}