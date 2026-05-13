#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "tusb.h"
#include "pico/bootrom.h"
#include "flexray_signal_gen.h"

#define BGE_PIN        2
#define STBN_PIN       3
#define LED_FR12_PIN   19
#define LED_FR34_PIN   20
#define RELAY_FR_1_2   17
#define RELAY_FR_3_4   18

#define TXD_FR1  28
#define TXEN_FR1 27
#define TXD_FR2  4
#define TXEN_FR2 5
#define TXD_FR3  10
#define TXEN_FR3 9
#define TXD_FR4  16
#define TXEN_FR4 22

// PWM configuration for LEDs – keep brightness low to avoid glare
#define LED_PWM_WRAP            1023u
#define LED_IDLE_MAX_LEVEL      128u   // peak duty in idle breathing
#define LED_ACTIVE_LEVEL        128u   // on level when transmitting
#define LED_BREATH_PERIOD_MS    2000u
#define LED_BREATH_HALF_PERIOD_MS (LED_BREATH_PERIOD_MS / 2u)

static uint led12_slice;
static uint led12_chan;
static uint led34_slice;
static uint led34_chan;
static bool pin_test_enabled;
static absolute_time_t pin_test_next;
static bool pin_test_level;
static fr_channel_pins_t active_pins[SIGNAL_GEN_MAX_CHANNELS];
static uint active_pin_count;

static inline void led_set_levels(uint16_t level12, uint16_t level34)
{
    if (level12 > LED_PWM_WRAP) level12 = LED_PWM_WRAP;
    if (level34 > LED_PWM_WRAP) level34 = LED_PWM_WRAP;

    pwm_set_chan_level(led12_slice, led12_chan, level12);
    pwm_set_chan_level(led34_slice, led34_chan, level34);
}

static uint16_t led_calc_idle_breath_level(void)
{
    uint32_t t = to_ms_since_boot(get_absolute_time()) % LED_BREATH_PERIOD_MS;
    uint32_t phase = (t < LED_BREATH_HALF_PERIOD_MS)
                     ? t
                     : (LED_BREATH_PERIOD_MS - t);
    uint32_t level = (phase * LED_IDLE_MAX_LEVEL) / LED_BREATH_HALF_PERIOD_MS;

    if (level < 2u) level = 2u;
    return (uint16_t)level;
}

static bool is_reserved_control_pin(uint pin)
{
    return pin == BGE_PIN || pin == STBN_PIN ||
           pin == LED_FR12_PIN || pin == LED_FR34_PIN ||
           pin == RELAY_FR_1_2 || pin == RELAY_FR_3_4;
}

static void pin_test_set(bool enabled)
{
    signal_gen_stop();
    pin_test_enabled = enabled;
    pin_test_level = false;
    pin_test_next = make_timeout_time_ms(0);

    for (uint ch = 0; ch < active_pin_count; ch++) {
        gpio_init(active_pins[ch].tx_pin);
        gpio_init(active_pins[ch].txen_pin);
        gpio_set_dir(active_pins[ch].tx_pin, GPIO_OUT);
        gpio_set_dir(active_pins[ch].txen_pin, GPIO_OUT);
        gpio_put(active_pins[ch].tx_pin, 0);
        gpio_put(active_pins[ch].txen_pin, enabled ? 0 : 1);
    }
}

static void pin_test_task(void)
{
    if (!pin_test_enabled || absolute_time_diff_us(get_absolute_time(), pin_test_next) > 0)
        return;
    pin_test_next = make_timeout_time_ms(50);
    pin_test_level = !pin_test_level;
    for (uint ch = 0; ch < active_pin_count; ch++) {
        gpio_put(active_pins[ch].tx_pin, pin_test_level);
        gpio_put(active_pins[ch].txen_pin, 0);
    }
}

// USB protocol
// CMD_SET_SLOT:   [0x03][slot 0-63][ch_mask][fid:2LE][ind][plen:2LE][payload...]
// CMD_CLEAR_SLOT: [0x04][slot 0-63]
// CMD_START:      [0x05]
// CMD_STOP:       [0x06]
// CMD_UPDATE:     [0x07][slot 0-63][plen:2LE][payload...]
// CMD_PING:       [0x02]
// CMD_BOOTLOADER: [0x08] — reboot into USB bootloader
// CMD_TARGET:     [0x09] — returns [OK][target][proto][major][minor][patch]
// CMD_SET_PINS:   [0x0a][channel 0-3][tx_gpio][txen_gpio]
//                 or [0x0a][tx0][txen0]...[tx3][txen3]
// CMD_CLEAR_ALL:  [0x0b]
// CMD_PIN_TEST:   [0x0c][mode 0/1] — drive configured TX pins at ~10 Hz for probing
// CMD_PIO_TEST:   [0x0d][mode 0/1] — drive ch0 through PIO FIFO for probing
// CMD_DIAG:       [0x0e] — returns [OK][txstall:4LE][late:4LE][completed:4LE][handled:4LE][last_render_us:4LE][max_render_us:4LE]
#define CMD_PING        0x02
#define CMD_SET_SLOT    0x03
#define CMD_CLEAR_SLOT  0x04
#define CMD_START       0x05
#define CMD_STOP        0x06
#define CMD_UPDATE      0x07
#define CMD_BOOTLOADER  0x08
#define CMD_TARGET      0x09
#define CMD_SET_PINS    0x0A
#define CMD_CLEAR_ALL   0x0B
#define CMD_PIN_TEST    0x0C
#define CMD_PIO_TEST    0x0D
#define CMD_DIAG        0x0E

#define RSP_OK          0x00
#define RSP_ERR_INVALID 0x01
#define RSP_PONG        0x03

#define FIRMWARE_PROTOCOL_VERSION 2
#define FIRMWARE_VERSION_MAJOR    0
#define FIRMWARE_VERSION_MINOR    2
#define FIRMWARE_VERSION_PATCH    0

#if PICO_RP2040
#define TARGET_FAMILY_CODE 0x20
#elif PICO_RP2350
#define TARGET_FAMILY_CODE 0x50
#else
#define TARGET_FAMILY_CODE 0x00
#endif

static void setup_pins(void)
{
    gpio_init(BGE_PIN);
    gpio_set_dir(BGE_PIN, GPIO_OUT);
    gpio_put(BGE_PIN, 0);

    gpio_init(STBN_PIN);
    gpio_set_dir(STBN_PIN, GPIO_OUT);
    gpio_put(STBN_PIN, 0);

    gpio_pull_up(TXEN_FR1);
    gpio_pull_up(TXEN_FR2);
    gpio_pull_up(TXEN_FR3);
    gpio_pull_up(TXEN_FR4);

    // Configure LEDs for PWM so we can run a low-brightness breathing pattern
    gpio_set_function(LED_FR12_PIN, GPIO_FUNC_PWM);
    gpio_set_function(LED_FR34_PIN, GPIO_FUNC_PWM);

    led12_slice = pwm_gpio_to_slice_num(LED_FR12_PIN);
    led12_chan  = pwm_gpio_to_channel(LED_FR12_PIN);
    led34_slice = pwm_gpio_to_slice_num(LED_FR34_PIN);
    led34_chan  = pwm_gpio_to_channel(LED_FR34_PIN);

    pwm_set_wrap(led12_slice, LED_PWM_WRAP);
    if (led34_slice != led12_slice) {
        pwm_set_wrap(led34_slice, LED_PWM_WRAP);
    }

    pwm_set_chan_level(led12_slice, led12_chan, 0);
    pwm_set_chan_level(led34_slice, led34_chan, 0);

    pwm_set_enabled(led12_slice, true);
    if (led34_slice != led12_slice) {
        pwm_set_enabled(led34_slice, true);
    }

    // Set relays to connect FlexRay bus
    gpio_init(RELAY_FR_1_2);
    gpio_set_dir(RELAY_FR_1_2, GPIO_OUT);
    gpio_put(RELAY_FR_1_2, 1);
    sleep_ms(500);
    gpio_init(RELAY_FR_3_4);
    gpio_set_dir(RELAY_FR_3_4, GPIO_OUT);
    gpio_put(RELAY_FR_3_4, 1);

    sleep_ms(100);
    gpio_put(BGE_PIN, 1);
    gpio_put(STBN_PIN, 1);
}

static void send_response(uint8_t status)
{
    uint8_t buf[2] = { status, 0x00 };
    tud_vendor_write(buf, 2);
    tud_vendor_write_flush();
}

static void handle_usb_data(const uint8_t *data, uint16_t len)
{
    if (len < 1) return;

    switch (data[0]) {
    case CMD_PING:
        send_response(RSP_PONG);
        break;

    case CMD_SET_SLOT: {
        // [cmd][slot][ch_mask][fid:2LE][ind][plen:2LE][payload...]
        if (len < 8) { send_response(RSP_ERR_INVALID); return; }
        uint8_t slot    = data[1];
        uint8_t ch_mask = data[2];
        if (slot >= SIGNAL_GEN_MAX_SLOTS || ch_mask == 0 || ch_mask > 0x0F) {
            send_response(RSP_ERR_INVALID); return;
        }
        uint16_t fid  = (uint16_t)(data[3] | ((uint16_t)data[4] << 8));
        uint8_t  ind  = data[5];
        uint16_t plen = (uint16_t)(data[6] | ((uint16_t)data[7] << 8));
        if (len < 8u + plen) { send_response(RSP_ERR_INVALID); return; }
        const uint8_t *payload = (plen > 0) ? &data[8] : NULL;
        bool ok = signal_gen_set_slot(slot, ch_mask, fid, ind, payload, plen);
        send_response(ok ? RSP_OK : RSP_ERR_INVALID);
        break;
    }

    case CMD_CLEAR_SLOT: {
        if (len < 2) { send_response(RSP_ERR_INVALID); return; }
        uint8_t slot = data[1];
        if (slot >= SIGNAL_GEN_MAX_SLOTS) {
            send_response(RSP_ERR_INVALID); return;
        }
        signal_gen_clear_slot(slot);
        send_response(RSP_OK);
        break;
    }

    case CMD_CLEAR_ALL:
        signal_gen_clear_all_slots();
        send_response(RSP_OK);
        break;

    case CMD_START:
        if (!signal_gen_can_start()) {
            send_response(RSP_ERR_INVALID);
            break;
        }
        if (pin_test_enabled)
            pin_test_set(false);
        send_response(RSP_OK);
        tud_task();
        tud_vendor_write_flush();
        signal_gen_start();
        break;

    case CMD_STOP:
        send_response(RSP_OK);
        tud_task();
        tud_vendor_write_flush();
        pin_test_set(false);
        break;

    case CMD_UPDATE: {
        // [cmd][slot][plen:2LE][payload...]
        if (len < 4) { send_response(RSP_ERR_INVALID); return; }
        uint8_t slot   = data[1];
        uint16_t plen  = (uint16_t)(data[2] | ((uint16_t)data[3] << 8));
        if (slot >= SIGNAL_GEN_MAX_SLOTS || len < 4u + plen) {
            send_response(RSP_ERR_INVALID); return;
        }
        const uint8_t *payload = (plen > 0) ? &data[4] : NULL;
        bool ok = signal_gen_update_slot_payload(slot, payload, plen);
        send_response(ok ? RSP_OK : RSP_ERR_INVALID);
        break;
    }

    case CMD_BOOTLOADER:
        send_response(RSP_OK);
        tud_task();
        tud_vendor_write_flush();
        sleep_ms(20);
        reset_usb_boot(0, 0);
        break;

    case CMD_TARGET: {
        uint8_t buf[6] = {
            RSP_OK,
            TARGET_FAMILY_CODE,
            FIRMWARE_PROTOCOL_VERSION,
            FIRMWARE_VERSION_MAJOR,
            FIRMWARE_VERSION_MINOR,
            FIRMWARE_VERSION_PATCH,
        };
        tud_vendor_write(buf, sizeof(buf));
        tud_vendor_write_flush();
        break;
    }

    case CMD_SET_PINS: {
        bool ok = false;
        fr_channel_pins_t next_pins[SIGNAL_GEN_MAX_CHANNELS];
        uint next_pin_count = active_pin_count;
        if (len == 4) {
            if (is_reserved_control_pin(data[2]) || is_reserved_control_pin(data[3])) {
                send_response(RSP_ERR_INVALID); return;
            }
            ok = signal_gen_set_channel_pins(data[1], data[2], data[3]);
            if (ok && data[1] < active_pin_count) {
                active_pins[data[1]].tx_pin = data[2];
                active_pins[data[1]].txen_pin = data[3];
            }
        } else if (len == 1u + (SIGNAL_GEN_MAX_CHANNELS * 2u)) {
            for (uint ch = 0; ch < SIGNAL_GEN_MAX_CHANNELS; ch++) {
                next_pins[ch].tx_pin = data[1u + (ch * 2u)];
                next_pins[ch].txen_pin = data[2u + (ch * 2u)];
                if (is_reserved_control_pin(next_pins[ch].tx_pin) ||
                    is_reserved_control_pin(next_pins[ch].txen_pin)) {
                    send_response(RSP_ERR_INVALID); return;
                }
            }
            ok = signal_gen_set_channel_pin_map(next_pins, SIGNAL_GEN_MAX_CHANNELS);
            if (ok) {
                memcpy(active_pins, next_pins, sizeof(next_pins));
                next_pin_count = SIGNAL_GEN_MAX_CHANNELS;
            }
        } else {
            send_response(RSP_ERR_INVALID); return;
        }
        if (ok) active_pin_count = next_pin_count;
        send_response(ok ? RSP_OK : RSP_ERR_INVALID);
        break;
    }

    case CMD_PIN_TEST:
        if (len < 2) { send_response(RSP_ERR_INVALID); return; }
        pin_test_set(data[1] != 0);
        send_response(RSP_OK);
        break;

    case CMD_PIO_TEST:
        if (len < 2) { send_response(RSP_ERR_INVALID); return; }
        pin_test_enabled = false;
        send_response(signal_gen_pio_test(0, data[1] != 0) ? RSP_OK : RSP_ERR_INVALID);
        break;

    case CMD_DIAG: {
        signal_gen_diag_t diag;
        signal_gen_diag(&diag);
        uint8_t buf[25] = { RSP_OK };
        uint32_t values[6] = {
            diag.txstall_count,
            diag.late_buffer_count,
            diag.completed_cycles,
            diag.handled_cycles,
            diag.last_render_us,
            diag.max_render_us,
        };
        for (uint i = 0; i < 6; i++) {
            buf[1u + i * 4u] = (uint8_t)values[i];
            buf[2u + i * 4u] = (uint8_t)(values[i] >> 8);
            buf[3u + i * 4u] = (uint8_t)(values[i] >> 16);
            buf[4u + i * 4u] = (uint8_t)(values[i] >> 24);
        }
        tud_vendor_write(buf, sizeof(buf));
        tud_vendor_write_flush();
        break;
    }

    default:
        send_response(RSP_ERR_INVALID);
        break;
    }
}

void tud_vendor_rx_cb(uint8_t itf, uint8_t const *buffer, uint16_t bufsize)
{
    (void)itf;
    // Some TinyUSB versions deliver data directly in the callback buffer,
    // while others push it into the rx_ff first.  Try rx_ff first for
    // multi-packet reassembly, then fall back to the raw buffer.
    bool handled = false;
    while (tud_vendor_available()) {
        uint8_t tmp[512];
        uint32_t n = tud_vendor_read(tmp, sizeof(tmp));
        if (n == 0) break;
        handle_usb_data(tmp, (uint16_t)n);
        handled = true;
    }
    if (!handled && bufsize > 0) {
        handle_usb_data(buffer, (uint16_t)bufsize);
    }
}

void tud_vendor_tx_cb(uint8_t itf, uint32_t sent_bytes)
{
    (void)itf; (void)sent_bytes;
}

void tud_mount_cb(void)   { printf("USB mounted\n"); }
void tud_umount_cb(void)  { printf("USB unmounted\n"); }
void tud_suspend_cb(bool remote_wakeup_en) { (void)remote_wakeup_en; }
void tud_resume_cb(void)  {}

int main(void)
{
    setup_pins();
    set_sys_clock_khz(100000, true);
    stdio_init_all();

    printf("\n=== FlexRay Signal Generator (200 Hz) ===\n");
    printf("4 channels x up to %u slots, cycle auto-rolls 0-63\n", SIGNAL_GEN_MAX_SLOTS);
    printf("System clock: %lu Hz\n", (unsigned long)clock_get_hz(clk_sys));

    tud_init(0);

    fr_channel_pins_t pins[4] = {
        { .tx_pin = TXD_FR1, .txen_pin = TXEN_FR1 },
        { .tx_pin = TXD_FR2, .txen_pin = TXEN_FR2 },
        { .tx_pin = TXD_FR3, .txen_pin = TXEN_FR3 },
        { .tx_pin = TXD_FR4, .txen_pin = TXEN_FR4 },
    };
    memcpy(active_pins, pins, sizeof(pins));
    active_pin_count = 4;
    signal_gen_init(pio0, pins, 4);

    printf("Ready — connect via WebUSB\n");

    while (true) {
        tud_task();
        pin_test_task();
        bool running = signal_gen_is_running();
        uint8_t tx_mask = running ? signal_gen_tick() : 0;

        if (running) {
            uint16_t lvl12 = (tx_mask & 0x03u) ? LED_ACTIVE_LEVEL : 0u;
            uint16_t lvl34 = (tx_mask & 0x0Cu) ? LED_ACTIVE_LEVEL : 0u;
            led_set_levels(lvl12, lvl34);
        } else {
            uint16_t breath = led_calc_idle_breath_level();
            led_set_levels(breath, breath);
        }
    }
    return 0;
}
