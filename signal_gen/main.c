#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "tusb.h"
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

// USB protocol
// CMD_SET_SLOT:   [0x03][ch 1-4][slot 0-3][fid:2LE][ind][plen:2LE][payload...]
// CMD_CLEAR_SLOT: [0x04][ch 1-4][slot 0-3]
// CMD_START:      [0x05]
// CMD_STOP:       [0x06]
// CMD_UPDATE:     [0x07][ch 1-4][slot 0-3][plen:2LE][payload...]
// CMD_PING:       [0x02]
#define CMD_PING        0x02
#define CMD_SET_SLOT    0x03
#define CMD_CLEAR_SLOT  0x04
#define CMD_START       0x05
#define CMD_STOP        0x06
#define CMD_UPDATE      0x07

#define RSP_OK          0x00
#define RSP_ERR_INVALID 0x01
#define RSP_PONG        0x03

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

    gpio_init(LED_FR12_PIN);
    gpio_set_dir(LED_FR12_PIN, GPIO_OUT);
    gpio_init(LED_FR34_PIN);
    gpio_set_dir(LED_FR34_PIN, GPIO_OUT);

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
        // [cmd][ch][slot][fid:2LE][ind][plen:2LE][payload...]
        if (len < 8) { send_response(RSP_ERR_INVALID); return; }
        uint8_t ch_num = data[1];
        uint8_t slot   = data[2];
        if (ch_num < 1 || ch_num > 4 || slot > 3) {
            send_response(RSP_ERR_INVALID); return;
        }
        uint16_t fid  = (uint16_t)(data[3] | ((uint16_t)data[4] << 8));
        uint8_t  ind  = data[5];
        uint16_t plen = (uint16_t)(data[6] | ((uint16_t)data[7] << 8));
        if (len < 8u + plen) { send_response(RSP_ERR_INVALID); return; }
        const uint8_t *payload = (plen > 0) ? &data[8] : NULL;
        bool ok = signal_gen_set_slot(ch_num - 1, slot, fid, ind, payload, plen);
        send_response(ok ? RSP_OK : RSP_ERR_INVALID);
        break;
    }

    case CMD_CLEAR_SLOT: {
        if (len < 3) { send_response(RSP_ERR_INVALID); return; }
        uint8_t ch_num = data[1];
        uint8_t slot   = data[2];
        if (ch_num < 1 || ch_num > 4 || slot > 3) {
            send_response(RSP_ERR_INVALID); return;
        }
        signal_gen_clear_slot(ch_num - 1, slot);
        send_response(RSP_OK);
        break;
    }

    case CMD_START:
        signal_gen_start();
        send_response(RSP_OK);
        break;

    case CMD_STOP:
        signal_gen_stop();
        send_response(RSP_OK);
        break;

    case CMD_UPDATE: {
        // [cmd][ch][slot][plen:2LE][payload...]
        if (len < 5) { send_response(RSP_ERR_INVALID); return; }
        uint8_t ch_num = data[1];
        uint8_t slot   = data[2];
        uint16_t plen  = (uint16_t)(data[3] | ((uint16_t)data[4] << 8));
        if (ch_num < 1 || ch_num > 4 || slot > 3 || len < 5u + plen) {
            send_response(RSP_ERR_INVALID); return;
        }
        const uint8_t *payload = (plen > 0) ? &data[5] : NULL;
        bool ok = signal_gen_update_slot_payload(ch_num - 1, slot, payload, plen);
        send_response(ok ? RSP_OK : RSP_ERR_INVALID);
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
    if (bufsize > 0)
        handle_usb_data(buffer, bufsize);
    while (tud_vendor_available()) {
        uint8_t tmp[512];
        uint32_t n = tud_vendor_read(tmp, sizeof(tmp));
        if (n == 0) break;
        handle_usb_data(tmp, (uint16_t)n);
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
    printf("4 channels x 4 slots, cycle auto-rolls 0-63\n");
    printf("System clock: %lu Hz\n", (unsigned long)clock_get_hz(clk_sys));

    tud_init(0);

    fr_channel_pins_t pins[4] = {
        { .tx_pin = TXD_FR1, .txen_pin = TXEN_FR1 },
        { .tx_pin = TXD_FR2, .txen_pin = TXEN_FR2 },
        { .tx_pin = TXD_FR3, .txen_pin = TXEN_FR3 },
        { .tx_pin = TXD_FR4, .txen_pin = TXEN_FR4 },
    };
    signal_gen_init(pio0, pins, 4);

    printf("Ready — connect via WebUSB\n");

    while (true) {
        tud_task();
        uint8_t tx_mask = signal_gen_tick();

        gpio_put(LED_FR12_PIN, (tx_mask & 0x03) != 0);
        gpio_put(LED_FR34_PIN, (tx_mask & 0x0C) != 0);
    }
    return 0;
}
