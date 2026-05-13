# FlexRay Signal Generator

A standalone FlexRay signal generator built on Raspberry Pi Pico 2 (RP2350). Generates proper FlexRay wire-level signals at 10 Mbit/s via PIO, controlled from a browser over WebUSB.

## Features

- **200 Hz continuous transmission** — frames repeat every 5 ms with auto-incrementing cycle count (0–63)
- **4 FR channels with channel-mask routing** — each slot can output on any combination of channels simultaneously with perfect alignment
- **Configurable slots and pins** — the web interface can add/remove frame slots and configure channel TX/TXEN GPIOs before starting
- **Proper FlexRay framing** — TSS, FSS, BSS, 8-bit data, FES, with interleaved active-low TXEN driven only during rendered frame output
- **CRC calculation** — 11-bit header CRC and 24-bit frame CRC computed on-device
- **WebUSB** — driverless browser pairing (Chrome), no host software required
- **Live payload update** — change data on a running slot without restarting

## Pin Assignments

| GPIO | Signal | Function |
|-----:|--------|----------|
| 2 | BGE | Transceiver bus-guard enable (high = on) |
| 3 | STBN | Transceiver standby (high = active) |
| 28 | TXD_FR1 | TX data to FR1 transceiver |
| 27 | TXEN_FR1 | TX enable for FR1 (active low) |
| 4 | TXD_FR2 | TX data to FR2 transceiver |
| 5 | TXEN_FR2 | TX enable for FR2 (active low) |
| 10 | TXD_FR3 | TX data to FR3 transceiver |
| 9 | TXEN_FR3 | TX enable for FR3 (active low) |
| 16 | TXD_FR4 | TX data to FR4 transceiver |
| 22 | TXEN_FR4 | TX enable for FR4 (active low) |
| 20 | LED | Status LED |

Compatible transceivers (active-low TX_EN): TLE9222, TJA1082, NCV7383.

The TXD/TXEN pins above are defaults. The web interface can reconfigure them
while transmission is stopped. GPIOs must be unique across TX/TXEN assignments,
TX and TXEN must differ, and board-control pins `2`, `3`, `17`, `18`, `19`,
and `20` are reserved.

## Build

```bash
mkdir -p build && cd build
cmake -G Ninja -DCMAKE_BUILD_TYPE=Release ..
ninja
```

Outputs:

- `build/flexray_signal_gen_rp2040.uf2`
- `build/flexray_signal_gen_rp2350.uf2`

## Flash

Use the helper script to build both UF2 files, detect the connected target, and
flash the matching firmware:

```bash
scripts/codex_run.sh
```

The script detects a running FlexRay WebUSB device with `CMD_TARGET`, or a
BOOTSEL USB mass-storage device via `picotool`/USB descriptors, then flashes the
matching RP2040 or RP2350 UF2.

You can also hold BOOTSEL, plug USB, and copy the matching UF2 to the mounted
drive manually.

Or with picotool:

```bash
picotool load -f build/flexray_signal_gen_rp2040.uf2
picotool load -f build/flexray_signal_gen_rp2350.uf2
```

## Web Interface

Open `web/signal_gen.html` in Chrome (or serve it over HTTPS), or use https://generator.pico-flexray.xyz/ to get the web interface.

1. Click **Connect** — Chrome will show the WebUSB pairing dialog
2. Set frame ID, payload (hex), and select which channels to output on (FR1–FR4 checkboxes)
3. Click **Start 200 Hz** for continuous transmission
4. Click **Stop** to halt

## USB Protocol

VID `0xCAFE` / PID `0x4004`, vendor-class bulk endpoints.

| CMD | Name | Payload |
|-----|------|---------|
| `0x02` | Ping | — (returns `0x03`) |
| `0x03` | Set Slot | `[slot 0-63][ch_mask][fid:2LE][ind][plen:2LE][data...]` |
| `0x04` | Clear Slot | `[slot 0-63]` |
| `0x05` | Start | — |
| `0x06` | Stop | — |
| `0x07` | Update Payload | `[slot 0-63][plen:2LE][data...]` |
| `0x08` | Bootloader | — (reboots into UF2 mode) |
| `0x09` | Target | — (returns `[0x00][target][protocol][major][minor][patch]`) |
| `0x0A` | Set Channel Pins | `[channel 0-3][tx_gpio][txen_gpio]` or `[tx0][txen0]...[tx3][txen3]` |
| `0x0B` | Clear All Slots | — |

`ch_mask` is a bitmask: bit 0 = FR1, bit 1 = FR2, bit 2 = FR3, bit 3 = FR4 (e.g. `0x0F` = all channels).

Response: `[status]` where `0x00` = OK.

For `Target`, `target` is `0x20` for RP2040 or `0x50` for RP2350. The web
interface logs the firmware version and protocol on connect.

The firmware accepts up to 64 configured slots. `Start` returns an error if the
highest configured slot index and current payload-derived static slot duration
cannot fit inside the 5 ms cycle.

## Project Structure

```
signal_gen/
  main.c                   Entry point, USB command dispatch, main loop
  flexray_signal_gen.c/h   Frame construction, PIO/DMA, 200 Hz scheduler
  flexray_signal_gen.pio   PIO program (FlexRay wire-level signal generation)
  flexray_frame.c/h        FlexRay CRC-11 and CRC-24 calculation
  flexray_crc_table.h      Precomputed CRC lookup tables
  usb_descriptors.c        WebUSB + WinUSB descriptors
  tusb_config.h            TinyUSB configuration
web/
  signal_gen.html           WebUSB browser interface
```
