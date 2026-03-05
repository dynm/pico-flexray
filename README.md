# FlexRay Signal Generator

A standalone FlexRay signal generator built on Raspberry Pi Pico 2 (RP2350). Generates proper FlexRay wire-level signals at 10 Mbit/s via PIO, controlled from a browser over WebUSB.

## Features

- **200 Hz continuous transmission** — frames repeat every 5 ms with auto-incrementing cycle count (0–63)
- **4 FR channels with channel-mask routing** — each slot can output on any combination of channels simultaneously with perfect alignment
- **Proper FlexRay framing** — TSS, FSS, BSS, 8-bit data, FES, with TX_EN driven via PIO side-set
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

## Build

```bash
mkdir -p build && cd build
cmake -G Ninja -DCMAKE_BUILD_TYPE=Release ..
ninja
```

Output: `build/flexray_signal_gen.uf2`

## Flash

Hold BOOTSEL, plug USB, copy `flexray_signal_gen.uf2` to the RPI-RP2 drive.

Or with picotool:

```bash
picotool load -f build/flexray_signal_gen.uf2
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
| `0x03` | Set Slot | `[slot 0-3][ch_mask][fid:2LE][ind][plen:2LE][data...]` |
| `0x04` | Clear Slot | `[slot 0-3]` |
| `0x05` | Start | — |
| `0x06` | Stop | — |
| `0x07` | Update Payload | `[slot 0-3][plen:2LE][data...]` |
| `0x08` | Bootloader | — (reboots into UF2 mode) |

`ch_mask` is a bitmask: bit 0 = FR1, bit 1 = FR2, bit 2 = FR3, bit 3 = FR4 (e.g. `0x0F` = all channels).

Response: `[status]` where `0x00` = OK.

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
