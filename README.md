[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/dynm/pico-flexray)

### pico-flexray — Low Cost FlexRay MITM Module
<img src="./imgs/pico-flexray.png" alt="pico-flexray module" width="600"/>
<img src="./imgs/pico-flexray-bmw-g30.webp" alt="pico-flexray BMW G30 example" width="600"/>
<img src="./imgs/openpilot-lateral-bmw-g30.webp" alt="OpenPilot BMW G30 example" width="600"/>

A Raspberry Pi Pico-based FlexRay man-in-the-middle (MITM) bridge that forwards frames between ECU and vehicle transceivers, with a Panda-compatible USB interface.

- Core features:
  - Continuous, bidirectional FlexRay frame forwarding (vehicle ↔ ECU)
  - USB interface is Panda-compatible
  - Man-in-the-middle bridge mode for FlexRay capture, forwarding, and injection workflows

### Hardware connections
1. For read-only FlexRay frame capture, connect a single transceiver to the vehicle’s bus, attach its BP/BM lines to the FlexRay lines in your vehicle.
2. To differentiate frames from the ECU and the vehicle, use a man-in-the-middle (MITM) setup: split the original FlexRay cable and connect each half to its own transceiver with separate BP/BM pairs—one transceiver for the ECU side, one for the vehicle side.

FR1/FR2 are the primary MITM pair. FR3/FR4 are an optional second pair; leaving FR3/FR4 unconnected does not affect normal FR1/FR2 operation.

Refer to your board’s pinout for physical pad/header locations. Signals below use Pico GPIO numbers as configured in `src/main.c`.

| GPIO | Signal | Direction | Side | Notes |
|---:|---|---|---|---|
| 2 | `BGE` | Output | Both | BGE to FlexRay transceivers (set High to enable)
| 3 | `STBN` | Output | Both | STBN to transceivers (set High to exit standby)
| 28 | `TXD_FR_1` | Output | FR1 | TXD to FR1 transceiver
| 27 | `TXEN_FR_1` | Output | FR1 | TX_EN for FR1 transceiver
| 26 | `RXD_FR_1` | Input | FR1 | RXD from FR1 transceiver
| 4 | `TXD_FR_2` | Output | FR2 | TXD to FR2 transceiver
| 5 | `TXEN_FR_2` | Output | FR2 | TX_EN for FR2 transceiver
| 6 | `RXD_FR_2` | Input | FR2 | RXD from FR2 transceiver
| 10 | `TXD_FR_3` | Output | FR3 | TXD to FR3 transceiver
| 9 | `TXEN_FR_3` | Output | FR3 | TX_EN for FR3 transceiver
| 8 | `RXD_FR_3` | Input | FR3 | RXD from FR3 transceiver
| 16 | `TXD_FR_4` | Output | FR4 | TXD to FR4 transceiver
| 22 | `TXEN_FR_4` | Output | FR4 | TX_EN for FR4 transceiver
| 21 | `RXD_FR_4` | Input | FR4 | RXD from FR4 transceiver
| 17 | `RELAY_FR_1_2` | Output | FR1/FR2 | Relay control for the FR1/FR2 pair
| 18 | `RELAY_FR_3_4` | Output | FR3/FR4 | Relay control for the FR3/FR4 pair
| 20 | `LED` | Output | Status | On-board/app status LED
| 7 | `ISR` | Output | Measurement | Use a logic analyzer to measure the frame preparation time consumption.

![Wiring diagram](imgs/wiring.png)
**Note:**  
You can use any FlexRay transceiver you have available. The following transceivers are pin-to-pin compatible and can be used interchangeably:
- TLE9222
- TJA1082
- NCV7383


### Build and flash

```bash
git clone https://github.com/dynm/pico-flexray/
cd pico-flexray
```

Option 1: Visual Studio Code
1. Install the [Raspberry Pi Pico extension](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico)
2. Open this repo, do not enable RISC-V instructions
3. Click the Pico extension tab on the left panel
4. Click "Switch Board" and select your Pico board
5. Hold BOOT, plug USB, then you can release BOOT
6. Click "Run Project (USB)"
7. Done!

Option 2: Command line
Prerequisites:
- Raspberry Pi Pico SDK 2.1.x (env var `PICO_SDK_PATH` or the VS Code Pico extension auto-setup)
- `picotool` for flashing, or UF2 drag-and-drop

Configure and build (default board is set in `CMakeLists.txt` to `pico2`):

```bash
cd pico-flexray
mkdir build && cd build
ninja -C build
```

Artifacts are produced in `build/` (e.g., `pico_flexray.uf2`, `pico_flexray.elf`).

Flash to device:
- UF2: Hold BOOT, plug USB, then copy `build/pico_flexray.uf2` to the RPI-RP2 mass storage device.
- Picotool: put the board in BOOTSEL or use reset-to-boot, then:

```bash
picotool load -f build/pico_flexray.uf2
```

Run-time:
- USB enumerates as a vendor-specific device (no CDC serial). Use UART for logs.
- On boot, the app prints pin assignments and status, enables transceivers, and starts forwarding.

### Adjusting pins or board

If you use a different board or wiring, update the GPIO defines at the top of `src/main.c` and/or modify set(PICO_BOARD pico2 CACHE STRING "Board type") in CMakeLists.txt. Rebuild and reflash.

### Streaming with Cabana

To visualize FlexRay data using Cabana:

1. Clone the OpenPilot repository and switch to the FlexRay-enabled branch:
   ```bash
   git clone https://github.com/dynm/openpilot
   cd openpilot
   git checkout cabana-flexray
   ```

2. Set up the environment:
   ```bash
   ./tools/op.sh setup
   ```

3. Build Cabana:
   ```bash
   source .venv/bin/activate
   scons -j$(nproc) tools/cabana/cabana
   ```

4. Launch Cabana:
   ```bash
   ./tools/cabana/cabana
   ```

### WebUSB frame blocker

Open `web/frame_blocker.html` in a WebUSB-capable browser and connect the Pico-FlexRay device. Add a FlexRay frame ID to block it on selected target forwarder directions.

The firmware uses the vendor OUT endpoint for runtime control:

| Op | Payload | Description |
|---:|---|---|
| `0x92` | `[u8 enabled][u8 count][count * ([u16 id][u8 direction_mask])]` | Replace the forwarding filter table. |
| `0x93` | none | Clear the forwarding filter table. |
| `0x95` | `[u8 direction_mask]` | Set whitelist-default directions; matching rules pass and all other frames in those directions block. |

The FR1/FR2 streamer raises an early PIO IRQ once the full 5-byte header is in the DMA ring. The IRQ handler verifies the header CRC, decodes the frame ID, and suppresses the selected target forwarder state machines until frame end. This blocks delivery of a valid PDU, but it is a mid-frame suppression rather than a clean empty slot.
