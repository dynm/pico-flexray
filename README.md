# FlexRay inject / frame build demo

Generic RP2350 FlexRay demo firmware extracted from the working implementation. It includes bridging, injection triggered by real frames, independent static frame generation, Panda USB and NCM/UDP interfaces, and an [interactive timing demo](web/flexray-demo/index.html). This branch is based on `main` and includes USB streaming fixes and USB NCM support alongside the generic injection and frame generation demo.

## Build

Requires Pico SDK 2.3.0, an RP2350 ARM toolchain, CMake, Ninja, and picotool 2.3.0 matching the SDK. The default board is `pico2_w`, running at 150 MHz.

```sh
cmake -S . -B build-on -G Ninja -DFLEXRAY_FRAME_GEN=ON
cmake --build build-on
cmake -S . -B build-off -G Ninja -DFLEXRAY_FRAME_GEN=OFF
cmake --build build-off
```

Each build directory contains `pico_flexray.uf2` and `pico_flexray.elf`. Set `PICO_SDK_PATH` and `PICO_TOOLCHAIN_PATH` for installations outside the default SDK paths. Use `picotool_DIR` to select a custom picotool installation.

| Mode | Features | DMA channels |
|---|---|---|
| ON (default) | FR1/FR2 bridge + inject + FR2 frame build | 10 |
| OFF | FR1..FR4 bridge + source identification and inject | 8 |

See [board_config.h](src/board_config.h) for pin definitions and [firmware modes](docs/firmware-modes.md) for resource allocation and wiring differences. All transmissions require assigned, non-overlapping TDMA slots.

## USB streaming and NCM support

Changes relative to `main` include:

- Vendor USB batches multiple records per transfer, checks space for a complete record before writing, and removes a queued frame only after the complete write succeeds. The receive callback clears the mirrored TinyUSB RX FIFO after processing its buffer.
- The USB device exposes both Panda Vendor and CDC-NCM interfaces. NCM connects to lwIP with DHCP on `192.168.7.0/24`, UDP streaming on port 5500, and injection/control on port 5501.
- UDP streaming uses bounded micro-batches and a pending queue to absorb temporary NCM backpressure. The network output path does not block waiting for the host. An actively consumed Vendor stream takes priority to avoid competing for Full-Speed USB bandwidth.
- A full capture FIFO evicts the oldest frame and counts the drop, retaining recent traffic. These changes reduce avoidable loss; bounded buffers do not guarantee lossless capture under sustained overload.

The transport implementation is preserved from the validated demo snapshot. This history correction changes no firmware, client, test, or PIO source files.

## Inject

The synthetic example triggers on FID6 with `cycle & 3 == 2`, using a previously observed FID8 template. It replaces the first four payload bytes and preserves the remaining 14 bytes. A real header prepares the packet; a real frame-end callback authorizes DMA. The original injector follows the target frame edges and outputs to FR1 in ON mode or FR3 in OFF mode. The FlexRay cycle and frame CRC are updated.

Each host override is consumed once and expires after 100 ms. Forwarding continues unchanged without fresh data. USB/UDP action `0x90` submits an entire 18-byte payload with one host transport CRC8 byte; `0x91` enables or disables injection. The transport CRC8 uses polynomial `0x1D`, initial value `0xF1`, and no final XOR.

```sh
python3 inject_demo_client.py --dry-run payload 000102030405060708090a0b0c0d0e0f1011
python3 inject_demo_client.py enable
python3 inject_demo_client.py payload 000102030405060708090a0b0c0d0e0f1011
python3 inject_demo_client.py disable
```

The default NCM/UDP destination is `192.168.7.1:5501`. Use `--transport usb` for vendor USB, which requires `pyusb` and libusb. Actions do not acknowledge successful injection; verify actual output using captures or statistics.

## Frame build

The implementation retains hardware FSS acquisition, fixed slot pacing, bounded phase correction, independent DMA, TXEN ownership, and echo exclusion. Defaults are FR2 FIDs `0xC/0xD`, 18-byte payloads, rep4/base3, `static_max_id=0x10`, and a 5 ms cycle. Frame generation starts disabled. Once enabled and synchronized, missing payload data selects a null frame at the same FSS deadline.

```sh
python3 frame_gen_client.py status
python3 frame_gen_client.py enable
python3 frame_gen_client.py payload --target-id 0xc --hex 000102030405060708090a0b0c0d0e0f1011
python3 frame_gen_client.py disable
```

See [frame build actions](docs/frame-gen-actions.md) for payload commands and the `0x94/0x95` formats, and the [design notes](docs/frame-gen-design.md) for implementation details. Frame build and inject have independent switches and data paths.

## Validation

```sh
python3 -m unittest discover -s tests -p 'test_*.py'
```

All 32 host tests and both ON/OFF firmware builds passed during extraction. ELF symbol checks passed for both modes; OFF excludes frame build. The demo JavaScript passed syntax checks. Browser interaction was not tested because the browser environment was unavailable.

Tests cover the actual C controller, MITM integration with real IRQ callbacks in both modes, command expiry, payload byte preservation, static scheduling, null frames and CRCs, and PIO cycle simulation. The FlexRay PIO sources, streamer, frame build, and slot scheduler retain the implementation present at extraction.

This branch has not undergone renewed electrical bench validation. Builds and host simulation do not establish hardware timing. After flashing an RP2350 bench device, use `cold_reset`, then verify USB re-enumeration and DMA operation.
