# Panda USB Implementation for FlexRay Bridge

This document describes the implementation of Panda USB protocol support for the FlexRay bridge project, enabling compatibility with Cabana and PandaStream tools.

## Overview

The FlexRay bridge now supports the Panda USB protocol, allowing it to appear as a Panda device to tools like Cabana. FlexRay frames are converted to CAN-FD format and transmitted over the Panda USB interface.

## Implementation Components

### 1. USB Descriptors (`src/usb_descriptors.c`)
- **VID/PID**: 0xbbaa/0xddcc (Panda-compatible)
- **Device Class**: Vendor-specific class
- **Endpoints**:
  - Control endpoint (0x00) - for Panda control commands
  - Bulk IN endpoint (0x81) - CAN data to host
  - Bulk OUT endpoint (0x03) - CAN data from host
- **Strings**: "comma.ai" manufacturer, "panda" product
- **Serial Number**: Generated from board unique ID

### 2. Panda USB Protocol (`src/panda_usb.c/.h`)

#### Control Commands Implemented:
- `GET_HW_TYPE (0xc1)` - Returns Red Panda (4) for CAN-FD support
- `CAN_RESET_COMMS (0xc0)` - Reset communication state
- `SET_SAFETY_MODEL (0xdc)` - Set to SILENT mode (3) 
- `SET_CAN_SPEED_KBPS (0xde)` - Configure CAN bus speeds
- `SET_DATA_SPEED_KBPS (0xf9)` - Configure CAN-FD data speeds
- `HEARTBEAT (0xf3)` - Host keepalive mechanism

#### CAN Packet Format:
```c
typedef struct __attribute__((packed)) {
    uint32_t addr;
    uint32_t busTime;
    uint8_t data_len_code;
    uint8_t bus;
    uint8_t extended:1;
    uint8_t rejected:1;
    uint8_t returned:1;
    uint8_t src:5;
    uint8_t checksum;
} can_header;
```

### 3. FlexRay to CAN Conversion (`src/usb_comms.c`)
- FlexRay frames are serialized and fragmented into CAN-FD frames
- Uses special CAN ID range (0x600+) for FlexRay identification
- Includes sequence numbers for frame reconstruction
- XOR checksum calculation for data integrity

### 4. TinyUSB Configuration (`src/tusb_config.h`)
- Vendor class enabled (`CFG_TUD_VENDOR=1`)
- Endpoint configuration matching Panda requirements
- Optimized for RP2040/RP2350 platform

## Usage with Cabana/PandaStream

1. **Flash the Firmware**:
   ```bash
   # Flash pico_flexray.uf2 to your Pico board
   ```

2. **Connect to Cabana**:
   - The device will appear as a Panda
   - Select it in Cabana's device list
   - FlexRay frames will appear as CAN-FD frames with IDs 0x600+

3. **PandaStream Compatibility**:
   - Device supports the standard Panda initialization sequence
   - Implements required control commands for stream setup
   - CAN-FD data rate configuration supported

## Protocol Flow

1. **Initialization**:
   ```
   Host → Device: GET_HW_TYPE
   Device → Host: 4 (Red Panda)
   
   Host → Device: CAN_RESET_COMMS
   Host → Device: SET_SAFETY_MODEL (SILENT)
   Host → Device: SET_CAN_SPEED_KBPS
   Host → Device: SET_DATA_SPEED_KBPS
   ```

2. **Data Streaming**:
   ```
   Device → Host: CAN packets with FlexRay data
   Host → Device: Periodic HEARTBEAT
   ```

## Hardware Requirements

- RP2040 or RP2350 based board (Raspberry Pi Pico/Pico 2)
- FlexRay transceiver connections as per main project documentation
- USB connection to host computer

## Build Instructions

```bash
mkdir build && cd build
cmake ..
ninja
```

Generated files:
- `pico_flexray.uf2` - Flash to Pico via drag-and-drop
- `pico_flexray.elf` - ELF binary for debugging

## Testing

### Basic USB Enumeration Test:
```bash
# On Linux/macOS, check if device enumerates
lsusb | grep bbaa:ddcc
```

### Panda Protocol Test:
```python
# Test with panda library
from panda import Panda
p = Panda()
print(f"Hardware type: {p.get_hw_type()}")
```

## Troubleshooting

### Common Issues:

1. **Device not recognized**:
   - Check USB cable connection
   - Verify `.uf2` file flashed correctly
   - Try different USB port

2. **Cabana can't connect**:
   - Ensure no other software is using the device
   - Check device appears in system USB device list
   - Restart Cabana

3. **No FlexRay data**:
   - Verify FlexRay bus connections
   - Check FlexRay bus is active
   - Monitor UART output for debug messages

## File Structure

```
src/
├── usb_descriptors.c     # USB device descriptors
├── panda_usb.c/.h        # Panda protocol implementation  
├── usb_comms.c/.h        # FlexRay to CAN conversion
├── panda_can.h           # CAN packet structures
├── tusb_config.h         # TinyUSB configuration
└── main.c                # Integration with FlexRay code
```

## Future Enhancements

- Support for bidirectional FlexRay communication
- Additional Panda control commands
- Performance optimizations for high-throughput scenarios
- Support for multiple FlexRay channels

## References

- [Panda Project](https://github.com/commaai/panda)
- [Cabana](https://github.com/commaai/openpilot/tree/master/tools/cabana)
- [TinyUSB Documentation](https://docs.tinyusb.org/)
- [RP2040 Datasheet](https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf) 