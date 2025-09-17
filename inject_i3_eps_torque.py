#!/usr/bin/env python3

import sys
import struct
import time

try:
    import usb.core  # type: ignore
    import usb.util  # type: ignore
except Exception:
    print("PyUSB is required. Install with: pip install pyusb", file=sys.stderr)
    sys.exit(1)


PANDA_VID = 0x3801
PANDA_PID = 0xDDCC

# TinyUSB vendor endpoints (see usb_descriptors.c)
EP_VENDOR_OUT = 0x03


def find_device():
    dev = usb.core.find(idVendor=PANDA_VID, idProduct=PANDA_PID)
    if dev is None:
        return None
    try:
        if hasattr(dev, "set_configuration"):
            dev.set_configuration()  # type: ignore[attr-defined]
    except Exception:
        pass
    return dev

# i3 eps: 0xFEFED0E7FEFE00FE74FED0E7FF7F1E00
raw_bytes = bytearray.fromhex('004400FEFED0E7FEFE00FE74FED0E7FF7F1E00')
def build_override_payload(frame_id: int, base: int, data_bytes: bytes) -> bytes:
    raw_bytes[3+8+4:3+8+4+2] = data_bytes
    if not (0 <= frame_id <= 0xFFFF):
        raise ValueError("frame_id out of range")
    if not (0 <= base <= 0xFF):
        raise ValueError("base out of range")
    if len(data_bytes) > 0xFFFF:
        raise ValueError("data too long")
    # op 0x90: [0x90][u16 id][u8 base][u16 len][len bytes]
    header = struct.pack('<BHBH', 0x90, frame_id, base, len(raw_bytes))
    return header + raw_bytes

torque_to_bytes = lambda torque: struct.pack('<H', int((torque + 8.130126121) * 4096))

def main() -> int:
    dev = find_device()
    if dev is None:
        print("Device not found. Is the Pico connected and running the app?", file=sys.stderr)
        return 1

    # 3s full cycle triangle wave: -1 -> +1 -> -1
    period_s = 3.0
    half_period_s = period_s / 2.0
    start_time = time.time()
    # Write to vendor OUT endpoint in a loop
    try:
        while True:
            time.sleep(0.02)
            t = (time.time() - start_time) % period_s
            if t < half_period_s:
                torque = -1.0 + 2.0 * (t / half_period_s)
            else:
                torque = 1.0 - 2.0 * ((t - half_period_s) / half_period_s)
            buf = build_override_payload(0x44, 0, torque_to_bytes(torque))
            print(f"Hex: {buf.hex()} Torque: {torque}")
            written = dev.write(EP_VENDOR_OUT, buf, timeout=1000)
            print(f"torque={torque:.3f}, written={written}")
            # if written == len(buf):
            #     break
        if written != len(buf):
            print(f"Partial write: {written}/{len(buf)} bytes", file=sys.stderr)
        else:
            print("Overrides sent: 0xAABB then 0xCCDD for frame 108 (base=0)")
    except usb.core.USBError as e:
        print(f"USB write failed: {e}", file=sys.stderr)
        return 2

    # Small delay to allow processing
    time.sleep(0.05)
    return 0


if __name__ == "__main__":
    sys.exit(main())


