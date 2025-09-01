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


def build_enable_payload(enabled: bool) -> bytes:
    return bytes([0x91, 0x01 if enabled else 0x00])


def build_override_payload(frame_id: int, base: int, data_bytes: bytes) -> bytes:
    if not (0 <= frame_id <= 0xFFFF):
        raise ValueError("frame_id out of range")
    if not (0 <= base <= 0xFF):
        raise ValueError("base out of range")
    if len(data_bytes) > 0xFFFF:
        raise ValueError("data too long")
    # op 0x90: [0x90][u16 id][u8 base][u16 len][len bytes]
    header = struct.pack('<BHBH', 0x90, frame_id, base, len(data_bytes))
    return header + data_bytes

angle_to_bytes = lambda angle: struct.pack('<H', int((angle + 1000) * 25))
torque_to_bytes = lambda torque: struct.pack('<H', int((torque + 196.596)/0.006))

def main() -> int:
    dev = find_device()
    if dev is None:
        print("Device not found. Is the Pico connected and running the app?", file=sys.stderr)
        return 1

    # Build one buffer containing enable + two overrides
    buf = bytearray()

    amplitude = 30.0
    half_period_s = 3.0
    start_time = time.time()

    # Write to vendor OUT endpoint
    try:
        written = dev.write(EP_VENDOR_OUT, buf, timeout=1000)
        while True:
            time.sleep(0.02)
            t = (time.time() - start_time) % (2 * half_period_s)
            if t < half_period_s:
                angle = -amplitude + (2 * amplitude) * (t / half_period_s)
            else:
                angle = amplitude - (2 * amplitude) * ((t - half_period_s) / half_period_s)
            angle_bytes = angle_to_bytes(angle)
            # print(f"Hex: {angle_bytes.hex()} Angle: {angle}")
            # buf = build_override_payload(0x48, 0, angle_to_bytes(angle))
            buf = build_override_payload(0x48, 1, angle_bytes)
            print(f"Hex: {buf.hex()} Angle: {angle}")
            written = dev.write(EP_VENDOR_OUT, buf, timeout=1000)
            # print(f"Written: {written}")
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


