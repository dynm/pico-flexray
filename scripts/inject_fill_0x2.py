#!/usr/bin/env python3
import argparse
import struct
import time

import usb.core


PANDA_VID = 0x3801
PANDA_PID = 0xDDCC
EP_VENDOR_OUT = 0x03
OP_OVERRIDE = 0x90
CRC8_INIT = 0xF1


def crc8_checksum(data: bytes, init_value: int = CRC8_INIT) -> int:
    crc = init_value & 0xFF
    for byte in data:
        crc ^= byte & 0xFF
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x1D) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def find_device():
    dev = usb.core.find(idVendor=PANDA_VID, idProduct=PANDA_PID)
    if dev is None:
        raise SystemExit(f"panda USB device not found: {PANDA_VID:04x}:{PANDA_PID:04x}")
    try:
        dev.set_configuration()
    except usb.core.USBError:
        pass
    return dev


def build_override_packet(frame_id: int, base: int, replacement: bytes) -> bytes:
    # Firmware validates bytes[0] as AUTOSAR E2E CRC over bytes[1:],
    # then pushes bytes[1 + replace_offset:] as the replacement slice.
    payload = bytes([crc8_checksum(replacement)]) + replacement
    header = struct.pack("<BHBH", OP_OVERRIDE, frame_id, base, len(payload))
    return header + payload


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Send FlexRay override packets for target frame 0x2 base 0/1 fills."
    )
    parser.add_argument("--frame-id", type=lambda s: int(s, 0), default=0x2)
    parser.add_argument("--len", type=int, default=26)
    parser.add_argument("--base0-byte", type=lambda s: int(s, 0), default=0xBB)
    parser.add_argument("--base1-byte", type=lambda s: int(s, 0), default=0xCC)
    parser.add_argument("--interval-ms", type=float, default=20.0)
    parser.add_argument("--count", type=int, default=0, help="0 means run until Ctrl-C")
    parser.add_argument("--timeout-ms", type=int, default=1000)
    args = parser.parse_args()

    if not (0 <= args.frame_id <= 0xFFFF):
        raise SystemExit("frame-id out of range")
    if not (0 <= args.len <= 253):
        raise SystemExit("len out of range")
    for name, value in (("base0-byte", args.base0_byte), ("base1-byte", args.base1_byte)):
        if not (0 <= value <= 0xFF):
            raise SystemExit(f"{name} out of range")

    packets = (
        build_override_packet(args.frame_id, 0, bytes([args.base0_byte]) * args.len),
        build_override_packet(args.frame_id, 1, bytes([args.base1_byte]) * args.len),
    )
    dev = find_device()
    interval_s = args.interval_ms / 1000.0
    sent = 0

    print(
        f"sending target=0x{args.frame_id:x} base0={args.len}*0x{args.base0_byte:02x} "
        f"base1={args.len}*0x{args.base1_byte:02x} interval={args.interval_ms:g}ms"
    )
    try:
        while args.count == 0 or sent < args.count:
            for packet in packets:
                dev.write(EP_VENDOR_OUT, packet, timeout=args.timeout_ms)
                sent += 1
                if args.count and sent >= args.count:
                    break
            if sent % 100 == 0:
                print(f"sent={sent}")
            time.sleep(interval_s)
    except KeyboardInterrupt:
        pass

    print(f"done: sent={sent}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
