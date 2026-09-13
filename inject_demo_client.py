#!/usr/bin/env python3
"""Submit synthetic payload bytes to the demo injector over USB or UDP."""
import argparse
import struct
from frame_gen_client import send_action


def host_crc8(payload: bytes) -> int:
    """Host transport integrity: polynomial 0x1d, initial 0xf1, no xor-out."""
    crc = 0xf1
    for byte in payload:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1d if crc & 0x80 else 0)) & 0xff
    return crc


def payload_action(payload: bytes) -> bytes:
    if len(payload) != 18:
        raise ValueError('demo payload must contain exactly 18 bytes')
    body = bytes([host_crc8(payload)]) + payload
    return struct.pack('<BHBH', 0x90, 8, 2, len(body)) + body


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--transport', choices=('udp', 'usb'), default='udp')
    parser.add_argument('--host', default='192.168.7.1')
    parser.add_argument('--port', type=int, default=5501)
    parser.add_argument('--dry-run', action='store_true', help='print action hex without transmitting')
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('enable')
    sub.add_parser('disable')
    payload = sub.add_parser('payload')
    payload.add_argument('hex', help='18 payload bytes; firmware replaces only bytes 0..3')
    args = parser.parse_args()
    try:
        action = payload_action(bytes.fromhex(args.hex)) if args.command == 'payload' else bytes([0x91, args.command == 'enable'])
    except ValueError as exc:
        parser.error(str(exc))
    if args.dry_run:
        print(action.hex())
    else:
        send_action(action, args.transport, args.host, args.port, 1.0)
        print('Action sent; transport has no acknowledgement of injection.')


if __name__ == '__main__':
    main()
