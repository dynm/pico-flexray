#!/usr/bin/env python3
"""Control frame generation over UDP/USB, or create a deterministic GPIO bench fixture."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import socket
import struct
import sys
import time

from build_replay_payload import build_header, calculate_frame_crc24


OP_CONFIG, OP_TEMPLATE, OP_ENABLE, OP_STATUS = range(0xA0, 0xA4)
OP_DIAGNOSTICS = 0xA4
DIAGNOSTICS = struct.Struct("<16I")
DIAGNOSTIC_FIELDS = ("sequence", "before_pulse", "after_pulse", "unconsumed", "incomplete_dma",
                     "reason", "fid", "cycle", "slot", "prepared_beat", "current_beat",
                     "packet_remaining", "pulse_remaining", "inducer_pc", "local_pc", "active")
OP_PAYLOAD, OP_SWITCH = 0x94, 0x95
VERSION = 7
STATUS_MAGIC = 0x53524650
ACK = struct.Struct("<BBH")
CONFIG = struct.Struct("<4H4BIIiI2H")
STATUS = struct.Struct("<IHH16I2i2I2H4I")
COMMAND_PENDING = 1 << 5
ERRORS = {
    0: "OK", 1: "UNAVAILABLE (timing not initialized)",
    2: "BAD_LENGTH", 3: "BAD_CONFIG", 4: "BAD_FRAME", 5: "BUSY",
    6: "NOT_CONFIGURED", 7: "NOT_SYNCED",
}
FLAG_NAMES = ("configured", "template_valid", "enabled", "locked",
              "tx_pending", "command_pending", "pacing", "second_template_valid", "resyncing")
STATUS_FIELDS = (
    "magic", "version", "size", "flags", "slot_cycles", "cycle_cycles", "samples", "cycle_samples",
    "rejected", "captured", "sent", "null_sent", "missed", "lost_stamps", "pace_ticks", "pace_skipped",
    "current_slot", "current_cycle", "reference_id", "phase_error_cycles", "cycle_shape_cycles",
    "phase_updates", "phase_missed", "template_len", "second_template_len", "last_error",
    "sync_losses", "sync_loss_reason",
    "header_slot_mismatches",
)


@dataclass(frozen=True)
class Config:
    target_id: int = 0xc
    static_max_id: int = 0x10
    payload_bytes: int = 18
    learn_samples: int = 12
    rx_channel: int = 0
    cycle_mask: int = 3
    cycle_base: int = 3
    tss_bits: int = 8
    slot_cycles: int = 0
    cycle_cycles: int = 0
    phase_cycles: int = 0
    tolerance_cycles: int = 32
    second_target_id: int = 0xd
    reserved: int = 0

    def pack(self) -> bytes:
        if not 2 <= self.target_id <= self.static_max_id:
            raise ValueError("target-id must be 2..static-max-id; FID 1 is not supported")
        if self.second_target_id and (not 2 <= self.second_target_id <= self.static_max_id
                                      or self.second_target_id == self.target_id):
            raise ValueError("second-target-id must be a different ID in 2..static-max-id, or 0")
        if (self.target_id, self.second_target_id, self.static_max_id,
                self.payload_bytes, self.cycle_mask, self.cycle_base) != (0xc, 0xd, 0x10, 18, 3, 3):
            raise ValueError("frame generation rules require C/D, static-max-id=0x10, payload=18, mask=3, base=3")
        try:
            return CONFIG.pack(*asdict(self).values())
        except struct.error as error:
            raise ValueError(f"configuration field outside wire range: {error}") from error


class ProtocolError(RuntimeError):
    pass


class DeviceError(ProtocolError):
    def __init__(self, status: int):
        self.status = status
        super().__init__(ERRORS.get(status, f"unknown device error {status}"))


def parse_status(payload: bytes) -> dict:
    if len(payload) != STATUS.size:
        raise ProtocolError(f"status must contain {STATUS.size} bytes, got {len(payload)}")
    result = dict(zip(STATUS_FIELDS, STATUS.unpack(payload)))
    if (result["magic"], result["version"], result["size"]) != (STATUS_MAGIC, VERSION, STATUS.size):
        raise ProtocolError("invalid status magic/version/size")
    result["flag_names"] = [name for bit, name in enumerate(FLAG_NAMES)
                            if result["flags"] & (1 << bit)]
    result["slot_us"] = result["slot_cycles"] / 150.0
    result["cycle_us"] = result["cycle_cycles"] / 150.0
    return result


class FrameGenClient:
    def __init__(self, host: str = "192.168.7.1", port: int = 5501,
                 timeout: float = 3.0):
        if timeout <= 0:
            raise ValueError("timeout must be positive")
        self.timeout = timeout
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.connect((host, port))

    def close(self) -> None:
        self.sock.close()

    @staticmethod
    def _remaining(deadline: float) -> float:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(
                "command deadline expired; before pace starts, pending commands need an RX header to apply. "
                "Start the GPIO26/selected RX input, then inspect status before retrying."
            )
        return remaining

    def request(self, op: int, body: bytes = b"", *, deadline: float | None = None) -> bytes:
        if deadline is None:
            deadline = time.monotonic() + self.timeout
        while True:
            self.sock.settimeout(self._remaining(deadline))
            self.sock.send(bytes([op]) + body)
            try:
                response = self.sock.recv(1024)
            except socket.timeout as error:
                # No request IDs: an unacknowledged mutation may already be queued.
                raise TimeoutError("UDP response timed out; command outcome is unknown. "
                                   "Inspect status before resending; RX input is required "
                                   "before pace starts to consume pending commands.") from error
            if len(response) < ACK.size:
                raise ProtocolError("truncated acknowledgement")
            reply_op, version, status = ACK.unpack_from(response)
            if reply_op != op or version != VERSION:
                raise ProtocolError("acknowledgement operation/version mismatch")
            if status == 5:
                time.sleep(min(0.01, self._remaining(deadline)))
                continue
            if status:
                raise DeviceError(status)
            expected = ACK.size + (STATUS.size if op == OP_STATUS else DIAGNOSTICS.size if op == OP_DIAGNOSTICS else 0)
            if len(response) != expected:
                raise ProtocolError(f"reply must contain {expected} bytes, got {len(response)}")
            return response[ACK.size:]

    def diagnostics(self) -> dict:
        values = DIAGNOSTICS.unpack(self.request(OP_DIAGNOSTICS))
        if values[0] & 1:
            raise ProtocolError("incomplete diagnostics snapshot")
        return dict(zip(DIAGNOSTIC_FIELDS, values))

    def status(self, *, deadline: float | None = None) -> dict:
        return parse_status(self.request(OP_STATUS, deadline=deadline))

    def apply(self, op: int, body: bytes) -> dict:
        deadline = time.monotonic() + self.timeout
        self.request(op, body, deadline=deadline)
        while True:
            state = self.status(deadline=deadline)
            if not state["flags"] & COMMAND_PENDING:
                if state["last_error"]:
                    raise DeviceError(state["last_error"])
                return state
            time.sleep(min(0.01, self._remaining(deadline)))


def payload_action(frame_id: int, payload: bytes) -> bytes:
    if frame_id not in (0xc, 0xd) or len(payload) != 18:
        raise ValueError("frame generation requires FID 0xc/0xd and exactly 18 payload bytes")
    return struct.pack("<BHBH", OP_PAYLOAD, frame_id, 3, len(payload)) + payload


def send_action(action: bytes, transport: str, host: str, port: int, timeout: float) -> None:
    """Same action bytes on either ingress. Like MITM actions, no ACK/retry."""
    if transport == "usb":
        import usb.core
        import usb.util
        device = usb.core.find(idVendor=0x3801, idProduct=0xDDCC)
        if device is None:
            raise OSError("Pico vendor USB device not found")
        try:
            usb.util.claim_interface(device, 0)
            if device.write(0x03, action, timeout=max(1, int(timeout * 1000))) != len(action):
                raise OSError("short USB bulk write")
        finally:
            usb.util.dispose_resources(device)
    else:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(timeout)
            sock.sendto(action, (host, port))


def make_frame(frame_id: int, cycle: int, payload: bytes) -> bytes:
    # Null-frame indicator = 1: payload contains valid data; sync/startup = 0.
    header = build_header(4, frame_id, cycle, len(payload))
    data = header + payload
    return data + calculate_frame_crc24(data).to_bytes(3, "big")


def frame_bits(frame: bytes, tss_bits: int) -> list[int]:
    bits = [0] * tss_bits + [1]  # TSS followed by one-bit FSS.
    for byte in frame:
        bits.extend((1, 0))  # Every byte has its own BSS.
        bits.extend((byte >> shift) & 1 for shift in range(7, -1, -1))
    bits.extend((0, 1))  # FES: one low bit then one high bit, then idle high.
    return bits


def generate_fixture(output: Path, *, cycles: int = 64, cycle_ns: int = 5_000_000,
                     slot_ns: int = 40_000, tss_lengths: tuple[int, ...] = (6, 8),
                     payload_bytes: int = 18) -> dict:
    bit_ns = 100
    if not 1 <= cycles <= 64 or cycle_ns <= 0 or slot_ns <= 0:
        raise ValueError("cycles must be 1..64 and period/slot times must be positive")
    if not tss_lengths or any(value < 3 or value > 15 for value in tss_lengths):
        raise ValueError("fixture TSS lengths must be 3..15 bits")
    if payload_bytes < 0 or payload_bytes > 254 or payload_bytes % 2:
        raise ValueError("payload length must be even and in 0..254 bytes")
    if 8 * slot_ns >= cycle_ns:
        raise ValueError("the target FSS must fit inside the cycle")
    frame_duration = (max(tss_lengths) + 1 + 10 * (payload_bytes + 8) + 2) * bit_ns
    if frame_duration >= slot_ns:
        raise ValueError("complete frame plus TSS must fit inside one static slot")
    output.mkdir(parents=True, exist_ok=True)
    payload = bytes(index & 0xFF for index in range(payload_bytes))
    template_path = output / "template_0x8.bin"
    template_path.write_bytes(make_frame(8, 2, payload))
    edges = [(0, 1)]
    frames = []
    for cycle in range(cycles):
        for index, frame_id in enumerate((4, 6)):
            tss = tss_lengths[(cycle + index) % len(tss_lengths)]
            fss_ns = cycle * cycle_ns + frame_id * slot_ns
            start_ns = fss_ns - tss * bit_ns
            frame = make_frame(frame_id, cycle, payload)
            bits = frame_bits(frame, tss)
            for bit, level in enumerate(bits):
                if level != edges[-1][1]:
                    edges.append((start_ns + bit * bit_ns, level))
            frames.append({"cycle": cycle, "id": frame_id, "tss_bits": tss,
                           "tss_ns": start_ns, "fss_ns": fss_ns,
                           "end_ns": start_ns + len(bits) * bit_ns,
                           "frame_hex": frame.hex()})
    duration_ns = cycles * cycle_ns
    # A repeated high endpoint encodes total playback duration, including idle.
    edges.append((duration_ns, 1))
    csv_path = output / "rx_edges.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("time_ns", "level"))
        writer.writerows(edges)
    vcd_path = output / "rx.vcd"
    with vcd_path.open("w", encoding="utf-8") as stream:
        stream.write("$timescale 1 ns $end\n$scope module bench $end\n"
                     "$var wire 1 ! RXD_FR1 $end\n$upscope $end\n"
                     "$enddefinitions $end\n")
        for timestamp, level in edges:
            stream.write(f"#{timestamp}\n{level}!\n")
    manifest = {
        "bit_ns": bit_ns, "cycle_ns": cycle_ns, "slot_ns": slot_ns,
        "cycles": cycles, "duration_ns": duration_ns, "payload_bytes": payload_bytes,
        "tss_lengths": list(tss_lengths), "present_ids": [4, 6], "absent_id": 8,
        "expected_anchor_to_target_ns": 2 * slot_ns,
        "selected_cycles": [cycle for cycle in range(cycles) if cycle & 3 == 2],
        "csv_format": "edge time_ns,level; hold each level until the next row",
        "template": str(template_path), "csv": str(csv_path), "vcd": str(vcd_path),
        "frames": frames,
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n",
                                          encoding="utf-8")
    return manifest


def integer(value: str) -> int:
    return int(value, 0)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.7.1")
    parser.add_argument("--port", type=int, default=5501)
    parser.add_argument("--timeout", type=float, default=3.0,
                        help="total command/apply deadline in seconds (default: 3)")
    commands = parser.add_subparsers(dest="command", required=True)
    configure = commands.add_parser("configure", help="queue configuration and wait for RX to apply")
    for name, default in asdict(Config()).items():
        if not name.startswith("reserved"):
            configure.add_argument("--" + name.replace("_", "-"), type=integer, default=default)
    template = commands.add_parser("template", help="submit one frame for the next eligible slot (header + payload + CRC)")
    source = template.add_mutually_exclusive_group(required=True)
    source.add_argument("--hex")
    source.add_argument("--file", type=Path)
    payload_parser = commands.add_parser("payload", help="send one 18-byte payload with action 0x94 (no ACK)")
    payload_parser.add_argument("--target-id", type=integer, required=True)
    payload_source = payload_parser.add_mutually_exclusive_group(required=True)
    payload_source.add_argument("--hex")
    payload_source.add_argument("--file", type=Path)
    payload_parser.add_argument("--transport", choices=("udp", "usb"), default="udp")
    for name in ("gen-enable", "gen-disable"):
        switch = commands.add_parser(name, help="send frame generation switch action 0x95 (no ACK)")
        switch.add_argument("--transport", choices=("udp", "usb"), default="udp")
    for name in ("enable", "disable", "status", "diagnostics"):
        commands.add_parser(name)
    clear = commands.add_parser("clear-template", help="use null for both targets or a selected FID")
    clear.add_argument("--target-id", type=integer)
    bench = commands.add_parser("fixture", help="generate a GPIO RX waveform and slot-8 template")
    bench.add_argument("--output", type=Path, default=Path("bench-frame-gen"))
    bench.add_argument("--cycles", type=integer, default=64)
    bench.add_argument("--cycle-ns", type=integer, default=5_000_000)
    bench.add_argument("--slot-ns", type=integer, default=40_000)
    bench.add_argument("--tss-bits", default="6,8", help="alternating received TSS lengths, e.g. 6,8")
    bench.add_argument("--payload-bytes", type=integer, default=18)
    args = parser.parse_args(argv)
    try:
        if args.command in ("payload", "gen-enable", "gen-disable"):
            if args.command == "payload":
                raw_payload = args.file.read_bytes() if args.file is not None else bytes.fromhex(args.hex)
                action = payload_action(args.target_id, raw_payload)
            else:
                action = bytes([OP_SWITCH, args.command == "gen-enable"])
            send_action(action, args.transport, args.host, args.port, args.timeout)
            print(json.dumps({"transport": args.transport, "action": action.hex(), "acknowledged": False}))
            return 0
        if args.command == "fixture":
            result = generate_fixture(args.output, cycles=args.cycles, cycle_ns=args.cycle_ns,
                                      slot_ns=args.slot_ns,
                                      tss_lengths=tuple(integer(v) for v in args.tss_bits.split(",")),
                                      payload_bytes=args.payload_bytes)
            print(json.dumps({key: value for key, value in result.items() if key != "frames"}, indent=2))
            return 0
        if args.command == "configure":
            body = Config(**{name: getattr(args, name) for name in asdict(Config())
                             if not name.startswith("reserved")}).pack()
            op = OP_CONFIG
        elif args.command == "template":
            body = args.file.read_bytes() if args.file is not None else bytes.fromhex(args.hex)
            if not 8 <= len(body) <= 262:
                raise ValueError("template must contain 8..262 bytes: header + payload + frame CRC")
            op = OP_TEMPLATE
        elif args.command == "clear-template":
            if args.target_id is not None and not 2 <= args.target_id <= 2047:
                raise ValueError("target-id must be 2..2047")
            body = b"" if args.target_id is None else struct.pack("<H", args.target_id)
            op = OP_TEMPLATE
        else:
            body = bytes([args.command == "enable"])
            op = OP_ENABLE
        client = FrameGenClient(args.host, args.port, args.timeout)
        try:
            result = (client.status() if args.command == "status" else
                      client.diagnostics() if args.command == "diagnostics" else client.apply(op, body))
        finally:
            client.close()
        print(json.dumps(result, indent=2))
        return 0
    except (OSError, ValueError, ProtocolError, ImportError) as error:
        print(f"frame generation: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
