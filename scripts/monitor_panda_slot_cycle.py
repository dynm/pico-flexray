#!/usr/bin/env python3
import argparse
from dataclasses import dataclass, field
import time

import usb.core
import usb.util


PANDA_VID = 0x3801
PANDA_PID = 0xDDCC
EP_IN = 0x81
READ_SIZE = 65536
MIN_BODY_LEN = 11


@dataclass
class SequenceState:
    expected: int | None = None
    step: int | None = None
    seen: int = 0
    errors: int = 0
    repeats: int = 0
    backwards: int = 0
    gaps: int = 0
    first: int | None = None
    last: int | None = None
    sources: dict[int, int] = field(default_factory=dict)


def parse_records(buffer: bytes):
    frames = []
    i = 0
    buflen = len(buffer)
    while i + 2 <= buflen:
        body_len = buffer[i] | (buffer[i + 1] << 8)
        if body_len < MIN_BODY_LEN:
            i += 1
            continue
        if i + 2 + body_len > buflen:
            break

        src = buffer[i + 2]
        header = buffer[i + 3:i + 8]
        frame_id = ((header[0] & 0x07) << 8) | header[1]
        payload_len_words = (header[2] >> 1) & 0x7F
        cycle_count = header[4] & 0x3F
        payload_bytes = payload_len_words * 2
        if 1 + 5 + payload_bytes + 3 != body_len:
            i += 1
            continue

        payload = bytes(buffer[i + 8:i + 8 + payload_bytes])
        frames.append((src, frame_id, cycle_count, payload))
        i += 2 + body_len
    return frames, i


def open_device():
    dev = usb.core.find(idVendor=PANDA_VID, idProduct=PANDA_PID)
    if dev is None:
        raise SystemExit(f"panda USB device not found: {PANDA_VID:04x}:{PANDA_PID:04x}")
    try:
        dev.set_configuration()
    except usb.core.USBError:
        pass
    return dev


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Monitor panda FlexRay frame cycle sequences."
    )
    parser.add_argument(
        "--frame-id",
        type=lambda s: int(s, 0),
        default=None,
        help="Frame id to monitor. Omit this to monitor all frame ids.",
    )
    parser.add_argument("--field", choices=("payload", "header-cycle"), default="header-cycle")
    parser.add_argument("--payload-index", type=int, default=0)
    parser.add_argument(
        "--cycle-step",
        type=lambda s: int(s, 0),
        default=None,
        help="Expected cycle increment. Omit this to infer one step per source/frame id.",
    )
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--flush-ms", type=int, default=500)
    parser.add_argument("--timeout-ms", type=int, default=1000)
    parser.add_argument("--print-ok", action="store_true")
    parser.add_argument(
        "--max-events",
        type=int,
        default=200,
        help="Maximum sequence error events to print. Use 0 to print none.",
    )
    parser.add_argument(
        "--summary-limit",
        type=int,
        default=32,
        help="Maximum per-frame summary rows to print, sorted by errors then seen.",
    )
    args = parser.parse_args()

    dev = open_device()
    flush_deadline = time.monotonic() + (args.flush_ms / 1000.0)
    while time.monotonic() < flush_deadline:
        try:
            dev.read(EP_IN, READ_SIZE, timeout=50)
        except usb.core.USBTimeoutError:
            pass
    deadline = time.monotonic() + args.duration
    data_buffer = b""
    states: dict[tuple[int, int], SequenceState] = {}
    printed_events = 0

    target = f"frame_id=0x{args.frame_id:x}" if args.frame_id is not None else "all frame ids"
    print(
        f"Monitoring {target} {args.field} "
        f"{'payload[' + str(args.payload_index) + ']' if args.field == 'payload' else ''} "
        f"for {args.duration:.1f}s"
    )
    while time.monotonic() < deadline:
        try:
            data = bytes(dev.read(EP_IN, READ_SIZE, timeout=args.timeout_ms))
        except usb.core.USBTimeoutError:
            continue
        if not data:
            continue
        data_buffer += data
        frames, consumed = parse_records(data_buffer)
        if consumed:
            data_buffer = data_buffer[consumed:]

        for src, frame_id, cycle_count, payload in frames:
            if args.frame_id is not None and frame_id != args.frame_id:
                continue
            if args.field == "payload" and len(payload) <= args.payload_index:
                continue

            if args.field == "header-cycle":
                value = cycle_count & 0x3F
                raw_value = value
            else:
                value = payload[args.payload_index] & 0x3F
                raw_value = payload[args.payload_index]

            key = (src, frame_id)
            state = states.setdefault(key, SequenceState())
            state.sources[src] = state.sources.get(src, 0) + 1
            if state.expected is None:
                state.first = value
                state.last = value
                state.seen += 1
                if args.cycle_step is not None:
                    state.step = args.cycle_step & 0x3F
                    state.expected = (value + state.step) & 0x3F
                if args.frame_id is not None:
                    print(
                        f"first frame_id=0x{frame_id:x} src={src} "
                        f"header_cycle={cycle_count:02x} payload=0x{raw_value:02x}"
                    )
                continue

            if state.step is None:
                inferred_step = (value - state.last) & 0x3F
                if inferred_step == 0:
                    state.errors += 1
                    state.repeats += 1
                    if args.max_events > 0 and printed_events < args.max_events:
                        print(
                            f"sequence repeat: "
                            f"frame_id=0x{frame_id:x} seen={state.seen} src={src} "
                            f"header_cycle={cycle_count:02x} payload=0x{raw_value:02x} "
                            f"expected=unknown previous=0x{state.last:02x}"
                        )
                        printed_events += 1
                    state.seen += 1
                    continue
                state.step = inferred_step
                state.expected = (value + state.step) & 0x3F
                state.last = value
                state.seen += 1
                continue

            if value != state.expected:
                state.errors += 1
                if value == state.last:
                    state.repeats += 1
                    kind = "repeat"
                elif ((value - state.expected) & 0x3F) < 32:
                    state.gaps += 1
                    kind = "gap"
                else:
                    state.backwards += 1
                    kind = "backward"
                if args.max_events > 0 and printed_events < args.max_events:
                    print(
                        f"sequence {kind}: "
                        f"frame_id=0x{frame_id:x} seen={state.seen} src={src} "
                        f"header_cycle={cycle_count:02x} payload=0x{raw_value:02x} "
                        f"expected=0x{state.expected:02x} previous=0x{state.last:02x}"
                    )
                    printed_events += 1
                state.expected = (value + state.step) & 0x3F
            else:
                state.expected = (state.expected + state.step) & 0x3F
                if args.print_ok:
                    print(f"ok frame_id=0x{frame_id:x} src={src} payload=0x{raw_value:02x}")
            state.last = value
            state.seen += 1

    total_seen = sum(state.seen for state in states.values())
    total_errors = sum(state.errors for state in states.values())
    total_repeats = sum(state.repeats for state in states.values())
    total_gaps = sum(state.gaps for state in states.values())
    total_backwards = sum(state.backwards for state in states.values())
    print(
        f"done: tracked={len(states)} seen={total_seen} errors={total_errors} "
        f"repeats={total_repeats} gaps={total_gaps} backwards={total_backwards}"
    )

    rows = sorted(
        states.items(),
        key=lambda item: (item[1].errors, item[1].seen),
        reverse=True,
    )
    for (src, frame_id), state in rows[: args.summary_limit]:
        if state.errors == 0 and args.frame_id is None:
            continue
        print(
            f"summary frame_id=0x{frame_id:x} src={src} seen={state.seen} "
            f"step={state.step} errors={state.errors} gaps={state.gaps} repeats={state.repeats} "
            f"backwards={state.backwards} first={state.first} last={state.last}"
        )

    return 0 if total_seen > 0 and total_errors == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
