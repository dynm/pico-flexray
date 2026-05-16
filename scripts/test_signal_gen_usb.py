#!/usr/bin/env python3
import argparse
import sys
import time

import usb.core
import usb.util


VID = 0xCAFE
PID = 0x4004
INTERFACE = 0
EP_OUT = 0x03
EP_IN = 0x81

CMD_SET_SLOT = 0x03
CMD_START = 0x05
CMD_STOP = 0x06
CMD_TARGET = 0x09
CMD_SET_PINS = 0x0A
CMD_CLEAR_ALL = 0x0B
CMD_PIN_TEST = 0x0C
CMD_PIO_TEST = 0x0D
CMD_DIAG = 0x0E

RSP_OK = 0x00

TARGET_NAMES = {
    0x20: "RP2040",
    0x50: "RP2350",
}

DEFAULT_PINS = (
    (28, 27),
    (4, 5),
    (10, 9),
    (16, 22),
)


class ProtocolError(RuntimeError):
    pass


def claim(dev) -> None:
    try:
        if dev.is_kernel_driver_active(INTERFACE):
            dev.detach_kernel_driver(INTERFACE)
    except (NotImplementedError, usb.core.USBError):
        pass
    usb.util.claim_interface(dev, INTERFACE)


def xfer(dev, name: str, packet: bytes, read_len: int = 64, timeout_ms: int = 1500) -> bytes:
    started = time.monotonic()
    print(f"{name}: tx={packet.hex(' ')}", flush=True)
    dev.write(EP_OUT, packet, timeout=timeout_ms)
    rsp = bytes(dev.read(EP_IN, read_len, timeout=timeout_ms))
    elapsed_ms = (time.monotonic() - started) * 1000.0
    print(f"{name}: rsp={rsp.hex(' ')} ({elapsed_ms:.1f} ms)")
    return rsp


def expect_ok(dev, name: str, packet: bytes, timeout_ms: int = 1500) -> bytes:
    rsp = xfer(dev, name, packet, timeout_ms=timeout_ms)
    if not rsp or rsp[0] != RSP_OK:
        status = "empty" if not rsp else f"0x{rsp[0]:02x}"
        raise ProtocolError(f"{name} rejected: status={status}, rsp={rsp.hex(' ')}")
    return rsp


def query_target(dev, label: str = "target") -> bytes:
    rsp = expect_ok(dev, label, bytes([CMD_TARGET]))
    if len(rsp) < 3:
        raise ProtocolError(f"{label} response too short: {rsp.hex(' ')}")
    target = TARGET_NAMES.get(rsp[1], f"unknown 0x{rsp[1]:02x}")
    version = ".".join(str(b) for b in rsp[3:6]) if len(rsp) >= 6 else "unknown"
    print(f"{label}: {target}, protocol={rsp[2]}, fw={version}")
    return rsp


def build_pin_packet(pins: tuple[tuple[int, int], ...]) -> bytes:
    data = bytearray([CMD_SET_PINS])
    for tx, txen in pins:
        data.extend((tx, txen))
    return bytes(data)


def parse_pin_list(value: str) -> tuple[int, int, int, int]:
    try:
        pins = tuple(int(part.strip(), 0) for part in value.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("pin list must contain integers") from exc
    if len(pins) != 4:
        raise argparse.ArgumentTypeError("pin list must contain exactly 4 values")
    for pin in pins:
        if pin < 0 or pin > 29:
            raise argparse.ArgumentTypeError("pins must be in GPIO range 0..29")
    return pins


def build_slot_packet(slot: int, payload_len: int, channel_mask: int, frame_base: int) -> bytes:
    frame_id = (frame_base + slot) & 0x7FF
    fill = frame_id & 0xFF
    payload = bytes([fill]) * payload_len
    return bytes([
        CMD_SET_SLOT,
        slot & 0xFF,
        channel_mask & 0x0F,
        frame_id & 0xFF,
        (frame_id >> 8) & 0xFF,
        0x00,
        payload_len & 0xFF,
        (payload_len >> 8) & 0xFF,
    ]) + payload


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Configure and start the FlexRay signal generator over vendor USB."
    )
    parser.add_argument("--slots", type=int, default=40, help="number of slots to configure")
    parser.add_argument("--payload-len", type=int, default=26, help="payload bytes per slot, must be even")
    parser.add_argument("--channel-mask", type=lambda s: int(s, 0), default=0x0F,
                        help="channel mask for every slot, default 0x0f")
    parser.add_argument("--frame-base", type=lambda s: int(s, 0), default=1,
                        help="first frame id, default 1")
    parser.add_argument("--tx-pin", type=parse_pin_list,
                        help='comma-separated TX GPIOs for FR1..FR4, for example "28,4,10,16"')
    parser.add_argument("--txen-pin", type=parse_pin_list,
                        help='comma-separated TX_EN GPIOs for FR1..FR4, for example "27,5,9,22"')
    parser.add_argument("--timeout-ms", type=int, default=2000)
    parser.add_argument("--no-initial-stop", action="store_true",
                        help="skip the initial STOP used to recover from a prior running test")
    parser.add_argument("--leave-running", action="store_true",
                        help="do not send STOP after the post-start verification")
    parser.add_argument("--pin-test", action="store_true",
                        help="skip slots/start and toggle configured TX pins at about 10 Hz")
    parser.add_argument("--pio-test", action="store_true",
                        help="skip slots/start and feed a visible pattern through PIO channel 1")
    parser.add_argument("--diag-delay", type=float, default=0.2,
                        help="seconds to wait after START before reading diagnostics")
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.slots < 1 or args.slots > 64:
        raise SystemExit("--slots must be 1..64")
    if args.payload_len < 0 or args.payload_len > 254 or (args.payload_len & 1):
        raise SystemExit("--payload-len must be an even value from 0..254")
    if args.channel_mask < 1 or args.channel_mask > 0x0F:
        raise SystemExit("--channel-mask must be 0x1..0x0f")
    if args.frame_base < 0 or args.frame_base > 0x7FF:
        raise SystemExit("--frame-base must be 0..0x7ff")
    if (args.tx_pin is None) != (args.txen_pin is None):
        raise SystemExit("--tx-pin and --txen-pin must be provided together")
    pins = tuple(zip(args.tx_pin, args.txen_pin)) if args.tx_pin else DEFAULT_PINS
    seen: set[int] = set()
    for index, (tx, txen) in enumerate(pins, start=1):
        if tx == txen:
            raise SystemExit(f"FR{index} TX and TX_EN must be different")
        for pin in (tx, txen):
            if pin in seen:
                raise SystemExit(f"GPIO {pin} is assigned more than once")
            seen.add(pin)


def main() -> int:
    args = parse_args()
    validate_args(args)

    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        print("FlexRay Signal Generator USB device not found (VID 0xCAFE, PID 0x4004).", file=sys.stderr)
        return 1

    try:
        if dev.get_active_configuration() is None:
            dev.set_configuration()
    except usb.core.USBError:
        dev.set_configuration()

    try:
        claim(dev)
        pins = tuple(zip(args.tx_pin, args.txen_pin)) if args.tx_pin else DEFAULT_PINS
        print(f"Testing {args.slots} slot(s), {args.payload_len}B payload, mask=0x{args.channel_mask:x}")
        print("Pins: " + " ".join(f"FR{i}=TX{tx}/EN{txen}" for i, (tx, txen) in enumerate(pins, start=1)))
        if not args.no_initial_stop:
            try:
                expect_ok(dev, "initial-stop", bytes([CMD_STOP]), args.timeout_ms)
            except (usb.core.USBError, ProtocolError) as exc:
                print(f"initial-stop ignored: {exc}")
        query_target(dev, "target-before")
        expect_ok(dev, "set-pins", build_pin_packet(pins), args.timeout_ms)
        if args.pin_test:
            expect_ok(dev, "pin-test-on", bytes([CMD_PIN_TEST, 1]), args.timeout_ms)
            print("Pin test running: TX pins toggle at ~10 Hz; TX_EN pins are held active low.")
            return 0
        if args.pio_test:
            expect_ok(dev, "pio-test-on", bytes([CMD_PIO_TEST, 1]), args.timeout_ms)
            print("PIO test running on FR1: TX should toggle from PIO FIFO; TX_EN should pulse active low.")
            return 0
        expect_ok(dev, "clear-all", bytes([CMD_CLEAR_ALL]), args.timeout_ms)

        for slot in range(args.slots):
            pkt = build_slot_packet(slot, args.payload_len, args.channel_mask, args.frame_base)
            expect_ok(dev, f"set-slot-{slot}", pkt, args.timeout_ms)

        expect_ok(dev, "start", bytes([CMD_START]), args.timeout_ms)
        query_target(dev, "target-after-start")
        if args.diag_delay > 0:
            time.sleep(args.diag_delay)
            diag = expect_ok(dev, "diag", bytes([CMD_DIAG]), args.timeout_ms)
            if len(diag) >= 5:
                stalls = int.from_bytes(diag[1:5], "little")
                if len(diag) >= 17:
                    late = int.from_bytes(diag[5:9], "little")
                    completed = int.from_bytes(diag[9:13], "little")
                    handled = int.from_bytes(diag[13:17], "little")
                    render_text = ""
                    if len(diag) >= 25:
                        last_render = int.from_bytes(diag[17:21], "little")
                        max_render = int.from_bytes(diag[21:25], "little")
                        render_text = (
                            f" last_render_us={last_render}"
                            f" max_render_us={max_render}"
                        )
                    print(
                        "diag: "
                        f"txstall_count={stalls} "
                        f"late_buffer_count={late} "
                        f"completed_cycles={completed} "
                        f"handled_cycles={handled}"
                        f"{render_text}"
                    )
                else:
                    print(f"diag: txstall_count={stalls}")

        if args.leave_running:
            print("Leaving generator running.")
        else:
            expect_ok(dev, "stop", bytes([CMD_STOP]), args.timeout_ms)
        print("USB protocol test passed.")
        return 0
    except usb.core.USBError as exc:
        print(f"USB error: {exc}", file=sys.stderr)
        return 1
    except ProtocolError as exc:
        print(f"Protocol error: {exc}", file=sys.stderr)
        return 1
    finally:
        try:
            usb.util.release_interface(dev, INTERFACE)
        except usb.core.USBError:
            pass
        try:
            usb.util.dispose_resources(dev)
        except usb.core.USBError:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
