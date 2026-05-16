#!/usr/bin/env python3
import argparse
import errno
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import usb.core
import usb.util


SIGNAL_GEN_VID = 0xCAFE
SIGNAL_GEN_PID = 0x4004
BOOTSEL_VID = 0x2E8A
BOOTSEL_PID_RP2040 = 0x0003
BOOTSEL_PID_RP2350 = 0x000F

INTERFACE = 0
EP_OUT = 0x03
EP_IN = 0x81
CMD_BOOTLOADER = 0x08
CMD_TARGET = 0x09
RSP_OK = 0x00
MIN_PROTOCOL_VERSION = 2

TARGET_CODES = {
    0x20: "rp2040",
    0x50: "rp2350",
}

UF2_NAMES = {
    "rp2040": "flexray_signal_gen_rp2040.uf2",
    "rp2350": "flexray_signal_gen_rp2350.uf2",
}


def is_disconnect(exc: usb.core.USBError) -> bool:
    return getattr(exc, "errno", None) == errno.ENODEV or "Errno 19" in str(exc)


def claim(dev) -> None:
    try:
        if dev.is_kernel_driver_active(INTERFACE):
            dev.detach_kernel_driver(INTERFACE)
    except (NotImplementedError, usb.core.USBError):
        pass
    usb.util.claim_interface(dev, INTERFACE)


def running_firmware_device():
    return usb.core.find(idVendor=SIGNAL_GEN_VID, idProduct=SIGNAL_GEN_PID)


def query_running_target(dev) -> str | None:
    try:
        claim(dev)
        dev.write(EP_OUT, bytes([CMD_TARGET]), timeout=1000)
        rsp = bytes(dev.read(EP_IN, 64, timeout=1000))
        if len(rsp) >= 3 and rsp[0] == RSP_OK and rsp[2] >= MIN_PROTOCOL_VERSION:
            return TARGET_CODES.get(rsp[1])
    except usb.core.USBError:
        return None
    finally:
        try:
            usb.util.release_interface(dev, INTERFACE)
        except usb.core.USBError:
            pass
        try:
            usb.util.dispose_resources(dev)
        except usb.core.USBError:
            pass
    return None


def wait_for_running_target(timeout_s: int) -> str | None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        dev = running_firmware_device()
        if dev is not None:
            target = query_running_target(dev)
            if target:
                return target
        time.sleep(0.5)
    return None


def reset_running_to_bootloader(dev) -> bool:
    try:
        claim(dev)
        try:
            dev.write(EP_OUT, bytes([CMD_BOOTLOADER]), timeout=1000)
        except usb.core.USBError as exc:
            if not is_disconnect(exc):
                raise
        try:
            dev.read(EP_IN, 1, timeout=250)
        except usb.core.USBError as exc:
            if not is_disconnect(exc):
                pass
        return True
    except usb.core.USBError as exc:
        print(f"Failed to send bootloader command: {exc}", file=sys.stderr)
        print("Close any browser tab or serial tool currently connected to the Pico, then retry.", file=sys.stderr)
        return False
    finally:
        try:
            usb.util.release_interface(dev, INTERFACE)
        except usb.core.USBError:
            pass
        try:
            usb.util.dispose_resources(dev)
        except usb.core.USBError:
            pass


def reboot_to_bootsel_with_picotool(picotool: str | None) -> bool:
    if not picotool:
        return False
    cmd = [
        picotool,
        "reboot",
        "-u",
        "-f",
        "--vid",
        f"0x{SIGNAL_GEN_VID:04x}",
        "--pid",
        f"0x{SIGNAL_GEN_PID:04x}",
    ]
    return subprocess.run(cmd).returncode == 0


def run_text(cmd: list[str]) -> tuple[int, str]:
    proc = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    return proc.returncode, proc.stdout


def detect_bootsel_with_picotool(picotool: str | None) -> str | None:
    if not picotool:
        return None
    code, out = run_text([picotool, "info", "-d"])
    if code != 0:
        return None
    lowered = out.lower()
    if "rp2350" in lowered:
        return "rp2350"
    if "rp2040" in lowered:
        return "rp2040"
    return None


def flatten_usb_items(value):
    if isinstance(value, dict):
        yield value
        for child in value.get("_items", []):
            yield from flatten_usb_items(child)
    elif isinstance(value, list):
        for item in value:
            yield from flatten_usb_items(item)


def detect_bootsel_with_usb_tree() -> str | None:
    if sys.platform != "darwin":
        return None
    try:
        proc = subprocess.run(
            ["system_profiler", "SPUSBDataType", "-json"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if proc.returncode != 0:
        return None
    try:
        data = json.loads(proc.stdout)
    except json.JSONDecodeError:
        return None

    found: set[str] = set()
    for item in flatten_usb_items(data.get("SPUSBDataType", [])):
        vendor = str(item.get("vendor_id", "")).lower()
        product = str(item.get("product_id", "")).lower()
        name = " ".join(str(item.get(k, "")) for k in ("_name", "manufacturer", "serial_num")).lower()
        if "0x2e8a" not in vendor and "raspberry pi" not in name:
            continue
        if "0x000f" in product or "rp2350" in name:
            found.add("rp2350")
        if "0x0003" in product or "rp2040" in name or "rp2 boot" in name:
            found.add("rp2040")

    if len(found) == 1:
        return next(iter(found))
    if len(found) > 1:
        raise RuntimeError("Multiple BOOTSEL devices with different targets were found; connect only one device.")
    return None


def mounted_uf2_volumes() -> list[Path]:
    volumes = Path("/Volumes")
    if not volumes.exists():
        return []
    return [p for p in volumes.iterdir() if (p / "INFO_UF2.TXT").exists()]


def detect_bootsel_with_mounted_info() -> tuple[str | None, Path | None]:
    matches: list[tuple[str | None, Path]] = []
    for volume in mounted_uf2_volumes():
        try:
            info = (volume / "INFO_UF2.TXT").read_text(errors="ignore").lower()
        except OSError:
            continue
        target = None
        if "rp2350" in info:
            target = "rp2350"
        elif "rp2040" in info or "rpi-rp2" in info:
            target = "rp2040"
        matches.append((target, volume))

    known = [(target, volume) for target, volume in matches if target]
    if len({target for target, _ in known}) == 1 and len(known) == 1:
        return known[0]
    if len(known) > 1:
        raise RuntimeError("Multiple mounted UF2 bootloader volumes were found; connect only one device.")
    if len(matches) == 1:
        return None, matches[0][1]
    return None, None


def detect_bootsel_target(picotool: str | None) -> tuple[str | None, Path | None]:
    target = detect_bootsel_with_picotool(picotool)
    if target:
        _, volume = detect_bootsel_with_mounted_info()
        return target, volume
    target = detect_bootsel_with_usb_tree()
    if target:
        _, volume = detect_bootsel_with_mounted_info()
        return target, volume
    return detect_bootsel_with_mounted_info()


def wait_for_bootsel_target(picotool: str | None, timeout_s: int) -> tuple[str | None, Path | None]:
    deadline = time.monotonic() + timeout_s
    last_target, last_volume = None, None
    while time.monotonic() < deadline:
        target, volume = detect_bootsel_target(picotool)
        last_target, last_volume = target, volume
        if target:
            return target, volume
        time.sleep(0.5)
    return last_target, last_volume


def flash_with_picotool(picotool: str | None, uf2: Path) -> bool:
    if not picotool:
        return False
    return subprocess.run([picotool, "load", "-f", "-x", str(uf2)]).returncode == 0


def copy_to_volume(uf2: Path, volume: Path) -> None:
    dest = volume / uf2.name
    shutil.copyfile(uf2, dest)


def resolve_picotool(value: str | None) -> str | None:
    if value and os.access(value, os.X_OK):
        return value
    found = shutil.which(value) if value else shutil.which("picotool")
    return found


def main() -> int:
    parser = argparse.ArgumentParser(description="Flash the matching RP2040/RP2350 UF2 automatically.")
    parser.add_argument("--build-dir", default="build", type=Path)
    parser.add_argument("--picotool")
    parser.add_argument("--timeout", default=15, type=int)
    args = parser.parse_args()

    build_dir = args.build_dir.resolve()
    picotool = resolve_picotool(args.picotool)
    target = None
    volume = None

    dev = running_firmware_device()
    if dev is not None:
        target = query_running_target(dev)
        if target:
            print(f"==> Running FlexRay firmware reports target: {target}")
        else:
            print("==> Running FlexRay firmware found, but target query is unavailable; will detect after BOOTSEL")
        dev = running_firmware_device()
        if dev is None or not reset_running_to_bootloader(dev):
            print("==> Vendor bootloader command failed; trying picotool reboot -u -f")
            if not reboot_to_bootsel_with_picotool(picotool):
                return 1
        time.sleep(1.0)
    else:
        target, volume = detect_bootsel_target(picotool)
        if target:
            print(f"==> Existing BOOTSEL device target: {target}")

    if not target:
        target, volume = wait_for_bootsel_target(picotool, args.timeout)
    if not target:
        if volume:
            print(f"BOOTSEL volume is mounted at {volume}, but target family could not be determined.", file=sys.stderr)
        else:
            print("No running FlexRay firmware device or BOOTSEL device was found.", file=sys.stderr)
        print("Connect one device, or hold BOOTSEL while plugging it in, then run again.", file=sys.stderr)
        return 1

    uf2 = build_dir / UF2_NAMES[target]
    if not uf2.exists():
        print(f"Expected UF2 was not found: {uf2}", file=sys.stderr)
        return 1

    print(f"==> Flashing {target}: {uf2}")
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        if flash_with_picotool(picotool, uf2):
            print("==> Flash complete")
            running_target = wait_for_running_target(args.timeout)
            if running_target:
                print(f"==> Verified running firmware target: {running_target}")
            else:
                print("==> Flash wrote successfully, but the running firmware did not answer the protocol query.", file=sys.stderr)
                print("    Unplug/replug the board, reconnect WebUSB, then retry if the page still reports old firmware.", file=sys.stderr)
            return 0
        if volume and volume.exists():
            copy_to_volume(uf2, volume)
            print(f"==> Copied UF2 to {volume}")
            return 0
        target, volume = detect_bootsel_target(picotool)
        time.sleep(0.5)

    print("Timed out waiting for the Pico bootloader.", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
