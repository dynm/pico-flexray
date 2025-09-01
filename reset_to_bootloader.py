#!/usr/bin/env python3

import sys
import time
from typing import Optional

try:
    import usb.core  # type: ignore
    import usb.util  # type: ignore
except Exception as exc:  # pragma: no cover
    print("PyUSB is required. Install with: pip install pyusb", file=sys.stderr)
    raise


# Panda USB VID/PID (matches usb_descriptors.c)
PANDA_VID = 0x3801
PANDA_PID = 0xDDCC

# Vendor control request to enter bootloader (matches panda_usb.h)
REQUEST_ENTER_BOOTLOADER = 0xD1

# bmRequestType: Host-to-Device | Type=Vendor | Recipient=Device
BM_REQUEST_TYPE_OUT_VENDOR_DEVICE = 0x40


def find_device(vid: int = PANDA_VID, pid: int = PANDA_PID) -> Optional["usb.core.Device"]:
    dev = usb.core.find(idVendor=vid, idProduct=pid)
    if dev is None:
        return None
    # Setting configuration is generally safe; ignore errors if already set
    try:
        if hasattr(dev, "set_configuration"):
            dev.set_configuration()  # type: ignore[attr-defined]
    except Exception:
        pass
    return dev


def enter_bootloader(dev: "usb.core.Device", mode: int = 0, timeout_ms: int = 2000) -> None:
    # wValue encodes mode. panda_usb.c expects 0 for bootloader
    # Zero-length OUT control transfer
    dev.ctrl_transfer(
        BM_REQUEST_TYPE_OUT_VENDOR_DEVICE,
        REQUEST_ENTER_BOOTLOADER,
        wValue=mode,
        wIndex=0,
        data_or_wLength=b"",
        timeout=timeout_ms,
    )


def main() -> int:
    print(f"Searching for device {PANDA_VID:04x}:{PANDA_PID:04x}...")
    dev = find_device()
    if dev is None:
        print("Device not found. Is the Pico connected and running the app?", file=sys.stderr)
        return 1

    print("Sending ENTER_BOOTLOADER_MODE (bRequest=0xD1, wValue=0)...")
    try:
        enter_bootloader(dev, mode=0)
    except Exception as exc:
        # Device may disconnect immediately after status stage; often OK
        print(f"Warning: control transfer raised: {exc}")

    # Give the device a moment to re-enumerate as UF2 bootloader
    time.sleep(0.5)
    print("If successful, the Pico should now be in BOOTSEL (UF2) mode.")
    return 0


if __name__ == "__main__":
    sys.exit(main())


