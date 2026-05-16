#!/usr/bin/env python3
import sys
import time
import errno

import usb.core
import usb.util


VID = 0xCAFE
PID = 0x4004
INTERFACE = 0
EP_OUT = 0x03
EP_IN = 0x81
CMD_BOOTLOADER = 0x08


def is_disconnect(exc: usb.core.USBError) -> bool:
    return getattr(exc, "errno", None) == errno.ENODEV or "Errno 19" in str(exc)


def main() -> int:
    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        print("FlexRay Signal Generator USB device not found (VID 0xCAFE, PID 0x4004).", file=sys.stderr)
        print("Connect the running firmware once so the bootloader command can be sent.", file=sys.stderr)
        return 1

    try:
        if dev.is_kernel_driver_active(INTERFACE):
            dev.detach_kernel_driver(INTERFACE)
    except (NotImplementedError, usb.core.USBError):
        pass

    try:
        usb.util.claim_interface(dev, INTERFACE)
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
    except usb.core.USBError as exc:
        print(f"Failed to send bootloader command: {exc}", file=sys.stderr)
        print("Close any browser tab or serial tool currently connected to the Pico, then retry.", file=sys.stderr)
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

    time.sleep(1.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
