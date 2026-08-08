#!/usr/bin/env python3
"""FlexRay rack-force injection for frame ID 0x08, cycle group m2.

DBC signal (the DBC includes cycle count as its first byte):
  STA_AddRackForce_Rq : 128|12@1+ (2,-4096) [-4096|4094] "N"

The firmware payload excludes that cycle-count byte, so DBC bit 128 maps to
payload bit 120.  The firmware rule updates payload bytes 13..16 with masks,
preserving unrelated bits on both edges of the replacement range.
"""
import sys
import struct
import time
import threading
import queue

try:
    import usb.core
    import usb.util
except Exception:
    print("PyUSB is required. Install with: pip install pyusb", file=sys.stderr)
    sys.exit(1)


PANDA_VID = 0x3801
PANDA_PID = 0xDDCC

EP_VENDOR_OUT = 0x03
EP_VENDOR_IN = 0x81

TARGET_FRAME_ID = 0x08
CYCLE_BASE = 0x02
PAYLOAD_LENGTH = 18

RACK_FORCE_FACTOR = 2.0
RACK_FORCE_OFFSET = -4096.0
RACK_FORCE_MIN = -4096.0
RACK_FORCE_MAX = 4094.0


def find_device():
    dev = usb.core.find(idVendor=PANDA_VID, idProduct=PANDA_PID)
    if dev is None:
        return None
    try:
        if hasattr(dev, "set_configuration"):
            dev.set_configuration()
    except Exception:
        pass
    return dev


def physical_to_raw(physical: float) -> int:
    physical = max(RACK_FORCE_MIN, min(RACK_FORCE_MAX, physical))
    raw = (physical - RACK_FORCE_OFFSET) / RACK_FORCE_FACTOR
    raw = int(round(raw))
    return max(0, min(0xFFF, raw))


def crc8_1d(data: bytes, init_value: int) -> int:
    crc = init_value & 0xFF
    for byte in data:
        crc ^= byte & 0xFF
        for _ in range(8):
            if (crc & 0x80) != 0:
                crc = ((crc << 1) ^ 0x1D) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_override_payload(frame_id: int, base: int, data_bytes: bytes) -> bytes:
    header = struct.pack('<BHBH', 0x90, frame_id, base, len(data_bytes))
    return header + data_bytes


def set_le_unsigned(data: bytearray, start_bit: int, length: int, raw: int) -> None:
    """Pack an Intel/little-endian unsigned field into a byte array."""
    if raw < 0 or raw >= (1 << length):
        raise ValueError(f"raw value {raw} does not fit in {length} bits")
    for bit in range(length):
        absolute_bit = start_bit + bit
        mask = 1 << (absolute_bit % 8)
        byte_index = absolute_bit // 8
        if raw & (1 << bit):
            data[byte_index] |= mask
        else:
            data[byte_index] &= ~mask


def build_rack_force_payload(rack_force_n: float) -> bytes:
    """Build the host payload consumed by injector_submit_override().

    The first byte is the USB-command integrity CRC.  The following 18 bytes
    use firmware-payload coordinates (DBC coordinates minus the cycle byte).
    """
    payload = bytearray(PAYLOAD_LENGTH)

    # DBC 115|1 -> firmware payload bit 107: request-quality valid.
    set_le_unsigned(payload, 107, 1, 1)  # STA_AddRackForce_Qual

    # Keep the front-wheel-angle request neutral: raw=(0-(-51.2))/0.025=2048.
    set_le_unsigned(payload, 108, 12, 2048)  # STA_FtWhlAngl_Rq

    # DBC 128|12 -> firmware payload bit 120 after removing cycle count.
    set_le_unsigned(payload, 120, 12, physical_to_raw(rack_force_n))

    crc = crc8_1d(payload, 0xF1)
    return bytes([crc]) + payload


class KeyInput(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.q: "queue.Queue[str]" = queue.Queue()

    def run(self):
        try:
            while True:
                ch = sys.stdin.read(1)
                if not ch:
                    break
                self.q.put(ch)
        except Exception:
            pass

    def get_nowait(self) -> str:
        try:
            return self.q.get_nowait()
        except queue.Empty:
            return ""


class DataMonitor(threading.Thread):
    def __init__(self, dev, silence_timeout_s: float = 0.7):
        super().__init__(daemon=True)
        self.dev = dev
        self.silence_timeout_s = silence_timeout_s
        self.last_data_time = time.time()
        self._stop = False

    def run(self):
        while not self._stop:
            try:
                data = self.dev.read(EP_VENDOR_IN, 64, timeout=100)
                if data and len(data) > 0:
                    self.last_data_time = time.time()
            except usb.core.USBTimeoutError:
                pass
            except usb.core.USBError:
                time.sleep(0.1)
            except Exception:
                time.sleep(0.1)

    def should_pause(self) -> bool:
        return (time.time() - self.last_data_time) > self.silence_timeout_s

    def stop(self):
        self._stop = True


def print_help():
    print("\nControls:")
    print("  + / - : increase / decrease amplitude")
    print("  f: faster sweep   s: slower sweep")
    print("  q: quit")
    print("")


def main() -> int:
    amp = 2500.0
    half_period_s = 5.0

    print("FlexRay EPS injection – additional rack-force sweep")
    print("====================================================")
    print(f"Frame 0x{TARGET_FRAME_ID:02X}, cycle m2, signal range ±{amp:.1f} N")
    print_help()

    dev = find_device()
    if dev is None:
        print("Device not found. Is the Pico connected and running?", file=sys.stderr)
        return 1

    key_thread = KeyInput()
    key_thread.start()

    monitor = DataMonitor(dev)
    monitor.start()

    start_time = time.time()
    last_second = int(time.time())
    sends_in_second = 0

    try:
        while True:
            ch = key_thread.get_nowait()
            if ch:
                if ch == 'q':
                    break
                elif ch == '+':
                    amp = min(1000.0, amp + 10.0)
                elif ch == '-':
                    amp = max(10.0, amp - 10.0)
                elif ch == 'f':
                    half_period_s = max(0.5, half_period_s * 0.8)
                elif ch == 's':
                    half_period_s = min(10.0, half_period_s * 1.25)

            t_rel = (time.time() - start_time) % (2 * half_period_s)
            if t_rel < half_period_s:
                force_cmd = -amp + (2 * amp) * (t_rel / half_period_s)
            else:
                force_cmd = amp - (2 * amp) * ((t_rel - half_period_s) / half_period_s)

            if monitor.should_pause():
                time.sleep(0.05)
            else:
                payload = build_rack_force_payload(force_cmd)
                buf = build_override_payload(TARGET_FRAME_ID, CYCLE_BASE, payload)
                # print(f"Override: {buf.hex()}")
                try:
                    dev.write(EP_VENDOR_OUT, buf, timeout=1000)
                    sends_in_second += 1
                except usb.core.USBError as e:
                    print(f"USB write failed: {e}", file=sys.stderr)
                    time.sleep(0.2)
                    dev = find_device()
                    if dev is None:
                        print("Reconnection failed. Exiting.", file=sys.stderr)
                        break

            now = time.time()
            sec = int(now)
            if sec != last_second:
                raw = physical_to_raw(force_cmd)
                paused = monitor.should_pause()
                print(f"rack_force={force_cmd:+8.1f}N  raw=0x{raw:03X}  amp={amp:.1f}N  halfT={half_period_s:.2f}s  fps={sends_in_second}  paused={paused}")
                sends_in_second = 0
                last_second = sec

            time.sleep(0.02)

    finally:
        try:
            monitor.stop()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
