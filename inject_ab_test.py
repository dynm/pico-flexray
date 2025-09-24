#!/usr/bin/env python3
import sys
import struct
import time
import math
import threading
import queue
import os
from opendbc.can.parser import CANParser
from opendbc.can.packer import CANPacker

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
EP_VENDOR_IN = 0x81


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

# Initialize CANPacker with local DBC path
_DBC_PATH = os.path.join(os.path.dirname(__file__), "dbc", "focus_on_lateral.dbc")
_PACKER = CANPacker(_DBC_PATH)
_DBC = _PACKER.dbc

# Expose full-field packing for ACC so you can tune every field
ACC_DEFAULTS = {
    "cycle_count": 0,
    "crc1": 0,
    "cnt1": 0,
    "always_0x9": 9,
    "steering_angle_req": 0.0,
    "steer_torque_req": 0.0,
    "TJA_ready": 0,
    "assist_mode": 0,
    "wayback_en1_lane_keeping_trigger": 0,
    "lane_keeping_triggered": 0,
    "like_assist_torque_reserve": 0xA0,
    "constants": 0x03ff17fe,
    "wayback_en_2": 0,
    "steering_engaged": 2,
    "maybe_assist_force_enhance": 0xa2,
    "maybe_assist_force_weaken": 0xfa,
}

#          B8 61 FC 7F 02 01 00 AO FE 17 FF 23 A2 FA
#          96 64 1f 80 00 01 00 a0 fe 17 ff 23 a2 fa

# 01 48 90 a8 61 fe 7f 00 01 00 a0 fe 17 ff 23 a2 fa


def pack_acc_payload(values: dict):
    """Pack the entire ACC payload bytes (17 bytes) using CANPacker.

    Note: crc1/cnt1 are not auto-computed for this custom DBC; set them explicitly if needed.
    """
    merged = dict(ACC_DEFAULTS)
    merged.update(values)
    msg = _PACKER.make_can_msg("ACC", 0, merged)
    return msg


def build_frame(angle_deg: float, torque_nm: float) -> bytes:
    values = {"steering_angle_req": angle_deg, "steer_torque_req": torque_nm}
    values["cycle_count"] = 1
    values["crc1"] = 0x48 & 0xFF
    values["cnt1"] = (0x48 >> 8) & 0b111
    data = pack_acc_payload(values)
    return data

print(build_frame(30, 0.2).hex())

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
    """Reads from IN endpoint to detect whether bus data is flowing. Sets paused if silent."""
    def __init__(self, dev: "usb.core.Device", silence_timeout_s: float = 0.7):
        super().__init__(daemon=True)
        self.dev = dev
        self.silence_timeout_s = silence_timeout_s
        self.last_data_time = time.time()
        self._stop = False

    def run(self):
        while not self._stop:
            try:
                # Small read to detect activity; timeout short
                data = self.dev.read(EP_VENDOR_IN, 64, timeout=100)  # type: ignore
                if data and len(data) > 0:
                    self.last_data_time = time.time()
            except usb.core.USBTimeoutError:
                pass
            except usb.core.USBError:
                # transient errors; keep trying
                time.sleep(0.1)
            except Exception:
                time.sleep(0.1)

    def should_pause(self) -> bool:
        return (time.time() - self.last_data_time) > self.silence_timeout_s

    def stop(self):
        self._stop = True


def print_help():
    print("\nControls:")
    print("  a: switch to Angle-only mode")
    print("  t: switch to Torque-only mode")
    print("  + / - : increase / decrease amplitude")
    print("  f: faster sweep   s: slower sweep")
    print("  q: quit")
    print("")


def main() -> int:
    # Initial params
    mode = "A"  # A: angle-only, T: torque-only
    angle_amp = 100.0  # deg peak for triangle wave
    torque_amp = 0.2  # Nm amplitude for torque-only mode
    MAX_TORQUE = 0.3  # Absolute safety clamp in Nm
    half_period_s = 3.0

    print("FlexRay A/B injection (Angle vs Torque)")
    print("=====================================")
    print_help()

    dev = find_device()
    if dev is None:
        print("Device not found. Is the Pico connected and running the app?", file=sys.stderr)
        return 1

    key_thread = KeyInput()
    key_thread.start()

    monitor = DataMonitor(dev)
    monitor.start()

    start_time = time.time()
    last_print = 0.0
    send_count = 0
    last_second = int(time.time())
    sends_in_second = 0

    try:
        while True:
            # Handle keys
            ch = key_thread.get_nowait()
            if ch:
                if ch == 'q':
                    break
                elif ch == 'a':
                    mode = "A"
                elif ch == 't':
                    mode = "T"
                elif ch == '+':
                    if mode == 'A':
                        angle_amp = min(45.0, angle_amp + 1.0)
                    else:
                        torque_amp = min(MAX_TORQUE, torque_amp + 0.05)
                elif ch == '-':
                    if mode == 'A':
                        angle_amp = max(1.0, angle_amp - 1.0)
                    else:
                        torque_amp = max(0.0, torque_amp - 0.05)
                elif ch == 'f':
                    half_period_s = max(0.5, half_period_s * 0.8)
                elif ch == 's':
                    half_period_s = min(10.0, half_period_s * 1.25)

            # Generate setpoints
            t_rel = (time.time() - start_time) % (2 * half_period_s)
            if t_rel < half_period_s:
                angle_cmd = -angle_amp + (2 * angle_amp) * (t_rel / half_period_s)
            else:
                angle_cmd = angle_amp - (2 * angle_amp) * ((t_rel - half_period_s) / half_period_s)

            if mode == 'A':
                torque_cmd = 0.0
            else:
                # In torque-only mode, hold small angle (0) and modulate torque as a slow sine
                torque_cmd = torque_amp * math.sin(2 * math.pi * (time.time() - start_time) / (2 * half_period_s))
                angle_cmd = 0.0

            # Safety clamp torque to ±MAX_TORQUE
            if torque_cmd > MAX_TORQUE:
                torque_cmd = MAX_TORQUE
            elif torque_cmd < -MAX_TORQUE:
                torque_cmd = -MAX_TORQUE

            # Pause injection if no bus data is observed recently
            if monitor.should_pause():
                # Brief idle before next check
                time.sleep(0.05)
            else:
                # Build ACC override payload via DBC packing slice
                payload = build_frame(angle_cmd, 0)
                buf = build_override_payload(0x48, 1, payload)
                print(f"Hex: {buf.hex()} Angle: {angle_cmd}, Torque: {torque_cmd}")
                try:
                    dev.write(EP_VENDOR_OUT, buf, timeout=1000)
                    send_count += 1
                    sends_in_second += 1
                except usb.core.USBError as e:
                    print(f"USB write failed: {e}", file=sys.stderr)
                    # Try to reconnect
                    time.sleep(0.2)
                    dev = find_device()
                    if dev is None:
                        print("Reconnection failed. Exiting.", file=sys.stderr)
                        break

            now = time.time()
            sec = int(now)
            if sec != last_second:
                fps = sends_in_second
                sends_in_second = 0
                last_second = sec
                paused = monitor.should_pause()
                print(f"mode={mode} angle_amp={angle_amp:.1f}deg torque_amp={min(torque_amp, MAX_TORQUE):.2f}Nm halfT={half_period_s:.2f}s send_fps={fps} paused={paused}")

            # pacing ~50 Hz
            time.sleep(0.02)

    finally:
        try:
            monitor.stop()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    sys.exit(main())


