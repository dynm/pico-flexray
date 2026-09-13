"""Host protocol and TDMA bench waveform checks; no Pico hardware is needed."""

import bisect
import csv
import io
from pathlib import Path
import socket
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import frame_gen_client as tx


def ack(op, status=0, body=b""):
    return struct.pack("<BBH", op, tx.VERSION, status) + body


def status_payload(flags=0, **values):
    fields = {name: 0 for name in tx.STATUS_FIELDS}
    fields.update(magic=tx.STATUS_MAGIC, version=tx.VERSION, size=tx.STATUS.size, flags=flags)
    fields.update(values)
    return tx.STATUS.pack(*(fields[name] for name in tx.STATUS_FIELDS))


class FakeSocket:
    def __init__(self, replies):
        self.replies = iter(replies)
        self.sent = []

    def connect(self, address):
        self.address = address

    def settimeout(self, timeout):
        self.timeout = timeout

    def send(self, payload):
        self.sent.append(payload)

    def recv(self, _size):
        result = next(self.replies)
        if isinstance(result, Exception):
            raise result
        return result

    def close(self):
        pass


class ClientTests(unittest.TestCase):
    def client(self, replies, timeout=1.0):
        sock = FakeSocket(replies)
        with patch.object(tx.socket, "socket", return_value=sock):
            client = tx.FrameGenClient(timeout=timeout)
        self.addCleanup(client.close)
        self.assertEqual(sock.address, ("192.168.7.1", 5501))
        return client, sock

    def test_diagnostics_snapshot_and_length(self):
        values = [2, 1, 0, 0, 0, 1, 12, 3, 12, 53, 54, 4, 0, 22, 7, 0]
        client, sock = self.client([ack(tx.OP_DIAGNOSTICS, body=tx.DIAGNOSTICS.pack(*values))])
        result = client.diagnostics()
        self.assertEqual(result["reason"], 1)
        self.assertEqual(result["fid"], 12)
        self.assertEqual(sock.sent, [b"\xa4"])
        client, _ = self.client([ack(tx.OP_DIAGNOSTICS, body=bytes(60))])
        with self.assertRaises(tx.ProtocolError): client.diagnostics()

    def test_config_wire_layout_and_signed_phase(self):
        self.assertEqual(tx.Config().pack(), bytes.fromhex(
            "0c00 1000 1200 0c00 00 03 03 08 "
            "00000000 00000000 00000000 20000000 0d000000"))
        config = tx.Config(slot_cycles=6000, phase_cycles=-15).pack()
        self.assertEqual(len(config), 32)
        self.assertEqual(struct.unpack_from("<IIi", config, 12), (6000, 0, -15))
        with self.assertRaisesRegex(ValueError, "FID 1 is not supported"):
            tx.Config(target_id=1).pack()

    def test_two_target_wire_and_targeted_clear(self):
        packed = tx.Config(target_id=0xc, second_target_id=0xd, static_max_id=0x10).pack()
        self.assertEqual(struct.unpack_from("<2H", packed, 28), (0xd, 0))
        for second in (1, 0xc, 0x11):
            with self.subTest(second=second), self.assertRaises(ValueError):
                tx.Config(target_id=0xc, second_target_id=second, static_max_id=0x10).pack()
        state = tx.parse_status(status_payload(flags=0x83, template_len=26, second_template_len=26))
        self.assertEqual(state["template_len"], 26)
        self.assertEqual(state["second_template_len"], 26)
        self.assertIn("second_template_valid", state["flag_names"])
        self.assertEqual(tx.STATUS.size, 108)
        with patch.object(tx, "FrameGenClient") as constructor, patch("sys.stdout", new=io.StringIO()):
            constructor.return_value.apply.return_value = {}
            self.assertEqual(tx.main(["clear-template", "--target-id", "0xd"]), 0)
            constructor.return_value.apply.assert_called_once_with(tx.OP_TEMPLATE, b"\x0d\x00")

    def test_payload_and_switch_actions_both_transports(self):
        for transport in ("udp", "usb"):
            with patch.object(tx, "send_action") as send, patch("sys.stdout", new=io.StringIO()):
                self.assertEqual(tx.main(["payload", "--target-id", "0xd", "--hex", "ab" * 18,
                                          "--transport", transport]), 0)
                send.assert_called_once_with(bytes.fromhex("940d00031200" + "ab" * 18),
                                             transport, "192.168.7.1", 5501, 3.0)
                for command, value in (("gen-enable", 1), ("gen-disable", 0)):
                    send.reset_mock()
                    self.assertEqual(tx.main([command, "--transport", transport]), 0)
                    self.assertEqual(send.call_args.args[0], bytes([0x95, value]))
        for fid, data in ((8, bytes(18)), (0xc, bytes(17)), (0xd, bytes(19))):
            with self.assertRaises(ValueError):
                tx.payload_action(fid, data)

    def test_status_validation(self):
        state = tx.parse_status(status_payload(flags=0x29, slot_cycles=6000))
        self.assertEqual(state["flag_names"], ["configured", "locked", "command_pending"])
        self.assertEqual(state["slot_us"], 40.0)
        for payload in (b"", status_payload(size=63), status_payload(version=1),
                        status_payload(magic=0)):
            with self.subTest(payload=payload), self.assertRaises(tx.ProtocolError):
                tx.parse_status(payload)

    def test_only_busy_retries(self):
        client, sock = self.client([ack(tx.OP_ENABLE, 5), ack(tx.OP_ENABLE)])
        client.request(tx.OP_ENABLE, b"\x01")
        self.assertEqual(sock.sent, [b"\xa2\x01", b"\xa2\x01"])
        for error in (1, 2, 3, 4, 6, 99):
            with self.subTest(error=error):
                client, sock = self.client([ack(tx.OP_CONFIG, error)])
                with self.assertRaises(tx.DeviceError) as raised:
                    client.request(tx.OP_CONFIG, tx.Config().pack())
                self.assertEqual(raised.exception.status, error)
                self.assertEqual(len(sock.sent), 1)

    def test_ack_is_followed_by_wait_for_application(self):
        client, sock = self.client([
            ack(tx.OP_CONFIG),
            ack(tx.OP_STATUS, body=status_payload(flags=tx.COMMAND_PENDING)),
            ack(tx.OP_STATUS, body=status_payload(flags=1)),
        ])
        state = client.apply(tx.OP_CONFIG, tx.Config().pack())
        self.assertEqual(state["flags"], 1)
        self.assertEqual([packet[0] for packet in sock.sent], [0xA0, 0xA3, 0xA3])

    def test_pending_command_has_finite_deadline(self):
        def replies():
            yield ack(tx.OP_ENABLE)
            while True:
                yield ack(tx.OP_STATUS, body=status_payload(flags=tx.COMMAND_PENDING))
        client, _ = self.client(replies(), timeout=0.01)
        with self.assertRaisesRegex(TimeoutError, "RX header"):
            client.apply(tx.OP_ENABLE, b"\x01")

    def test_packet_timeout_does_not_repeat_mutation(self):
        client, sock = self.client([socket.timeout()])
        with self.assertRaisesRegex(TimeoutError, "outcome is unknown"):
            client.request(tx.OP_CONFIG, tx.Config().pack())
        self.assertEqual(len(sock.sent), 1)

    def test_bad_ack_and_apply_error(self):
        for response in (b"\xa2", b"\xa2\x01\x00\x00", ack(tx.OP_TEMPLATE),
                         ack(tx.OP_ENABLE) + b"\x00"):
            with self.subTest(response=response):
                client, _ = self.client([response])
                with self.assertRaises(tx.ProtocolError):
                    client.request(tx.OP_ENABLE, b"\x01")
        client, sock = self.client([
            ack(tx.OP_TEMPLATE), ack(tx.OP_STATUS, body=status_payload(last_error=4)),
        ])
        with self.assertRaises(tx.DeviceError):
            client.apply(tx.OP_TEMPLATE, b"\x00" * 8)
        self.assertEqual(len(sock.sent), 2)

    def test_short_template_rejected_before_udp(self):
        with patch.object(tx.socket, "socket") as sock, patch("sys.stderr", new=io.StringIO()):
            self.assertEqual(tx.main(["template", "--hex", "00"]), 1)
            sock.assert_not_called()


def crc24_bitwise(data):
    crc = 0xFEDCBA
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc = ((crc << 1) ^ (0x5D6DCB if crc & 0x800000 else 0)) & 0xFFFFFF
    return crc


class FixtureTests(unittest.TestCase):
    def test_full_64_cycle_waveform_and_tss_independent_fss(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = tx.generate_fixture(Path(directory))
            self.assertEqual(manifest["duration_ns"], 320_000_000)
            self.assertEqual(len(manifest["frames"]), 128)
            self.assertEqual(manifest["selected_cycles"], list(range(2, 64, 4)))
            with Path(manifest["csv"]).open(newline="") as stream:
                edges = [(int(row["time_ns"]), int(row["level"]))
                         for row in csv.DictReader(stream)]
            times = [edge[0] for edge in edges]
            self.assertEqual(times, sorted(set(times)))
            self.assertEqual(edges[0], (0, 1))
            self.assertEqual(edges[-1], (320_000_000, 1))

            def level_at(timestamp):
                return edges[bisect.bisect_right(times, timestamp) - 1][1]

            for record in manifest["frames"]:
                with self.subTest(cycle=record["cycle"], frame_id=record["id"]):
                    frame = bytes.fromhex(record["frame_hex"])
                    fss = record["fss_ns"]
                    self.assertEqual(fss, record["cycle"] * 5_000_000 + record["id"] * 40_000)
                    self.assertEqual(fss - record["tss_ns"], record["tss_bits"] * 100)
                    self.assertEqual(level_at(record["tss_ns"] - 1), 1)
                    self.assertEqual(level_at(fss - 1), 0)
                    self.assertEqual(level_at(fss), 1)
                    self.assertEqual(level_at(fss + 199), 1)  # FSS plus BSS high.
                    self.assertEqual(level_at(fss + 200), 0)  # BSS low.
                    decoded = bytearray()
                    for index in range(len(frame)):
                        start = fss + 100 + index * 1000
                        self.assertEqual([level_at(start + 50), level_at(start + 150)], [1, 0])
                        value = 0
                        for bit in range(8):
                            value = (value << 1) | level_at(start + 250 + bit * 100)
                        decoded.append(value)
                    self.assertEqual(decoded, frame)
                    self.assertEqual(decoded[4] & 63, record["cycle"])
                    self.assertEqual(((decoded[0] & 7) << 8) | decoded[1], record["id"])
                    self.assertIn(record["id"], (4, 6))
                    self.assertEqual(int.from_bytes(frame[-3:], "big"), crc24_bitwise(frame[:-3]))
                    fes = fss + 100 + len(frame) * 1000
                    self.assertEqual([level_at(fes + 50), level_at(fes + 150),
                                      level_at(fes + 250)], [0, 1, 1])
                    self.assertEqual(record["end_ns"], fes + 200)
            template = Path(manifest["template"]).read_bytes()
            self.assertEqual(len(template), 26)
            self.assertEqual(template[1], 8)
            self.assertEqual(int.from_bytes(template[-3:], "big"), crc24_bitwise(template[:-3]))
            vcd = Path(manifest["vcd"]).read_text()
            self.assertIn("$timescale 1 ns $end", vcd)
            self.assertEqual(vcd.count("\n#"), len(edges))
            self.assertIn("#320000000\n1!", vcd)

    def test_invalid_fixture_cannot_silently_overlap_slots(self):
        with tempfile.TemporaryDirectory() as directory:
            for options in ({"slot_ns": 20_000}, {"payload_bytes": 17}, {"cycles": 65},
                            {"tss_lengths": ()}, {"tss_lengths": (0,)},
                            {"cycle_ns": 300_000}):
                with self.subTest(options=options), self.assertRaises(ValueError):
                    tx.generate_fixture(Path(directory), **options)


if __name__ == "__main__":
    unittest.main()
