"""Cycle-level checks of the actual static timing PIO source, without hardware.

Run: python3 -m unittest discover -s tests -p 'test_frame_gen_timing_pio.py'
The path model includes ideal two-clock input synchronizers. Electrical pad
propagation and DMA bus contention remain logic-analyzer measurements.
"""

import bisect
import itertools
import pathlib
import re
import unittest


SOURCE = pathlib.Path(__file__).resolve().parents[1] / "src/flexray_fss_timing.pio"
LEGACY_STREAMER = SOURCE.parents[1] / "tests/fixtures/flexray_bss_streamer_legacy.pio"
MASK32 = (1 << 32) - 1


def read_programs(source=SOURCE):
    programs = {}
    current = None
    c_sdk = False
    for raw in source.read_text().splitlines():
        line = raw.split(";", 1)[0].strip()
        if line.startswith("% c-sdk"):
            c_sdk = True
            continue
        if line.startswith("%}"):
            c_sdk = False
            continue
        if c_sdk:
            continue
        if not line:
            continue
        if line.startswith(".program "):
            current = {"instructions": [], "labels": {}, "constants": {}}
            programs[line.split()[1]] = current
        elif current is None:
            continue
        elif line.startswith(".define public "):
            _, _, name, value = line.split()
            current["constants"][name] = int(value, 0)
        elif line == ".wrap_target":
            current["wrap_target"] = len(current["instructions"])
        elif line == ".wrap":
            current["wrap"] = len(current["instructions"]) - 1
        elif line.startswith(".out "):
            current["shift_right"] = "right" in line.split()
        elif line.startswith(".in "):
            current["push_threshold"] = int(line.split()[-1])
        elif line.startswith("."):
            continue
        elif line.endswith(":"):
            current["labels"][line.removesuffix(":").removeprefix("public ")] = len(
                current["instructions"]
            )
        else:
            delay = re.search(r"\[(\d+)\]", line)
            cycles = 1 + (int(delay[1]) if delay else 0)
            operation = re.sub(r"\[\d+\]", "", line).strip()
            current["instructions"].append((operation.replace(",", "").split(), cycles))
    return programs


class Machine:
    """The small instruction subset used by the source above.

    Register changes are recorded at instruction execution. External IRQ input
    and pin functions are sampled on each execution cycle; device-level
    inter-SM propagation is intentionally outside this single-SM interpreter.
    """

    def __init__(self, program, *, x=MASK32, y=0, pin=lambda _: 1, fifo_depth=None,
                 status=None, external_irq=lambda _index, _time: False, sm_number=0,
                 aux_pin=lambda _time: 1, tx=(), wait_pin_delay=0):
        self.program = program
        self.reg = {"x": x, "y": y, "isr": 0, "osr": 0}
        self.tx = list(tx)
        self.out_count = 32
        self.pulls = []
        self.pin = pin
        self.wait_pin_delay = wait_pin_delay
        # WAIT JMPPIN uses the independently configured RXD; capture maps
        # WAIT PIN 0 to TXEN. Legacy programs use RXD for both sources.
        self.in_pin = aux_pin if any(op[:3] == ["wait", "0", "jmppin"]
                                     for op, _ in program["instructions"]) else pin
        self.status = status
        self.external_irq = external_irq
        self.sm_number = sm_number
        self.fifo_depth = fifo_depth
        self.pc = 0
        self.time = 0
        self.irq = {}
        self.rx = []
        self.dropped = 0
        self.pushes = []
        self.decrements = []
        self.pin_writes = []
        self.wait_successes = []
        self.irq_writes = []
        self.irq_events = []
        self.status_reads = []
        self.txen_writes = []
        self.in_count = 0

    def number(self, token):
        return self.program["constants"].get(token, int(token) if token.isdigit() else 0)

    def step(self):
        op, cycles = self.program["instructions"][self.pc]
        if "side" in op:
            pos = op.index("side")
            self.txen_writes.append((self.time, int(op[pos + 1])))
            op = op[:pos]
        next_pc = self.pc + 1
        jump_taken = False
        if op[0] == "jmp":
            if len(op) == 2:
                take = True
            elif op[1] == "pin":
                take = bool(self.pin(self.time))
            elif op[1] == "!x":
                take = self.reg["x"] == 0
            elif op[1] == "x!=y":
                take = self.reg["x"] != self.reg["y"]
            elif op[1] in ("x--", "y--"):
                register = op[1][0]
                old = self.reg[register]
                self.reg[register] = (old - 1) & MASK32
                if register == "x":
                    self.decrements.append(self.time)
                take = old != 0
            else:
                raise AssertionError(f"unsupported condition {op}")
            if take:
                jump_taken = True
                next_pc = self.program["labels"][op[-1]]
        elif op[0] == "set":
            value = self.number(op[2])
            if op[1] == "pins":
                self.pin_writes.append((self.time, value))
            else:
                self.reg[op[1]] = value
        elif op[0] == "mov":
            if op[2] == "status":
                value = MASK32 if (self.status(self.time) if self.status else
                                   not self.tx) else 0
                self.status_reads.append((self.time, value))
            elif op[2] == "pins":
                value = int(self.pin(self.time))
            elif op[2] in ("null", "!null"):
                value = MASK32 if op[2] == "!null" else 0
            else:
                value = self.reg[op[2]]
            if op[1] == "pins":
                self.pin_writes.append((self.time, value))
            else:
                self.reg[op[1]] = value
            if op[1] == "isr":
                self.in_count = 0
        elif op[0] == "pull":
            if not self.tx:
                self.time += 1
                return
            self.reg["osr"] = self.tx.pop(0)
            self.out_count = 0
            self.pulls.append(self.time)
        elif op[0] == "out":
            if self.out_count >= 32:
                if not self.tx:
                    self.time += 1
                    return
                self.reg["osr"] = self.tx.pop(0)
                self.out_count = 0
            count = self.number(op[2])
            if self.program.get("shift_right", False):
                value = self.reg["osr"] & ((1 << count) - 1)
                self.reg["osr"] >>= count
            else:
                value = self.reg["osr"] >> (32 - count)
                self.reg["osr"] = (self.reg["osr"] << count) & MASK32
            self.out_count += count
            if op[1] == "pins":
                self.pin_writes.append((self.time, value))
            else:
                self.reg[op[1]] = value
        elif op[0] == "in":
            if (self.fifo_depth is not None and len(self.rx) >= self.fifo_depth):
                self.time += 1
                return
            assert op[1:] == ["pins", "1"]
            self.reg["isr"] = ((self.reg["isr"] << 1) | int(self.pin(self.time))) & MASK32
            self.in_count += 1
            if self.in_count == self.program.get("push_threshold", 8):
                self.rx.append(self.reg["isr"])
                self.pushes.append((self.time, self.reg["isr"]))
                self.reg["isr"] = self.in_count = 0
        elif op[0] == "nop":
            pass
        elif op[0] == "push":
            self.pushes.append((self.time, self.reg["isr"]))
            if self.fifo_depth is None or len(self.rx) < self.fifo_depth:
                self.rx.append(self.reg["isr"])
            else:
                self.dropped += 1
            self.reg["isr"] = 0
        elif op[0] == "irq":
            scope = "prev" if "prev" in op else "next" if "next" in op else "self"
            if scope != "self":
                op = [item for item in op if item != scope]
            index = self.number(op[2])
            if op[-1] == "rel":
                index = (index & ~3) | ((index + self.sm_number) & 3)
            value = op[1] == "set"
            self.irq[index] = value
            if value:
                self.irq_writes.append((self.time, index))
            self.irq_events.append((self.time, scope, index, value))
        elif op[0] == "wait":
            if op[2] == "irq":
                index = self.number(op[3])
                value = bool(self.irq.get(index, False) or self.external_irq(index, self.time))
            elif op[2] == "jmppin":
                value = bool(self.pin(self.time))
            else:
                assert op[2:] == ["pin", "0"]
                value = bool(self.in_pin(self.time - self.wait_pin_delay))
            if value == bool(int(op[1])):
                if op[2] == "irq" and op[1] == "1":
                    self.irq[index] = False
                self.wait_successes.append(self.time)
            else:
                next_pc = self.pc
                cycles = 1  # WAIT delay applies only after success; poll every clock
        else:
            raise AssertionError(f"unsupported instruction {op}")
        self.pc = (self.program.get("wrap_target", 0)
                   if self.pc == self.program.get("wrap", len(self.program["instructions"]) - 1)
                   and not jump_taken and next_pc == self.pc + 1 else next_pc)
        self.time += cycles

    def run(self, end, event=None):
        while self.time < end:
            if event:
                event(self)
            self.step()

class Waveform:
    def __init__(self, writes=(), delay=0):
        self.writes = list(writes)
        self.starts = [t + delay for t, _ in self.writes]
        self.levels = [v for _, v in self.writes]

    def pin(self, time):
        i = bisect.bisect_right(self.starts, time) - 1
        return self.levels[i] if i >= 0 else 1


def frame_edges(start, tss, data):
    bits = [0] * tss + [1]
    for byte in data:
        bits += [1, 0] + [(byte >> b) & 1 for b in range(7, -1, -1)]
    bits += [0, 1]
    return [(start + 15 * i, bit) for i, bit in enumerate(bits)]


def descriptors(tss, length):
    spans = [(0, 15 * tss), (1, 30)]
    for i in range(length):
        spans.append((0, 150 if i + 1 == length else 135))
        if i + 1 < length:
            spans.append((1, 15))
    spans.append((1, 165))
    return [((clocks - 5) << 1) | level for level, clocks in spans] + [1]


def injector_words(raw):
    raw += b'\0' * (-len(raw) % 4)
    return [
        int.from_bytes(raw[i:i + 4], 'big') for i in range(0, len(raw), 4)]


class TimingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.programs = read_programs()
        cls.inducer = read_programs(SOURCE.with_name('flexray_frame_gen_output.pio'))['flexray_frame_gen_inducer']
        cls.txen = read_programs(SOURCE.with_name("flexray_frame_gen_output.pio"))["flexray_frame_gen_txen"]
        cls.streamer = read_programs(SOURCE.with_name('flexray_bss_streamer.pio'))['flexray_bss_streamer']
        cls.forwarder = read_programs(SOURCE.with_name('flexray_forwarder_with_injector.pio'))['flexray_forwarder_with_injector']
        header = SOURCE.with_name('flexray_frame_gen.h').read_text()
        cls.streamer_entry = next(i for i, (op, _) in enumerate(cls.streamer['instructions']) if op == ['wait', '0', 'pin', '0'])
        cls.forwarder_entry = int(re.search(r'#define FLEXRAY_FRAME_GEN_FORWARDER_ENTRY (\d+)u', header)[1])

    def test_memory_and_original_idle_qualification(self):
        self.assertEqual(len(self.streamer['instructions']), 28)
        self.assertEqual(len(self.forwarder['instructions']), 22)
        self.assertEqual(len(self.inducer['instructions']), 12)
        self.assertEqual(len(self.programs['flexray_fss_capture']['instructions']) + 22, 32)
        self.assertEqual(len(self.programs['flexray_slot_pace']['instructions']) + 12 + 2 + 2, 32)
        self.assertEqual(len(self.txen['instructions']) + 28, 32)
        self.assertEqual(self.streamer['instructions'][6][0], ['jmp', 'pin', 'entry_point'])
        self.assertEqual(self.streamer['instructions'][self.streamer_entry], (['wait', '0', 'pin', '0'], 1))
        self.assertEqual(self.forwarder['instructions'][self.forwarder_entry], (['wait', '0', 'pin', '0'], 1))

    def test_static_receiver_matches_original_cycles(self):
        compact = self.streamer
        legacy = read_programs(LEGACY_STREAMER)['flexray_bss_streamer']
        self.assertEqual(len(legacy['instructions']), 30)
        self.assertEqual(len(compact['instructions']), 28)
        for length in (8, 10, 26, 262):
            data = bytes((i * 73 + 29) & 255 for i in range(length))
            wave = Waveform(frame_edges(500, 8, data) + frame_edges(5000 + length * 150, 8, data))
            old = Machine(legacy, pin=wave.pin)
            new = Machine(compact, pin=wave.pin)
            for m in (old, new): m.run(8000 + length * 300)
            self.assertEqual(old.rx, new.rx)
            self.assertEqual(old.irq_events, new.irq_events)
            self.assertEqual(old.txen_writes, new.txen_writes)

    def test_txen_writes_only_on_ownership_edges(self):
        control = Machine(self.txen, pin=Waveform([(0,1),(500,0),(2500,1),(5000,0),(7000,1)]).pin)
        control.run(10000)
        self.assertEqual(control.pin_writes, [(501,0),(2501,1),(5001,0),(7001,1)])
        self.assertFalse(control.txen_writes)  # No side-set writes during stalled WAIT.
        self.assertFalse(control.irq_events)
        self.assertFalse(control.pushes)

    def test_capture_rejects_shortened_bss_inside_long_frames(self):
        # An asynchronously observed BSS LOW can be shorter than 15 clocks.
        # Sampling idle only once per bit can alias past that LOW and report
        # another FSS inside an all-ones data run. No run here is 11 bits HIGH.
        capture_program = self.programs['flexray_fss_capture']
        for period_numerator in (5999, 5998, 5996, 5980):
            for low_clocks in (12, 13, 14, 15):
                with self.subTest(period_numerator=period_numerator, low_clocks=low_clocks):
                    start = 500
                    edges = [(start, 0), (start + 120, 1)]
                    # Slightly different source/receiver clocks make the BSS
                    # edge walk through all phases of the idle polling loop.
                    for i in range(262):
                        fall = start + 150 + i * period_numerator // 40
                        edges += [(fall, 0), (fall + low_clocks, 1)]
                    edges += [(fall + 135, 0), (fall + 150, 1)]
                    following = edges[-1][0] + 500
                    edges += frame_edges(following, 8, bytes([0x96]) * 8)
                    m = Machine(capture_program, pin=Waveform(edges).pin)
                    m.run(following + 2000)
                    markers = [t for t, scope, irq, value in m.irq_events if irq == 0 and value]
                    self.assertEqual(markers, [start + 121, following + 121])

    def test_capture_wait_has_one_clock_resolution_for_all_word_phases(self):
        # Two real frames with different TSS, idle before each one. FSS is
        # varied across all 32 sampler word positions; no /8 quantization.
        for phase in range(32):
            tss1, tss2 = 6 + phase % 10, 15 - phase % 10
            fss1, fss2 = 1000 + phase, 7000 + phase
            edges = frame_edges(fss1 - tss1 * 15, tss1, bytes(8))
            edges += frame_edges(fss2 - tss2 * 15, tss2, bytes(8))
            capture = Machine(self.programs['flexray_fss_capture'], pin=Waveform(edges).pin)
            capture.run(fss2 + 100)
            irqs = [t for t, i in capture.irq_writes if i == 0]
            self.assertEqual(irqs, [fss1 + 1, fss2 + 1])
            marker = Waveform([(0, 0)] + capture.txen_writes, delay=2)
            sampler = Machine(self.programs['flexray_fss_sampler'], pin=marker.pin)
            sampler.run(fss2 + 100)
            bits = ''.join(f'{value:032b}' for value in sampler.rx)
            starts = [m.start() for m in re.finditer('1{15}', bits)]
            self.assertEqual(starts, [fss1 + 3, fss2 + 3])

    def test_capture_requalifies_after_short_low_and_suppresses_local_echo(self):
        # Echo lasts through TXEN low. Short LOW followed by HIGH must return
        # to idle qualification, not accept another falling edge immediately.
        edges = [(0, 1), (400, 0), (401, 1), (440, 0), (500, 1)]
        edges += frame_edges(900, 8, bytes(8))
        capture = Machine(self.programs['flexray_fss_capture'], pin=Waveform(edges).pin)
        capture.run(2200)
        self.assertEqual([t for t, i in capture.irq_writes if i == 0], [1021])
        echo = frame_edges(400, 8, bytes(8)) + frame_edges(3000, 6, bytes(8))
        capture = Machine(self.programs['flexray_fss_capture'], pin=Waveform(echo).pin,
                          aux_pin=lambda t: not (390 <= t < 1900))
        capture.run(4300)
        self.assertEqual([t for t, i in capture.irq_writes if i == 0], [3091])

    def path(self, tss, raw, phase=0):
        fss, cycle = 1000, 8000
        capture = Machine(self.programs['flexray_fss_capture'],
                          pin=Waveform(frame_edges(fss - 90, 6, bytes(8)), delay=2).pin)
        capture.run(2200)
        fss_irq = next(t for t, i in capture.irq_writes if i == 0)
        pace = Machine(self.programs['flexray_slot_pace'],
                       x=cycle - 15 * tss - 15 + (tss & 1) + phase, y=MASK32,
                       external_irq=lambda i, t: i == 0 and t == fss_irq + 1,
                       tx=[6000 - 9] * 4)
        pace.run(fss + cycle + 500)
        beats = [t + 1 for t, scope, i, v in pace.irq_events if scope == 'self' and i == 3 and v]
        inducer = Machine(self.inducer, tx=descriptors(tss, len(raw)),
                          external_irq=lambda i, t: i == 3 and t in beats)
        inducer.pc = self.inducer['labels']['wait_pace']
        end = fss + cycle + len(raw) * 150 + 500
        inducer.run(end)
        internal = Waveform(inducer.pin_writes, delay=2)
        # The preceding slot preloads the original injector FIFO. Its first
        # instruction is WAIT TSS, so this causes no early TXD/TXEN edge.
        # Bench TSS widths fit a one-clock WAIT PIN input latency relative
        # to JMP PIN. This changes odd/even dominant-loop phase; common pad
        # delay is still outside this model and requires hardware calibration.
        forwarder = Machine(self.forwarder, pin=internal.pin, wait_pin_delay=1,
                            tx=[len(raw) - 1] + injector_words(raw))
        forwarder.pc = self.forwarder_entry
        forwarder.run(end)
        ownership = Waveform([(0, 1)] + inducer.txen_writes, delay=2)
        streamer = Machine(self.txen, pin=ownership.pin)
        streamer.run(end)
        return pace, inducer, forwarder, streamer

    def test_original_injector_bytes_tss_fss_and_txen(self):
        # All allowed TSS durations, both divider phases, short/full packets.
        for length, tss, phase in itertools.product((8, 26, 262), range(6, 16), (-1, 0, 1)):
            raw = bytes((i * 73 + 29) & 255 for i in range(length))
            pace, inducer, forwarder, streamer = self.path(tss, raw, phase)
            with self.subTest(length=length, tss=tss, phase=phase):
                self.assertEqual(len(forwarder.pulls), 1)
                self.assertFalse(streamer.rx)
                # The first physical rising edge after TSS is the FSS edge.
                low = next(t for t, v in forwarder.pin_writes if v == 0)
                fss = next(t for t, v in forwarder.pin_writes if v == 1 and t > low)
                self.assertEqual(fss, 9001 + phase)
                self.assertEqual(fss - low, tss * 15 - (tss & 1))
                output = Waveform(forwarder.pin_writes)
                # Decode using the injector's actual first data bit and BSS.
                for b, expected in enumerate(raw):
                    start = fss + 45 + 150 * b
                    value = sum(output.pin(start + 7 + 15 * bit) << (7 - bit) for bit in range(8))
                    self.assertEqual(value, expected)
                enable = next(t for t, v in streamer.pin_writes if v == 0)
                release = next(t for t, v in streamer.pin_writes if v == 1 and t > enable)
                lock = next(t for t, i in inducer.irq_writes if i == 7)
                unlock = next(t for t, scope, i, v in inducer.irq_events if i == 7 and not v)
                self.assertLess(lock, enable)
                self.assertLessEqual(enable, low)
                self.assertGreater(release, fss + 15 + length * 150 + 30)
                self.assertGreaterEqual(release, unlock)

    def test_original_programs_send_adjacent_frames_without_sm_restart(self):
        first, second = bytes(range(26)), bytes(range(128, 154))
        def load_at_times(packets):
            next_packet = 0
            def load(machine):
                nonlocal next_packet
                if next_packet < 2 and machine.time >= (2000, 12000)[next_packet]:
                    machine.tx.extend(packets[next_packet])
                    next_packet += 1
            return load
        signal = Machine(self.inducer,
                         external_irq=lambda i, t: i == 3 and t in (1000, 7000, 13000, 19000))
        signal.run(22000, load_at_times([descriptors(8, 26)] * 2))
        internal = Waveform(signal.pin_writes, delay=2)
        forwarder = Machine(self.forwarder, pin=internal.pin, wait_pin_delay=1)
        forwarder.pc = self.forwarder_entry
        forwarder.run(22000, load_at_times([[25] + injector_words(raw) for raw in (first, second)]))
        self.assertEqual(len(forwarder.pulls), 2)
        self.assertEqual([t for t, i in signal.irq_writes if i == 7], [7003, 13003])
        self.assertEqual(len([t for t, i in signal.irq_writes if i == 2]), 2)
        self.assertFalse([t for t, i in signal.irq_writes if i in (3, 4)])
        self.assertEqual(len([t for t, scope, i, v in signal.irq_events if i == 7 and not v]), 2)
        output = Waveform(forwarder.pin_writes)
        fss_times = []
        for beat, raw in zip((7000, 13000), (first, second)):
            low = next(t for t, v in forwarder.pin_writes if t >= beat and not v)
            fss = next(t for t, v in forwarder.pin_writes if t > low and v)
            fss_times.append(fss)
            for byte, expected in enumerate(raw):
                start = fss + 45 + 150 * byte
                actual = sum(output.pin(start + 7 + 15 * bit) << (7 - bit) for bit in range(8))
                self.assertEqual(actual, expected)
        self.assertEqual(fss_times[1] - fss_times[0], 6000)

    def test_single_signal_prepared_in_previous_slot_fires_once_on_next_pace(self):
        for prepared_at in (1005, 2000, 4000, 6990):
            with self.subTest(prepared_at=prepared_at):
                signal = Machine(self.inducer,
                                 external_irq=lambda i, t: i == 3 and t in (1000, 7000, 13000, 19000))
                prepared = False
                def prepare(machine):
                    nonlocal prepared
                    if not prepared and machine.time >= prepared_at:
                        machine.tx.extend(descriptors(8, 26))
                        prepared = True
                signal.run(22000, prepare)
                self.assertEqual(signal.pin_writes[0], (7004, 0))
                self.assertEqual([t for t, i in signal.irq_writes if i == 7], [7003])
                self.assertEqual(signal.pin_writes[-1][1], 1)
                self.assertFalse(signal.tx)
                self.assertFalse(signal.pushes)  # no token / selector DMA
        empty = Machine(self.inducer,
                        external_irq=lambda i, t: i == 3 and t in (1000, 7000, 13000))
        empty.run(15000)
        self.assertFalse(empty.pin_writes)
        self.assertFalse(empty.irq_writes)

    def test_cycle_envelopes_have_exact_static_spacing_and_one_shaping_correction(self):
        n, slot, cycle, reference = 8, 1800, 20000, 6
        def envelope(correction=0):
            words = []
            for current in list(range(reference, n + 1)) + list(range(1, reference)):
                if current != n:
                    words.append(slot - 9)
                else:
                    guards = [((g + 1) * slot // 64 - g * slot // 64) - 4 for g in range(64)]
                    words += [0] + guards + [0, cycle - n * slot - 13 + correction]
            return words
        # One +21-clock phase shaping at the FIRST cycle boundary, then
        # nominal C forever, as the reset DMA restores the table word.
        tables = envelope(21) + envelope() * 15
        pace = Machine(self.programs['flexray_slot_pace'], x=50, y=MASK32,
                       tx=tables, external_irq=lambda i, t: i == 0 and t == 300)
        pace.run(200000)
        beats = [t for t, scope, i, v in pace.irq_events if scope == 'self' and i == 3 and v]
        for i, (a, b) in enumerate(zip(beats, beats[1:])):
            current = (reference - 1 + i) % n + 1
            expected = slot if current != n else cycle - (n - 1) * slot
            if i == n - reference:
                expected += 21
            self.assertEqual(b - a, expected)
        # Exact count within each 5-ms-style envelope; no extra tail beats.
        self.assertEqual([(~v) & MASK32 for _, v in pace.pushes], list(range(len(beats))))
        stalled_cpu = Machine(self.programs['flexray_slot_pace'], x=50, y=MASK32,
                              tx=tables, fifo_depth=4,
                              external_irq=lambda i, t: i == 0 and t == 300)
        stalled_cpu.run(200000)
        self.assertEqual(pace.irq_events, stalled_cpu.irq_events)
        self.assertGreater(stalled_cpu.dropped, 0)

    def test_shaping_word_is_read_after_phase_window_even_for_last_reference(self):
        for n, reference in ((2, 1), (2, 2), (8, 6), (9, 9)):
            slot, cycle = 1800, n * 1800 + 4000
            ram = []
            for current in list(range(reference, n + 1)) + list(range(1, reference)):
                if current != n:
                    ram.append(slot - 9)
                else:
                    ram += [0] + [((g + 1) * slot // 64 - g * slot // 64) - 4 for g in range(64)] + [0]
                    tail_index = len(ram)
                    ram.append(cycle - n * slot - 13)
            nominal = ram[tail_index]
            pace = Machine(self.programs['flexray_slot_pace'], x=50, y=MASK32,
                           external_irq=lambda i, t: i == 0 and t == 300)
            read_index, publication = 0, None
            reads = []
            def dma(machine):
                nonlocal read_index, publication
                beats = [t for t, scope, i, v in machine.irq_events if scope == 'self' and i == 3 and v]
                if publication is None and beats and machine.time >= beats[0] + 1000:
                    ram[tail_index] = nominal + 21
                    publication = machine.time
                # Five queued words conservatively include four TX FIFO
                # words plus one OSR word of possible automatic prefetch.
                while len(machine.tx) < 5:
                    if read_index == tail_index:
                        reads.append((machine.time, ram[read_index]))
                    machine.tx.append(ram[read_index])
                    read_index += 1
                    if read_index == len(ram):
                        # Actual chain: reset shaping word, reload this block.
                        ram[tail_index] = nominal
                        read_index = 0
            pace.run(cycle * 3, dma)
            self.assertIsNotNone(publication)
            self.assertGreater(reads[0][0], publication)
            self.assertEqual(reads[0][1], nominal + 21)
            self.assertTrue(all(v == nominal for _, v in reads[1:]))

    def test_echo_exclusion_and_real_next_slot_with_unchanged_streamer(self):
        _, inducer, tx, local = self.path(8, bytes(range(26)))
        txd = Waveform(tx.pin_writes)
        enable = next(t for t, v in local.pin_writes if v == 0)
        release = next(t for t, v in local.pin_writes if v == 1 and t > enable)
        lock = next(t for t, i in inducer.irq_writes if i == 7)
        unlock = next(t for t, _, i, v in inducer.irq_events if i == 7 and not v)
        real_start = release + 350
        data = bytes(range(8))
        real = Waveform(frame_edges(real_start, 6, data))
        for delay in range(0, 61, 3):
            def rxd(t):
                if t >= real_start:
                    return real.pin(t)
                return txd.pin(t - delay) if enable + delay <= t < release + delay else 1
            capture = Machine(self.programs['flexray_fss_capture'], pin=rxd,
                              aux_pin=lambda t: not (enable <= t < release))
            capture.run(real_start + 1500)
            self.assertEqual([t for t, i in capture.irq_writes if i == 0], [real_start + 91])
            receiver = Machine(self.streamer, pin=rxd,
                               external_irq=lambda i, t: i == 7 and lock <= t < unlock)
            receiver.run(real_start + 1500)
            self.assertEqual(bytes(v & 255 for v in receiver.rx), data)
            self.assertTrue(all(t >= real_start for t, v in receiver.txen_writes if v == 0))

    def test_phase_probe_expected_edge_and_missing_reference(self):
        for tss, offset in itertools.product(range(6, 16), (-9, 0, 11)):
            pace, _, _, _ = self.path(tss, bytes(8))
            phase_irq = next(t for t, i in pace.irq_writes if i == 4)
            # Real external reference FSS=9000+offset. Capture and GPIO
            # synchronizers add five clocks from RXD pad to marker input.
            marker = Waveform([(0, 0), (9005 + offset, 1), (9020 + offset, 0)])
            phase = Machine(self.programs['flexray_fss_phase_sampler'], pin=marker.pin,
                            fifo_depth=8, external_irq=lambda i, t: i == 4 and t == phase_irq + 1)
            phase.run(9400)
            bits = ''.join(f'{v:032b}' for v in phase.rx)
            self.assertEqual(len(bits), 256)
            self.assertEqual(bits.count('1'), 15)
            self.assertEqual(bits.index('1'), tss * 15 - 19 - (tss & 1) + offset)
        missing = Machine(self.programs['flexray_fss_phase_sampler'], pin=lambda t: 0,
                          fifo_depth=8, external_irq=lambda i, t: i == 4 and t == 100)
        missing.run(1000)
        self.assertEqual(missing.rx, [0] * 8)


if __name__ == '__main__':
    unittest.main()
