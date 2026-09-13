"""Malformed RX waveforms, not electrical bench validation.

Run: python3 -m unittest discover -s tests -p 'test_frame_gen_streamer_mutations.py'
Both programs are interpreted directly from their PIO source. Recovery bytes
are also checked against an independent sentinel, not only against each other.
"""
import random
import unittest

from test_frame_gen_timing_pio import Machine, Waveform, SOURCE, LEGACY_STREAMER, read_programs, frame_edges


class StreamerMutationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.old = read_programs(LEGACY_STREAMER)['flexray_bss_streamer']
        cls.new = read_programs(SOURCE.with_name('flexray_bss_streamer.pio'))['flexray_bss_streamer']
        cls.sentinel = bytes([0x00, 0xff, 0xaa, 0x55, 0x80, 0x01, 0x96, 0x69])

    def check_wave(self, prefix, recovery, *, partial_exit=0):
        wave = Waveform(prefix + frame_edges(recovery, 8, self.sentinel))
        machines = []
        for program in (self.old, self.new):
            m = Machine(program, pin=wave.pin)
            ends = []
            injected = False
            def observe(sm):
                nonlocal injected
                # Negative control: emulate a future, unsafe mid-byte abort.
                if partial_exit and not injected and sm.in_count == partial_exit:
                    sm.pc = program['labels']['frame_end']
                    injected = True
                if sm.pc == program['labels']['frame_end']:
                    ends.append((sm.time, sm.in_count))
            m.run(recovery + 2000, observe)
            if not partial_exit:
                self.assertTrue(ends)
                self.assertTrue(all(count == 0 for _, count in ends), ends)
            recovered = bytes(value & 255 for t, value in m.pushes if t >= recovery)
            machines.append((m, recovered))
        old, new = machines
        self.assertEqual(old[1], self.sentinel)
        if partial_exit:
            self.assertNotEqual(new[1], self.sentinel)
        else:
            self.assertEqual(new[1], self.sentinel)
            self.assertEqual(old[0].pushes, new[0].pushes)
            self.assertEqual(old[0].irq_events, new[0].irq_events)
            self.assertEqual(old[0].txen_writes, new[0].txen_writes)

    def test_cut_at_every_pio_clock_then_stuck_high_or_low(self):
        source = frame_edges(200, 8, bytes([0x96, 0x00, 0xff, 0xaa, 0x55, 0x81, 0x01, 0x69]))
        # Exhaust every clock, including TSS, FSS, BSS and every sampled bit.
        for cut in range(200, source[-1][0] + 1):
            for level in (0, 1):
                with self.subTest(cut=cut, level=level):
                    prefix = [(t, v) for t, v in source if t < cut]
                    prefix += [(cut, level), (cut + 200, 1)]
                    self.check_wave(prefix, cut + 800)

    def test_glitches_and_missing_bss(self):
        source = frame_edges(200, 8, bytes([0xaa, 0x55, 0x00, 0xff, 0x96, 0x69, 0x80, 0x01]))
        original = Waveform(source)
        for start in range(200, source[-1][0], 15):
            for width in (1, 7, 15, 31, 100):
                with self.subTest(start=start, width=width):
                    end = start + width
                    edges = [(t, v) for t, v in source if not start <= t <= end]
                    edges += [(start, 1 - original.pin(start)), (end, original.pin(end))]
                    self.check_wave(sorted(edges) + [(1800, 1)], 2500)

    def test_random_malformed_bursts_then_recovery(self):
        rng = random.Random(0x2350)
        for case in range(200):
            t = 200
            edges = []
            for _ in range(100):
                edges.append((t, rng.randrange(2)))
                t += rng.choice((1, 2, 7, 14, 15, 16, 31, 96, 165, 300))
            with self.subTest(case=case):
                self.check_wave(edges + [(t, 1)], t + 800)

    def test_frame_lengths_patterns_and_short_recovery_gaps(self):
        for length in (0, 1, 4, 5, 8, 26, 128, 262):
            for pattern in (0x00, 0xff, 0xaa, 0x55):
                source = frame_edges(200, 8, bytes([pattern]) * length)
                for gap in (180, 240, 600):
                    with self.subTest(length=length, pattern=pattern, gap=gap):
                        # The close probe can be missed by BOTH receivers while
                        # they requalify idle. Compare its complete trace, then
                        # require exact bytes from a second, well-spaced frame.
                        probe = frame_edges(source[-1][0] + gap, 8, self.sentinel)
                        self.check_wave(source + probe, probe[-1][0] + 800)

    def test_negative_control_partial_byte_abort_is_detected(self):
        source = frame_edges(200, 8, bytes([0xaa]))
        for count in range(1, 8):
            with self.subTest(count=count):
                self.check_wave(source, 1500, partial_exit=count)


if __name__ == '__main__':
    unittest.main()
