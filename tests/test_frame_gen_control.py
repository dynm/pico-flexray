"""Compile the firmware's actual C scheduler with register/FIFO fakes.

The generated headers use the actual PIO sources. This tests
CPU command/ownership behavior; test_frame_gen_timing_pio.py tests PIO cycles.
"""
from pathlib import Path
import os
import subprocess
import sys
import tempfile
import unittest

from test_frame_gen_timing_pio import read_programs

ROOT = Path(__file__).resolve().parents[1]


class StaticControlTests(unittest.TestCase):
    def test_firmware_control_path(self):
        with tempfile.TemporaryDirectory(prefix="frame-gen-control-") as directory:
            out = Path(directory)
            # Compile the actual ingress parsing loops, stubbing only the MITM
            # destination. This checks USB/NCM framing without faking USB/lwIP.
            ingress = []
            for filename, name in (("panda_usb.c", "vendor_actions"), ("udp_server.c", "ncm_actions")):
                source = (ROOT / "src" / filename).read_text()
                start = source.index("while ((uint16_t)(len - off)")
                end = source.index("{", start)
                depth = 1
                end += 1
                while depth:
                    depth += (source[end] == "{") - (source[end] == "}")
                    end += 1
                ingress.append(f"static void {name}(const uint8_t *data, uint16_t len) {{\n"
                               "uint32_t off = 0;\n" + source[start:end] + "\n}\n")
            (out / "frame_gen_ingress.h").write_text("\n".join(ingress))
            for name in ("hardware/clocks.h", "hardware/dma.h", "hardware/irq.h",
                         "hardware/pio.h", "hardware/sync.h", "hardware/gpio.h",
                         "hardware/structs/sio.h", "pico/stdlib.h", "pico/time.h",
                         "pico/multicore.h"):
                header = out / name
                header.parent.mkdir(exist_ok=True, parents=True)
                header.write_text("/* supplied by frame_gen_control_fakes.h */\n")
            sections = out / "pico/platform/sections.h"
            sections.parent.mkdir(parents=True, exist_ok=True)
            sections.write_text("#define __no_inline_not_in_flash_func(name) name\n")
            for source in ("flexray_fss_timing.pio", "flexray_frame_gen_output.pio",
                           "flexray_bss_streamer.pio", "flexray_forwarder_with_injector.pio"):
                lines = ["#pragma once"]
                for name, program in read_programs(ROOT / "src" / source).items():
                    for constant, value in program["constants"].items():
                        lines.append(f"#define {name}_{constant} {value}u")
                    for label, offset in program["labels"].items():
                        lines.append(f"#define {name}_offset_{label} {offset}u")
                    size = len(program["instructions"])
                    lines.append(f"static const pio_program_t {name}_program = {{{size}u}};")
                    lines.append(f"static inline pio_sm_config {name}_program_get_default_config(uint offset) "
                                 "{ return (pio_sm_config){.start = offset}; }")
                contents = (ROOT / "src" / source).read_text()
                if "% c-sdk {" in contents:
                    lines.append(contents.split("% c-sdk {", 1)[1].split("%}", 1)[0])
                (out / (source + ".h")).write_text("\n".join(lines) + "\n")
            # Feed the demo CLI's actual wire payload into the real injector.
            action = subprocess.check_output(
                [sys.executable, str(ROOT / "inject_demo_client.py"),
                 "--dry-run", "payload", "52" * 18], text=True).strip()
            wire = bytes.fromhex(action)
            (out / "demo_inject_action.h").write_text(
                "static const uint8_t demo_inject_action[] = {" +
                ",".join(str(byte) for byte in wire) + "};\n")
            binary = out / "test-control"
            # Existing ARM frame logging has a different uint32_t printf
            # convention on the host. Only its CRC functions are used here.
            crc_object = out / "frame.o"
            crc_result = subprocess.run(
                [os.environ.get("CC", "cc"), "-std=c11", "-Wno-format",
                 "-O1", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                 "-I" + str(out), "-I" + str(ROOT / "src"), "-c",
                 str(ROOT / "src/flexray_frame.c"), "-o", str(crc_object)],
                capture_output=True, text=True)
            self.assertEqual(crc_result.returncode, 0, crc_result.stderr)
            command = [os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra", "-Werror",
                       "-Wstrict-prototypes", "-O1", "-fsanitize=address,undefined",
                       "-fno-omit-frame-pointer", "-I" + str(out),
                       "-I" + str(ROOT / "src"), str(ROOT / "tests/test_frame_gen_control.c"),
                       str(crc_object), str(ROOT / "src/flexray_slot_schedule.c"),
                       str(ROOT / "src/flexray_frame_gen_packet.c"),
                       str(ROOT / "src/rtt_protocol.c"), "-o", str(binary)]
            for filename, mode, expected in (
                ("test_frame_gen_control.c", 1, "frame generation C control tests passed"),
                ("test_frame_gen_bridge.c", 1, "independent MITM/frame generation integration tests passed"),
                ("test_frame_gen_bridge.c", 0, "four-channel MITM/secondary capture tests passed"),
                ("test_flexray_slot_schedule.c", 1, "flexray static schedule tests passed"),
                ("test_flexray_frame_gen_packet.c", 1, "flexray static wire tests passed"),
            ):
                with self.subTest(source=filename, frame_gen=mode):
                    invocation = [arg.replace("test_frame_gen_control.c", filename) for arg in command]
                    invocation.insert(1, f"-DFLEXRAY_FRAME_GEN={mode}")
                    compile_result = subprocess.run(invocation, capture_output=True, text=True)
                    self.assertEqual(compile_result.returncode, 0, compile_result.stderr)
                    result = subprocess.run([str(binary)], capture_output=True, text=True)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertIn(expected, result.stdout)


if __name__ == "__main__":
    unittest.main()
