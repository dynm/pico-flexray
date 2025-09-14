#!/usr/bin/env python3
"""
FlexRay wire-encoding builder for Pico replay.

Input: frame id, cycle count, payload hex. Produces a 32-bit word stream where:
 - Idle is logic-1 (0xFFFFFFFF words)
 - TSS is 8 zeros (0x00)
 - FSS is one '1' bit
 - BSS is two bits: '1' then '0'
 - BSS is inserted before every 8-bit byte (header bytes, payload bytes, crc bytes)
 - Header CRC-11 and Frame CRC-24 match the C implementation (tables copied)

Example:
  python build_replay_payload.py --id 0x37 --cycle 0x3B --payload BFD8700991FFFFFF01F0F1FFFFFF8FBD

CSV input (id,cycle,payload_hex) also supported:
  python build_replay_payload.py --csv csv/frames.csv
"""

import argparse
import sys
from typing import List, Tuple

# --- CRC Tables (copied from src/flexray_crc_table.h) ---

FLEXRAY_CRC11_TABLE = [
    0x000, 0x385, 0x70A, 0x48F, 0x591, 0x614, 0x29B, 0x11E,
    0x0A7, 0x322, 0x7AD, 0x428, 0x536, 0x6B3, 0x23C, 0x1B9,
    0x14E, 0x2CB, 0x644, 0x5C1, 0x4DF, 0x75A, 0x3D5, 0x050,
    0x1E9, 0x26C, 0x6E3, 0x566, 0x478, 0x7FD, 0x372, 0x0F7,
    0x29C, 0x119, 0x596, 0x613, 0x70D, 0x488, 0x007, 0x382,
    0x23B, 0x1BE, 0x531, 0x6B4, 0x7AA, 0x42F, 0x0A0, 0x325,
    0x3D2, 0x057, 0x4D8, 0x75D, 0x643, 0x5C6, 0x149, 0x2CC,
    0x375, 0x0F0, 0x47F, 0x7FA, 0x6E4, 0x561, 0x1EE, 0x26B,
    0x538, 0x6BD, 0x232, 0x1B7, 0x0A9, 0x32C, 0x7A3, 0x426,
    0x59F, 0x61A, 0x295, 0x110, 0x00E, 0x38B, 0x704, 0x481,
    0x476, 0x7F3, 0x37C, 0x0F9, 0x1E7, 0x262, 0x6ED, 0x568,
    0x4D1, 0x754, 0x3DB, 0x05E, 0x140, 0x2C5, 0x64A, 0x5CF,
    0x7A4, 0x421, 0x0AE, 0x32B, 0x235, 0x1B0, 0x53F, 0x6BA,
    0x703, 0x486, 0x009, 0x38C, 0x292, 0x117, 0x598, 0x61D,
    0x6EA, 0x56F, 0x1E0, 0x265, 0x37B, 0x0FE, 0x471, 0x7F4,
    0x64D, 0x5C8, 0x147, 0x2C2, 0x3DC, 0x059, 0x4D6, 0x753,
    0x1F5, 0x270, 0x6FF, 0x57A, 0x464, 0x7E1, 0x36E, 0x0EB,
    0x152, 0x2D7, 0x658, 0x5DD, 0x4C3, 0x746, 0x3C9, 0x04C,
    0x0BB, 0x33E, 0x7B1, 0x434, 0x52A, 0x6AF, 0x220, 0x1A5,
    0x01C, 0x399, 0x716, 0x493, 0x58D, 0x608, 0x287, 0x102,
    0x369, 0x0EC, 0x463, 0x7E6, 0x6F8, 0x57D, 0x1F2, 0x277,
    0x3CE, 0x04B, 0x4C4, 0x741, 0x65F, 0x5DA, 0x155, 0x2D0,
    0x227, 0x1A2, 0x52D, 0x6A8, 0x7B6, 0x433, 0x0BC, 0x339,
    0x280, 0x105, 0x58A, 0x60F, 0x711, 0x494, 0x01B, 0x39E,
    0x4CD, 0x748, 0x3C7, 0x042, 0x15C, 0x2D9, 0x656, 0x5D3,
    0x46A, 0x7EF, 0x360, 0x0E5, 0x1FB, 0x27E, 0x6F1, 0x574,
    0x583, 0x606, 0x289, 0x10C, 0x012, 0x397, 0x718, 0x49D,
    0x524, 0x6A1, 0x22E, 0x1AB, 0x0B5, 0x330, 0x7BF, 0x43A,
    0x651, 0x5D4, 0x15B, 0x2DE, 0x3C0, 0x045, 0x4CA, 0x74F,
    0x6F6, 0x573, 0x1FC, 0x279, 0x367, 0x0E2, 0x46D, 0x7E8,
    0x71F, 0x49A, 0x015, 0x390, 0x28E, 0x10B, 0x584, 0x601,
    0x7B8, 0x43D, 0x0B2, 0x337, 0x229, 0x1AC, 0x523, 0x6A6,
]

FLEXRAY_CRC11_4BIT_TABLE = [
    0x000, 0x385, 0x70A, 0x48F, 0x591, 0x614, 0x29B, 0x11E,
    0x0A7, 0x322, 0x7AD, 0x428, 0x536, 0x6B3, 0x23C, 0x1B9,
]

FLEXRAY_CRC24_TABLE = [
    0x000000, 0x5D6DCB, 0xBADB96, 0xE7B65D, 0x28DAE7, 0x75B72C, 0x920171, 0xCF6CBA,
    0x51B5CE, 0x0CD805, 0xEB6E58, 0xB60393, 0x796F29, 0x2402E2, 0xC3B4BF, 0x9ED974,
    0xA36B9C, 0xFE0657, 0x19B00A, 0x44DDC1, 0x8BB17B, 0xD6DCB0, 0x316AED, 0x6C0726,
    0xF2DE52, 0xAFB399, 0x4805C4, 0x15680F, 0xDA04B5, 0x87697E, 0x60DF23, 0x3DB2E8,
    0x1BBAF3, 0x46D738, 0xA16165, 0xFC0CAE, 0x336014, 0x6E0DDF, 0x89BB82, 0xD4D649,
    0x4A0F3D, 0x1762F6, 0xF0D4AB, 0xADB960, 0x62D5DA, 0x3FB811, 0xD80E4C, 0x856387,
    0xB8D16F, 0xE5BCA4, 0x020AF9, 0x5F6732, 0x900B88, 0xCD6643, 0x2AD01E, 0x77BDD5,
    0xE964A1, 0xB4096A, 0x53BF37, 0x0ED2FC, 0xC1BE46, 0x9CD38D, 0x7B65D0, 0x26081B,
    0x3775E6, 0x6A182D, 0x8DAE70, 0xD0C3BB, 0x1FAF01, 0x42C2CA, 0xA57497, 0xF8195C,
    0x66C028, 0x3BADE3, 0xDC1BBE, 0x817675, 0x4E1ACF, 0x137704, 0xF4C159, 0xA9AC92,
    0x941E7A, 0xC973B1, 0x2EC5EC, 0x73A827, 0xBCC49D, 0xE1A956, 0x061F0B, 0x5B72C0,
    0xC5ABB4, 0x98C67F, 0x7F7022, 0x221DE9, 0xED7153, 0xB01C98, 0x57AAC5, 0x0AC70E,
    0x2CCF15, 0x71A2DE, 0x961483, 0xCB7948, 0x0415F2, 0x597839, 0xBECE64, 0xE3A3AF,
    0x7D7ADB, 0x201710, 0xC7A14D, 0x9ACC86, 0x55A03C, 0x08CDF7, 0xEF7BAA, 0xB21661,
    0x8FA489, 0xD2C942, 0x357F1F, 0x6812D4, 0xA77E6E, 0xFA13A5, 0x1DA5F8, 0x40C833,
    0xDE1147, 0x837C8C, 0x64CAD1, 0x39A71A, 0xF6CBA0, 0xABA66B, 0x4C1036, 0x117DFD,
    0x6EEBCC, 0x338607, 0xD4305A, 0x895D91, 0x46312B, 0x1B5CE0, 0xFCEABD, 0xA18776,
    0x3F5E02, 0x6233C9, 0x858594, 0xD8E85F, 0x1784E5, 0x4AE92E, 0xAD5F73, 0xF032B8,
    0xCD8050, 0x90ED9B, 0x775BC6, 0x2A360D, 0xE55AB7, 0xB8377C, 0x5F8121, 0x02ECEA,
    0x9C359E, 0xC15855, 0x26EE08, 0x7B83C3, 0xB4EF79, 0xE982B2, 0x0E34EF, 0x535924,
    0x75513F, 0x283CF4, 0xCF8AA9, 0x92E762, 0x5D8BD8, 0x00E613, 0xE7504E, 0xBA3D85,
    0x24E4F1, 0x79893A, 0x9E3F67, 0xC352AC, 0x0C3E16, 0x5153DD, 0xB6E580, 0xEB884B,
    0xD63AA3, 0x8B5768, 0x6CE135, 0x318CFE, 0xFEE044, 0xA38D8F, 0x443BD2, 0x195619,
    0x878F6D, 0xDAE2A6, 0x3D54FB, 0x603930, 0xAF558A, 0xF23841, 0x158E1C, 0x48E3D7,
    0x599E2A, 0x04F3E1, 0xE345BC, 0xBE2877, 0x7144CD, 0x2C2906, 0xCB9F5B, 0x96F290,
    0x082BE4, 0x55462F, 0xB2F072, 0xEF9DB9, 0x20F103, 0x7D9CC8, 0x9A2A95, 0xC7475E,
    0xFAF5B6, 0xA7987D, 0x402E20, 0x1D43EB, 0xD22F51, 0x8F429A, 0x68F4C7, 0x35990C,
    0xAB4078, 0xF62DB3, 0x119BEE, 0x4CF625, 0x839A9F, 0xDEF754, 0x394109, 0x642CC2,
    0x4224D9, 0x1F4912, 0xF8FF4F, 0xA59284, 0x6AFE3E, 0x3793F5, 0xD025A8, 0x8D4863,
    0x139117, 0x4EFCDC, 0xA94A81, 0xF4274A, 0x3B4BF0, 0x66263B, 0x819066, 0xDCFDAD,
    0xE14F45, 0xBC228E, 0x5B94D3, 0x06F918, 0xC995A2, 0x94F869, 0x734E34, 0x2E23FF,
    0xB0FA8B, 0xED9740, 0x0A211D, 0x574CD6, 0x98206C, 0xC54DA7, 0x22FBFA, 0x7F9631,
]


def calculate_header_crc11(header_bytes_first_three: bytes) -> int:
    """Replicates calculate_flexray_header_crc from C code.
    Expects header[0], header[1], header[2] with header[2] containing only (payload_len<<1) before CRC bits are set.
    Returns 11-bit CRC as int.
    """
    if len(header_bytes_first_three) != 3:
        raise ValueError("header_bytes_first_three must be exactly 3 bytes")
    b0, b1, b2 = header_bytes_first_three
    data_word = ((b0 & 0b11111) << 16) | (b1 << 8) | b2
    data_word >>= 1

    crc = 0x1A
    byte0 = (data_word >> 12) & 0xFF
    index = ((crc >> 3) & 0xFF) ^ byte0
    crc = ((crc << 8) & 0x7FF) ^ FLEXRAY_CRC11_TABLE[index]

    byte1 = (data_word >> 4) & 0xFF
    index = ((crc >> 3) & 0xFF) ^ byte1
    crc = ((crc << 8) & 0x7FF) ^ FLEXRAY_CRC11_TABLE[index]

    last_bits = data_word & 0xF
    tbl_idx = ((crc >> 7) & 0xF) ^ last_bits
    crc = ((crc << 4) & 0x7FF) ^ FLEXRAY_CRC11_4BIT_TABLE[tbl_idx]
    return crc & 0x7FF


def calculate_frame_crc24(data: bytes) -> int:
    """Replicates calculate_flexray_frame_crc from C code.
    Initial value 0xFEDCBA, LUT-based, returns 24-bit int.
    """
    crc = 0xFEDCBA
    for byte in data:
        idx = ((crc >> 16) ^ byte) & 0xFF
        crc = ((crc << 8) & 0xFFFFFF) ^ FLEXRAY_CRC24_TABLE[idx]
    return crc & 0xFFFFFF


def build_header(indicators: int, frame_id: int, cycle_count: int, payload_len_bytes: int) -> bytes:
    if payload_len_bytes % 2 != 0:
        raise ValueError(f"FlexRay payload length must be even (words), length: {payload_len_bytes}")
    payload_len_words = payload_len_bytes // 2

    byte0 = ((indicators & 0x1F) << 3) | ((frame_id >> 8) & 0x07)
    byte1 = frame_id & 0xFF
    byte2 = (payload_len_words << 1) & 0xFE  # CRC bit10 not set yet

    crc11 = calculate_header_crc11(bytes([byte0, byte1, byte2]))
    byte2 = ((payload_len_words << 1) & 0xFE) | ((crc11 >> 10) & 0x01)
    byte3 = (crc11 >> 2) & 0xFF
    byte4 = ((crc11 & 0x03) << 6) | (cycle_count & 0x3F)
    return bytes([byte0, byte1, byte2, byte3, byte4])


def bytes_to_bits_msb_first(data: bytes) -> List[int]:
    bits: List[int] = []
    for byte in data:
        for i in range(7, -1, -1):
            bits.append((byte >> i) & 1)
    return bits


def pack_bits_to_words_msb_first(bits: List[int]) -> List[int]:
    words: List[int] = []
    # pad to 32-bit boundary with idle '1's
    if len(bits) % 32 != 0:
        pad = 32 - (len(bits) % 32)
        bits = bits + [1] * pad
    for i in range(0, len(bits), 32):
        word = 0
        for b in bits[i:i+32]:
            word = ((word << 1) | (b & 1)) & 0xFFFFFFFF
        words.append(word)
    return words


def build_wire_words(indicators: int, frame_id: int, cycle_count: int, payload_hex: str,
                     idle_prefix_words: int = 8, total_words: int = 64) -> List[int]:
    # Parse payload
    payload_hex = payload_hex.strip().replace(" ", "")
    if len(payload_hex) % 2 != 0:
        raise ValueError("Payload hex must have even length")
    payload = bytes.fromhex(payload_hex)

    # Header
    header = build_header(indicators, frame_id, cycle_count, len(payload))
    # Frame CRC24 over header+payload
    crc24 = calculate_frame_crc24(header + payload)
    crc_bytes = bytes([(crc24 >> 16) & 0xFF, (crc24 >> 8) & 0xFF, crc24 & 0xFF])

    # Build bitstream
    bits: List[int] = []

    # Idle prefix
    bits += [1] * (idle_prefix_words * 32)

    # TSS: 8 zeros
    bits += [0] * 8

    # FSS: 1 high bit
    bits.append(1)

    # Sequence of bytes to transmit: header(5), payload, crc(3)
    full_bytes = header + payload + crc_bytes

    # For every byte, insert BSS then 8 data bits (MSB-first)
    for byte in full_bytes:
        bits += [1, 0]  # BSS per byte
        for i in range(7, -1, -1):
            bits.append((byte >> i) & 1)

    # Pack into 32-bit words
    words = pack_bits_to_words_msb_first(bits)

    # Pad or trim to total_words
    if total_words > 0:
        if len(words) < total_words:
            words += [0xFFFFFFFF] * (total_words - len(words))
        elif len(words) > total_words:
            words = words[:total_words]
    return words


def format_c_array(words: List[int], name: str = "replay_buffer") -> str:
    lines = []
    lines.append(f"uint32_t {name}[{len(words)}] __attribute__((aligned(256))) = {{")
    row = []
    for i, w in enumerate(words):
        row.append(f"0x{w:08X}")
        if (i + 1) % 8 == 0:
            lines.append("    " + ", ".join(row) + ",")
            row = []
    if row:
        lines.append("    " + ", ".join(row) + ",")
    lines.append("};")
    return "\n".join(lines)


def parse_single_args(argv: List[str]) -> Tuple[int, int, str, int, int, int]:
    ap = argparse.ArgumentParser(description="Build FlexRay replay 32-bit words")
    ap.add_argument("--id", required=False, help="Frame ID (e.g. 0x37 or 55)")
    ap.add_argument("--cycle", required=False, help="Cycle count (0-63, e.g. 0x3B)")
    ap.add_argument("--payload", required=False, help="Payload hex string (no spaces)")
    ap.add_argument("--indicators", default="0", help="5-bit indicators (default 0)")
    ap.add_argument("--idle_words", type=int, default=8, help="Idle prefix words of 0xFFFFFFFF")
    ap.add_argument("--total_words", type=int, default=64, help="Total words in output array (pad with idle)")
    ap.add_argument("--csv", help="CSV file with id,cycle,payload per line")
    args = ap.parse_args(argv)

    if args.csv:
        return (-1, -1, "", int(args.indicators, 0), args.idle_words, args.total_words)

    if not (args.id and args.cycle and args.payload):
        ap.error("Either provide --csv or all of --id --cycle --payload")

    frame_id = int(args.id, 0)
    cycle = int(args.cycle, 0) & 0x3F
    indicators = int(args.indicators, 0) & 0x1F
    return (frame_id, cycle, args.payload, indicators, args.idle_words, args.total_words)


def process_csv(path: str, indicators: int, idle_words: int, total_words: int) -> None:
    outputs: List[str] = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 3:
                continue
            frame_id = int(parts[0], 0)
            cycle = int(parts[1], 0) & 0x3F
            payload_hex = parts[2]
            words = build_wire_words(indicators, frame_id, cycle, payload_hex, idle_words, total_words)
            outputs.append(format_c_array(words, name=f"replay_buffer_id_{frame_id:03d}"))
    print("\n\n".join(outputs))


def main(argv: List[str]) -> None:
    # Keep original hint lines for user context
    # frame id, cycle count, payload
    # 0x37,0x3B,BFD8700991FFFFFF01F0F1FFFFFF8FBD
    frame_id, cycle, payload_hex, indicators, idle_words, total_words = parse_single_args(argv)
    if frame_id == -1:
        # CSV mode
        ap = argparse.ArgumentParser(add_help=False)
        ap.add_argument("--csv")
        ns, _ = ap.parse_known_args(argv)
        process_csv(ns.csv, indicators, idle_words, total_words)
        return

    words = build_wire_words(indicators, frame_id, cycle, payload_hex, idle_words, total_words)
    print(format_c_array(words))


if __name__ == "__main__":
    main(sys.argv[1:])
