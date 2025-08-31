def reflect8(n):
    return int('{:08b}'.format(n)[::-1], 2)

def crc8(data, poly, init, xor_out, refin=False, refout=False):
    crc = init
    for byte in data:
        if refin:
            byte = reflect8(byte)
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ poly
            else:
                crc = crc << 1
            crc &= 0xFF # keep it 8-bit
    if refout:
        crc = reflect8(crc)
    return crc ^ xor_out


def find_crc_params():
    hex_data = [
        # "F6007FFF7FFF", "6B017FFF7FFF", "D1027FFF7FFF", "4C037FFF7FFF",
        # "B8047FFF7FFF", "25057FFF7FFF", "9F067FFF7FFF", "02077FFF7FFF",
        # "6A087FFF7FFF", "F7097FFF7FFF", "4D0A7FFF7FFF", "D00B7FFF7FFF",
        # "240C7FFF7FFF", "B90D7FFF7FFF", "030E7FFF7FFF",
        # "3C61007D10C0F0201311FCFDFFFFFFFF",
        # "DB62007D10C0F020DB12EC00FFFFFFFF",
        # "8663007D10C0F0208613EC00FFFFFFFF",
        # "0864007D10C0F0200814EC00FFFFFFFF",
        # "5565007D10C0F0205515EC00FFFFFFFF",
        # "B266007D10C0F020B216EC00FFFFFFFF"

        # "0864007D10C0F0200814EC00FFFFFFFF",
        # "5565007D10C0F0205515EC00FFFFFFFF",
        # "B266007D10C0F020B216EC00FFFFFFFF",
        # "EF67007D10C0F020EF17EC00FFFFFFFF",
        # "B368007D10C0F020B318EC00FFFFFFFF",
        # "EE69007D10C0F020EE19EC00FFFFFFFF",

        # "6160007D10C0F0206110EC00FFFFFFFF",
        # "3C61007D10C0F0203C11EC00FFFFFFFF",
        # "DB62007D10C0F020DB12EC00FFFFFFFF",
        # "8663007D10C0F0208613EC00FFFFFFFF",
        # "0864007D10C0F0200814EC00FFFFFFFF",
        # "5565007D10C0F0205515EC00FFFFFFFF",
        # "B266007D10C0F020B216EC00FFFFFFFF",

        # "F4F5F47E0080FFFF4EF5F47E0080FFFF",
        # "F0F6F47E0080FFFF",
        # "9AF7F47E0080FFFF",
        # "E6F8F47E0080FFFF",
        # "8CF9F47E0080FFFF",
        # "32FAF47E0080FFFF",
        # "58FBF47E0080FFFF",

        # "2FF1FFFFFFFFFFFFFFFF8888FFFFFFFF",
        # "30F2FFFFFFFFFFFFFFFF8888FFFFFFFF",
        # "CEF3FFFFFFFFFFFFFFFF8888FFFFFFFF",
        # "0EF4FFFFFFFFFFFFFFFF8888FFFFFFFF",
        # "F0F5FFFFFFFFFFFFFFFF8888FFFFFFFF",
        # "EFF6FFFFFFFFFFFFFFFF8888FFFFFFFF",
        # "11F7FFFFFFFFFFFFFFFF8888FFFFFFFF",
        # "72F8FFFFFFFFFFFFFFFF8888FFFFFFFF",
        # "8CF9FFFFFFFFFFFFFFFF8888FFFFFFFF",
        # "93FAFFFFFFFFFFFFFFFF8888FFFFFFFF",
        # "6DFBFFFFFFFFFFFFFFFF8888FFFFFFFF",
        # "ADFCFFFFFFFFFFFFFFFF8888FFFFFFFF",
        # "53FDFFFFFFFFFFFFFFFF8888FFFFFFFF",
        # "03FEFF7FFF7FFF7FFF7F4444FFFFFFFF",
        # "9EF0FF7FFF7FFF7FFF7F4444FFFFFFFF",
        # "60F1FF7FFF7FFF7FFF7F4444FFFFFFFF",
        # "7FF2FF7FFF7FFF7FFF7F4444FFFFFFFF",
        # "81F3FF7FFF7FFF7FFF7F4444FFFFFFFF",
        # "41F4FF7FFF7FFF7FFF7F4444FFFFFFFF",
        "F29CA861FE7F00000000FE17FF130000",
        "3794A861FE7F00000000FE17FF130000",
        "DB90A861FE7F00000000FE17FF130000",
        "2599A861FE7F00000000FE17FF130000",
        "AD92A861FE7F00000000FE17FF130000",
        "539BA861FE7F00000000FE17FF130000",
        
    ]

    parsed_data = []
    for line in hex_data:
        bytes_data = bytes.fromhex(line)
        crc = bytes_data[0]
        data = bytes_data[1:]
        parsed_data.append({'crc': crc, 'data': data})

    max_len = len(parsed_data[0]['data'])
    poly = 0x1d

    print(f"--- Searching with polynomial: {hex(poly)} ---")

    matches = []

    for data_len in range(1, max_len + 1):
        for reverse_bytes in [False, True]:
            print(f"Testing data length: {data_len}, reversed bytes: {reverse_bytes}")
            for xor_out in range(1):
                for init in range(256):
                    for refin, refout in [(False, False), (True, True), (True, False), (False, True)]:
                        match = True
                        for sample in parsed_data:
                            data_slice = sample['data'][:data_len]
                            if reverse_bytes:
                                data_to_check = data_slice[::-1]
                            else:
                                data_to_check = data_slice

                            calculated_crc = crc8(data_to_check, poly, init, xor_out, refin, refout)
                            if calculated_crc != sample['crc']:
                                match = False
                                break
                        
                        if match:
                            print(f"\n{'*' * 20}")
                            print(f"Found a match!")
                            print(f"  Data length: {data_len}")
                            print(f"  Polynomial: {hex(poly)}")
                            print(f"  Init value: {hex(init)}")
                            print(f"  XOR out: {hex(xor_out)}")
                            print(f"  Reflected In: {refin}")
                            print(f"  Reflected Out: {refout}")
                            print(f"  Reversed Bytes: {reverse_bytes}")
                            print(f"{'*' * 20}\n")
                            matches.append({
                                'data_len': data_len,
                                'poly': poly,
                                'init': init,
                                'xor_out': xor_out,
                                'refin': refin,
                                'refout': refout,
                                'reverse_bytes': reverse_bytes,
                            })
    
    if matches:
        print(f"\n--- Total matches: {len(matches)} ---")
        return matches
    
    print("\n--- No matching CRC parameters found. ---")
    return []

if __name__ == "__main__":
    find_crc_params()
