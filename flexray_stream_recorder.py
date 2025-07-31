import struct
import usb.core
import usb.util
import time
import sys
import csv
from datetime import datetime

# Panda USB VID/PID
PANDA_VID = 0x3801
PANDA_PID = 0xddcc

TARGET_ENDPOINT = 0x81

'''
typedef struct
{
    uint32_t frame_crc; // 24 bits

    uint16_t frame_id;                  // 11 bits
    uint16_t header_crc;                // 11 bits

    uint8_t indicators;                 // 5 bit
    uint8_t payload_length_words;       // 7 bits (number of 16-bit words)
    uint8_t cycle_count;                // 6 bits
    uint8_t source; // from ecu or vehicle
    uint8_t payload[MAX_FRAME_PAYLOAD_BYTES];
} flexray_frame_t;
'''
format_string = "IHHBBBB64s"
struct_size = struct.calcsize(format_string)

def calculate_flexray_header_crc(frame):
    """
    Calculates the FlexRay header CRC based on the C implementation.
    The frame parameter is a dictionary-like object with keys:
    'indicators', 'frame_id', 'payload_length_words'.
    """
    data_word = 0
    data_word |= int(frame['indicators']) << 19
    data_word |= frame['frame_id'] << 7
    data_word |= frame['payload_length_words']

    crc = 0x1A
    poly = 0x385

    for i in range(19, -1, -1):
        data_bit = (data_word >> i) & 1
        crc_msb = (crc >> 10) & 1

        crc <<= 1
        if (data_bit ^ crc_msb):
            crc ^= poly
    
    return crc & 0x7FF

def calculate_flexray_full_crc(frame):
    """
    Calculates the FlexRay full CRC24 for the entire frame using plain bit calculation.
    Reassembles the frame data from parsed components: 5-byte header + payload
    Uses polynomial 0x5D6DCB and initial value 0xFEDCBA
    """
    crc = 0xFEDCBA
    poly = 0x5D6DCB
    
    # Reassemble the 5-byte header according to FlexRay format
    # Byte 0: indicators (5 bits) << 3 | frame_id highest 3 bits
    byte0 = (frame['indicators'] << 3) | ((frame['frame_id'] >> 8) & 0x07)
    
    # Byte 1: frame_id lowest 8 bits
    byte1 = frame['frame_id'] & 0xFF
    
    # Byte 2: payload_length_words (7 bits) << 1 | header_crc bit 10
    byte2 = (frame['payload_length_words'] << 1) | ((frame['header_crc'] >> 10) & 0x01)
    
    # Byte 3: header_crc bits 9-2
    byte3 = (frame['header_crc'] >> 2) & 0xFF
    
    # Byte 4: header_crc bits 1-0 << 6 | cycle_count (6 bits)
    byte4 = ((frame['header_crc'] & 0x03) << 6) | (frame['cycle_count'] & 0x3F)
    
    # Create the data to calculate CRC over: header + payload
    frame_data = bytes([byte0, byte1, byte2, byte3, byte4]) + frame['payload']
    
    # Calculate CRC over the assembled data
    for byte_val in frame_data:
        for bit in range(8):
            data_bit = (byte_val >> (7 - bit)) & 1
            crc_msb = (crc >> 23) & 1
            
            crc <<= 1
            if data_bit ^ crc_msb:
                crc ^= poly
    
    return crc & 0xFFFFFF


def parse_frame(data):
    unpacked = struct.unpack(format_string, data)
    frame_dict = {
        "frame_crc": unpacked[0],
        "frame_id": unpacked[1],
        "header_crc": unpacked[2],
        "indicators": unpacked[3],
        "payload_length_words": unpacked[4],
        "cycle_count": unpacked[5],
        "source": unpacked[6],
        "payload": unpacked[7],
    }
    payload_len = frame_dict['payload_length_words'] * 2
    frame_dict['payload'] = frame_dict['payload'][:payload_len]

    calculated_crc = calculate_flexray_header_crc(frame_dict)
    frame_dict['calculated_header_crc'] = calculated_crc
    frame_dict['header_crc_valid'] = (calculated_crc == frame_dict['header_crc'])
    if not frame_dict['header_crc_valid']:
        return None
    
    calculated_frame_crc = calculate_flexray_full_crc(frame_dict)
    frame_dict['calculated_frame_crc'] = calculated_frame_crc
    frame_dict['frame_crc_valid'] = (calculated_frame_crc == frame_dict['frame_crc'])
    if not frame_dict['frame_crc_valid']:
        print(f"Frame CRC invalid: {frame_dict['frame_crc']} != {calculated_frame_crc}")
        return None
    
    return frame_dict

def try_parse_frame(data):
    for i in range(0, len(data)-struct_size+1):
        frame_dict = parse_frame(data[i:i+struct_size])
        if frame_dict and frame_dict['header_crc_valid']:
            return frame_dict, i
    return None, -1

def find_usb_device():
    print("Searching for USB device...")
    
    dev = usb.core.find(idVendor=PANDA_VID, idProduct=PANDA_PID)
    if dev is None:
        print(f"Error: Device not found VID:PID {PANDA_VID:04x}:{PANDA_PID:04x}")
        print("Please ensure the device is connected and correctly identified.")
        return None
    
    print(f"Found device: {dev}")
    
    # 设置配置
    try:
        if hasattr(dev, 'set_configuration'):
            dev.set_configuration()  # type: ignore
            print("Device configuration successful")
    except usb.core.USBError as e:
        print(f"Warning: Failed to set configuration: {e}")
    except Exception as e:
        print(f"Warning: Exception occurred during device configuration: {e}")
    
    return dev

def read_and_parse_data_continuously(dev, csv_writer):
    """Continuously read data from endpoint and parse FlexRay frames"""
    print(f"\nStarting to read data from endpoint 0x{TARGET_ENDPOINT:02x} and parse FlexRay frames...")
    print("Press Ctrl+C to stop")
    print("=" * 80)

    data_buffer = b''
    total_frames = 0
    start_time = time.time()

    latest_frames = {}
    sorted_frame_ids = []
    last_display_time = 0
    max_seen_payload_hex_len = len('Payload')

    try:
        while True:
            frames_found_in_batch = False
            try:
                # Read data from endpoint
                data = dev.read(TARGET_ENDPOINT, 512, timeout=1000)  # type: ignore
                if data:
                    data_buffer += bytes(data)

                    # Process data in buffer
                    while len(data_buffer) >= struct_size:
                        frame, offset = try_parse_frame(data_buffer)
                        
                        if frame:
                            frames_found_in_batch = True
                            total_frames += 1
                            timestamp = datetime.now().isoformat()
                            
                            payload_hex = frame['payload'].hex()
                            if len(payload_hex) > max_seen_payload_hex_len:
                                max_seen_payload_hex_len = len(payload_hex)
                            
                            # CSV Logging
                            row = [
                                timestamp,
                                frame['source'],
                                bin(frame['indicators'])[2:].zfill(5),
                                frame['frame_id'],
                                frame['payload_length_words'],
                                f"0x{frame['header_crc']:x}",
                                frame['cycle_count'],
                                payload_hex,
                                f"0x{frame['frame_crc']:x}"
                            ]
                            csv_writer.writerow(row)
                            
                            # Update for real-time display
                            frame_id = frame['frame_id']
                            frame['timestamp'] = timestamp # Add timestamp for display
                            if frame_id not in latest_frames:
                                sorted_frame_ids.append(frame_id)
                                sorted_frame_ids.sort()
                            latest_frames[frame_id] = frame
                            
                            # Remove parsed frame from buffer (including garbage data before it)
                            data_buffer = data_buffer[offset + struct_size:]
                        else:
                            # No valid frame found in current buffer.
                            # 保留最后一部分数据，因为它可能是下一帧的开头。
                            if len(data_buffer) > struct_size:
                                data_buffer = data_buffer[-(struct_size-1):]
                            break # Need more data to form a complete frame
                
                # Rate limit screen updates
                current_time = time.time()
                if frames_found_in_batch and (current_time - last_display_time > 0.1): # Update ~10 times/sec
                    # ANSI escape code to clear screen and move to top-left
                    print("\033[H\033[J", end="")
                    
                    # Display Header
                    header_line = f"{'Source':<10} | {'Frame ID':<10} | {'Timestamp':<28} | {'Cycle':<8} | {'Payload Words':<15} | {'Payload':<{max_seen_payload_hex_len}}"
                    print(header_line)
                    print("-" * len(header_line))

                    # Display data for each slot
                    for fid in sorted_frame_ids:
                        f = latest_frames.get(fid)
                        if f:
                            payload_hex = f['payload'].hex()
                            line = (
                                f"{f['source']:<10} | "
                                f"{f['frame_id']:<10} | "
                                f"{f['timestamp']:<28} | "
                                f"{f['cycle_count']:<8} | "
                                f"{f['payload_length_words']:<15} | "
                                f"{payload_hex:<{max_seen_payload_hex_len}}"
                            )
                            print(line)
                    
                    print("\n" + "=" * (len(header_line)))
                    print(f"Total frames processed: {total_frames} | Frames/sec: {total_frames / (current_time - start_time):.1f}")
                    last_display_time = current_time
                
            except usb.core.USBTimeoutError:
                # Timeout is normal, continue trying
                continue
            except usb.core.USBError as e:
                print(f"\nUSB error: {e}")
                print("Device may have been disconnected, trying to reconnect...")
                dev = None
                while dev is None:
                    time.sleep(1)
                    dev = find_usb_device()
                if dev:
                    print("Device reconnected.")
                else:
                    print("Failed to reconnect device. Exiting.")
                    break

    except KeyboardInterrupt:
        print(f"\n\nUser interrupted")
        
    finally:
        elapsed = time.time() - start_time
        frame_rate = total_frames / elapsed if elapsed > 0 else 0
        print(f"\nFinal statistics:")
        print(f"  Total frames: {total_frames}")
        print(f"  Elapsed time: {elapsed:.2f} seconds")
        print(f"  Frame rate: {frame_rate:.1f} frames/second")


def main():
    print("FlexRay USB data stream recorder")
    print("=" * 40)
    
    # Setup CSV file
    csv_filename = f"flexray_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    try:
        csv_file = open(csv_filename, 'w', newline='', encoding='utf-8')
    except IOError as e:
        print(f"Error: Failed to create CSV file {csv_filename}: {e}")
        sys.exit(1)

    csv_writer = csv.writer(csv_file)
    
    # CSV Header
    header = [
        'timestamp', 'source', 'indicators', 'frame_id', 'payload_length_words', 'header_crc', 'cycle_count', 'payload', 'frame_crc'
    ]
    csv_writer.writerow(header)
    print(f"Recording data to {csv_filename}")

    # Find device
    dev = find_usb_device()
    if dev is None:
        csv_file.close()
        sys.exit(1)
    
    # Start continuously reading and parsing data
    try:
        read_and_parse_data_continuously(dev, csv_writer)
    except Exception as e:
        print(f"\nUnhandled error: {e}")
    finally:
        if csv_file and not csv_file.closed:
            csv_file.close()
            print(f"\nLog file {csv_filename} closed.")

if __name__ == "__main__":
    data_hex_str = "57c0c1000c00580600111100000000000000000000000000000000000000000000000000000000000000000000000e408b630010286600100100000001000000b0a7a9156eeb6106fb250010"
    data = bytes.fromhex(data_hex_str)
    frame_dict = parse_frame(data)
    print(f"frame_dict: {frame_dict}")
    main()
