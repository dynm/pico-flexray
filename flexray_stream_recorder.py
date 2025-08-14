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
CSV_BUFFER_SIZE = 1000  # Batch write to CSV every 1000 records

# High-throughput tuning
READ_SIZE = 65536  # Maximize bulk IN size to reduce per-call overhead
RAW_BENCH_MODE = False  # When True, skip CSV and per-frame hex, only count FPS
STATS_INTERVAL_SEC = 1.0   # How often to print a brief stats line
MIN_BODY_LEN = 11  # src(1) + header(5) + crc24(3) + minimal payload(0)

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
# Variable-length records: [u16 len_le][u8 src][5B header][payload][3B crc]

def parse_varlen_records(buffer, frames_out):
    """
    Incrementally parse variable-length FlexRay records from an arbitrary byte buffer.
    Returns the number of bytes consumed.
    """
    i = 0
    buflen = len(buffer)
    while i + 2 <= buflen:
        body_len = buffer[i] | (buffer[i+1] << 8)
        if body_len < MIN_BODY_LEN:
            i += 1
            continue
        if i + 2 + body_len > buflen:
            break
        src = buffer[i+2]
        header = buffer[i+3:i+8]
        indicators = header[0] >> 3
        frame_id = ((header[0] & 0x07) << 8) | header[1]
        payload_len_words = (header[2] >> 1) & 0x7F
        header_crc = ((header[2] & 0x01) << 10) | (header[3] << 2) | ((header[4] >> 6) & 0x03)
        cycle_count = header[4] & 0x3F
        payload_bytes = payload_len_words * 2
        if 5 + payload_bytes + 3 != body_len - 1:
            # length mismatch, skip one byte
            i += 1
            continue
        payload = bytes(buffer[i+8:i+8+payload_bytes])
        crc_bytes = buffer[i+8+payload_bytes:i+8+payload_bytes+3]
        frame_crc = (crc_bytes[0] << 16) | (crc_bytes[1] << 8) | crc_bytes[2]
        frames_out.append({
            'source': src,
            'indicators': indicators,
            'frame_id': frame_id,
            'payload_length_words': payload_len_words,
            'header_crc': header_crc,
            'cycle_count': cycle_count,
            'payload': payload,
            'frame_crc': frame_crc,
            'header_crc_valid': True,
            'frame_crc_valid': True,
        })
        i += 2 + body_len
    return i

def find_usb_device():
    print("Searching for USB device...")
    
    dev = usb.core.find(idVendor=PANDA_VID, idProduct=PANDA_PID)
    if dev is None:
        print(f"Error: Device not found VID:PID {PANDA_VID:04x}:{PANDA_PID:04x}")
        print("Please ensure the device is connected and correctly identified.")
        return None
    
    print(f"Found device: {dev}")
    
    # device configuration
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
    csv_buffer = []
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
                data = dev.read(TARGET_ENDPOINT, READ_SIZE, timeout=1000)  # type: ignore
                if data:
                    data_buffer += bytes(data)
                    batch_timestamp = datetime.now().isoformat()

                    # Process data in buffer using variable-length records
                    frames = []
                    consumed = parse_varlen_records(data_buffer, frames)
                    if consumed > 0:
                        data_buffer = data_buffer[consumed:]
                    for frame in frames:
                        frames_found_in_batch = True
                        total_frames += 1
                        if not RAW_BENCH_MODE:
                            timestamp = batch_timestamp
                            payload_hex = frame['payload'].hex()
                            if len(payload_hex) > max_seen_payload_hex_len:
                                max_seen_payload_hex_len = len(payload_hex)
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
                            csv_buffer.append(row)
                            if len(csv_buffer) >= CSV_BUFFER_SIZE:
                                csv_writer.writerows(csv_buffer)
                                csv_buffer.clear()
                            frame_id = frame['frame_id']
                            frame['timestamp'] = timestamp
                            if frame_id not in latest_frames:
                                sorted_frame_ids.append(frame_id)
                                sorted_frame_ids.sort()
                            latest_frames[frame_id] = frame
                
                # Rate limit screen updates
                current_time = time.time()
                if frames_found_in_batch and (current_time - last_display_time > STATS_INTERVAL_SEC):
                    fps = total_frames / (current_time - start_time)
                    if RAW_BENCH_MODE:
                        print(f"FPS(avg): {fps:.1f}")
                    else:
                        print(f"Frames processed: {total_frames} | FPS(avg): {fps:.1f} | Unique IDs: {len(sorted_frame_ids)}")
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
        # Write any remaining rows in the buffer
        if csv_writer and csv_buffer:
            csv_writer.writerows(csv_buffer)
            print(f"Wrote {len(csv_buffer)} remaining records to CSV.")
            csv_buffer.clear()

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
    main()
