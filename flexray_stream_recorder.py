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

# 目标端点
TARGET_ENDPOINT = 0x81

format_string = "?????xHBxHB254sxIBxxx"
struct_size = struct.calcsize(format_string)

def calculate_flexray_header_crc(frame):
    """
    Calculates the FlexRay header CRC based on the C implementation.
    The frame parameter is a dictionary-like object with keys:
    'sync_frame_indicator', 'startup_frame_indicator', 'frame_id', 'payload_length_words'.
    """
    data_word = 0
    data_word |= int(frame['sync_frame_indicator']) << 19
    data_word |= int(frame['startup_frame_indicator']) << 18
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


def parse_frame(data):
    unpacked = struct.unpack(format_string, data)
    frame_dict = {
        "startup_frame_indicator": unpacked[0],
        "sync_frame_indicator": unpacked[1],
        "null_frame_indicator": unpacked[2],
        "payload_preamble_indicator": unpacked[3],
        "reserved_bit": unpacked[4],
        "frame_id": unpacked[5],
        "payload_length_words": unpacked[6],
        "header_crc": unpacked[7],
        "cycle_count": unpacked[8],
        "payload": unpacked[9],
        "payload_crc": unpacked[10],
        "source": unpacked[11],
    }
    payload_len = frame_dict['payload_length_words'] * 2
    frame_dict['payload'] = frame_dict['payload'][0:payload_len]

    calculated_crc = calculate_flexray_header_crc(frame_dict)
    frame_dict['calculated_header_crc'] = calculated_crc
    frame_dict['header_crc_valid'] = (calculated_crc == frame_dict['header_crc'])
    return frame_dict

def try_parse_frame(data):
    for i in range(0, len(data)-struct_size+1):
        frame_dict = parse_frame(data[i:i+struct_size])
        if frame_dict['header_crc_valid']:
            return frame_dict, i
    return None, -1

def find_usb_device():
    """查找USB设备"""
    print("正在搜索USB设备...")
    
    dev = usb.core.find(idVendor=PANDA_VID, idProduct=PANDA_PID)
    if dev is None:
        print(f"错误: 未找到设备 VID:PID {PANDA_VID:04x}:{PANDA_PID:04x}")
        print("请确保设备已连接并正确识别。")
        return None
    
    print(f"找到设备: {dev}")
    
    # 设置配置
    try:
        if hasattr(dev, 'set_configuration'):
            dev.set_configuration()  # type: ignore
            print("设备配置成功")
    except usb.core.USBError as e:
        print(f"警告: 无法设置配置: {e}")
    except Exception as e:
        print(f"警告: 设备配置时出现异常: {e}")
    
    return dev

def read_and_parse_data_continuously(dev, csv_writer):
    """持续从端点读取数据并解析FlexRay帧"""
    print(f"\n开始从端点 0x{TARGET_ENDPOINT:02x} 持续读取数据并解析FlexRay帧...")
    print("按 Ctrl+C 停止")
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
                # 从端点读取数据
                data = dev.read(TARGET_ENDPOINT, 512, timeout=1000)  # type: ignore
                if data:
                    data_buffer += bytes(data)

                    # 在缓冲区中处理数据
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
                                int(frame['startup_frame_indicator']),
                                int(frame['sync_frame_indicator']),
                                int(frame['null_frame_indicator']),
                                int(frame['payload_preamble_indicator']),
                                int(frame['reserved_bit']),
                                frame['frame_id'],
                                frame['payload_length_words'],
                                f"0x{frame['header_crc']:x}",
                                frame['cycle_count'],
                                payload_hex,
                                f"0x{frame['payload_crc']:x}"
                            ]
                            csv_writer.writerow(row)
                            
                            # Update for real-time display
                            frame_id = frame['frame_id']
                            frame['timestamp'] = timestamp # Add timestamp for display
                            if frame_id not in latest_frames:
                                sorted_frame_ids.append(frame_id)
                                sorted_frame_ids.sort()
                            latest_frames[frame_id] = frame
                            
                            # 从缓冲区移除已解析的帧（包括找到它之前的垃圾数据）
                            data_buffer = data_buffer[offset + struct_size:]
                        else:
                            # 在当前缓冲区中未找到有效帧。
                            # 保留最后一部分数据，因为它可能是下一帧的开头。
                            if len(data_buffer) > struct_size:
                                data_buffer = data_buffer[-(struct_size-1):]
                            break # 需要更多数据来形成一个完整的帧
                
                # Rate limit screen updates
                current_time = time.time()
                if frames_found_in_batch and (current_time - last_display_time > 0.1): # Update ~10 times/sec
                    # ANSI escape code to clear screen and move to top-left
                    print("\033[H\033[J", end="")
                    
                    # Display Header
                    header_line = f"{'Slot ID':<10} | {'Timestamp':<28} | {'Cycle':<8} | {'Payload Words':<15} | {'Payload':<{max_seen_payload_hex_len}}"
                    print(header_line)
                    print("-" * len(header_line))

                    # Display data for each slot
                    for fid in sorted_frame_ids:
                        f = latest_frames.get(fid)
                        if f:
                            payload_hex = f['payload'].hex()
                            line = (
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
                # 超时是正常的，继续尝试
                continue
            except usb.core.USBError as e:
                print(f"\nUSB错误: {e}")
                print("设备可能已断开，正在尝试重新连接...")
                dev = None
                while dev is None:
                    time.sleep(1)
                    dev = find_usb_device()
                if dev:
                    print("设备已重新连接。")
                else:
                    print("无法重新连接设备。正在退出。")
                    break

    except KeyboardInterrupt:
        print(f"\n\n用户中断")
        
    finally:
        elapsed = time.time() - start_time
        frame_rate = total_frames / elapsed if elapsed > 0 else 0
        print(f"\n最终统计:")
        print(f"  总帧数: {total_frames}")
        print(f"  运行时间: {elapsed:.2f} 秒")
        print(f"  帧速率: {frame_rate:.1f} 帧/秒")


def main():
    """主函数"""
    print("FlexRay USB数据流记录器")
    print("=" * 40)
    
    # Setup CSV file
    csv_filename = f"flexray_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    try:
        csv_file = open(csv_filename, 'w', newline='', encoding='utf-8')
    except IOError as e:
        print(f"错误: 无法创建CSV文件 {csv_filename}: {e}")
        sys.exit(1)

    csv_writer = csv.writer(csv_file)
    
    # CSV Header
    header = [
        'timestamp', 'startup_frame_indicator', 'sync_frame_indicator', 
        'null_frame_indicator', 'payload_preamble_indicator', 'reserved_bit', 
        'frame_id', 'payload_length_words', 'header_crc', 'cycle_count', 
        'payload', 'payload_crc'
    ]
    csv_writer.writerow(header)
    print(f"将记录数据到 {csv_filename}")

    # 查找设备
    dev = find_usb_device()
    if dev is None:
        csv_file.close()
        sys.exit(1)
    
    # 开始持续读取并解析数据
    try:
        read_and_parse_data_continuously(dev, csv_writer)
    except Exception as e:
        print(f"\n未处理的错误: {e}")
    finally:
        if csv_file and not csv_file.closed:
            csv_file.close()
            print(f"\n日志文件 {csv_filename} 已关闭。")

if __name__ == "__main__":
    main()
