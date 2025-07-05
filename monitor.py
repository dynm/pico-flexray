import serial
import argparse
from datetime import datetime
import sys
import os

def record_uart(port, baudrate, outfile, live):
    """
    Records UART data, adding a timestamp for each data snapshot.
    Optionally displays a live, in-place view of the latest snapshot.

    A new snapshot is detected when a line matches the header:
    'ID,Len,HeaderCRC,Cyc,Data,PayloadCRC,SRC'
    """
    header = 'ID,Len,HeaderCRC,Cyc,Data,PayloadCRC,SRC'
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Listening on {port} at {baudrate} bps...")
    except serial.SerialException as e:
        print(f"Error opening serial port {port}: {e}", file=sys.stderr)
        sys.exit(1)

    snapshot_buffer = []
    f = None

    if outfile:
        try:
            f = open(outfile, 'a')
            print(f"Recording to {outfile}")
            # Ensure there's a newline at the start if the file is not empty
            if os.path.getsize(outfile) > 0:
                f.write("\n")
        except IOError as e:
            print(f"Error opening output file {outfile}: {e}", file=sys.stderr)
            if ser.is_open:
                ser.close()
            sys.exit(1)

    try:
        while True:
            try:
                line = ser.readline()
                if not line:
                    continue  # Timeout occurred, just continue listening

                line_str = line.decode('utf-8', errors='ignore').strip()

                if line_str:
                    if line_str == header:
                        # A new snapshot begins. If we are in live mode and have a
                        # complete snapshot in the buffer, display it now.
                        if live and snapshot_buffer:
                            os.system('cls' if os.name == 'nt' else 'clear')
                            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                            header_line = f"--- Live Snapshot at {timestamp} ---"
                            print(header_line)
                            for s_line in snapshot_buffer:
                                print(s_line)
                            print("-" * len(header_line))
                        
                        # Start new snapshot
                        snapshot_buffer = [line_str]

                        # If logging to file, write timestamp before the header
                        if f:
                            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                            f.write(f"Timestamp: {timestamp}\n")
                    else:
                        snapshot_buffer.append(line_str)

                    # Write to file if enabled
                    if f:
                        f.write(line_str + '\n')
                        f.flush()  # Ensure data is written to the file immediately

            except serial.SerialException as e:
                print(f"Serial error: {e}", file=sys.stderr)
                break
            except UnicodeDecodeError as e:
                print(f"Unicode decode error: {e}. Raw data: {line}", file=sys.stderr)

    except KeyboardInterrupt:
        print("\nStopping recording.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")
        if f:
            f.close()
            print(f"Log file {outfile} closed.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Record and/or display live UART data with timestamps for each snapshot.",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument('port', help='Serial port to read from (e.g., /dev/ttyUSB0, COM3)')
    parser.add_argument('--baudrate', type=int, default=115200, help='Baud rate for the serial connection (default: 115200)')
    parser.add_argument('--outfile', default='uart_log.txt', help='File to save the recorded data (default: uart_log.txt)')
    parser.add_argument('--live', action='store_true', help='Display live data snapshots in the terminal (in-place).')
    parser.add_argument('--no-log', action='store_true', help='Disable logging to a file.')

    args = parser.parse_args()

    outfile = None if args.no_log else args.outfile

    if not outfile and not args.live:
        print("Error: At least one of logging or --live mode must be enabled.", file=sys.stderr)
        parser.print_help()
        sys.exit(1)

    record_uart(args.port, args.baudrate, outfile, args.live)