
import sys

# The script reads raw data from a logic analyzer, which is assumed to be
# a binary file representing a continuous stream of bits. It downsamples
# the data using center-point sampling. For every 10 bits, the 5th bit
# is taken as the sampled value.

def main():
    """
    Reads, downsamples using center-point sampling, and writes the signal data.
    """
    input_filename = "raw_signal.bin"
    output_filename = "downsampled_raw_signal.bin"
    window_size = 10
    sample_point = 5  # 5th bit (1-indexed)

    try:
        print(f"Reading data from {input_filename}...")
        with open(input_filename, "rb") as f:
            data = f.read()
        print(f"Read {len(data)} bytes.")
    except FileNotFoundError:
        print(f"Error: Input file '{input_filename}' not found.")
        sys.exit(1)

    print(f"Downsampling data using center-point sampling from {window_size}-bit windows...")
    
    output_bits = []
    
    # Create a generator for the bit stream
    def bit_stream(data):
        for byte in data:
            # The raw data is bitswapped per byte for Sigrok compatibility.
            # Iterate from LSB to MSB to achieve this.
            for i in range(8):
                yield (byte >> i) & 1

    bits = bit_stream(data)
    processed_bits = 0
    
    while True:
        try:
            # Read a chunk of bits
            chunk = [next(bits) for _ in range(window_size)]
            processed_bits += window_size
            # The 5th bit is at index 4 (0-indexed)
            sampled_bit = chunk[sample_point - 1]
            output_bits.append(sampled_bit)
        except StopIteration:
            # Reached the end of the bit stream.
            # The last partial chunk is discarded.
            break
            
    print(f"Total bits processed: {processed_bits}")
    print(f"Total bits sampled: {len(output_bits)}")

    # Pack the collected bits into bytes
    downsampled_data = bytearray()
    for i in range(0, len(output_bits), 8):
        byte_val = 0
        byte_chunk = output_bits[i:i+8]
        # Pack bits into a byte, with the first bit in the chunk being the MSB.
        for j, bit in enumerate(byte_chunk):
            byte_val |= (bit << (7 - j))
        downsampled_data.append(byte_val)

    try:
        with open(output_filename, "wb") as f:
            f.write(downsampled_data)
        print(f"Downsampling complete.")
        print(f"Wrote {len(downsampled_data)} bytes to {output_filename}.")
    except IOError as e:
        print(f"Error writing to file {output_filename}: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
