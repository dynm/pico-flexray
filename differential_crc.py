import crc
import os

crc24_calculator = crc.Calculator(
    crc.Configuration(
        width=24,
        polynomial=0x5D6DCB,
        init_value=0xFEDCBA, # The non-zero init value
        final_xor_value=0x000000,
        reverse_input=False,
        reverse_output=False,
    )
)

# define message structure and constants
HEADER_LEN = 1
COUNT_LEN = 1
PAYLOAD_LEN = 32
TOTAL_MSG_LEN = HEADER_LEN + COUNT_LEN + PAYLOAD_LEN

# --- calculate delta crc ---
def create_lookup_tables():
    print("--- Generating Lookup Tables and Constants ---")
    
    # generate delta crc lookup table
    delta_crc_table = []
    for count_val in range(2**6):
        delta_message = bytearray(TOTAL_MSG_LEN)
        delta_message[HEADER_LEN] = count_val
        delta_crc = crc24_calculator.checksum(bytes(delta_message))
        delta_crc_table.append(delta_crc)
    print("Delta CRC Lookup Table generated.")

    # calculate correction constant: "zero crc"
    zero_message = bytes(TOTAL_MSG_LEN)
    crc_of_zeros = crc24_calculator.checksum(zero_message)
    print(f"Correction Constant 'CRC_OF_ZEROS' is: 0x{crc_of_zeros:06X}\n")

    return delta_crc_table, crc_of_zeros

# --- main verification logic ---
if __name__ == "__main__":
    DELTA_CRC_TABLE, CRC_OF_ZEROS = create_lookup_tables()

    header_data = os.urandom(HEADER_LEN)
    payload_data = os.urandom(PAYLOAD_LEN)
    test_cycle_count = 42 # random count
    
    print(f"--- Verification for a Random Message ---")
    print(f"Testing with Cycle Count: {test_cycle_count}\n")
    
    # --- method 1: direct calculation (The Ground Truth) ---
    print("--- Method 1: Direct Calculation ---")
    direct_message = header_data + bytes([test_cycle_count]) + payload_data
    direct_crc_result = crc24_calculator.checksum(direct_message)
    print(f"Directly Calculated CRC: 0x{direct_crc_result:06X}\n")

    # --- method 2: corrected differential method ---
    print("--- Method 2: Corrected Differential Method ---")
    
    # 1. define base message and delta message
    # M = (H|0|P) XOR (0|c|0) XOR (0|0|0)
    # CRC(M) = CRC(H|0|P) XOR CRC(0|c|0) XOR CRC(0|0|0) <-- The formula
    base_message = header_data + bytes([0]) + payload_data
    
    # 2. calculate base message crc
    base_crc = crc24_calculator.checksum(base_message)
    print(f"Base CRC (for count=0): 0x{base_crc:06X}")
    
    # 3. lookup delta crc from table
    delta_crc_from_table = DELTA_CRC_TABLE[test_cycle_count]
    print(f"Delta CRC from table for count {test_cycle_count}: 0x{delta_crc_from_table:06X}")
    print(f"CRC_OF_ZEROS constant: 0x{CRC_OF_ZEROS:06X}")

    # 4. calculate final result
    differential_crc_result = base_crc ^ delta_crc_from_table ^ CRC_OF_ZEROS
    print(f"Final CRC (Base^Delta^Zeros): 0x{differential_crc_result:06X}\n")
    
    # --- final comparison ---
    print("--- Comparison ---")
    print(f"Direct CRC       : 0x{direct_crc_result:06X}")
    print(f"Differential CRC : 0x{differential_crc_result:06X}")

    if direct_crc_result == differential_crc_result:
        print("\n✅ Success! The corrected differential method works perfectly.")
    else:
        print("\n❌ Error! The results do not match.")