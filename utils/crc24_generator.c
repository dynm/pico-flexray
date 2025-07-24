// crc24_generator.c
// Compile on your PC: gcc crc24_generator.c -o crc24_generator
// Run: ./crc24_generator

#include <stdio.h>
#include <stdint.h>

#define FLEXRAY_CRC24_POLYNOMIAL 0x5D6DCB

int main() {
    uint32_t crc_table[256];

    printf("Generating FlexRay CRC-24 Lookup Table...\n");
    printf("Polynomial: 0x%X\n\n", FLEXRAY_CRC24_POLYNOMIAL);

    for (int i = 0; i < 256; ++i) {
        uint32_t crc = (uint32_t)i << 16; // Move byte to the top of the 24-bit word
        for (int j = 0; j < 8; ++j) {
            if ((crc & 0x800000) != 0) { // Check MSB of 24-bit word
                crc = (crc << 1) ^ FLEXRAY_CRC24_POLYNOMIAL;
            } else {
                crc = crc << 1;
            }
        }
        crc_table[i] = crc & 0xFFFFFF; // Ensure it's a 24-bit value
    }

    printf("// FlexRay CRC-24 Lookup Table (Polynomial: 0x%X, Initial: 0xABCDEF)\n", FLEXRAY_CRC24_POLYNOMIAL);
    printf("static const uint32_t flexray_crc24_table[256] = {\n");
    for (int i = 0; i < 256; ++i) {
        if (i % 8 == 0) {
            printf("    ");
        }
        printf("0x%06X, ", crc_table[i]);
        if (i % 8 == 7) {
            printf("\n");
        }
    }
    printf("};\n");

    return 0;
}