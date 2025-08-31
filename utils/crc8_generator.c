// crc8_generator.c
// Compile on your PC: gcc crc8_generator.c -o crc8_generator
// Run: ./crc8_generator

#include <stdio.h>
#include <stdint.h>

#define CRC8_POLY 0x1D   // SAE J1850 polynomial (MSB-first)

int main() {
    uint8_t crc_table[256];

    printf("Generating CRC-8 (SAE J1850) Lookup Table...\n");
    printf("Polynomial: 0x%X\n\n", CRC8_POLY);

    for (int i = 0; i < 256; ++i) {
        uint8_t crc = (uint8_t)i;
        for (int j = 0; j < 8; ++j) {
            if ((crc & 0x80) != 0) {
                crc = (uint8_t)((crc << 1) ^ CRC8_POLY);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
        crc_table[i] = crc;
    }

    printf("// CRC-8 SAE J1850 Lookup Table (Poly: 0x%X, Init: 0xFF, XorOut: 0xFF)\n", CRC8_POLY);
    printf("static const uint8_t flexray_crc8_table[256] = {\n");
    for (int i = 0; i < 256; ++i) {
        if (i % 8 == 0) {
            printf("    ");
        }
        printf("0x%02X, ", crc_table[i]);
        if (i % 8 == 7) {
            printf("\n");
        }
    }
    printf("};\n");

    return 0;
}


