// crc11_generator.c
// Compile on your PC: gcc crc11_generator.c -o crc11_generator
// Run: ./crc11_generator

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define FLEXRAY_CRC11_POLY 0x385

int main() {
    uint16_t crc_table[256];

    printf("Generating FlexRay CRC-11 Lookup Table...\n");
    printf("Polynomial: 0x%X\n\n", FLEXRAY_CRC11_POLY);

    for (int i = 0; i < 256; ++i) {
        uint16_t crc = (uint16_t)i << 3; // Align byte to the top of the 11-bit CRC
        for (int j = 0; j < 8; ++j) {
            if ((crc & 0x400) != 0) { // Check MSB of 11-bit word (bit 10)
                crc = (crc << 1) ^ FLEXRAY_CRC11_POLY;
            } else {
                crc = crc << 1;
            }
        }
        crc_table[i] = crc & 0x7FF; // Ensure it's an 11-bit value
    }

    printf("// FlexRay Header CRC-11 Lookup Table (Poly: 0x%X, Init: 0x1A)\n", FLEXRAY_CRC11_POLY);
    printf("static const uint16_t flexray_crc11_table[256] = {\n");
    for (int i = 0; i < 256; ++i) {
        if (i % 8 == 0) {
            printf("    ");
        }
        printf("0x%03X, ", crc_table[i]);
        if (i % 8 == 7) {
            printf("\n");
        }
    }
    printf("};\n");

    printf("// FlexRay Header CRC-11 4 bit Lookup Table (Poly: 0x%X, Init: 0x1A)\n", FLEXRAY_CRC11_POLY);
    /**
     * @brief 
     * static void generate_flexray_crc_4bit_table(uint16_t table[16]) {
            const uint16_t poly = 0x385;
            for (int i = 0; i < 16; ++i) {
                uint16_t reg = (uint16_t)i << 7;  // 4位输入左移7位（11-4）
                for (int j = 0; j < 4; ++j) {
                    bool msb = (reg >> 10) & 1;
                    reg <<= 1;
                    if (msb) {
                        reg ^= poly;
                    }
                    reg &= 0x7FF;  // 保持11位
                }
                table[i] = reg;
            }
        }
     */
    uint16_t flexray_crc11_4bit_table[16];
    for (int i = 0; i < 16; ++i) {
        uint16_t reg = (uint16_t)i << 7;
        for (int j = 0; j < 4; ++j) {
            bool msb = (reg >> 10) & 1;
            reg <<= 1;
            if (msb) {
                reg ^= FLEXRAY_CRC11_POLY;
            }
            reg &= 0x7FF;
        }
        flexray_crc11_4bit_table[i] = reg;
    }
    printf("static const uint16_t flexray_crc11_4bit_table[16] = {\n");
    for (int i = 0; i < 16; ++i) {
        if (i % 8 == 0) {
            printf("    ");
        }
        printf("0x%03X, ", flexray_crc11_4bit_table[i]);
        if (i % 8 == 7) {
            printf("\n");
        }
    }
    printf("};\n");
    return 0;
}