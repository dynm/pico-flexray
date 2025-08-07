#include "../src/flexray_frame.h"
#include <stdio.h>
#include <stddef.h>

#define PRINT_FIELD_INFO(struct_type, field, accumulated_size) \
    do { \
        size_t offset = offsetof(struct_type, field); \
        size_t size = sizeof(((struct_type *)0)->field); \
        printf("  %s offset = %zu, size = %zu\n", #field, offset, size); \
        if (offset > accumulated_size) { \
            printf("    -> padding before this field: %zu bytes\n", offset - accumulated_size); \
        } \
        accumulated_size = offset + size; \
    } while (0)

void print_frame_structure() {
    size_t accumulated_size = 0;
    printf("sizeof(flexray_frame_t) = %zu\n", sizeof(flexray_frame_t));

    PRINT_FIELD_INFO(flexray_frame_t, frame_crc, accumulated_size);
    PRINT_FIELD_INFO(flexray_frame_t, frame_id, accumulated_size);
    PRINT_FIELD_INFO(flexray_frame_t, header_crc, accumulated_size);
    PRINT_FIELD_INFO(flexray_frame_t, indicators, accumulated_size);
    PRINT_FIELD_INFO(flexray_frame_t, payload_length_words, accumulated_size);
    PRINT_FIELD_INFO(flexray_frame_t, cycle_count, accumulated_size);
    PRINT_FIELD_INFO(flexray_frame_t, source, accumulated_size);
    PRINT_FIELD_INFO(flexray_frame_t, payload, accumulated_size);

    size_t total_size = sizeof(flexray_frame_t);
    if (total_size > accumulated_size) {
        printf("  -> padding at the end of the struct: %zu bytes\n", total_size - accumulated_size);
    }

    if (total_size == sizeof(flexray_frame_t)) {
        printf("The struct has no padding.\n");
    } else {
        printf("The struct has padding.\n");
    }
}

int main() {
    print_frame_structure();
    return 0;
}
