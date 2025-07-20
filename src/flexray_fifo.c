#include "flexray_fifo.h"
#include "pico/sync.h"
#include <string.h>
#include <stdio.h>

void flexray_fifo_init(flexray_fifo_t *fifo) {
    memset(fifo, 0, sizeof(flexray_fifo_t));
    printf("FlexRay FIFO initialized\n");
}

bool flexray_fifo_is_empty(const flexray_fifo_t *fifo) {
    return fifo->write_pos == fifo->read_pos;
}

bool flexray_fifo_is_full(const flexray_fifo_t *fifo) {
    return ((fifo->write_pos + 1) % FLEXRAY_FIFO_SIZE) == fifo->read_pos;
}

uint32_t flexray_fifo_count(const flexray_fifo_t *fifo) {
    if (fifo->write_pos >= fifo->read_pos) {
        return fifo->write_pos - fifo->read_pos;
    }
    return FLEXRAY_FIFO_SIZE - (fifo->read_pos - fifo->write_pos);
}

bool flexray_fifo_push(flexray_fifo_t *fifo, const flexray_frame_t *frame) {
    uint32_t primask = save_and_disable_interrupts();

    if (flexray_fifo_is_full(fifo)) {
        // FIFO is full, drop the incoming frame.
        fifo->stats.frames_dropped++;
        restore_interrupts(primask);
        return false;
    }

    memcpy(&fifo->frames[fifo->write_pos], frame, sizeof(flexray_frame_t));
    fifo->write_pos = (fifo->write_pos + 1) % FLEXRAY_FIFO_SIZE;
    fifo->stats.total_frames_received++;
    
    restore_interrupts(primask);
    return true;
}

bool flexray_fifo_pop(flexray_fifo_t *fifo, flexray_frame_t *frame) {
    uint32_t primask = save_and_disable_interrupts();

    if (flexray_fifo_is_empty(fifo)) {
        restore_interrupts(primask);
        return false; // FIFO empty
    }

    memcpy(frame, &fifo->frames[fifo->read_pos], sizeof(flexray_frame_t));
    fifo->read_pos = (fifo->read_pos + 1) % FLEXRAY_FIFO_SIZE;
    fifo->stats.frames_transmitted++;
    
    restore_interrupts(primask);
    return true;
}

void flexray_fifo_get_stats(const flexray_fifo_t *fifo, fifo_stats_t *stats) {
    uint32_t primask = save_and_disable_interrupts();
    memcpy(stats, &fifo->stats, sizeof(fifo_stats_t));
    restore_interrupts(primask);
} 