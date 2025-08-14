#ifndef FLEXRAY_FIFO_H
#define FLEXRAY_FIFO_H

#include "flexray_frame.h"
#include "pico/stdlib.h"

// FIFO stats
typedef struct {
    uint32_t total_frames_received;
    uint32_t frames_dropped;
    uint32_t frames_transmitted;
} fifo_stats_t;

// FlexRay FIFO struct
typedef struct {
    flexray_frame_t frames[FLEXRAY_FIFO_SIZE];
    volatile uint32_t write_pos;
    volatile uint32_t read_pos;
    fifo_stats_t stats;
} flexray_fifo_t;

void flexray_fifo_init(flexray_fifo_t *fifo);
bool flexray_fifo_push(flexray_fifo_t *fifo, const flexray_frame_t *frame);
bool flexray_fifo_pop(flexray_fifo_t *fifo, flexray_frame_t *frame);
// Peek the frame at the head without removing it. Returns false if empty.
bool flexray_fifo_peek(const flexray_fifo_t *fifo, flexray_frame_t *frame);
uint32_t flexray_fifo_count(const flexray_fifo_t *fifo);
bool flexray_fifo_is_full(const flexray_fifo_t *fifo);
bool flexray_fifo_is_empty(const flexray_fifo_t *fifo);
void flexray_fifo_get_stats(const flexray_fifo_t *fifo, fifo_stats_t *stats);

#endif // FLEXRAY_FIFO_H 