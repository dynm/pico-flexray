#ifndef FLEXRAY_FRAME_GEN_H
#define FLEXRAY_FRAME_GEN_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define FLEXRAY_FRAME_GEN_OP_PAYLOAD 0x94u
#define FLEXRAY_FRAME_GEN_OP_SWITCH 0x95u
#define FLEXRAY_FRAME_GEN_OP_CONFIG 0xa0u
#define FLEXRAY_FRAME_GEN_OP_TEMPLATE 0xa1u
#define FLEXRAY_FRAME_GEN_OP_ENABLE 0xa2u
#define FLEXRAY_FRAME_GEN_OP_STATUS 0xa3u
#define FLEXRAY_FRAME_GEN_OP_DIAGNOSTICS 0xa4u
#define FLEXRAY_FRAME_GEN_VERSION 7u
#define FLEXRAY_FRAME_GEN_TARGETS 2u
// Entry PC in the unchanged injector. Start on WAIT TSS so initialization
// does not write shared TXD. PIO tests verify this offset.
#define FLEXRAY_FRAME_GEN_FORWARDER_ENTRY 3u
#define FLEXRAY_FRAME_GEN_STATUS_MAGIC 0x53524650u

typedef struct __attribute__((packed)) {
    uint16_t target_id, static_max_id, payload_bytes, learn_samples;
    uint8_t rx_channel, cycle_mask, cycle_base, tss_bits;
    uint32_t slot_cycles, cycle_cycles;
    int32_t phase_cycles;
    uint32_t tolerance_cycles;
    uint16_t second_target_id, reserved; // 0 disables the second reservation.
} flexray_frame_gen_config_t;
_Static_assert(sizeof(flexray_frame_gen_config_t) == 32, "static config wire size");
enum {
    FLEXRAY_FRAME_GEN_OK, FLEXRAY_FRAME_GEN_UNAVAILABLE, FLEXRAY_FRAME_GEN_BAD_LENGTH,
    FLEXRAY_FRAME_GEN_BAD_CONFIG, FLEXRAY_FRAME_GEN_BAD_FRAME, FLEXRAY_FRAME_GEN_BUSY,
    FLEXRAY_FRAME_GEN_NOT_CONFIGURED, FLEXRAY_FRAME_GEN_NOT_SYNCED,
};
enum {
    FLEXRAY_FRAME_GEN_CONFIGURED = 1u << 0,
    FLEXRAY_FRAME_GEN_TEMPLATE_VALID = 1u << 1,
    FLEXRAY_FRAME_GEN_ENABLED = 1u << 2,
    FLEXRAY_FRAME_GEN_LOCKED = 1u << 3,
    FLEXRAY_FRAME_GEN_TX_PENDING = 1u << 4,
    FLEXRAY_FRAME_GEN_COMMAND_PENDING = 1u << 5,
    FLEXRAY_FRAME_GEN_PACING = 1u << 6,
    FLEXRAY_FRAME_GEN_SECOND_TEMPLATE_VALID = 1u << 7,
    FLEXRAY_FRAME_GEN_RESYNCING = 1u << 8,
};
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version, size;
    uint32_t flags, slot_cycles, cycle_cycles, samples, cycle_samples;
    uint32_t rejected, captured, sent, null_sent, missed, lost_stamps;
    uint32_t pace_ticks, pace_skipped, current_slot, current_cycle, reference_id;
    int32_t phase_error_cycles, cycle_shape_cycles;
    uint32_t phase_updates, phase_missed;
    uint16_t template_len, second_template_len;
    uint32_t last_error;
    uint32_t sync_losses, sync_loss_reason; // 1: reference timeout; 2: cycle discontinuity
    uint32_t header_slot_mismatches; // IRQ-time observation only, not an FSS timestamp
} flexray_frame_gen_status_t;
_Static_assert(sizeof(flexray_frame_gen_status_t) == 108, "static status wire size");
// Read-only fault snapshot. Sequence is even when fully published.
typedef struct {
    uint32_t sequence, counts[4];
    uint32_t reason, fid, cycle, slot, prepared_beat, current_beat;
    uint32_t packet_remaining, pulse_remaining, inducer_pc, local_pc, active;
} flexray_frame_gen_diagnostics_t;
_Static_assert(sizeof(flexray_frame_gen_diagnostics_t) <= sizeof(flexray_frame_gen_status_t), "diagnostics reply capacity");
void flexray_frame_gen_init(void);
// Shared USB bulk/NCM action parser. Returns consumed bytes, or zero for
// unknown/truncated actions. Semantic rejection is reported in status.last_error.
size_t flexray_frame_gen_action(const uint8_t *data, size_t length);
void flexray_frame_gen_forwarder_local_config(bool enable);
void flexray_frame_gen_on_rx_header(bool is_fr2, const uint8_t header[5], bool valid);
size_t flexray_frame_gen_command(const uint8_t *request, size_t length,
                                uint8_t *reply, size_t capacity);
#endif
