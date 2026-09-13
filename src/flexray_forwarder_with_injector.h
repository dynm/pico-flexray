#ifndef FLEXRAY_FORWARDER_WITH_INJECTOR_H
#define FLEXRAY_FORWARDER_WITH_INJECTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/pio.h"

typedef enum {
    INJECT_TRANSPORT_UNKNOWN = 0,
    INJECT_TRANSPORT_VENDOR = 1,
    INJECT_TRANSPORT_UDP = 2,
} injector_transport_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t vendor_submitted;
    uint32_t udp_submitted;
    uint32_t accepted;
    uint32_t rejected_crc;
    uint32_t rejected_rule;
    uint32_t rejected_length;
    uint32_t superseded;
    uint32_t stale_discarded;
    uint32_t consumed;
    uint32_t prepared;
    uint32_t injected;
    uint32_t queue_depth;
    uint32_t queue_high_water;
    uint32_t disabled_skips;
} injector_stats_t;

#define INJECTOR_STATS_MAGIC 0x4A4E4950u
#define INJECTOR_STATS_VERSION 2u

_Static_assert(sizeof(injector_stats_t) == 64,
               "injector stats must fit one USB control transfer");

// Cache a frame's raw bytes (header+payload+CRC) when rules match
void try_cache_last_target_frame(uint16_t frame_id, uint8_t cycle_count, uint16_t frame_length, uint8_t *captured_bytes);

// Prepare a matching injection while the configured previous frame is still on the bus.
bool prepare_inject_frame(uint16_t frame_id, uint8_t cycle_count);

// Drop any stale preparation before processing a new header.
void discard_prepared_injection(void);

// Start the prepared injection when the trigger frame ends.
void inject_prepared_frame(void);

void setup_forwarder_with_injector(PIO pio,
    uint rx_pin_from_fr1, uint tx_pin_to_fr2,
    uint rx_pin_from_fr2, uint tx_pin_to_fr1,
    uint rx_pin_from_fr3, uint tx_pin_to_fr4,
    uint rx_pin_from_fr4, uint tx_pin_to_fr3);

// Submit a host integrity byte plus a complete payload. The matching rule
// extracts only its configured replacement slice.
bool injector_submit_override(uint16_t id, uint8_t base, uint16_t len, const uint8_t *bytes);
bool injector_submit_override_from(uint16_t id, uint8_t base, uint16_t len,
                                   const uint8_t *bytes,
                                   injector_transport_t transport);

// Enable/disable injection at runtime
void injector_set_enabled(bool enabled);
bool injector_is_enabled(void);
void injector_clear_overrides(void);
void injector_get_stats(injector_stats_t *stats);
// Call only while the FlexRay generator is stopped.
void injector_reset_stats_and_queue(void);


#endif // FLEXRAY_FORWARDER_WITH_INJECTOR_H
