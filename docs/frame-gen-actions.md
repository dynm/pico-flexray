# Static frame generation rules and host actions (protocol v7)

See the [design notes](frame-gen-design.md) for the architecture and the [validation summary](bench-validation.md) for measured coverage.

Rules are compiled into `src/flexray_frame_gen_rules.h`: FR2 FIDs `0xC` / `0xD`, each with an 18-byte payload, cycle repetition 4, and base 3. The static segment ends at the bench's FID `0x10`. Reservations are configured at startup, with transmission disabled by default; the host does not allocate slots. Hardware automatically acquires the slot length, 5 ms cycle, and FSS phase.

Once enabled and synchronized, both slots **transmit every cycle**:

| Cycle count | Unconsumed host data available | No unconsumed data |
| --- | --- | --- |
| 3, 7, 11...63 | Consume once and send a normal frame | Null |
| Other cycles | Null; retain data for the next base=3 cycle | Null |

Null frames have an all-zero 18-byte payload, NFI=0, and PPI=0. The header retains the FID, length, and current cycle, with a correct frame CRC. Only the payload is all zero, not the complete wire frame. Normal frames use NFI=1 and PPI=0. The two targets receive data independently, at up to 50 Hz of normal data each. Normal and null frames together total 200 Hz per target.

Data is consumed during preparation in the preceding logical slot and is not replayed even if that transmission attempt fails. A prepared DMA packet remains owned until frame end. Late submissions apply to the next base=3 cycle. Unconsumed data has no separate wall-clock expiry or absolute cycle addressing. Loss of synchronization discards stale data, and new payloads are rejected while unsynchronized. Disabling stops normal and null transmission and clears applied pending data. Supply fresh data after enabling again.

## Shared USB bulk / NCM input

USB vendor bulk OUT endpoint `0x03` (VID `0x3801` / PID `0xDDCC`) and NCM UDP `192.168.7.1:5501` use identical action bytes and the same parsing/submission function. `0x90/0x91` remain MITM actions; `0x92/0x93` remain RTT actions.

| Action | Wire format (all multibyte integers are little-endian) |
| --- | --- |
| `0x94` | `[94][u16 FID][u8 base][u16 length][payload]` |
| `0x95` | `[95][u8 enabled]`; accepts only 0 or 1 |

`0x94` accepts only FID=C/D, base=3, and length=18. The host does not generate the FlexRay header or frame CRC. To send C, append 18 payload bytes to `94 0c 00 03 12 00`; D uses the prefix `94 0d 00 03 12 00`. Enable and disable are `95 01` / `95 00`. They control only frame generation, not MITM.

A bulk transfer or UDP datagram may concatenate complete actions, including a C+D pair totaling 48 bytes and existing MITM actions. Do not split an action across input buffers. Each target has a publication mailbox, so C and D may be submitted consecutively. If an earlier publication for the same target has not been applied, or a control command is pending, a new payload returns internal BUSY and is discarded without overwriting DMA data. Once applied but not yet consumed, data may be replaced by a newer submission; it does not accumulate in a queue.

Like MITM actions, `0x94/0x95` send no ACK and perform no automatic retries. `0xA3` status exposes `last_error`, `command_pending`, and cumulative transmission counters. It is not a per-action acknowledgment and must not be used to trigger blind retries. `0xA0` remains available for timing diagnostics; attempts to change FID, length, static_max, or base/mask are rejected. `0xA1` full-frame/clear and `0xA2` acknowledged enable/disable remain available for diagnostics, subject to the same static rules. Before pacing starts, control commands require a real RX header to be applied. Start the reference input before enabling.

## Host examples

```sh
# NCM/UDP: start the reference input, enable, then submit one C payload
python3 frame_gen_client.py gen-enable
python3 frame_gen_client.py payload --target-id 0xc --hex 0102030405060708090a0b0c0d0e0f101112
python3 frame_gen_client.py status
python3 frame_gen_client.py gen-disable

# USB bulk (requires PyUSB): identical protocol
python3 frame_gen_client.py gen-enable --transport usb
python3 frame_gen_client.py payload --transport usb --target-id 0xd --hex 0102030405060708090a0b0c0d0e0f101112
python3 frame_gen_client.py gen-disable --transport usb
```

Each CLI invocation submits once. For continuous data, assemble actions in the host's existing bulk/UDP loop. Send enable separately and wait for it to be applied. Do not submit a payload while enable is still pending.

Software tests cover USB/NCM parsing, consecutive submissions for both targets, single consumption, null frames outside base cycles, disable, late data, 63-to-0 wraparound, CRCs, DMA ownership, and MITM independence. See the [validation summary](bench-validation.md) for bench coverage boundaries and [sleep and resynchronization](frame-gen-resync.md) for automatic recovery.
