# Inject / frame build demo implementation

## Inject

[src/flexray_injector_rules.h](../src/flexray_injector_rules.h) defines a synthetic example. It caches an observed FID8 template with an 18-byte payload, prepares a replacement on a real FID6 header when `cycle & 3 == 2`, and authorizes DMA in that frame's real frame-end callback. ON outputs to FR1; OFF outputs to FR3. Only the first four payload bytes are replaced. All other payload bytes are preserved, and the current cycle and FlexRay frame CRC are updated.

Host commands are consumed once and expire after 100 ms. Disabling injection clears the queue and prepared state. Existing queue synchronization is retained. The target slot must provide real frame edges to drive the original injector, within an assigned, non-overlapping TDMA schedule.

## Frame build

`src/flexray_frame_gen.c`, `flexray_slot_schedule.c`, and `flexray_frame_gen_packet.c` retain the implementation present at extraction. Hardware FSS markers measure slot intervals using the lowest five observed static IDs. After lock, acquisition DMA stops and the sampling SM is reused for continuous slot pacing. Short reference phase captures adjust only the next cycle boundary. Each correction is consumed once, within-cycle slot intervals remain fixed, and the cycle tail produces no pace events.

FR2 reserves FIDs 0xC/0xD by default, with 18-byte payloads, rep4/base3, `static_max_id=0x10`, and a target cycle period of 5 ms. Once enabled and synchronized, the reserved slots transmit every cycle. Missing payload data selects a null frame at the same deadline. Null frames clear NFI/PPI, zero the payload, and retain the current cycle and correct frame CRC. Each template stores one packet plus 64 three-byte CRCs.

The preceding logical slot preloads the FIFO. Independent packet DMA and a finite framing waveform drive the original injector. The GPIO17 ownership signal controls TXEN through a four-instruction controller for the full frame. IRQ7 excludes local echo. A separate completion IRQ releases ownership after post-FES idle. Real RX IRQs do not replace the frame build completion notification.

## Timing and validation

The extracted branch preserves all FlexRay PIO sources, the streamer, the slot scheduler, and the frame build C implementation. See [firmware modes](firmware-modes.md) for resources, [actions](frame-gen-actions.md) for host commands, and [resynchronization](frame-gen-resync.md) for recovery behavior.

Host tests cover the actual C control path, IRQ/injection isolation in both modes, packet/null/cycle CRC handling, PIO instruction cycles, and streamer equivalence. Compilation and simulation do not establish electrical validation of this branch. No renewed flashing or logic analyzer tests have been performed for this branch.
