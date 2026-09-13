# Static frame generation synchronization loss and sleep recovery (protocol v7)

The DUT can remain powered with frame generation enabled. When the bus sleeps or reference timing becomes unreliable, firmware pauses local frame generation and automatically resynchronizes when the bus returns. No host reconfiguration, repeated enable command, or manual `cold_reset` is required for this recovery.

See the [frame generation design](frame-gen-design.md) for the architecture and the [validation summary](bench-validation.md) for bench coverage.

## Synchronization states

- While synchronized, static slots follow the fixed hardware FSS-anchored pace. Bounded FSS phase captures adjust only the next cycle boundary; within-cycle slot intervals remain fixed. C/D rep=4, base=3, and null rules remain unchanged.
- A discontinuity between an actual static frame's cycle and the hardware pace's predicted label immediately signals synchronization loss. CPU latency may advance the slot before a header interrupt is handled; such slot mismatches are diagnostic only and do not reset timing. The hardware FSS calibration watchdog detects phase loss.
- Missing valid FSS phase calibration briefly retains prediction. After three learned cycles (about 15 ms on the target network), synchronization is considered lost. This is a loss watchdog; CPU time never determines a transmitted FSS edge.
- On loss, stop new pace events, clear unconsumed triggers, and wait 1 microsecond for triggers already crossing PIO boundaries to settle. Cancel packets that have not started. Frames already transmitting finish through the inducer's independent DONE IRQ2 without truncating TXEN ownership. Real RX frame-end callbacks do not complete frame generation.
- Reuse the existing sampling SM/DMA to learn slot and cycle periods from the lowest five distinct static FIDs actually observed. The reference may change to another valid ID. A new hardware FSS starts pacing, a real header binds FID/cycle, and one valid phase calibration is required before local output resumes. Recovery observes the bus again even if the previous configuration specified fixed measurements; it does not reuse the old period.

While synchronized, missing host data still produces null frames on time. During synchronization loss, both normal and null frames pause because a null frame in the wrong slot can also collide. Enable intent is retained. After recovery, transmission resumes with null frames unless fresh payload data is available. Disable still turns local output off.

## Data and concurrency

The synchronization generation increments on loss. Previously applied data becomes invalid immediately. Each USB/NCM target mailbox and diagnostic A1 template carries the generation from the start of construction, preventing stale data from being published across sleep recovery. New payloads during loss/recovery return NOT_SYNCED; a mailbox with a pending command may return BUSY first.

Publication banks retain existing SPSC and `prepared_packet` ownership guarantees. No locks or interrupt masking are added. Templates still store one packet plus 64 three-byte CRCs, with no cyclic packet DMA added.

Real reception in ON/OFF uses the shared 28-word streamer. The original forwarder/injector program, MITM real-header/frame-end triggers, and failsafe are retained. See the [design notes](frame-gen-design.md) for context on the streamer and independent DONE path. The 1 microsecond shutdown wait appears only in the exceptional recovery path to handle concurrent CPU decisions and hardware transmission starts. It is not inserted into normal PIO frame timing and does not pause an ACTIVE framing SM.

## Status interface

Protocol version 7 expands the status body from 96 to 108 bytes, using Python format `<IHH16I2i2I2H4I>`. The appended fields are `sync_losses`, `sync_loss_reason`, and `header_slot_mismatches`.

Loss reasons are 1=calibration timeout and 2=actual cycle discontinuity. `header_slot_mismatches` records only the difference in slot labels when a header is handled. Flag bit 8 is `resyncing`. Both `enabled` and `resyncing` may be true: enable intent is retained while local output waits for synchronization. `locked`/`pacing` describe acquisition and pacing state; completed recovery also requires `resyncing=0`. Error code 7 is `NOT_SYNCED`; clients must use the matching protocol version.

`sync_loss_reason` retains the most recent loss reason after successful recovery. `sync_losses` is cumulative. Missing references during sleep increase `phase_missed`; distinguish sleep-window increments from increments during normal synchronized operation. See the [validation summary](bench-validation.md) for measurement coverage.

## Acquisition candidates and bootstrap

Candidates are sorted by the FID values observed during the current acquisition, retaining at most the lowest five distinct static FIDs. A newly observed lower FID replaces the largest candidate. Previously valid cycle measurements are retained. Candidates freeze after lock and clear when acquisition restarts after synchronization loss.

Acquisition can use fewer than five candidates, but automatic slot-length measurement requires at least two distinct FIDs in the same cycle. One candidate becomes the reference after lock. There is no vote across five references every cycle, and IDs need not be 1..5. Candidates remain bounded by `static_max_id`, excluding dynamic-segment frames. The source implementation included bench checks for the lowest-five selection, cold startup with high reference IDs, and sleep recovery; see the [validation scope](bench-validation.md) for this extracted branch's limits.

Candidate filtering uses `restart_bootstrap()`. Before a reference is bound, a non-candidate FID or invalid FSS/header resets only the unbound pace/FSS wait and phase sampling, retaining allocated PIO programs and DMA. Previously, each non-candidate header invoked a full `reset_timing()`, adding excessive initialization work under dense static traffic. On the source bench, prediction could remain at cycle 15 when FID2 cycle 16 arrived, causing repeated reacquisition. Full resource reconstruction remains in use for actual acquisition/runtime mode transitions.
