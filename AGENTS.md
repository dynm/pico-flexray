# FlexRay timing invariants

- Preserve the established streamer/forwarder instructions and their tested
  receive/forward timing. Prefer spare SMs reusing the original programs.
  Make unavoidable instruction changes explicitly in the PIO source; do not
  patch instructions at runtime. Identify and justify every difference.
- Select mutually exclusive firmware modes with `FLEXRAY_FRAME_GEN`: ON keeps
  FR1/FR2 plus independent static frame generation; OFF restores the established FR1..FR4
  bridge, source identification and routing. Keep shared fixes in one codebase.
  The four-channel firmware must compile out frame generation code, PIO programs and DMA;
  do not add runtime mode checks or attempt four-channel/build pin sharing.
- Keep MITM on its established real-header / frame-end injection callbacks and
  command failsafe. frame generation uses an independent pace, spare SMs and its own packet
  DMA; it must not consume MITM overrides, authorize MITM DMA, or replace the
  existing trigger rules with pace. Local framing IRQs belong only to build.
- FlexRay is TDMA. The forwarding and injection timing assumes assigned,
  non-overlapping slots; preserve this assumption when changing the firmware.
- Anchor static transmission to hardware-observed FSS timing. Header and
  frame-end interrupts may prepare/authorize work, but their arrival times must
  not determine the transmitted FSS edge.
- Qualify idle, then WAIT for TSS and FSS edges as in the streamer. Do not
  combine edge detection with a multi-clock measurement polling loop. Measure
  slot intervals only during acquisition; after lock, stop acquisition DMA and
  replace/reuse the measurement SM for transmission timing or diagnostics.
- Learn the static-slot and cycle periods using hardware FSS observations of
  the lowest five distinct observed static IDs during acquisition. Reuse the sampling SM for a continuous static
  slot pace after lock, including absent and unreserved slots. Use short FSS
  phase captures to reshape the next cycle boundary, not an unbounded PLL.
  The target cycle is 5 ms / 200 Hz; cycle count 0..63 is only a label. Keep
  every within-cycle static interval fixed, emit no slot pace in the cycle tail,
  and consume each phase correction once. Missing references retain prediction.
  Only selected reservations induce the original injector; pace itself stays
  off the bus. Target IDs may precede the reference ID. Local TX currently
  supports IDs 2..static_max_id only; FID 1 is deferred. At the preceding
  logical slot, preload the original injector FIFO and prepare one signal for
  the next pace. Do not add gap tables, cyclic replay, or packet-selector DMA.
- Keep the selected TSS duration separate from the FSS deadline. Account for
  every PIO instruction and fixed signal-path delay in the timing calculation.
- FR2 local transmission must own TXD and active-low TXEN for the full frame.
  Reuse the bridge's existing IRQ7 echo exclusion before TSS through post-FES
  idle, and suppress local RXD echo in the FSS capture as well as forwarding.
  Preserve real traffic in intervening TDMA slots; do not mute reception for
  the entire anchor-to-target reservation.
- For an enabled, synchronized static-slot reservation, unavailable or late
  payload data must select a prebuilt null frame at the same FSS deadline.
  Do not turn payload lateness into a skipped slot or a delayed frame. Null
  frames clear NFI and PPI, retain the static payload length, zero the payload,
  and carry the current cycle and correct CRC.
- Store one packet plus 64 three-byte frame CRCs per frame generation template, including
  null. Set the cycle/CRC bytes before arming DMA; keep its source unchanged
  through frame end. Do not expand templates into 64 complete frames.
- Use existing single-producer/single-consumer and slot ownership guarantees.
  Do not add mutexes, spinlocks, interrupt masking, or per-bit arbitration unless
  a concrete concurrent access requires it; explain that access when adding one.
- PIO cycle simulation and firmware builds do not establish electrical timing.
  Verify new timing paths with a signal generator and logic analyzer before
  describing them as hardware-validated.
- On the RP2350 bench, use the Raspberry Pi OpenOCD target's `cold_reset`
  after flashing or when restarting hardware tests. Ordinary `reset` uses
  SYSRESETREQ and can leave DMA/USB in a bad state with this Debug Probe.
  Verify USB re-enumeration and DMA operation after the full reset.
