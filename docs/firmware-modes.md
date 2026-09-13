# Firmware modes

Select either mode at compile time with `FLEXRAY_FRAME_GEN`, which defaults to `OFF`.

| Mode | Features | PIO0/1/2 instruction words | DMA channels |
|---|---|---|---|
| ON | FR1/FR2 bridge, inject, FR2 static frame build | Acquisition: 13/32/32; runtime: 30/32/32 | 10 |
| OFF | FR1..FR4 bridge, source identification, inject | 28/28/22 | 8 |

PIO0 SM2 is unused. ON uses GPIO16/17 for local framing and ownership, and GPIO10 for the FSS marker. OFF restores the FR3/FR4 pins. See [board_config.h](../src/board_config.h) for pin definitions. Source flags and four-channel routing retain the original implementation.

OFF compiles out the frame build C modules, PIO programs, and DMA. Injection uses real header/frame-end callbacks and a 100 ms host command timeout. Frame build has its own enable switch, mailboxes, and DMA.
