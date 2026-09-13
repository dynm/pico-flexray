# Demo branch validation scope

See the [README](../README.md#validation) for this branch's validation record. Host C tests use register/FIFO fakes with AddressSanitizer and UndefinedBehaviorSanitizer enabled. PIO tests check cycle semantics. ON/OFF builds use the RP2350 ARM toolchain.

This demo branch has not undergone renewed electrical bench validation. For bench tests, use OpenOCD `cold_reset` after flashing the RP2350, verify USB re-enumeration and DMA operation, then use a signal generator and logic analyzer to check FSS, TXEN, and echo exclusion.
