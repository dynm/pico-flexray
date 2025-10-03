# Pico-FlexRay: A Low-Cost FlexRay Man-in-the-Middle (MITM) Module Based on Raspberry Pi Pico

## Preamble: The FlexRay Challenge

In the world of autonomous driving and driver-assistance systems, the OpenPilot project has brought powerful capabilities to many vehicles. However, support for cars using the FlexRay bus has always been lacking. I believe this isn't because the FlexRay protocol is inherently too complex, but rather that the expensive, specialized hardware and convoluted development workflows have deterred developers, leading to a shortage of community effort in this area.

### FlexRay vs. CAN: The MITM Hurdle

Compared to familiar protocols like CAN or CAN-FD, performing a Man-in-the-Middle (MITM) attack on FlexRay is significantly more challenging.

*   **CAN/CAN-FD**: These protocols use a bus arbitration mechanism (CSMA/CD) and are not time-sensitive. We can easily disconnect a node and bridge the two ends with two CAN controllers and transceivers. Acting like a network switch, this setup can store and forward bidirectional data, allowing us to intercept, modify, and inject specific frames to achieve lateral and longitudinal control.

*   **FlexRay**: This is a protocol based on **TDMA (Time-Division Multiple Access)**. Its high efficiency comes from a pre-defined time schedule that dictates which node (ECU) can transmit a specific Frame ID in a specific time slot. This mechanism avoids the overhead of bus arbitration and dramatically increases bus utilization. However, this very time-sensitivity means we cannot simply "store-and-forward" data. Any minute delay could destroy the entire network's synchronization. We must transmit a specific Frame ID at a precise moment.

> **Note**: For this discussion, we are temporarily ignoring the FlexRay Dynamic Segment, which uses a mini-slotting mechanism to achieve a form of bus access arbitration.

## Inspiration and Acknowledgements

Before starting my project, I was greatly inspired by the pioneering work of others who explored FlexRay using FPGAs. All of my explorations are built upon their foundational efforts, and I extend my sincere gratitude to the authors of these projects:

*   **Audi A4 B8 FlexRay Hacking**: [https://icanhack.nl/blog/audi-flexray/](https://icanhack.nl/blog/audi-flexray/)
*   **Hacking an Audi: a man-in-the-middle attack on FlexRay**: [https://comma-ai.medium.com/hacking-an-audi-performing-a-man-in-the-middle-attack-on-flexray-2710b1d29f3f](https://comma-ai.medium.com/hacking-an-audi-performing-a-man-in-the-middle-attack-on-flexray-2710b1d29f3f)
*   **FPGA based FlexRay Interceptor**: [https://github.com/dolson8874/flexray-interceptor/](https://github.com/dolson8874/flexray-interceptor/)

## The Hardware Selection Journey: From AG32 to Pico

My initial attempt involved a Chinese-made MCU, the **AG32VF407**. It's pin-compatible with the STM32F407 and innovatively includes integrated CPLD resources, allowing for flexible peripheral expansion. However, its development environment was complex to set up, and the workflow required juggling multiple software tools. After a preliminary attempt, I decided to look for a more streamlined alternative.

The breakthrough came when I stumbled upon a Nintendo Switch hacking project that used a **Raspberry Pi Pico** for a Fault Injection attack. I was immediately curious how this little MCU managed high-precision bit-banging and was introduced to its magical **PIO (Programmable I/O)** peripheral.

After getting my hands on a Pico dev board, I experienced an unprecedented level of development convenience:

*   **Cross-Platform Environment**: Simply installing the official extension in VS Code automatically deploys the entire toolchain.
*   **One-Click Workflow**: The extension's built-in buttons make everything from compiling to flashing incredibly simple.
*   **Effortless Flashing**: No JTAG/SWD needed. Just hold the `BOOTSEL` button on the Pico while connecting it to a computer, and it appears as a mass storage device. Drag and drop the `.uf2` firmware file onto it, and you're done.

This elegant and efficient development experience solidified my decision to use the Pico to tackle FlexRay.

## Step 1: Implementing a Read-Only FlexRay Sniffer

### FlexRay Physical Layer & Frame Structure

Before we dive in, let's have a quick recap of FlexRay basics.

*   **Physical Signal**: FlexRay uses a differential signal, defining "Dominant" as logic 0 and "Recessive" as logic 1. The bus is high when idle. A standard bit duration is 100ns.

*   **Frame Structure**: A FlexRay frame consists of `TSS + FSS + n * (BSS + 8-bit Data) + FES`.
    *   `TSS` (Transmission Start Sequence): A sequence of several `0`s marking the start of transmission.
    *   `FSS` (Frame Start Sequence): A single `1`.
    *   `BSS` (Byte Start Sequence): Separates data bytes, fixed as `10`. This means every 8-bit payload byte is prefixed with `10`.
    *   `FES` (Frame End Sequence): `01`, after which the bus returns to idle.

    The actual data is contained within the `n * (BSS + 8-bit Data)` part. After stripping the BSS, we get the true frame content, structured as follows:
    1.  **Header (40 bits)**
        *   **Indicators (5 bits)**: Includes the `Null Indicator`, which, when set to `1`, signifies an invalid payload.
        *   **Frame ID (11 bits)**: The unique identifier for the frame.
        *   **Payload Length (7 bits)**: Length of the payload in FlexRay Words (1 Word = 2 Bytes). Max length is `(2^7 - 1) * 2 = 254` bytes.
        *   **Header CRC (11 bits)**: Protects the Indicators, Frame ID, and Length.
        *   **Cycle Count (6 bits)**: A counter for the communication cycle. This value is **not** protected by the Header CRC but is crucial for FlexRay Multiplexing.
    2.  **Payload (0 - 254 bytes)**: The actual application data.
    3.  **Frame CRC (24 bits)**: Protects both the Header and Payload.

### PIO-based Implementation

With the frame structure understood, we can start parsing it with the Pico's PIO. First, we need a FlexRay transceiver to convert the differential bus signals into TTL-level signals the Pico can understand. I used a **TJA1082**, which is pin-to-pin compatible with chips like the TLE9222 and NCV7383.

My initial thought was to have the PIO State Machine detect the TSS by looking for a continuous stream of low bits, similar to how an **Active Star Coupler** works. However, this approach would truncate the beginning of the TSS. As a MITM module, we cannot alter the network configuration, so preserving the original signal length is paramount.

My final, more robust solution was:

1.  **Idle State Detection**: Implement an Idle state detector in the PIO State Machine. If the bus stays high for 11 bit-times, it's considered Idle.
2.  **TSS Detection**: As soon as the bus transitions from Idle to low, we assume the TSS has begun. The PIO runs at 100MHz, and I used center-point sampling. This introduces a 50ns phase shift (half a bit-time) but doesn't alter the signal shape, which is perfectly tolerable for FlexRay.
3.  **Edge Alignment**: After detecting the TSS, use a `wait` instruction to sync on the rising edge of the FSS. This precisely aligns our sampling clock with the FlexRay signal edge, eliminating cumulative error.
4.  **Data Stream Parsing**: Next, use a polling loop with a timeout to detect the falling edge of each BSS. After each detection, sample 8 bits of data and push them to the PIO FIFO.
5.  **DMA Transfer**: The PIO's FIFO works seamlessly with DMA, allowing the BSS stream to be automatically written to a specified location in RAM without CPU intervention.
6.  **Interrupt Notification**: Upon detecting the FES, which marks the end of a frame, the PIO sends an IRQ to the CPU. The CPU's interrupt handler then processes the fully received FlexRay frame.

> **A PIO Pro-Tip**: The `wait pins` instruction should be used with caution as it has no timeout feature. If the expected signal never arrives, the program will stall indefinitely. For safety-critical applications, it's better to use a loop that checks the I/O state combined with a `jmp` and a counter to implement a timeout.

With this, we've successfully built a FlexRay sniffer. We can now connect the transceiver in read-only mode to any FlexRay bus (BM/BP) in the vehicle and start exploring the data stream.

## Step 2: Conquering the Challenge of Transparent Forwarding

Read-only sniffing isn't enough. FlexRay is a broadcast bus, so the data seen at any node is a mix from all ECUs. To distinguish which data comes from which ECU, we must implement a true MITM module.

This requires:
1.  Physically disconnecting the target ECU from the vehicle's bus.
2.  Connecting the ECU and the vehicle bus to two separate transceivers on our MITM module.
3.  Having the Pico forward data in both directions:
    *   When data arrives from the vehicle bus, forward it to the ECU.
    *   When the ECU transmits data, forward it to the vehicle bus.

My initial implementation was naive: I used the PIO's `side-set` feature to toggle the transceiver's transmit enable pin (`TX_EN`) based on the bus idle state. However, I overlooked a fatal flaw: **the transceiver echoes its own TX signal back on its RX pin!**

This echo completely broke my bidirectional forwarding logic. A delayed, forwarded signal was reflected back, superimposed on the original signal, and then sent back to the source. This created a catastrophic feedback loop that wreaked havoc on the bus communication, causing the instrument cluster to light up with dozens of fault codes. Thankfully, clearing the codes and letting the car sleep for a few minutes resolved the issue.

After a painful debugging session, I began the arduous task of figuring out how to prevent this echo. The PIO instruction set is extremely limited; I even considered adding external transistors to physically disconnect the RX line based on the `TX_EN` state.

While re-reading the Pico datasheet, I had a eureka moment: **the `irq set` and `wait irq` instructions can be used to implement a perfect mutex!**

The final solution was incredibly clean and elegant:
*   I created two identical PIO State Machines, one for the "Vehicle -> ECU" direction and the other for "ECU -> Vehicle".
*   They both compete for the same IRQ flag.
*   When one State Machine begins forwarding, it first `set`s the IRQ. When it's done, it clears the IRQ (or more accurately, waits for a signal that clears it).
*   The other State Machine `wait`s on this IRQ before starting its work. If the IRQ is already taken, it will be stalled until the first State Machine is finished.

**With the addition of just two instructions, I achieved mutually exclusive operation and cleanly solved the echo problem.**

The Pico-FlexRay was now a "transparent proxy," invisible to the vehicle's network. By tracking which transceiver a frame came from (`transceiver_index`), we can easily distinguish the source of the data. By inserting it between the ADAS module and the vehicle bus, we can now precisely analyze which frames the ADAS module uses to control the car.

## Step 3: Implementing the Man-in-the-Middle Attack

With all the groundwork laid, only the final step remained: substituting specific frame data.

The logical approach is to simply drop the original frame with a specific ID during forwarding and inject our own modified version.

My first idea was to perform **on-the-fly modification**: detect the target Frame ID in the PIO State Machine and instantly replace parts of the payload as it was being forwarded. However, this proved unfeasible on my car because most ADAS-related payloads use **AutoSAR E2E (End-to-End) protection**.

E2E protection typically adds an `Alive Counter` and a `CRC` to the payload. A common format is `[CRC8] [Alive Counter] [Payload Data]`. This means the CRC8 checksum is at the *beginning* of the payload, but its calculation covers the *entire* payload. We can't pre-calculate the correct CRC8 without knowing the full payload content in advance.

So, I shifted to a **"Cache-Modify-Inject"** strategy:

1.  **Cache**: After the BSS Streamer parses the frame, cache all incoming frames in memory.
2.  **Prepare**: When the target frame to be modified is **about to arrive** (triggered by the end of the preceding frame), we prepare the data for injection.
3.  **Modify**: Read the most recently cached version of the target frame from memory, modify its payload, and then recalculate and fix all related fields:
    *   `Cycle Count`
    *   E2E `CRC8`
    *   E2E `Alive Counter`
    *   `FlexRay Frame CRC24`
4.  **Inject**: When the time slot for the target frame arrives, transmit this newly constructed frame.

This process requires knowing the `previous_id` and `target_id`, which are easily found using tools like `cabana` (FlexRay Frame IDs are typically sequential).

### Injection Strategies and Trade-offs

My current implementation supports two injection modes:

*   **Partial Modification**
    *   **Pros**: Safer. We can modify only the direct control commands while preserving other signals in the frame, such as `gating` signals that enable or disable ADAS commands. This way, even if OpenPilot sends an erroneous command, it won't be executed if the car's ADAS function isn't active. Another benefit is preserving native features. If the car's AEB (Automatic Emergency Braking) signal is in the same frame as longitudinal control, our module can keep the native AEB active while OpenPilot is running (with just a one-cycle delay).
    *   **Latency**: Since we modify a frame cached from the previous cycle, there is a one-cycle latency (typically 2ms - 20ms, depending on the manufacturer's configuration).

*   **Full Replacement**
    *   **Pros**: More flexible. It allows for finer control from the OpenPilot side using `opendbc`, such as adjusting steering damping at different speeds.
    *   **Cons**: Requires more comprehensive signal reverse-engineering and frame construction.

## Current Limitations and Future Work

The only known issue at present is that **the frame with the lowest ID cannot be injected**. My injection trigger relies on the arrival of the "previous frame," and the very first frame has no predecessor.

However, I believe this is a rare requirement. For functional safety and design reasons, critical ADAS commands are highly unlikely to be transmitted using the very first Frame ID on the bus.

If a future scenario absolutely requires injecting the first frame, it can be achieved by synchronizing with the FlexRay Communication Cycle for precise timing—a potential direction for future improvement.

## Conclusion

By leveraging the powerful and user-friendly PIO peripheral of the Raspberry Pi Pico, we have successfully created a fully functional, low-cost FlexRay Man-in-the-Middle module. It can act as a transparent proxy for bus analysis and also modify and inject frames in real-time, paving the way for running OpenPilot on FlexRay-based vehicles and for further research into automotive network security. I hope this sharing can be helpful to the community.