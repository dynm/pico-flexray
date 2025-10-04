# Pico-FlexRay: 基于 Raspberry Pi Pico 的低成本 FlexRay 中间人攻击（MITM）模块

## 前言：为什么是 FlexRay？

在自动驾驶和辅助驾驶领域，OpenPilot 项目为许多车型带来了强大的功能。然而，对于采用 FlexRay 总线的车辆，OpenPilot 的支持一直不尽人意。我个人认为，这并非因为 FlexRay 通信协议本身过于复杂，而是其昂贵的专用硬件和复杂的开发流程，劝退了大多数开发者，导致社区缺乏足够的开发力量投入其中。

### FlexRay vs. CAN：MITM 的挑战

与我们熟悉的 CAN 或 CAN-FD 总线相比，FlexRay 的中间人攻击（MITM）要困难得多。

*   **CAN/CAN-FD**：采用总线仲裁机制（CSMA/CD），对时间不敏感。我们可以轻松地断开一个节点，用两个 CAN 控制器和收发器连接断开的两端，像网络交换机一样对双向数据进行“存储-转发”。通过这种方式，我们可以拦截、修改并发送特定的数据帧，从而实现对车辆横向（Lateral）和纵向（Longitudinal）的控制。

*   **FlexRay**：这是一种基于 **TDMA（时分多址）** 的协议。它的高效源于一张预先定义好的时间调度表，约定了网络中每个节点（ECU）只能在特定的时间片（Time Slot）内发送特定 ID 的数据帧。这种机制避免了总线仲裁的开销，极大地提高了总线利用率。但也正因为这种对时间的高度敏感性，我们无法简单地进行“存储-转发”，任何微小的延迟都可能破坏整个网络的同步，导致通信失败。我们必须在精确的时刻，发送精确的 Frame。

> **注**：此处我们暂时不考虑 FlexRay 的动态段（Dynamic Segment），它通过迷你的时隙（Mini-Slotting）实现了一种类似仲裁的总线访问机制。

## 灵感与致谢

在开始我的项目之前，许多先行者基于 FPGA 的探索给了我巨大的启发。我所有关于 FlexRay 的研究都建立在他们工作的基础之上，在此向这些项目的作者们表示衷心的感谢：

*   **Audi Q8 FlexRay Hacking**: [https://icanhack.nl/blog/audi-flexray/](https://icanhack.nl/blog/audi-flexray/)
*   **Hacking an Audi: a man-in-the-middle attack on FlexRay**: [https://comma-ai.medium.com/hacking-an-audi-performing-a-man-in-the-middle-attack-on-flexray-2710b1d29f3f](https://comma-ai.medium.com/hacking-an-audi-performing-a-man-in-the-middle-attack-on-flexray-2710b1d29f3f)
*   **FPGA based FlexRay Interceptor**: [https://github.com/dolson8874/flexray-interceptor/](https://github.com/dolson8874/flexray-interceptor/)

## 硬件选型之路：从 AG32 到 Pico

我最初的尝试是使用一款国产 MCU——**AG32VF407**。它与 STM32F407 引脚兼容，并且创新地内置了部分 CPLD 资源，可以灵活地扩展外设，潜力巨大。然而，其开发环境的部署相对复杂，整个开发流程需要协同使用多款软件，在初步尝试后，我决定寻找更便捷的方案。

转机出现在我偶然看到的一個 Nintendo Switch 破解项目，该项目利用 **Raspberry Pi Pico** 实现了 Fault Injection 攻击。我立刻对 Pico 如何实现高精度的 Bit-banging 产生了浓厚兴趣，并因此了解到了它强大的 **PIO (Programmable I/O)** 外设。

在购买了 Pico 开发板并上手尝试后，我体验到了前所未有的开发便利性：

*   **跨平台开发环境**：仅需在 VS Code 中安装一个官方插件，即可自动完成所有工具链的部署。
*   **一键式操作**：插件内置了快捷按钮，从编译到刷写，整个流程一气呵成。
*   **极简的固件刷写**：无需 JTAG/SWD 等调试器。只需按住 Pico 上的 `BOOTSEL` 按钮再连接电脑，它就会模拟成一个U盘（Mass Storage Device）。将编译好的 `.uf2` 固件文件拖入这个U盘，即可完成刷写。

这简洁高效的开发体验，让我坚定了使用 Pico 来攻克 FlexRay 的决心。

## 第一步：实现 FlexRay 帧的只读嗅探

### FlexRay 物理层信号与帧结构

在动手之前，我们先快速回顾一下 FlexRay 的基础知识。

*   **物理信号**：FlexRay 总线为差分信号，定义“显性”（Dominant）为逻辑 0，“隐性”（Recessive）为逻辑 1。总线空闲（Idle）时为高电平。一个 bit 的标准时长为 100ns。

*   **帧结构**：一个 FlexRay 帧由 `TSS + FSS + n * (BSS + 8-bit Data) + FES` 构成。
    *   `TSS` (Transmission Start Sequence)：持续数个 bit 的 `0`，标志着传输开始。
    *   `FSS` (Frame Start Sequence)：一个 `1`。
    *   `BSS` (Byte Start Sequence)：用于分隔数据字节，固定为 `10`。这意味着每 8-bit 的 payload 前都会插入一个 `10` 前缀。
    *   `FES` (Frame End Sequence)：`01`，之后总线回到 Idle 状态。

    FlexRay 的有效数据躺在 `n * (BSS + 8-bit Data)` 中。去掉 BSS 后，我们得到真正的帧内容，其结构如下：
    1.  **Header (40 bits)**
        *   **Indicators (5 bits)**: 其中包含 `Null Indicator`，当其为 `1` 时，表示该帧的 payload 无效。
        *   **Frame ID (11 bits)**: 帧的唯一标识符。
        *   **Payload Length (7 bits)**: payload 的长度，单位是 FlexRay Word (1 Word = 2 Bytes)。最大长度为 `(2^7 - 1) * 2 = 254` 字节。
        *   **Header CRC (11 bits)**: 保护 Indicators, Frame ID, Length。
        *   **Cycle Count (6 bits)**: 循环计数器。这个值**不被** Header CRC 保护，但在 FlexRay Multiplexing（帧复用）中至关重要。
    2.  **Payload (0 - 254 bytes)**: 真正的应用数据。
    3.  **Frame CRC (24 bits)**: 保护 Header 和 Payload。

### 基于 PIO 的实现

了解了帧结构后，我们就可以开始用 Pico 的 PIO 来解析它了。首先，需要一个 FlexRay 收发器（Transceiver）将总线上的差分信号转换为 Pico 能识别的 TTL 电平。我使用的是 **TJA1082**，它与 TLE9222, NCV7383 等芯片 pin-to-pin 兼容。

最初，我想到用 PIO State Machine 来探测 TSS持续n个Low来表示Frame开始，就像FlexRay网络中 **Active Star Coupler** 那样，但是会截断（Truncate）TSS 的一部分前导 `0`。作为一个 MITM 模块，我们不能修改网络配置，因此必须尽可能保持信号的原始长度。

最终，我采用了更可靠的方案：

1.  **Idle 状态检测**：在 PIO State Machine 中实现 Idle 状态检测。只要总线持续 11 个 bit 的高电平，就判定总线进入 Idle。
2.  **TSS 探测**：一旦总线由 Idle 变为低电平，就认为 TSS 开始了。PIO 以 100MHz 的频率工作，我采用中心点采样法，这会引入 50ns 的相位偏移（半个 bit 时间），但不会改变信号波形，对于 FlexRay 来说完全可以容忍。
3.  **边沿对齐**：探测到 TSS 后，使用 `wait` 指令等待 FSS 的上升沿。这可以精确地将我们的采样时钟与 FlexRay 信号边沿对齐，消除累积误差。
4.  **数据流解析**：接下来，通过带超时的轮询来探测每个 BSS 的下降沿，每次探测到后，采样 8-bit 数据，并通过 PIO FIFO 发送出去。
5.  **DMA 传输**：PIO 的 FIFO 可以与 DMA 无缝配合，无需 CPU 干预，BSS Stream 会被自动写入内存中的指定位置。
6.  **中断通知**：当探测到 FES，标志着一个帧的结束时，PIO 会向 CPU 发送一个 IRQ 信号。CPU 在中断服务程序（ISR）中开始处理刚刚接收到的完整 FlexRay 帧。

> **PIO 开发避坑指南**：`wait pins` 指令需要谨慎使用，它没有超时功能。如果等待的信号永远不来，程序将永久阻塞。对于某些安全关键场景，需要使用循环检测 I/O 状态，并结合 `jmp` 和计数器来实现带超时的等待。

至此，我们已经成功实现了一个 FlexRay 嗅探器。现在可以把收发器以只读模式（Read-only）连接到车辆的任意 FlexRay 总线（BM/BP）上，开始探索车辆的数据流了。

## 第二步：攻克透明转发的难关

只读嗅探还不够，因为 FlexRay 是一个广播总线，在任意节点观察到的数据都是所有 ECU 数据的混合。为了区分特定数据来自哪个 ECU，我们必须实现一个真正的中间人（MITM）模块。

这意味着：
1.  将目标 ECU 与车辆总线物理断开。
2.  将 ECU 和车辆总线分别连接到 MITM 模块的两个收发器上。
3.  Pico 负责在两个方向上进行数据转发：
    *   当车辆总线有数据时，转发给 ECU。
    *   当 ECU 发送数据时，转发给车辆总线。

我最初的实现非常简单：通过 PIO 的 `side-set` 功能，根据 Idle 状态的变化来控制收发器的发送使能引脚（`TX_EN`）。然而，我忽略了一个致命问题：**收发器会把自己的 TX 信号通过 RX 反射（echo）回来！**

这个 echo 彻底破坏了我的双向转发逻辑。一个方向上转发出去的带延迟的信号，被反射回来，再与原始信号叠加，最终发送回信号的来源端。这形成了一个灾难性的反馈循环，严重干扰了总线通信，导致车辆仪表盘爆出了几十个故障码。幸运的是，在清除故障码并让车辆休眠几分钟后，一切恢复了正常。

在经历了痛苦的调试后，我开始艰难地探索如何避免 echo。PIO 的指令集非常有限，我甚至一度想通过增加外部晶体管来根据 `TX_EN` 的状态物理切断 RX 信号。

在反复阅读 Pico 的数据手册时，我突然茅塞顿开：**`irq set` 和 `wait irq` 指令可以实现一个完美的互斥锁（Mutex）！**

最终的解决方案极其简洁和优雅：
*   我创建了两个完全相同的 PIO State Machine，一个负责“车辆 -> ECU”方向，另一个负责“ECU -> 车辆”方向。
*   它们共同竞争同一个 IRQ 标志位。
*   当一个 State Machine 开始转发数据时，它会首先 `set` 这个 IRQ。当它转发结束时，再 `clear` 这个 IRQ（实际上是 `wait` 另一个清除 IRQ 的信号）。
*   另一个 State Machine 在开始工作前，会 `wait` 在这个 IRQ 上。如果 IRQ 已经被占用，它就会被阻塞，直到前一个 State Machine 完成工作。

**通过增加两条指令，我完美地实现了互斥运行，从根本上解决了 echo 问题。**

现在，Pico-FlexRay 终于成为了一个对车辆无感知的“透明代理”。通过记录数据帧来自哪个收发器（`transceiver_index`），我们可以轻松地区分信号的来源。将它串联在 ADAS 模块和车辆总线之间，就能精确分析出 ADAS 是通过哪些 Frame 来控制车辆的了。

## 第三步：实现中间人攻击

万事俱备，只差最后一步：替换特定 Frame 的数据。

很自然地，我们只需要在转发数据时，丢弃原始的特定 ID 的 Frame，然后注入我们修改后的 Frame。

我最初的想法是进行 **On-the-fly 修改**：在 PIO State Machine 中实时探测目标 Frame ID，然后在转发过程中即时替换 payload 的特定部分。然而，在我的车上这并不可行。因为大多数 ADAS 相关的 payload 都使用了 **AutoSAR E2E (End-to-End) 保护**。

E2E 保护通常会在 payload 的开头和/或结尾加入 `Alive Counter` 和 `CRC`。例如，常见的格式是 `[CRC8] [Alive Counter] [Payload Data]`。这意味着 CRC8 校验值在 payload 的最前面，但它的计算范围却包含了整个 payload。我们无法在不知道完整 payload 的情况下预先计算出正确的 CRC8。

因此，我转变思路，采用了 **“缓存-修改-注入”（Cache-Modify-Inject）** 策略：

1.  **缓存（Cache）**：在 BSS Streamer 解析出帧数据后，将所有帧都缓存到内存中。
2.  **准备（Prepare）**：当需要被修改的目标帧（Target Frame）**即将到来**时（通常是它的前一个 Frame 结束时触发），我们开始准备要注入的数据。
3.  **修改（Modify）**：从缓存中读取最近一次收到的同 ID 目标帧，修改其 payload 内容，然后重新计算并修复所有相关字段：
    *   `Cycle Count`
    *   `CRC8` (E2E)
    *   `Alive Counter` (E2E)
    *   `FlexRay Frame CRC24`
4.  **注入（Inject）**：在目标帧所属的时间片到来时，发送这个全新构造的帧。

这个过程需要知道前后两个 Frame 的 ID（`previous_id` 和 `target_id`），这些信息在 `cabana` 这类工具中可以轻易观察到（FlexRay Frame ID 通常是递增的）。

### 注入策略与权衡

目前，我的实现支持两种注入模式：

*   **部分修改（Partial Modification）**
    *   **优点**：更安全。我们可以只修改控制车辆的指令，而保留 Frame 中的其他信号，比如一些用于控制 ADAS 指令是否生效的 `gating` 信号。这样，即使 OpenPilot 发送了错误的指令，只要车辆本身未开启 ADAS 功能，这些指令也不会被执行。另一个好处是，如果原车的 AEB（自动紧急制动）信号与纵向控制信号在同一个 Frame 中，我们的模块在开启 OpenPilot 时依然能保留原车的 AEB 功能（仅有大约一个 cycle 的延迟）。
    *   **延迟**：由于我们是基于上一个 cycle 缓存的帧进行修改，所以注入的帧会有一个 cycle 的延迟（通常为 5ms - 20ms，取决于车企的配置）。

*   **完整替换（Full Replacement）**
    *   **优点**：更灵活。可以方便地使用 `opendbc` 在 OpenPilot 侧进行更细微的控制，例如根据不同车速精细调整转向阻尼等。
    *   **缺点**：需要更全面的信号解析和构造工作。

## 当前局限与未来展望

目前唯一的已知问题是，**ID 最低的那个 Frame 无法被注入**。因为我的注入触发机制依赖于“前一个 Frame”的到来，而第一个 Frame 没有“前一个”。

不过，我个人认为这个需求几乎不存在。出于功能安全和系统设计的考虑，关键的 ADAS 指令极少会使用总线上的第一个 Frame ID 进行传输。

如果未来确实遇到了必须注入第一个 Frame 的场景，可以通过同步 FlexRay 的通信周期（Communication Cycle）来实现更精确的定时注入，这也是未来可以改进的方向。

## 总结

通过利用 Raspberry Pi Pico 强大而易用的 PIO 外设，我们成功以极低的成本，实现了一个功能完备的 FlexRay 中间人攻击模块。它不仅可以作为透明代理来分析总线数据，还能实时修改和注入数据帧，为在 FlexRay 车型上运行 OpenPilot 或进行其他车载网络安全研究铺平了道路。希望我的分享能对社区有所帮助。