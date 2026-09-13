# FlexRay hardware timing lab

An interactive English demo built with plain HTML, CSS, and JavaScript. It requires no third-party libraries, external fonts, build step, or network connection. Open `index.html` directly, or run this from the repository root:

```sh
python3 -m http.server 8765 --bind 127.0.0.1
```

Then visit [the local demo](http://127.0.0.1:8765/web/flexray-demo/).

- Single bridge MITM: one bidirectional FR1/FR2 bridge, using the `FLEXRAY_FRAME_GEN=ON` resource layout with frame generation disabled.
- Frame generation: FSS capture, measurement from the lowest five observed IDs, sampler-to-pace reuse, preparation, full transmission, independent DONE, bounded phase capture, one-time tail correction, and sleep recovery.
- Dual bridge MITM: FR1/FR2 plus FR3/FR4, with `FLEXRAY_FRAME_GEN=OFF` and frame generation compiled out.

All modes show three PIO blocks with four SMs each. Green highlights participants in the current step; it does not imply other allocated SMs are stopped. Select an SM to inspect GPIO, DMA, and IRQ details. Connection colors distinguish signal levels, IRQs, and data. Playback adds moving markers and a timeline cursor. Controls support pause, stepping, speed changes, host data supply, cycle labels, reference phase offset, and simulated bus sleep.

The host toggle represents continuous submission of fresh valid data, not replay of a consumed payload. The MITM example assumes a valid cached template and a fresh override. Its current rule triggers on the primary bridge's FID6/base2, injecting toward FR1 in ON mode or FR3 in OFF mode.

Frame generation steps illustrate separate concepts. The phase step zooms into a reference slot; it does not imply that references always follow C/D. Waveforms, animation speed, and pauses are educational illustrations, not instruction-cycle simulation, logic analyzer recordings, or live device control. See the [design notes](../../docs/frame-gen-design.md).

`index.html` defines the page structure, `style.css` the responsive layout, and `app.js` the steps, resource maps, SVG connections/waveforms, and interactive state.

The memory lab shows 256 sampled bits, eight words, autopush, valid ISR bits, DMA commit state, little-endian bytes, CLZ, and phase rejection conditions. Step by bit or word, or play through normal, cross-word, incomplete 14-bit, extra-marker, out-of-tolerance, expired-DMA, and missing-marker scenarios. Acquisition CTZ and runtime CLZ are explained separately.

Fourteen expandable topics expose implementation details. The offline source snapshot comes from this demo branch. PIO0 SM2 is unused. PIO0 uses 13/30 words during ON acquisition/runtime and 28 words in OFF mode. ON/OFF use 10/8 DMA channels respectively.
