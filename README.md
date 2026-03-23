# Siren

A multi-algorithm drone oscillator for the [Music Thing Modular Workshop System Computer](https://www.musicthing.co.uk/Workshop-Computer/). Ported from [vhikk-drone](https://github.com/moseshoyt/vhikk-drone) (a monome norns script inspired by the [Forge TME Vhikk X](https://www.forge-tme.com/product/vhikk-x/)).

Siren provides 6 switchable oscillator bank algorithms, each with a distinct timbral character, all controllable via the Workshop Computer's knobs, CV inputs, and triggers. Designed to be a rich drone source within a modular system — effects processing, modulation, and filtering are left to external modules.

## Oscillator Banks

| LED | Bank | Description |
|-----|------|-------------|
| 1 | SINE | Multi-sine cluster (4 oscs). Harmonic ratios with phase feedback. Purest drone. |
| 2 | CLST | Cluster scanning (4 oscs). Tightly detuned for beating/phaser textures. |
| 3 | DTON | Diatonic (4 oscs). Just intonation intervals with wavefolding. Musical chords. |
| 4 | ANLG | Analogue (2 oscs). Cross-modulation blending into ring modulation. Classic analog character. |
| 5 | WSHP | Waveshaping (2 oscs). FM-like timbres via tanh waveshaping and wavefolding. Metallic and bright. |
| 6 | WAVE | Wavetable (4 oscs). Waveform scanning with bit reduction and cross-modulation. Complex and digital. |

## Controls

### Switch Positions

| Position | Main Knob | X Knob | Y Knob |
|----------|-----------|--------|--------|
| **Up** | WARP (cross-mod / distortion) | SPAN (frequency spread) | MORPH (waveform scan) |
| **Middle** | SEED (structural randomization) | SCAN (timbral morphing) | BASIS (root pitch) |
| **Down** | *(momentary)* Tap to cycle through oscillator banks | | |

The lit LED indicates which of the 6 banks is currently active.

### Parameters

Each parameter is interpreted differently per bank — this is core to the exploration-focused design.

- **WARP**: Distortion / modulation intensity.
  - SINE: Phase feedback (subtle harmonics → complex overtones)
  - CLST: Sub-oscillator amplitude modulation (thickening → distortion)
  - DTON: Wavefolding with smooth quadratic onset
  - ANLG: Cross-modulation blending smoothly into ring modulation
  - WSHP: FM modulation index (0.5x → 6x)
  - WAVE: Bit reduction (CCW) or frequency cross-modulation (CW)
- **SPAN**: Frequency spread between oscillators.
  - SINE: Scales harmonic deviation (tighter → wider cluster)
  - CLST: Cluster tightness (unison → wide beating)
  - DTON: Crossfades between tight intervals (1, 3rd, 5th, 7th) and wide intervals (2nd, 4th, 6th, 8ve)
  - ANLG: Symmetric detuning between the two oscillators
  - WSHP: Modulator frequency ratio (fifth → double octave)
  - WAVE: Detuning scaled by harmonic number (upper partials spread more)
- **MORPH**: Waveform and timbral character.
  - SINE: Harmonic emphasis (fundamentals → upper harmonics)
  - CLST: Per-oscillator frequency offset (changes beating pattern)
  - DTON/ANLG: Waveform scan (sine → triangle → sawtooth)
  - WSHP: Carrier waveform (sine → triangle → sawtooth)
  - WAVE: Wavetable position (sine → tri → saw → pulse)
- **SEED**: Structural randomization. Per-oscillator detuning using asymmetric prime-number multipliers. Changes harmonic relationships without affecting the fundamental.
- **SCAN**: Timbral morphing.
  - SINE: Inter-oscillator ring modulation mix
  - CLST: Waveform selection per oscillator (sine → tri → saw, offset per osc)
  - DTON: 2nd and 3rd harmonic addition
  - ANLG: Modulator waveform scan (sine → tri → saw)
  - WSHP: Wavefold intensity
  - WAVE: Wavetable offset per oscillator + pulse width
- **BASIS**: Root pitch / frequency. Exponential mapping from ~55 Hz to ~440 Hz (3 octave range, extendable with CV).

### Knob Pickup

When switching between Up and Middle switch positions, knobs use "pickup" behavior (like the Arturia MicroFreak). The parameter holds its current value until the physical knob crosses near it, preventing sudden jumps.

### Jacks

| Jack | Function |
|------|----------|
| **CV In 1** | Pitch modulation (added to BASIS) |
| **CV In 2** | WARP modulation |
| **Pulse In 1** | Gate — drone on/off |
| **Pulse In 2** | Trigger — randomize SEED |
| **Audio In 1** | Processor input — external audio is waveshaped/folded by WARP and SCAN, blended by MORPH, summed with drone. Mono input creates stereo output via offset processing. |
| **Audio In 2** | SPAN modulation — external CV or audio modulates detuning/spread. Patch an LFO for evolving textures or a sequencer for rhythmic spread changes. |
| **Audio Out 1** | Left audio output |
| **Audio Out 2** | Right audio output |

When no gate is patched to Pulse In 1, the drone runs continuously. To use as a pure audio processor, gate the drone off via Pulse In 1 and patch audio into Audio In 1.

## Building

Requires the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk).

```bash
mkdir build && cd build
cmake ..
make
```

Flash the resulting `siren.uf2` to the Workshop Computer by holding BOOT while connecting USB, then dragging the file to the mounted drive.

## Technical Details

- All DSP uses fixed-point integer arithmetic (Q15 audio, Q16.16 phase accumulators)
- Waveforms generated from 1024-point lookup tables with linear interpolation
- Nonlinearities (tanh, wavefold) via lookup tables
- Only the active oscillator bank computes each sample (no parallel bank overhead)
- Knob pickup prevents parameter jumps when switching pages
- Estimated CPU usage: ~20-40% of budget per sample at 48 kHz
- No dynamic memory allocation

## Origins

The oscillator algorithms are adapted from the SuperCollider engine in vhikk-drone, which itself is inspired by the Forge TME Vhikk X Eurorack module's SEED/SCAN paradigm of structural vs. timbral randomization.

## License

MIT
