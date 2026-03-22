# Siren

A multi-algorithm drone oscillator for the [Music Thing Modular Workshop System Computer](https://www.musicthing.co.uk/Workshop-Computer/). Ported from [vhikk-drone](https://github.com/moseshoyt/vhikk-drone) (a monome norns script inspired by the [Forge TME Vhikk X](https://www.forge-tme.com/product/vhikk-x/)).

Siren provides 6 switchable oscillator bank algorithms, each with a distinct timbral character, all controllable via the Workshop Computer's knobs, CV inputs, and triggers. Designed to be a rich drone source within a modular system — effects processing, modulation, and filtering are left to external modules.

## Oscillator Banks

| LED | Bank | Description |
|-----|------|-------------|
| 1 | SINE | Multi-sine cluster (4 oscs). Harmonic ratios with phase feedback. Purest drone. |
| 2 | CLST | Cluster scanning (4 oscs). Tightly detuned for beating/phaser textures. |
| 3 | DTON | Diatonic (4 oscs). Just intonation intervals with wavefolding. Musical chords. |
| 4 | ANLG | Analogue (2 oscs). Cross-modulation and ring modulation. Classic analog character. |
| 5 | WSHP | Waveshaping (2 oscs). FM-like timbres via tanh waveshaping. Metallic and bright. |
| 6 | WAVE | Wavetable (4 oscs). Waveform scanning with bit reduction. Complex and digital. |

## Controls

### Switch Positions

| Position | Main Knob | X Knob | Y Knob |
|----------|-----------|--------|--------|
| **Up** | WARP (cross-mod / distortion) | SPAN (frequency spread) | MORPH (waveform scan) |
| **Middle** | SEED (structural randomization) | SCAN (timbral morphing) | BASIS (root pitch) |
| **Down** | *(momentary)* Tap to cycle through oscillator banks | | |

The lit LED indicates which of the 6 banks is currently active.

### Parameters

- **WARP**: Cross-modulation and distortion intensity. Behavior varies per bank — from phase feedback (SINE) to ring modulation (ANLG) to FM index (WSHP).
- **SPAN**: Frequency spread between oscillators. Controls cluster tightness (CLST), interval width (DTON), or detuning amount (ANLG).
- **MORPH**: Waveform scanning. Crossfades between sine, triangle, and sawtooth waveshapes.
- **SEED**: Structural randomization. Affects per-oscillator detuning ratios and harmonic relationships.
- **SCAN**: Timbral morphing. Adds harmonics, ring modulation, or wavefolding depending on the bank.
- **BASIS**: Root pitch / frequency. Exponential mapping from ~55 Hz to ~440 Hz (3 octave range, extendable with CV).

### Jacks

| Jack | Function |
|------|----------|
| **CV In 1** | Pitch modulation (added to BASIS) |
| **CV In 2** | WARP modulation |
| **Pulse In 1** | Gate — drone on/off |
| **Pulse In 2** | Trigger — randomize SEED |
| **Audio In 1** | Processor input — external audio is waveshaped/folded by WARP and SCAN, blended by MORPH, summed with drone. Mono input creates stereo output via offset processing. |
| **Audio In 2** | FM input — modulates oscillator frequencies. FM depth scales with WARP. Patch an LFO for vibrato or another oscillator for FM synthesis. |
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
- Only the active oscillator bank computes each sample (no parallel bank overhead)
- Estimated CPU usage: ~20-40% of budget per sample at 48 kHz
- No dynamic memory allocation

## Origins

The oscillator algorithms are adapted from the SuperCollider engine in vhikk-drone, which itself is inspired by the Forge TME Vhikk X Eurorack module's SEED/SCAN paradigm of structural vs. timbral randomization.

## License

MIT
