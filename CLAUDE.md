# CLAUDE.md - Siren

## Project Overview

**Siren** is a multi-algorithm drone oscillator for the Music Thing Modular Workshop System Computer (RP2040-based Eurorack module). It's loosely based on the oscillator cores from FORGE TME Vhikk X, adapted for the Workshop Computer's hardware constraints and modular synthesis context.

See [WORKSHOP_COMPUTER_AI_DIRECTIVE.md](WORKSHOP_COMPUTER_AI_DIRECTIVE.md) for the full Workshop Computer platform reference, ComputerCard API details, hardware errata, and coding standards.

## Architecture

### Platform Constraints
- **CPU**: RP2040 dual-core ARM Cortex-M0+ @ 133 MHz, no hardware FPU
- **RAM**: 262 KB total
- **Audio**: 12-bit I/O at 48 kHz via SPI DAC
- **Budget**: ~2,778 cycles per sample per core (~20 microseconds)

### Design Decisions
- **Fixed-point integer math throughout** — no floating point in the audio path
- **One oscillator bank active at a time** — unlike the norns version which runs all 6 in parallel with SelectX crossfading. Bank transitions use fade-out → switch → fade-in (~84ms total) to stay within the RP2040's CPU budget.
- **No internal effects/LFOs** — the module lives in a modular system where external modules handle delays, reverbs, filters, and modulation
- **Wavetable lookup** for all waveforms (sine, saw, triangle) with linear interpolation
- **Lookup tables for nonlinearities** (tanh, wavefold) stored in flash

### Files

```
siren/
├── main.cpp              # Main application (ComputerCard subclass, control mapping, routing)
├── oscillators.h         # 6 oscillator bank implementations (SINE, CLST, DTON, ANLG, WSHP, WAVE)
├── wavetables.h          # 1024-point lookup tables + phase/frequency utilities
├── dsp.h                 # Fixed-point math (Q15 multiply, clip, lerp, smoothing)
├── ComputerCard.h        # Workshop Computer hardware abstraction (from upstream)
├── CMakeLists.txt        # Pico SDK build configuration
├── pico_sdk_import.cmake # SDK locator (from upstream)
└── .gitignore
```

### Signal Flow

```
[Audio In 2 (SPAN)] → SPAN modulation ──┐
[Knobs/CV] → [Parameter Smoothing] → [Active Oscillator Bank (1 of 6)]
    → [Envelope] ──┐
                    ├── [Sum] → [Q15 to 12-bit] → [Audio Out L/R]
[Audio In 1] → [Wavefold (WARP) → Tanh (SCAN) → Dry/Wet (MORPH)] ──┘
               (mono → stereo via offset processing)

[BASIS + CV1] ──────────────────────────────────→ [CV Out 1: Pitch CV]
[Envelope level] ───────────────────────────────→ [CV Out 2: Envelope]
[Oscillator phase[0] MSB] ─────────────────────→ [Pulse Out 1: Sub clock]
[Oscillator phase[0] bit 30] ──────────────────→ [Pulse Out 2: 1/2 divider]
```

### Fixed-Point Conventions
- **Q15**: Signed 16-bit audio samples (-32768 to 32767 representing -1.0 to ~+1.0)
- **Q16.16**: Signed 32-bit for frequency ratios and phase accumulators
- **12-bit**: Hardware I/O range (-2048 to 2047) for audio, CV, and knob values (0-4095 unsigned)

## Oscillator Banks

Each bank is a struct with a `process(const OscParams&, int16_t& out_l, int16_t& out_r)` method. All banks respond to the same 6 parameters (WARP, SPAN, MORPH, SEED, SCAN, BASIS) but interpret them differently — this is core to the Vhikk X design philosophy.

| Index | Struct | Oscs | Character |
|-------|--------|------|-----------|
| 0 | BankSine | 4 | Harmonic sine cluster with phase feedback |
| 1 | BankCluster | 4 | Tight beating/phaser cluster |
| 2 | BankDiatonic | 4 | Just intonation intervals with wavefold |
| 3 | BankAnalogue | 2 | Cross-mod and ring-mod |
| 4 | BankWaveshape | 2 | FM-like tanh waveshaping |
| 5 | BankWavetable | 4 | Waveform scanning with bit reduction |

### Per-Bank Parameter Behavior

**SINE** — Harmonic ratios [1, 1.5, 2, 3]. Phase feedback via WARP (scaled by `PHASE_FRAC_BITS` for proper accumulator range). SPAN scales each oscillator's deviation from fundamental (spreading, not shifting). MORPH weights per-oscillator amplitudes (fundamentals → upper harmonics) with gain normalization. SCAN adds inter-oscillator ring modulation.

**CLST** — 4 tightly detuned oscillators. SPAN controls cluster width (unison → wide beating). WARP applies sub-oscillator amplitude modulation (`sub * p.warp >> 12`). MORPH applies per-oscillator frequency ratio offsets for different beating patterns. SCAN selects per-oscillator waveform via circular scan (sine → tri → saw → sine, offset per osc).

**DTON** — Just intonation intervals. SPAN crossfades between tight intervals (1, 3rd, 5th, 7th) and wide intervals (2nd, 4th, 6th, octave). WARP applies wavefold with quadratic drive onset (`p.warp * p.warp >> 10`) for smooth low-end response. SCAN adds 2nd harmonic (up to 75% mix). MORPH scans carrier waveform (sine → triangle → sawtooth).

**ANLG** — 2 oscillators with detuning. WARP crossfades smoothly between cross-modulation (CCW) and ring modulation (CW) with a 1500-2500 blend zone (no midpoint discontinuity). SPAN controls symmetric detuning. MORPH scans waveform (sine → tri → saw). SCAN scans modulator waveform.

**WSHP** — Carrier + modulator in ~fifth relationship. WARP controls FM modulation index (0.5x → 6x). SPAN sets modulator frequency ratio (`*16` scaling for 1.5x-2.5x range). MORPH scans carrier waveform. SCAN controls wavefold intensity (continuous from 0, no dead zone). SEED applies asymmetric detuning via prime multipliers.

**WAVE** — 4 oscillators with harmonic ratios [1, 2, 3, 5]. WARP splits into bit reduction (CCW, full 0-2047 range) and frequency cross-modulation (CW). SPAN scales detuning per harmonic number (`*(i+1)` so upper partials spread more). MORPH scans wavetable position via circular scan (sine → tri → saw → pulse → sine). SCAN offsets wavetable position per oscillator + pulse width. SEED uses per-oscillator prime multipliers `{0, 7, -5, 11}` for asymmetric detuning.

### SEED Implementation

All banks use asymmetric per-oscillator prime-number multipliers for SEED, creating non-uniform detuning that changes harmonic relationships without shifting the fundamental. Each bank has its own multiplier set (e.g., WAVE uses `{0, 7, -5, 11}`, keeping osc 0 anchored while others shift by different amounts and directions).

### Bank Crossfade

When switching banks (via switch-down or Pulse In 2 long hold), the current bank fades out, the bank switches at silence, and the new bank fades in (~42ms each way, ~84ms total). Only one bank ever runs at a time — the RP2040 can't handle two banks simultaneously without overrunning the sample callback. During crossfade, LEDs show the transition (old dims, new brightens). If the gate is closed (env_level == 0), the swap is instant since there's no audio to crossfade. Additional bank cycle requests are ignored during an active crossfade.

### Knob Pickup

When switching between Up and Middle switch positions, knobs use pickup behavior (like the Arturia MicroFreak). The `KnobPickup` struct holds the current parameter value and a `picked_up` flag. On page change, knobs are "released" — the parameter holds its value until the physical knob crosses within ~2% of it, preventing sudden jumps.

## Control Mapping

### Switch
- **Up**: Knobs control WARP / SPAN / MORPH (Main / X / Y)
- **Middle**: Knobs control SEED / SCAN / BASIS (Main / X / Y)
- **Down (momentary)**: Cycle oscillator bank with ~84ms crossfade (fade-out → switch → fade-in); LEDs show transition (old dims, new brightens)

### CV/Trigger
- CV In 1 → pitch (added to BASIS)
- CV In 2 → WARP modulation
- CV Out 1 → pitch CV (mirrors BASIS + CV1 modulation)
- CV Out 2 → envelope level
- Pulse In 1 → gate on/off
- Pulse In 2 → dual-purpose: short pulse randomizes SEED, long hold (≥500ms) cycles bank with crossfade
- Pulse Out 1 → sub-oscillator clock (square wave at fundamental)
- Pulse Out 2 → 1/2 divider (one octave below fundamental)

### Audio Input
- Audio In 1 → processor input: waveshaped/folded by WARP+SCAN, dry/wet via MORPH, summed with drone (stereo from mono via offset processing)
- Audio In 2 → SPAN modulation: external CV/audio modulates detuning/spread

## Development

### Building
Requires Pico SDK. Set `PICO_SDK_PATH` environment variable or use the VS Code extension.

```bash
mkdir build && cd build
cmake ..
make
```

### Making Changes

1. **Adding/modifying oscillator banks**: Edit `oscillators.h`. Each bank is self-contained. Keep all DSP in integer math — no `float` or `double` in the audio path.
2. **Adding parameters**: Add to `OscParams` struct, map in `update_controls()` in `main.cpp`.
3. **Changing control mapping**: Edit `update_controls()` and `update_leds()` in `main.cpp`.
4. **Wavetable changes**: Edit `init_wavetables()` in `wavetables.h`. Tables are computed at startup, stored in RAM.

### Performance Guidelines
- The `ProcessSample()` callback must complete in ~20 microseconds
- Mark time-critical functions with `__not_in_flash_func()`
- Avoid division in the audio path (use shifts or multiply-by-reciprocal)
- Avoid `float`/`double` — software FPU emulation is 50-100x slower than integer
- Test CPU headroom by toggling a GPIO pin in ProcessSample() and measuring with a scope

### Relationship to vhikk-drone
The oscillator algorithms in `oscillators.h` are integer-math ports of the SuperCollider UGen chains in `vhikk-drone/lib/Engine_VhikkDrone.sc`. Key differences:
- SC's `SelectX.ar` crossfading between all 6 banks → sequential bank switching with ~84ms fade-out/fade-in crossfade (only one bank runs at a time)
- SC's `SinOsc.ar` → wavetable lookup with linear interpolation
- SC's `.tanh` / `.fold2` → lookup table approximations
- SC's `Lag.kr` → one-pole integer lowpass filter
- No delay processors, LFOs, or mod matrix (handled by external modules)

## Dependencies

- [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) (v2.2.0+)
- [ComputerCard library](https://github.com/TomWhitwell/Workshop_Computer) (v0.2.7, included as header)

## References

- [Forge TME Vhikk X](https://www.forge-tme.com/product/vhikk-x/) — hardware inspiration
- [Vhikk X Algorithms](https://forge-tme.com/vhikk-x/algos_launch.html) — algorithm descriptions
- [Workshop Computer](https://www.musicthing.co.uk/Workshop-Computer/) — target hardware
- [ComputerCard API](https://github.com/TomWhitwell/Workshop_Computer/tree/main/Demonstrations+HelloWorlds/PicoSDK/ComputerCard) — hardware abstraction
