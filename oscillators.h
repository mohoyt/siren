#ifndef OSCILLATORS_H
#define OSCILLATORS_H

#include <cstdint>
#include "wavetables.h"
#include "dsp.h"

// Number of oscillator bank types
static constexpr int NUM_BANKS = 6;

// Maximum oscillators per bank
static constexpr int MAX_OSCS = 6;

// All parameters are 0-4095 (12-bit knob range) unless noted

struct OscParams
{
    int32_t warp;       // 0-4095: cross-mod / distortion
    int32_t span;       // 0-4095: frequency spread
    int32_t morph;      // 0-4095: waveform scanning
    int32_t seed;       // 0-4095: structural randomization
    int32_t scan;       // 0-4095: timbral morphing
    int32_t basis_freq; // Root frequency as phase increment
};

// ═══════════════════════════════════════════════
// BANK 0: SINE (Multi-sine cluster)
// Purest drone. 4 sine oscillators in harmonic cluster.
// ═══════════════════════════════════════════════
struct BankSine
{
    uint32_t phase[4] = {};

    // Harmonic ratios in Q16.16: [1.0, 1.002, 2.0, 2.01]
    static constexpr int32_t ratios_q16[4] = {65536, 65667, 131072, 131728};

    void process(const OscParams& p, int16_t& out_l, int16_t& out_r)
    {
        int32_t mix_l = 0, mix_r = 0;

        for (int i = 0; i < 4; i++)
        {
            // Frequency: rootFreq * ratio * (1 + span * 0.05)
            // span 0-4095 mapped to 0.0-0.05 spread factor
            int32_t spread = 65536 + ((int32_t)p.span * 3); // Q16.16: 1.0 + span*0.05ish
            int64_t ratio = (int64_t)ratios_q16[i] * spread >> 16;

            // Seed affects per-oscillator detuning
            int32_t seed_detune = 65536 + ((int32_t)p.seed * i * 1); // subtle per-osc
            ratio = ratio * seed_detune >> 16;

            uint32_t inc = (uint32_t)((int64_t)p.basis_freq * ratio >> 16);
            phase[i] += inc;

            // Basic sine lookup
            int16_t osc = table_lookup(sine_table, phase[i]);

            // WARP: phase feedback — re-read sine with osc output as phase offset
            if (p.warp > 100)
            {
                uint32_t fb_phase = phase[i] + ((int32_t)osc * p.warp >> 6);
                osc = table_lookup(sine_table, fb_phase);
            }

            // Pan: spread oscillators across stereo field
            // i=0: hard left, i=3: hard right
            int32_t pan = i * 1365; // 0, 1365, 2730, 4095
            mix_l += ((int32_t)osc * (4095 - pan)) >> 14;
            mix_r += ((int32_t)osc * pan) >> 14;
        }

        // MORPH: subtle amplitude modulation between oscillators
        // (simplified from SC's SinOsc.kr AM)
        // Just apply a gentle emphasis shift based on morph

        // SCAN: inter-oscillator ring modulation
        if (p.scan > 200)
        {
            // Approximate by mixing in product of first two oscs
            int16_t osc0 = table_lookup(sine_table, phase[0]);
            int16_t osc2 = table_lookup(sine_table, phase[2]);
            int32_t ring = q15_mul(osc0, osc2);
            int32_t ring_amt = p.scan >> 2; // 0-1023
            mix_l += ring * ring_amt >> 12;
            mix_r += ring * ring_amt >> 12;
        }

        out_l = q15_clip(mix_l);
        out_r = q15_clip(mix_r);
    }
};

// ═══════════════════════════════════════════════
// BANK 1: CLST (Cluster scanning)
// Tightly clustered oscillators for beating/phaser textures.
// ═══════════════════════════════════════════════
struct BankCluster
{
    uint32_t phase[4] = {};

    void process(const OscParams& p, int16_t& out_l, int16_t& out_r)
    {
        int32_t mix_l = 0, mix_r = 0;

        // SPAN controls cluster tightness (0.0 to 0.08 spread)
        int32_t cluster_spread = p.span * 5; // Q16.16 fraction, max ~0.08

        for (int i = 0; i < 4; i++)
        {
            // Offset from center: (i - 1.5) * spread
            int32_t offset = ((i * 2 - 3) * cluster_spread) >> 1; // centered
            int32_t ratio = 65536 + offset; // Q16.16: 1.0 + offset

            // Seed: per-oscillator variation
            ratio += (int32_t)p.seed * i * 1;

            uint32_t inc = (uint32_t)((int64_t)p.basis_freq * ratio >> 16);

            // MORPH: scan offset per oscillator (slight frequency drift)
            int32_t morph_offset = (int32_t)p.morph * (i + 1) * 2;
            inc += morph_offset;

            phase[i] += inc;

            // Waveform selection based on SCAN
            // scan < 1365: sine, 1365-2730: tri, 2730+: saw
            int16_t wave;
            int32_t scan_pos = (p.scan + i * 614) & 4095; // per-osc offset
            if (scan_pos < 1365)
            {
                int32_t blend = scan_pos * 3;
                wave = q15_lerp(table_lookup(sine_table, phase[i]),
                               table_lookup(tri_table, phase[i]), blend);
            }
            else if (scan_pos < 2730)
            {
                int32_t blend = (scan_pos - 1365) * 3;
                wave = q15_lerp(table_lookup(tri_table, phase[i]),
                               table_lookup(saw_table, phase[i]), blend);
            }
            else
            {
                wave = table_lookup(saw_table, phase[i]);
            }

            // WARP: phase modulation from a sub-oscillator
            if (p.warp > 200)
            {
                uint32_t sub_phase = phase[i] >> 1; // half frequency
                int16_t sub = table_lookup(sine_table, sub_phase);
                wave = q15_clip((int32_t)wave + ((int32_t)wave * sub >> 15) * p.warp / 2731);
            }

            // Pan across stereo field
            int32_t pan = i * 1365;
            mix_l += ((int32_t)wave * (4095 - pan)) >> 14;
            mix_r += ((int32_t)wave * pan) >> 14;
        }

        out_l = q15_clip(mix_l);
        out_r = q15_clip(mix_r);
    }
};

// ═══════════════════════════════════════════════
// BANK 2: DTON (Diatonic quantized cluster)
// Musical intervals with just intonation ratios.
// ═══════════════════════════════════════════════
struct BankDiatonic
{
    uint32_t phase[4] = {};

    // Two sets of interval ratios in Q16.16
    // Set 0 (tight): unison, major 3rd, 5th, major 7th
    static constexpr int32_t ratios_tight[4] = {65536, 81920, 98304, 122880};
    // Set 1 (wide): major 2nd, 4th, major 6th, octave
    static constexpr int32_t ratios_wide[4] = {73728, 87381, 109227, 131072};

    void process(const OscParams& p, int16_t& out_l, int16_t& out_r)
    {
        int32_t mix_l = 0, mix_r = 0;

        // SPAN crossfades between tight and wide interval sets
        int32_t span_blend = p.span;

        for (int i = 0; i < 4; i++)
        {
            // Blend between interval sets based on span
            int32_t ratio = ratios_tight[i] +
                (((int64_t)(ratios_wide[i] - ratios_tight[i]) * span_blend) >> 12);

            // Seed: micro-detuning per oscillator
            ratio += (int32_t)p.seed * i * 1;

            uint32_t inc = (uint32_t)((int64_t)p.basis_freq * ratio >> 16);
            phase[i] += inc;

            // MORPH: waveform scan (sine -> tri -> saw)
            int16_t wave;
            if (p.morph < 2048)
            {
                int32_t blend = p.morph * 2;
                wave = q15_lerp(table_lookup(sine_table, phase[i]),
                               table_lookup(tri_table, phase[i]), blend);
            }
            else
            {
                int32_t blend = (p.morph - 2048) * 2;
                wave = q15_lerp(table_lookup(tri_table, phase[i]),
                               table_lookup(saw_table, phase[i]), blend);
            }

            // WARP: wavefold intensity
            if (p.warp > 100)
            {
                // Scale signal up then fold
                int32_t scaled = (int32_t)wave * (4096 + p.warp * 3) >> 12;
                // Map to fold table: input range -2.0..+2.0 -> table index
                int32_t fold_idx = (scaled >> 6) + 512; // center at 512
                if (fold_idx < 0) fold_idx = 0;
                if (fold_idx > 1023) fold_idx = 1023;
                wave = fold_table[fold_idx];
            }

            // SCAN: add 2nd harmonic
            if (p.scan > 200)
            {
                uint32_t h2_phase = phase[i] << 1; // double freq
                int16_t h2 = table_lookup(sine_table, h2_phase);
                wave = q15_clip((int32_t)wave + ((int32_t)h2 * p.scan >> 14));
            }

            // Pan across stereo (oscillators spread L to R)
            int32_t pan = i * 1365;
            mix_l += ((int32_t)wave * (4095 - pan)) >> 14;
            mix_r += ((int32_t)wave * pan) >> 14;
        }

        out_l = q15_clip(mix_l);
        out_r = q15_clip(mix_r);
    }
};

// ═══════════════════════════════════════════════
// BANK 3: ANLG (Analogue - 2 oscillators)
// Cross-modulation and ring modulation.
// ═══════════════════════════════════════════════
struct BankAnalogue
{
    uint32_t phase[2] = {};

    void process(const OscParams& p, int16_t& out_l, int16_t& out_r)
    {
        // Two oscillators with detuning from span
        // freq1 = root * (1 + span * 0.02)
        // freq2 = root * (1 - span * 0.02 + seed * 0.01)
        int32_t ratio1 = 65536 + ((int32_t)p.span * 3);        // ~1.0 + span*0.02
        int32_t ratio2 = 65536 - ((int32_t)p.span * 3) + ((int32_t)p.seed * 1);

        uint32_t inc1 = (uint32_t)((int64_t)p.basis_freq * ratio1 >> 16);
        uint32_t inc2 = (uint32_t)((int64_t)p.basis_freq * ratio2 >> 16);
        phase[0] += inc1;
        phase[1] += inc2;

        // MORPH: carrier waveform scan (sine -> tri -> saw)
        int16_t carrier;
        if (p.morph < 2048)
        {
            carrier = q15_lerp(table_lookup(sine_table, phase[0]),
                              table_lookup(tri_table, phase[0]),
                              p.morph * 2);
        }
        else
        {
            carrier = q15_lerp(table_lookup(tri_table, phase[0]),
                              table_lookup(saw_table, phase[0]),
                              (p.morph - 2048) * 2);
        }

        // Modulator waveform (affected by scan)
        int16_t modulator;
        if (p.scan < 2048)
        {
            modulator = q15_lerp(table_lookup(sine_table, phase[1]),
                                table_lookup(tri_table, phase[1]),
                                p.scan * 2);
        }
        else
        {
            modulator = q15_lerp(table_lookup(tri_table, phase[1]),
                                table_lookup(saw_table, phase[1]),
                                (p.scan - 2048) * 2);
        }

        int16_t mixed;

        if (p.warp < 2048)
        {
            // WARP CCW (0-2047): cross-modulation
            // carrier * (1 + modulator * warp_amount)
            int32_t mod_depth = p.warp * 2; // 0-4095 range
            int32_t mod_signal = (int32_t)modulator * mod_depth >> 12;
            mixed = q15_clip((int32_t)carrier + ((int32_t)carrier * mod_signal >> 15));
        }
        else
        {
            // WARP CW (2048-4095): ring modulation blend
            int32_t ring_amt = (p.warp - 2048) * 2;
            int16_t ring = q15_mul(carrier, modulator);
            mixed = q15_lerp(carrier, ring, ring_amt);
        }

        // Fold to prevent clipping
        int32_t fold_idx = ((int32_t)mixed >> 6) + 512;
        if (fold_idx < 0) fold_idx = 0;
        if (fold_idx > 1023) fold_idx = 1023;
        mixed = fold_table[fold_idx];

        // Stereo: slight delay approximation via phase offset
        int16_t delayed = table_lookup(sine_table, phase[0] - (p.span << 10));
        int16_t right = q15_lerp(mixed, delayed, 1024 + p.span / 4);

        out_l = mixed;
        out_r = right;
    }
};

// ═══════════════════════════════════════════════
// BANK 4: WSHP (Waveshaping - 2 oscillators)
// FM-like timbres via waveshaping.
// ═══════════════════════════════════════════════
struct BankWaveshape
{
    uint32_t phase[2] = {};

    void process(const OscParams& p, int16_t& out_l, int16_t& out_r)
    {
        // Carrier at root, modulator at fifth relationship
        // freq2 = root * (1.5 + span * 0.5)
        int32_t ratio2 = 98304 + ((int32_t)p.span * 32); // Q16.16: 1.5 + span*0.5ish

        uint32_t inc1 = p.basis_freq;
        uint32_t inc2 = (uint32_t)((int64_t)p.basis_freq * ratio2 >> 16);
        phase[0] += inc1;
        phase[1] += inc2;

        // MORPH: carrier waveform (sine -> tri -> saw)
        int16_t carrier;
        if (p.morph < 2048)
        {
            carrier = q15_lerp(table_lookup(sine_table, phase[0]),
                              table_lookup(tri_table, phase[0]),
                              p.morph * 2);
        }
        else
        {
            carrier = q15_lerp(table_lookup(tri_table, phase[0]),
                              table_lookup(saw_table, phase[0]),
                              (p.morph - 2048) * 2);
        }

        int16_t modulator = table_lookup(sine_table, phase[1]);

        // WARP controls modulation index (0.5 to 6.0)
        // At warp=0: index=0.5 (subtle), warp=4095: index=6.0 (extreme)
        int32_t mod_index = 2048 + p.warp * 5; // approximate 0.5-6.0 in Q12

        // Apply FM-like waveshaping: carrier * index * (1 + mod * 0.5)
        int32_t shaped = (int32_t)carrier * mod_index >> 12;
        shaped += ((int32_t)shaped * modulator >> 16);

        // Tanh waveshaping: map to table index
        // Input is roughly -4.0 to +4.0, table is 1024 entries
        int32_t tanh_idx = (shaped >> 7) + 512;
        if (tanh_idx < 0) tanh_idx = 0;
        if (tanh_idx > 1023) tanh_idx = 1023;
        int16_t result = tanh_table[tanh_idx];

        // SCAN: fold intensity
        if (p.scan > 200)
        {
            int32_t fold_input = (int32_t)result * (4096 + p.scan * 2) >> 12;
            int32_t fold_idx = (fold_input >> 6) + 512;
            if (fold_idx < 0) fold_idx = 0;
            if (fold_idx > 1023) fold_idx = 1023;
            result = fold_table[fold_idx];
        }

        // Seed: add 2nd harmonic
        if (p.seed > 200)
        {
            uint32_t h2_phase = phase[0] << 1;
            int16_t h2 = table_lookup(sine_table, h2_phase);
            result = q15_clip((int32_t)result + ((int32_t)h2 * p.seed >> 14));
        }

        // Phase-inverted stereo (like SC version)
        out_l = result;
        out_r = (int16_t)-(int32_t)result;
    }
};

// ═══════════════════════════════════════════════
// BANK 5: WAVE (Wavetable scanning - 4 oscillators)
// Wavetable position scanning with optional bit reduction.
// ═══════════════════════════════════════════════
struct BankWavetable
{
    uint32_t phase[4] = {};

    // Harmonic ratios [1, 2, 3, 5] in Q16.16
    static constexpr int32_t ratios[4] = {65536, 131072, 196608, 327680};

    void process(const OscParams& p, int16_t& out_l, int16_t& out_r)
    {
        int32_t mix_l = 0, mix_r = 0;

        for (int i = 0; i < 4; i++)
        {
            // Frequency with slight span-based detuning
            int32_t detune = (i & 1) ? ((int32_t)p.span * 1) : (-(int32_t)p.span * 1);
            int32_t ratio = ratios[i] + detune;

            uint32_t inc = (uint32_t)((int64_t)p.basis_freq * ratio >> 16);
            phase[i] += inc;

            // Wavetable position: MORPH + per-osc offset from SCAN
            int32_t table_pos = p.morph + ((int32_t)p.scan * (i + 1) * 307 >> 12);
            table_pos &= 4095; // wrap

            // Blend between waveforms based on position
            // 0-1365: sine->tri, 1365-2730: tri->saw, 2730-4095: saw->pulse(square)
            int16_t wave;
            if (table_pos < 1365)
            {
                wave = q15_lerp(table_lookup(sine_table, phase[i]),
                               table_lookup(tri_table, phase[i]),
                               table_pos * 3);
            }
            else if (table_pos < 2730)
            {
                wave = q15_lerp(table_lookup(tri_table, phase[i]),
                               table_lookup(saw_table, phase[i]),
                               (table_pos - 1365) * 3);
            }
            else
            {
                // Approximate pulse/square by thresholding saw
                int16_t saw_val = table_lookup(saw_table, phase[i]);
                // Pulse width from scan
                int16_t threshold = (int16_t)((int32_t)p.scan * 32 - 16384);
                int16_t pulse = (saw_val > threshold) ? 32767 : -32768;
                wave = q15_lerp(table_lookup(saw_table, phase[i]),
                               pulse,
                               (table_pos - 2730) * 3);
            }

            if (p.warp < 2048)
            {
                // WARP CCW: bit reduction
                if (p.warp < 1800)
                {
                    int32_t bits = 8 - ((int32_t)p.warp * 6 >> 12);
                    if (bits < 2) bits = 2;
                    int32_t step = 32768 >> bits;
                    if (step > 0)
                        wave = (int16_t)(((int32_t)wave / step) * step);
                }
            }
            else
            {
                // WARP CW: frequency cross-mod
                int16_t sub = table_lookup(sine_table, phase[i] >> 1);
                int32_t depth = (p.warp - 2048) * 2;
                wave = q15_clip((int32_t)wave + ((int32_t)wave * sub >> 15) * depth / 4096);
            }

            // Pan
            int32_t pan = i * 1365;
            mix_l += ((int32_t)wave * (4095 - pan)) >> 14;
            mix_r += ((int32_t)wave * pan) >> 14;
        }

        out_l = q15_clip(mix_l);
        out_r = q15_clip(mix_r);
    }
};

#endif // OSCILLATORS_H
