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

        // MORPH: controls per-oscillator amplitude weighting
        // 0 = emphasize fundamentals (oscs 0,1), 4095 = emphasize harmonics (oscs 2,3)
        // Each osc gets a different gain based on morph position
        int32_t gains[4];
        gains[0] = 4095 - p.morph;                          // fundamental, fades out
        gains[1] = (p.morph < 2048) ? p.morph * 2 : (4095 - p.morph) * 2; // peaks mid
        gains[2] = (p.morph > 1365) ? (p.morph - 1365) * 3 >> 1 : 0;      // rises late
        gains[3] = p.morph;                                  // harmonics, fades in
        // Normalize so total is roughly constant
        // Compute reciprocal with enough precision to avoid volume jumps
        int32_t gain_sum = gains[0] + gains[1] + gains[2] + gains[3];
        if (gain_sum < 1) gain_sum = 1;
        // High-precision reciprocal: range ~455-1024, hundreds of distinct values
        int32_t gain_recip = (4 << 20) / gain_sum;

        for (int i = 0; i < 4; i++)
        {
            // SPAN: scale the deviation of each ratio from the fundamental
            // At span=0, ratios are tight. At span=4095, deviations are doubled.
            int32_t base_ratio = ratios_q16[i];
            int32_t deviation = base_ratio - 65536; // how far from 1.0
            // Scale deviation: 0.5x at span=0, 2.0x at span=4095
            int32_t span_scale = 2048 + ((int32_t)p.span * 3 >> 1); // Q12: 0.5 to ~1.5
            int64_t ratio = 65536 + ((int64_t)deviation * span_scale >> 12);

            // Seed: per-oscillator detuning (moderate range, max ~5% per osc)
            int32_t seed_detune = 65536 + ((int32_t)p.seed * i * 2);
            ratio = ratio * seed_detune >> 16;

            uint32_t inc = (uint32_t)((int64_t)p.basis_freq * ratio >> 16);
            phase[i] += inc;

            // Basic sine lookup
            int16_t osc = table_lookup(sine_table, phase[i]);

            // WARP: phase feedback — re-read sine with osc output as phase offset
            // Gentle scaling: at max warp, feedback is ~±0.5 of a full cycle
            if (p.warp > 50)
            {
                // Scale: osc (±32767) * warp (0-4095) >> 14 gives ±8191 max
                // Then shift into phase accumulator range
                int32_t fb_amount = (int32_t)osc * p.warp >> 14;
                uint32_t fb_phase = phase[i] + (fb_amount << (PHASE_FRAC_BITS - 2));
                osc = table_lookup(sine_table, fb_phase);
            }

            // Apply morph gain (multiply by precomputed reciprocal instead of dividing)
            // Split to avoid overflow: (osc * gains >> 8) * recip >> 12
            int32_t gained = ((int32_t)osc * gains[i] >> 8) * gain_recip >> 12;

            // Pan: spread oscillators across stereo field
            int32_t pan = i * 1365; // 0, 1365, 2730, 4095
            mix_l += (gained * (4095 - pan)) >> 14;
            mix_r += (gained * pan) >> 14;
        }

        // SCAN: inter-oscillator ring modulation
        if (p.scan > 100)
        {
            int16_t osc0 = table_lookup(sine_table, phase[0]);
            int16_t osc2 = table_lookup(sine_table, phase[2]);
            int32_t ring = q15_mul(osc0, osc2);
            // Stronger scaling: scan 0-4095 maps to 0-1.0 mix amount
            mix_l += (int32_t)ring * p.scan >> 12;
            mix_r += (int32_t)ring * p.scan >> 12;
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
            int32_t offset = ((i * 2 - 3) * cluster_spread) >> 1;
            int32_t ratio = 65536 + offset; // Q16.16: 1.0 + offset

            // Seed: per-oscillator variation (moderate, asymmetric)
            static constexpr int32_t clst_seed_scale[4] = {0, 3, -2, 5};
            ratio += (int32_t)p.seed * clst_seed_scale[i];

            uint32_t inc = (uint32_t)((int64_t)p.basis_freq * ratio >> 16);

            // MORPH: per-oscillator frequency offset creating beating pattern
            // Each osc gets a different morph-scaled detuning
            // At max morph, offsets are significant enough to create audible beating
            int32_t morph_ratio = 65536 + ((int32_t)p.morph * (i * 2 - 3) * 3);
            inc = (uint32_t)((int64_t)inc * morph_ratio >> 16);

            phase[i] += inc;
            int mip = mip_for_inc(inc); // band-limited level for this osc's pitch

            // Waveform selection based on SCAN (circular: sine → tri → saw → sine)
            int16_t wave;
            int32_t scan_pos = (p.scan + i * 614) & 4095;
            if (scan_pos < 1365)
            {
                int32_t blend = scan_pos * 3;
                wave = q15_lerp(table_lookup(sine_table, phase[i]),
                               bl_lookup(tri_mips, phase[i], mip), blend);
            }
            else if (scan_pos < 2730)
            {
                int32_t blend = (scan_pos - 1365) * 3;
                wave = q15_lerp(bl_lookup(tri_mips, phase[i], mip),
                               bl_lookup(saw_mips, phase[i], mip), blend);
            }
            else
            {
                int32_t blend = (scan_pos - 2730) * 3;
                wave = q15_lerp(bl_lookup(saw_mips, phase[i], mip),
                               table_lookup(sine_table, phase[i]), blend);
            }

            // WARP: amplitude modulation from a sub-oscillator
            // Creates harmonic thickening and distortion
            if (p.warp > 100)
            {
                uint32_t sub_phase = phase[i] >> 1; // half frequency
                int16_t sub = table_lookup(sine_table, sub_phase);
                // Scale: at max warp, sub modulates the wave by ±100%
                int32_t mod = ((int32_t)sub * p.warp) >> 12;
                wave = q15_clip((int32_t)wave + ((int32_t)wave * mod >> 15));
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
    Halfband2x decim_l, decim_r; // 2x oversampling decimators

    // Two sets of interval ratios in Q16.16
    // Set 0 (tight): unison, major 3rd, 5th, major 7th
    static constexpr int32_t ratios_tight[4] = {65536, 81920, 98304, 122880};
    // Set 1 (wide): major 2nd, 4th, major 6th, octave
    static constexpr int32_t ratios_wide[4] = {73728, 87381, 109227, 131072};

    void process(const OscParams& p, int16_t& out_l, int16_t& out_r)
    {
        // SPAN crossfades between tight and wide interval sets
        int32_t span_blend = p.span;

        // Precompute per-oscillator increments, half-steps and band-limited levels
        uint32_t inc[4], hinc[4];
        int mip[4];
        for (int i = 0; i < 4; i++)
        {
            // Blend between interval sets based on span
            // Difference fits in 17 bits (max 65536), span in 12 bits — 32-bit multiply is safe
            int32_t ratio = ratios_tight[i] +
                (((ratios_wide[i] - ratios_tight[i]) * span_blend) >> 12);

            // Seed: micro-detuning per oscillator
            static constexpr int32_t dton_seed_scale[4] = {0, 3, -2, 5};
            ratio += (int32_t)p.seed * dton_seed_scale[i];

            inc[i] = (uint32_t)((int64_t)p.basis_freq * ratio >> 16);
            hinc[i] = inc[i] >> 1;
            mip[i] = mip_for_inc(inc[i]);
        }

        // WARP wavefold drive depends only on params — hoist out of the loops
        int32_t drive = 4096 + ((int32_t)p.warp * p.warp >> 10); // quadratic onset

        // Run the wavefold + 2nd-harmonic stage at 2x rate, then decimate.
        int16_t sub_l[2], sub_r[2];
        for (int s = 0; s < 2; s++)
        {
            int32_t mix_l = 0, mix_r = 0;
            for (int i = 0; i < 4; i++)
            {
                phase[i] += (s == 0) ? hinc[i] : (inc[i] - hinc[i]);

                // MORPH: waveform scan (sine -> tri -> saw), band-limited
                int16_t wave;
                if (p.morph < 2048)
                {
                    int32_t blend = p.morph * 2;
                    wave = q15_lerp(table_lookup(sine_table, phase[i]),
                                   bl_lookup(tri_mips, phase[i], mip[i]), blend);
                }
                else
                {
                    int32_t blend = (p.morph - 2048) * 2;
                    wave = q15_lerp(bl_lookup(tri_mips, phase[i], mip[i]),
                                   bl_lookup(saw_mips, phase[i], mip[i]), blend);
                }

                // WARP: wavefold intensity (gradual onset)
                {
                    int32_t scaled = (int32_t)wave * drive >> 12;
                    int32_t fold_idx = (scaled >> 6) + 512;
                    if (fold_idx < 0) fold_idx = 0;
                    if (fold_idx > 1023) fold_idx = 1023;
                    wave = fold_table[fold_idx];
                }

                // SCAN: add 2nd harmonic for richer timbre
                {
                    uint32_t h2_phase = phase[i] << 1;
                    int16_t h2 = table_lookup(sine_table, h2_phase);
                    // scan 0-4095: h2 up to 75% mix
                    int32_t h2_amt = (int32_t)h2 * p.scan >> 12;
                    wave = q15_clip((int32_t)wave + h2_amt);
                }

                // Pan across stereo (oscillators spread L to R)
                int32_t pan = i * 1365;
                mix_l += ((int32_t)wave * (4095 - pan)) >> 14;
                mix_r += ((int32_t)wave * pan) >> 14;
            }
            sub_l[s] = q15_clip(mix_l);
            sub_r[s] = q15_clip(mix_r);
        }

        out_l = decim_l.process(sub_l[0], sub_l[1]);
        out_r = decim_r.process(sub_r[0], sub_r[1]);
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
        // SPAN: symmetric detuning between the two oscillators
        int32_t ratio1 = 65536 + ((int32_t)p.span * 3);
        // SEED: shifts osc2 relationship (max ~10% detune)
        int32_t ratio2 = 65536 - ((int32_t)p.span * 3) + ((int32_t)p.seed * 4);

        uint32_t inc1 = (uint32_t)((int64_t)p.basis_freq * ratio1 >> 16);
        uint32_t inc2 = (uint32_t)((int64_t)p.basis_freq * ratio2 >> 16);
        phase[0] += inc1;
        phase[1] += inc2;
        int mip0 = mip_for_inc(inc1); // band-limited levels per oscillator
        int mip1 = mip_for_inc(inc2);

        // MORPH: carrier waveform scan (sine -> tri -> saw)
        int16_t carrier;
        if (p.morph < 2048)
        {
            carrier = q15_lerp(table_lookup(sine_table, phase[0]),
                              bl_lookup(tri_mips, phase[0], mip0),
                              p.morph * 2);
        }
        else
        {
            carrier = q15_lerp(bl_lookup(tri_mips, phase[0], mip0),
                              bl_lookup(saw_mips, phase[0], mip0),
                              (p.morph - 2048) * 2);
        }

        // Modulator waveform (affected by scan)
        int16_t modulator;
        if (p.scan < 2048)
        {
            modulator = q15_lerp(table_lookup(sine_table, phase[1]),
                                bl_lookup(tri_mips, phase[1], mip1),
                                p.scan * 2);
        }
        else
        {
            modulator = q15_lerp(bl_lookup(tri_mips, phase[1], mip1),
                                bl_lookup(saw_mips, phase[1], mip1),
                                (p.scan - 2048) * 2);
        }

        int16_t mixed;

        // WARP: cross-modulation (low) blending into ring modulation (high)
        // Crossfade zone around the midpoint (1500-2500) to avoid discontinuity
        {
            // Cross-mod amount: full at 0, fades out by 2500
            // 4095/2500 ≈ 107/65536 * 65536/1 — use (x * 107) >> 16 ≈ x * 1.634/1
            // Simpler: (x << 12) / 2500 ≈ x * 26 >> 4 (since 4096/2500 ≈ 1.638)
            int32_t xmod_amt = (p.warp < 2500) ? ((2500 - p.warp) * 27 >> 4) : 0;
            if (xmod_amt > 4095) xmod_amt = 4095;
            int32_t mod_signal = (int32_t)modulator * xmod_amt >> 12;
            int16_t xmod_out = q15_clip((int32_t)carrier + ((int32_t)carrier * mod_signal >> 15));

            // Ring mod amount: zero below 1500, full by 4095
            // 4095/2595 ≈ 1.578 ≈ 26/16
            int32_t ring_amt = (p.warp > 1500) ? ((p.warp - 1500) * 26 >> 4) : 0;
            if (ring_amt > 4095) ring_amt = 4095;
            int16_t ring = q15_mul(carrier, modulator);
            int16_t ring_out = q15_lerp(carrier, ring, ring_amt);

            // Blend: in the crossfade zone both contribute
            if (p.warp < 1500)
                mixed = xmod_out;
            else if (p.warp > 2500)
                mixed = ring_out;
            else
            {
                // 4095/1000 ≈ 4.095 ≈ 33/8
                int32_t blend = (p.warp - 1500) * 33 >> 3;
                if (blend > 4095) blend = 4095;
                mixed = q15_lerp(xmod_out, ring_out, blend);
            }
        }

        // Fold to prevent clipping
        int32_t fold_idx = ((int32_t)mixed >> 6) + 512;
        if (fold_idx < 0) fold_idx = 0;
        if (fold_idx > 1023) fold_idx = 1023;
        mixed = fold_table[fold_idx];

        // Stereo: slight delay approximation via phase offset
        int16_t delayed = table_lookup(sine_table, phase[0] - (p.span << 10));
        int16_t right = q15_lerp(mixed, delayed, 1024 + (p.span >> 2));

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
    Halfband2x decim; // 2x oversampling decimator (mono; right = -left)

    void process(const OscParams& p, int16_t& out_l, int16_t& out_r)
    {
        // Carrier at root, modulator relationship controlled by SPAN
        // span=0: fifth (1.5x), span=4095: octave+fifth (3.0x)
        // Scaled so the range is musically useful without jumping to inharmonic
        int32_t ratio2 = 98304 + ((int32_t)p.span * 16); // Q16.16: 1.5 to ~2.5

        uint32_t inc1 = p.basis_freq;
        uint32_t inc2 = (uint32_t)((int64_t)p.basis_freq * ratio2 >> 16);
        uint32_t h1 = inc1 >> 1; // half-steps for 2x oversampling
        uint32_t h2 = inc2 >> 1;
        int mip = mip_for_inc(inc1); // carrier band-limited level

        // WARP controls modulation index (0.5 to 6.0)
        // At warp=0: index=0.5 (subtle), warp=4095: index=6.0 (extreme)
        int32_t mod_index = 2048 + p.warp * 5; // approximate 0.5-6.0 in Q12

        // Run the FM -> tanh -> fold cascade at 2x rate (96 kHz internal) to push
        // the aliasing byproducts above 24 kHz before decimating back to 48 kHz.
        int16_t sub_out[2];
        for (int s = 0; s < 2; s++)
        {
            // Advance by half a step each sub-sample; exact total = inc per sample
            phase[0] += (s == 0) ? h1 : (inc1 - h1);
            phase[1] += (s == 0) ? h2 : (inc2 - h2);

            // MORPH: carrier waveform (sine -> tri -> saw), band-limited
            int16_t carrier;
            if (p.morph < 2048)
            {
                carrier = q15_lerp(table_lookup(sine_table, phase[0]),
                                  bl_lookup(tri_mips, phase[0], mip),
                                  p.morph * 2);
            }
            else
            {
                carrier = q15_lerp(bl_lookup(tri_mips, phase[0], mip),
                                  bl_lookup(saw_mips, phase[0], mip),
                                  (p.morph - 2048) * 2);
            }

            int16_t modulator = table_lookup(sine_table, phase[1]);

            // Apply FM-like waveshaping: carrier * index * (1 + mod * 0.5)
            int32_t shaped = (int32_t)carrier * mod_index >> 12;
            shaped += (int32_t)((int64_t)shaped * modulator >> 16);

            // Tanh waveshaping: map to table index
            // Input is roughly -4.0 to +4.0, table is 1024 entries
            int32_t tanh_idx = (shaped >> 7) + 512;
            if (tanh_idx < 0) tanh_idx = 0;
            if (tanh_idx > 1023) tanh_idx = 1023;
            int16_t result = tanh_table[tanh_idx];

            // SCAN: fold intensity (continuous from 0)
            {
                int32_t fold_input = (int32_t)result * (4096 + p.scan * 2) >> 12;
                int32_t fold_idx = (fold_input >> 6) + 512;
                if (fold_idx < 0) fold_idx = 0;
                if (fold_idx > 1023) fold_idx = 1023;
                result = fold_table[fold_idx];
            }

            // Seed: add 2nd harmonic (continuous from 0)
            {
                uint32_t h2_phase = phase[0] << 1;
                int16_t h2s = table_lookup(sine_table, h2_phase);
                result = q15_clip((int32_t)result + ((int32_t)h2s * p.seed >> 14));
            }

            sub_out[s] = result;
        }

        // Decimate 96 kHz -> 48 kHz, then phase-inverted stereo (like SC version)
        int16_t mono = decim.process(sub_out[0], sub_out[1]);
        out_l = mono;
        out_r = (int16_t)-(int32_t)mono;
    }
};

// ═══════════════════════════════════════════════
// BANK 5: WAVE (Wavetable scanning - 4 oscillators)
// Wavetable position scanning with optional bit reduction.
// ═══════════════════════════════════════════════
struct BankWavetable
{
    uint32_t phase[4] = {};
    Halfband2x decim_l, decim_r; // 2x oversampling decimators

    // Harmonic ratios [1, 2, 3, 5] in Q16.16
    static constexpr int32_t ratios[4] = {65536, 131072, 196608, 327680};

    void process(const OscParams& p, int16_t& out_l, int16_t& out_r)
    {
        // Precompute per-oscillator increments, half-steps and band-limited levels
        uint32_t inc[4], hinc[4];
        int mip[4];
        for (int i = 0; i < 4; i++)
        {
            // Frequency with span-based detuning and seed variation
            // SPAN: alternating ± detuning, stronger on upper harmonics
            int32_t detune = (i & 1) ? ((int32_t)p.span * (i + 1)) : (-(int32_t)p.span * (i + 1));
            // SEED: shifts harmonic ratios — at max seed, ratios drift significantly
            // Use different prime multipliers per osc for non-uniform detuning
            static constexpr int32_t seed_scale[4] = {0, 7, -5, 11};
            int32_t seed_offset = (int32_t)p.seed * seed_scale[i];
            int32_t ratio = ratios[i] + detune + seed_offset;

            inc[i] = (uint32_t)((int64_t)p.basis_freq * ratio >> 16);
            hinc[i] = inc[i] >> 1;
            mip[i] = mip_for_inc(inc[i]);
        }

        int32_t threshold = ((int32_t)p.scan << 4) - 32768; // pulse width (PWM)

        // Run the pulse/bit-crush/cross-mod stage at 2x rate, then decimate. The
        // bit reduction's quantization harmonics are intentional character; the
        // 2x rate just keeps the hard-edged pulse from aliasing as harshly.
        int16_t sub_l[2], sub_r[2];
        for (int s = 0; s < 2; s++)
        {
            int32_t mix_l = 0, mix_r = 0;
            for (int i = 0; i < 4; i++)
            {
                phase[i] += (s == 0) ? hinc[i] : (inc[i] - hinc[i]);

                // Wavetable position: MORPH + per-osc offset from SCAN
                int32_t table_pos = p.morph + ((int32_t)p.scan * (i + 1) * 307 >> 12);
                table_pos &= 4095; // wrap

                // Blend between waveforms (circular: sine→tri→saw→pulse→sine).
                // Saw/tri use band-limited tables; the pulse threshold reads the
                // clean ramp (saw_table) so PWM duty stays monotonic (no ripple).
                int16_t wave;
                if (table_pos < 1024)
                {
                    wave = q15_lerp(table_lookup(sine_table, phase[i]),
                                   bl_lookup(tri_mips, phase[i], mip[i]),
                                   table_pos * 4);
                }
                else if (table_pos < 2048)
                {
                    wave = q15_lerp(bl_lookup(tri_mips, phase[i], mip[i]),
                                   bl_lookup(saw_mips, phase[i], mip[i]),
                                   (table_pos - 1024) * 4);
                }
                else if (table_pos < 3072)
                {
                    int16_t saw_bl = bl_lookup(saw_mips, phase[i], mip[i]);
                    int16_t saw_ramp = table_lookup(saw_table, phase[i]);
                    int16_t pulse_wave = (saw_ramp > threshold) ? 32767 : -32768;
                    wave = q15_lerp(saw_bl, pulse_wave, (table_pos - 2048) * 4);
                }
                else
                {
                    int16_t saw_ramp = table_lookup(saw_table, phase[i]);
                    int16_t pulse_wave = (saw_ramp > threshold) ? 32767 : -32768;
                    wave = q15_lerp(pulse_wave,
                                   table_lookup(sine_table, phase[i]),
                                   (table_pos - 3072) * 4);
                }

                if (p.warp < 2048)
                {
                    // WARP CCW: bit reduction (8-bit down to 2-bit)
                    int32_t bits = 8 - ((int32_t)p.warp * 6 >> 11); // use full 0-2047 range
                    if (bits < 2) bits = 2;
                    if (bits < 8)
                    {
                        // Quantize using shift instead of division (step is always power of 2)
                        int32_t shift = 15 - bits; // equivalent to log2(32768 >> bits)
                        wave = (int16_t)((wave >> shift) << shift);
                    }
                }
                else
                {
                    // WARP CW: frequency cross-mod
                    int16_t sub = table_lookup(sine_table, phase[i] >> 1);
                    int32_t depth = (p.warp - 2048) * 2;
                    wave = q15_clip((int32_t)wave + (((int32_t)wave * sub >> 15) * depth >> 12));
                }

                // Pan
                int32_t pan = i * 1365;
                mix_l += ((int32_t)wave * (4095 - pan)) >> 14;
                mix_r += ((int32_t)wave * pan) >> 14;
            }
            sub_l[s] = q15_clip(mix_l);
            sub_r[s] = q15_clip(mix_r);
        }

        out_l = decim_l.process(sub_l[0], sub_l[1]);
        out_r = decim_r.process(sub_r[0], sub_r[1]);
    }
};

#endif // OSCILLATORS_H
