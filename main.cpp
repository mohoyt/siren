#include "ComputerCard.h"
#include "wavetables.h"
#include "dsp.h"
#include "oscillators.h"

// Siren: A drone oscillator for Music Thing Modular Workshop System Computer
// Ported from the vhikk-drone norns engine (inspired by Forge TME Vhikk X)
//
// Controls:
//   Switch UP:     Knobs = WARP / SPAN / MORPH
//   Switch MIDDLE: Knobs = SEED / SCAN / BASIS (pitch)
//   Switch DOWN:   Momentary tap cycles oscillator bank (LED shows which)
//
//   CV1: Pitch (added to BASIS)
//   CV2: WARP modulation
//   Pulse1: Gate (drone on/off)
//   Pulse2: Randomize SEED on trigger
//
//   Audio In 1/2: Ring-modulated with drone output
//   Audio Out 1/2: Stereo drone output

class Siren : public ComputerCard
{
public:
    Siren()
    {
        init_wavetables();

        // Default parameter values (midpoint)
        warp_raw = 0;
        span_raw = 1200;   // moderate spread
        morph_raw = 2048;  // center waveform
        seed_raw = 0;
        scan_raw = 0;
        basis_raw = 2048;  // ~110 Hz

        gate_open = true;  // start droning
    }

    virtual void __not_in_flash_func(ProcessSample)()
    {
        // ─── Read controls (smoothed) ───
        update_controls();

        // ─── Build oscillator params ───
        OscParams params;
        params.warp = warp_smooth.process(warp_raw);
        params.span = span_smooth.process(span_raw);
        params.morph = morph_smooth.process(morph_raw);
        params.seed = seed_val; // seed changes discretely, no smooth
        params.scan = scan_smooth.process(scan_raw);

        // BASIS: convert to phase increment
        // Range: ~55 Hz to ~440 Hz (2 octaves below to 2 octaves above 110)
        // basis_raw 0-4095 maps exponentially
        int32_t basis = basis_smooth.process(basis_raw);

        // Add CV1 pitch modulation
        if (Connected(Input::CV1))
        {
            basis += CVIn1(); // ±2048 added to 0-4095 range
            if (basis < 0) basis = 0;
            if (basis > 8191) basis = 8191;
        }

        // Exponential frequency mapping: 55 Hz at 0, 110 at 2048, 220 at 4095
        // Approximate 2^(basis/2048) * 55 Hz using piecewise linear
        int32_t freq_hz = exp_freq(basis);
        params.basis_freq = freq_hz_to_phase_inc(freq_hz);

        // CV2 modulates WARP
        if (Connected(Input::CV2))
        {
            int32_t cv_warp = params.warp + (CVIn2() + 2048); // shift CV to 0-4095ish
            if (cv_warp < 0) cv_warp = 0;
            if (cv_warp > 4095) cv_warp = 4095;
            params.warp = cv_warp;
        }

        // ─── Generate audio ───
        int16_t out_l = 0, out_r = 0;

        if (gate_open)
        {
            // Envelope: simple attack/release
            if (env_level < 32767)
            {
                env_level += 8; // ~85ms attack at 48kHz
                if (env_level > 32767) env_level = 32767;
            }
        }
        else
        {
            if (env_level > 0)
            {
                env_level -= 3; // ~225ms release
                if (env_level < 0) env_level = 0;
            }
        }

        if (env_level > 0)
        {
            switch (current_bank)
            {
                case 0: bank_sine.process(params, out_l, out_r); break;
                case 1: bank_cluster.process(params, out_l, out_r); break;
                case 2: bank_diatonic.process(params, out_l, out_r); break;
                case 3: bank_analogue.process(params, out_l, out_r); break;
                case 4: bank_waveshape.process(params, out_l, out_r); break;
                case 5: bank_wavetable.process(params, out_l, out_r); break;
            }

            // Apply envelope
            out_l = q15_mul(out_l, (int16_t)env_level);
            out_r = q15_mul(out_r, (int16_t)env_level);

            // Audio input ring modulation
            if (Connected(Input::Audio1))
            {
                int16_t ain = bit12_to_q15(AudioIn1());
                out_l = q15_mul(out_l, ain);
            }
            if (Connected(Input::Audio2))
            {
                int16_t ain = bit12_to_q15(AudioIn2());
                out_r = q15_mul(out_r, ain);
            }
        }

        // Output (Q15 -> 12-bit)
        AudioOut1(q15_to_12bit(out_l));
        AudioOut2(q15_to_12bit(out_r));

        // ─── LEDs: show current bank ───
        update_leds();
    }

private:
    // Oscillator bank instances
    BankSine bank_sine;
    BankCluster bank_cluster;
    BankDiatonic bank_diatonic;
    BankAnalogue bank_analogue;
    BankWaveshape bank_waveshape;
    BankWavetable bank_wavetable;

    // Current bank selection (0-5)
    int current_bank = 0;

    // Raw parameter values (0-4095)
    int32_t warp_raw, span_raw, morph_raw, seed_raw, scan_raw, basis_raw;
    int32_t seed_val = 0; // actual seed value (changes on trigger)

    // Parameter smoothers
    KnobSmoother warp_smooth, span_smooth, morph_smooth, scan_smooth, basis_smooth;

    // Envelope
    int32_t env_level = 0;
    bool gate_open;

    // Switch state tracking for bank cycling
    bool switch_was_down = false;

    // LED update counter (don't update every sample)
    uint16_t led_counter = 0;

    // Bank names for reference:
    // 0=SINE, 1=CLST, 2=DTON, 3=ANLG, 4=WSHP, 5=WAVE
    static constexpr int led_map[6] = {0, 1, 2, 3, 4, 5};

    void update_controls()
    {
        Switch sw = SwitchVal();

        if (sw == Switch::Up)
        {
            // Page 1: WARP / SPAN / MORPH
            warp_raw = KnobVal(Knob::Main);
            span_raw = KnobVal(Knob::X);
            morph_raw = KnobVal(Knob::Y);
        }
        else if (sw == Switch::Middle)
        {
            // Page 2: SEED / SCAN / BASIS
            seed_raw = KnobVal(Knob::Main);
            seed_val = seed_raw; // seed updates directly
            scan_raw = KnobVal(Knob::X);
            basis_raw = KnobVal(Knob::Y);
        }

        // Switch down = momentary bank cycle
        if (sw == Switch::Down)
        {
            if (!switch_was_down)
            {
                // Rising edge: cycle bank
                current_bank = (current_bank + 1) % NUM_BANKS;
                switch_was_down = true;
            }
        }
        else
        {
            switch_was_down = false;
        }

        // Pulse1: gate
        if (Connected(Input::Pulse1))
        {
            gate_open = PulseIn1();
        }

        // Pulse2: randomize seed on rising edge
        if (PulseIn2RisingEdge())
        {
            // Simple LCG for random seed
            static uint32_t rng = 42;
            rng = rng * 1664525 + 1013904223;
            seed_val = (rng >> 20) & 4095;
        }
    }

    void update_leds()
    {
        led_counter++;
        if (led_counter < 480) return; // Update ~100 Hz
        led_counter = 0;

        for (int i = 0; i < 6; i++)
        {
            if (i == current_bank)
                LedBrightness(i, 4095);
            else
                LedOff(i);
        }
    }

    // Approximate exponential frequency mapping
    // input 0-4095 -> ~55-440 Hz (3 octave range)
    // Uses piecewise linear approximation of 2^(x/1365) * 55
    int32_t exp_freq(int32_t input)
    {
        if (input < 0) input = 0;
        if (input > 8191) input = 8191;

        // Split into octave and fraction
        // Each 1365 units = 1 octave (for 0-4095 = 3 octaves)
        // For extended range (with CV): each 1365 = 1 octave
        int32_t octave = input / 1365;
        int32_t frac = input % 1365;

        // Base frequency for each octave: 55, 110, 220, 440, 880...
        int32_t base = 55;
        for (int i = 0; i < octave && i < 6; i++) base <<= 1;

        // Linear interpolation within octave (approximate exponential)
        // Next octave frequency is base * 2
        int32_t freq = base + ((base * frac) / 1365);

        return freq;
    }
};

int main()
{
    Siren siren;
    siren.EnableNormalisationProbe();
    siren.Run();
}
