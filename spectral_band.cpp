#include "aurora.h"
#include "daisysp.h"
#include "arm_math.h"

using namespace daisy;
using namespace aurora;
using namespace daisysp;

Hardware hw;

#define FFT_SIZE 2048
#define BUFFER_SIZE 256
#define OVERLAP_SIZE (FFT_SIZE - BUFFER_SIZE)
#define CIRCULAR_BUFFER_SIZE (FFT_SIZE + BUFFER_SIZE)

arm_rfft_fast_instance_f32 S;
float circular_buffer[CIRCULAR_BUFFER_SIZE];
int write_ptr = 0;
int buffer_count = 0;
float fft_input[FFT_SIZE];
float fft_output[FFT_SIZE];
float ifft_output[FFT_SIZE];
float mask[FFT_SIZE];
float phase_accum[FFT_SIZE / 2];
float overlap_buffer[OVERLAP_SIZE];

#define COMB_FILTER_SIZE 8192
float comb_filter_buffer[COMB_FILTER_SIZE];
int comb_filter_write_index = 0;

// Initialize comb filter buffer
void InitCombFilter() {
    memset(comb_filter_buffer, 0, sizeof(comb_filter_buffer));
    comb_filter_write_index = 0;
}

// Comb filter function
float CombFilter(float input, float time, float reflect) {
    int delay_samples = static_cast<int>(time * (COMB_FILTER_SIZE - 2)) + 1;
    int read_index = (comb_filter_write_index - delay_samples + COMB_FILTER_SIZE) % COMB_FILTER_SIZE;

    float delayed_sample = comb_filter_buffer[read_index];
    float output = input + delayed_sample * reflect;

    comb_filter_buffer[comb_filter_write_index] = output;
    comb_filter_write_index = (comb_filter_write_index + 1) % COMB_FILTER_SIZE;

    return output;
}

// Tilt EQ function
float TiltEQ(float sample, float atmosphere) {
    // Simple first-order high-pass and low-pass filters for tilt EQ
    static float hp_output = 0.0f;
    static float lp_output = 0.0f;

    float alpha = atmosphere; // Tilt amount

    // Low-pass filter
    lp_output += alpha * (sample - lp_output);

    // High-pass filter
    hp_output = sample - lp_output;

    // Combine low-pass and high-pass for tilt EQ
    return (1.0f - atmosphere) * lp_output + atmosphere * hp_output;
}


void InitFFT() {
    arm_rfft_fast_init_f32(&S, FFT_SIZE);
}

void CreateMask(int band_size, float offset, float mix) {
    int total_bins = FFT_SIZE / 2;
    int max_offset = MAX(100, band_size);
    int offset_bins = static_cast<int>(offset * max_offset) * 8; // loop over 2 times when turning knob from 0 to 1

    // Kill DC component
    mask[0] = 0;
    mask[1] = 0;

    for (int i = 2; i < total_bins * 2; i += 2) {
        int bin_index = i / 2 + offset_bins;
        float frequency = bin_index * (48000.0f / FFT_SIZE); // Convert bin index to frequency
        
        // Adjust the strength of logarithmic scaling to be more subtle
        float scale_factor = log2f(1.0f + frequency / 1000.0f);
        int dynamic_band_size = band_size + static_cast<int>(band_size * scale_factor * 0.3); // Adjust the scaling factor to be more subtle
        
        int band_index = bin_index / dynamic_band_size; // Floor to get the index of the band
        bool even_band = band_index % 2 == 0;

        // beautiful GPT ternary
        // [0, 1] for 0-50%, [1, 8] for 50-100%
        float weight = even_band ? 1.0f : (mix < 0.5f ? mix * 2.0f : 1.0f + (mix - 0.5f) * 100.0f); // 0-1 for 0-50%, 1-8 for 50-100%

        mask[i] = weight;      // Real part
        mask[i + 1] = weight;  // Imaginary part
    }
}

void ApplyMaskSIMD(float32_t* fft_data, float32_t* mask) {
    arm_mult_f32(fft_data, mask, fft_data, FFT_SIZE);
}

void ApplyWindowFunction(float* buffer, int size) {
    for (int i = 0; i < size; ++i) {
        float hamming = 0.54 - 0.46 * cos(2.0 * M_PI * i / (size - 1));
        buffer[i] *= hamming;
    }
}

void MaintainPhaseContinuity(float* fft_data, float* phase_accum, int size, int hop_size) {
    for (int i = 0; i < size; i += 2) {
        int bin_index = i / 2;
        float real = fft_data[i];
        float imag = fft_data[i + 1];
        float magnitude = sqrtf(real * real + imag * imag);
        float phase = atan2f(imag, real);

        // Calculate the expected phase advance
        float expected_phase_advance = 2.0f * M_PI * bin_index * hop_size / FFT_SIZE;
        float phase_diff = phase - phase_accum[bin_index] - expected_phase_advance;
        phase_diff = fmod(phase_diff + M_PI, 2.0f * M_PI) - M_PI; // Wrap phase_diff to [-pi, pi]
        phase_accum[bin_index] += expected_phase_advance + phase_diff;

        // Adjust the phase
        float adjusted_phase = phase_accum[bin_index];
        fft_data[i] = magnitude * cosf(adjusted_phase);
        fft_data[i + 1] = magnitude * sinf(adjusted_phase);
    }
}

void ApplyGriffinLim(float* magnitudes, float* phases, int fft_size, int max_iterations) {
    // Temporary buffers for FFT and IFFT results
    float temp_fft_output[fft_size];
    float temp_ifft_output[fft_size];

    // Initialize temporary buffers with initial values
    for (int i = 0; i < fft_size; i += 2) {
        temp_fft_output[i] = magnitudes[i / 2] * cosf(phases[i / 2]);
        temp_fft_output[i + 1] = magnitudes[i / 2] * sinf(phases[i / 2]);
    }

    for (int iter = 0; iter < max_iterations; ++iter) {
        // Perform IFFT to get time-domain signal
        arm_rfft_fast_f32(&S, temp_fft_output, temp_ifft_output, 1);

        // Perform FFT to get new magnitude and phase estimates
        arm_rfft_fast_f32(&S, temp_ifft_output, temp_fft_output, 0);

        // Update magnitudes to the original target magnitudes and keep the newly computed phases
        for (int i = 0; i < fft_size; i += 2) {
            float real = temp_fft_output[i];
            float imag = temp_fft_output[i + 1];
            float magnitude = magnitudes[i / 2]; // Use the target magnitudes
            float phase = atan2f(imag, real);
            temp_fft_output[i] = magnitude * cosf(phase);
            temp_fft_output[i + 1] = magnitude * sinf(phase);
        }
    }

    // Final update of magnitudes with refined phases
    for (int i = 0; i < fft_size; i += 2) {
        magnitudes[i / 2] = sqrtf(temp_fft_output[i] * temp_fft_output[i] + temp_fft_output[i + 1] * temp_fft_output[i + 1]);
        phases[i / 2] = atan2f(temp_fft_output[i + 1], temp_fft_output[i]);
    }
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) {
    hw.ProcessAllControls();

    float band_size = fmap(hw.GetKnobValue(KNOB_BLUR) + hw.GetCvValue(CV_BLUR), 2.0, FFT_SIZE / 2, Mapping::LINEAR);
    band_size = 2 + band_size / 6; // Limit to more useful range
    float offset = hw.GetKnobValue(KNOB_WARP) + hw.GetCvValue(CV_WARP);
    float mix = hw.GetKnobValue(KNOB_MIX) + hw.GetCvValue(CV_MIX);

    float time = hw.GetKnobValue(KNOB_TIME); // Time parameter for comb filter
    float reflect = hw.GetKnobValue(KNOB_REFLECT); // Reflect parameter for comb filter
    float atmosphere = hw.GetKnobValue(KNOB_ATMOSPHERE); // Atmosphere parameter for tilt EQ

    time += hw.GetCvValue(CV_TIME);
    reflect += hw.GetCvValue(CV_REFLECT);
    atmosphere += hw.GetCvValue(CV_ATMOSPHERE);

    CreateMask(static_cast<int>(band_size), offset, mix);

    // Copy incoming audio samples into the circular buffer
    for (size_t i = 0; i < size; ++i) {
        circular_buffer[write_ptr] = in[0][i];
        write_ptr = (write_ptr + 1) % CIRCULAR_BUFFER_SIZE;
    }


    // Only process FFT after the initial N-1 buffers
    if (buffer_count < FFT_SIZE / BUFFER_SIZE) {
        memcpy(out[0], in[0], BUFFER_SIZE * sizeof(float));
        buffer_count++;
        return;
    }

    // Perform FFT on the most recent FFT_SIZE samples
    int start_index = (write_ptr + CIRCULAR_BUFFER_SIZE - FFT_SIZE) % CIRCULAR_BUFFER_SIZE;
    for (int i = 0; i < FFT_SIZE; ++i) {
        int index = (start_index + i) % CIRCULAR_BUFFER_SIZE;
        fft_input[i] = circular_buffer[index];
    }

    // Check RMS before so we can normalize after
    float32_t rms_before = 0;
    arm_rms_f32(fft_input, FFT_SIZE, &rms_before);

    ApplyWindowFunction(fft_input, FFT_SIZE);

    arm_rfft_fast_f32(&S, fft_input, fft_output, 0);

    // Apply mask using SIMD
    ApplyMaskSIMD(fft_output, mask);

    // Prepare magnitude and phase arrays for Griffin-Lim
    float magnitudes[FFT_SIZE / 2];
    float phases[FFT_SIZE / 2];
    for (int i = 0; i < FFT_SIZE; i += 2) {
        float real = fft_output[i];
        float imag = fft_output[i + 1];
        magnitudes[i / 2] = sqrtf(real * real + imag * imag);
        phases[i / 2] = atan2f(imag, real);
    }

    // Apply Griffin-Lim algorithm
    ApplyGriffinLim(magnitudes, phases, FFT_SIZE, 1);

    // Construct the complex signal from refined magnitudes and phases for IFFT
    for (int i = 0; i < FFT_SIZE; i += 2) {
        fft_output[i] = magnitudes[i / 2] * cosf(phases[i / 2]);
        fft_output[i + 1] = magnitudes[i / 2] * sinf(phases[i / 2]);
    }

    // Perform inverse FFT
    arm_rfft_fast_f32(&S, fft_output, ifft_output, 1);

    // Overlap-add to reconstruct the time-domain signal
    for (int i = 0; i < OVERLAP_SIZE; ++i) {
        ifft_output[i] += overlap_buffer[i];
    }

    float rms_after = 0.0f;
    arm_rms_f32(ifft_output, FFT_SIZE, &rms_after);

    if (mix > 0.5f)
    {
        // Normalize the output to match the input RMS
        float normalize_factor = rms_before / rms_after;
        for (int i = 0; i < FFT_SIZE; ++i) {
            ifft_output[i] *= normalize_factor;
        }
    }

    // Apply comb filter and tilt EQ
    for (size_t i = 0; i < BUFFER_SIZE; ++i) {
        float combed_signal = CombFilter(ifft_output[BUFFER_SIZE + i], time, reflect);
        out[0][i] = TiltEQ(combed_signal, atmosphere);
    }

    // Save the end of the current window for the next overlap
    for (int i = 0; i < OVERLAP_SIZE; ++i) {
        overlap_buffer[i] = ifft_output[BUFFER_SIZE + i];
    }

    // Copy the input to the right output buffer
    memcpy(out[1], in[0], BUFFER_SIZE * sizeof(float)); // Original input to right channel
}

int main(void) {
    hw.Init();

    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    hw.SetAudioBlockSize(BUFFER_SIZE);
    hw.UpdateHidRates();

    InitFFT();
    InitCombFilter();

    hw.StartAudio(AudioCallback);

    while (1) { }
}
