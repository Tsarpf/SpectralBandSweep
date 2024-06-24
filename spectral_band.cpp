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
        float weight = even_band ? mix : (1.0f - mix);

        // Apply smoothing (e.g., linear interpolation between bands)
        //int next_bin_index = (bin_index + 1) / dynamic_band_size;
        //bool next_even_band = next_bin_index % 2 == 0;
        //float next_weight = next_even_band ? mix : (1.0f - mix);
        //// Interpolate weights between current and next band
        //float smoothed_weight = (weight + next_weight) / 2.0f;
        //mask[i] = smoothed_weight;      // Real part
        //mask[i + 1] = smoothed_weight;  // Imaginary part

        // Brickwall version, maybe the Griffin-Lim will save us.
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

    float band_size = fmap(hw.GetKnobValue(KNOB_BLUR), 2.0, FFT_SIZE / 2, Mapping::LINEAR);
    band_size = 2 + band_size / 6; // Limit to more useful range
    float offset = hw.GetKnobValue(KNOB_WARP);
    float mix = hw.GetKnobValue(KNOB_MIX);

    CreateMask(static_cast<int>(band_size), offset, mix);

    // Copy incoming audio samples into the circular buffer
    for (size_t i = 0; i < size; ++i) {
        circular_buffer[write_ptr] = in[0][i];
        write_ptr = (write_ptr + 1) % CIRCULAR_BUFFER_SIZE;
    }

    // Increment the buffer count
    buffer_count++;

    // Only process FFT after the initial N-1 buffers
    if (buffer_count < FFT_SIZE / BUFFER_SIZE) {
        memcpy(out[0], in[0], BUFFER_SIZE * sizeof(float));
        return;
    }

    // Perform FFT on the most recent FFT_SIZE samples
    int start_index = (write_ptr + CIRCULAR_BUFFER_SIZE - FFT_SIZE) % CIRCULAR_BUFFER_SIZE;
    for (int i = 0; i < FFT_SIZE; ++i) {
        int index = (start_index + i) % CIRCULAR_BUFFER_SIZE;
        fft_input[i] = circular_buffer[index];
    }

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

    // Save the end of the current window for the next overlap
    for (int i = 0; i < OVERLAP_SIZE; ++i) {
        overlap_buffer[i] = ifft_output[BUFFER_SIZE + i];
    }

    // Output the middle BUFFER_SIZE samples to the audio buffer
    for (size_t i = 0; i < BUFFER_SIZE; ++i) {
        out[0][i] = ifft_output[BUFFER_SIZE + i];
    }

    if (buffer_count >= (FFT_SIZE / BUFFER_SIZE) * 10) {
        buffer_count = (FFT_SIZE / BUFFER_SIZE); // Don't overflow, ugly af.
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

    hw.StartAudio(AudioCallback);

    while (1) { }
}
