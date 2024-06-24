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
float fft_output[FFT_SIZE];
float ifft_output[FFT_SIZE];
float out_buffer[CIRCULAR_BUFFER_SIZE];
float window[FFT_SIZE];
float mask[FFT_SIZE];
float phase_accum[FFT_SIZE / 2];

void InitFFT() {
    arm_rfft_fast_init_f32(&S, FFT_SIZE);
}

void CreateMask(int band_size, float offset, float mix) {
    int total_bins = FFT_SIZE / 2;

    int max_offset = MAX(50, band_size);
    int offset_bins = static_cast<int>(offset * max_offset) * 2; // loop over 2 times when turning knob from 0 to 1

    // Kill DC component
    mask[0] = 0;
    mask[1] = 0;
    for (int i = 2; i < total_bins * 2; i += 2) {
        int bin_index = i / 2 + offset_bins;
        int band_index = bin_index / band_size; // floor to get the index of the band
        bool even_band = band_index % 2 == 0;
        float weight = even_band ? mix : (1.0f - mix);

        // Apply smoothing (e.g., linear interpolation between bands)
        int next_band_index = (bin_index + 1) / band_size;
        bool next_even_band = next_band_index % 2 == 0;
        float next_weight = next_even_band ? mix : (1.0f - mix);

        // Interpolate weights between current and next band
        float smoothed_weight = (weight + next_weight) / 2.0f;

        mask[i] = smoothed_weight;      // Real part
        mask[i + 1] = smoothed_weight;  // Imaginary part
    }
}

void ApplyMaskSIMD(float32_t* fft_data, float32_t* mask) {
    arm_mult_f32(fft_data, mask, fft_data, FFT_SIZE);
}

void ApplyWindowFunction(float* buffer, int size) {
    for (int i = 0; i < size; ++i) {
        buffer[i] *= window[i];
    }
}

void GenerateWindowFunction(float* window, int size) {
    for (int i = 0; i < size; ++i) {
        window[i] = 0.54 - 0.46 * cos(2.0 * M_PI * i / (size - 1)); // Hamming window
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

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) {
    hw.ProcessAllControls();

    float band_size = fmap(hw.GetKnobValue(KNOB_BLUR), 2.0, FFT_SIZE / 4, Mapping::LINEAR);
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
    if (buffer_count >= FFT_SIZE / BUFFER_SIZE) {
        // Perform FFT on the most recent FFT_SIZE samples
        int start_index = (write_ptr + CIRCULAR_BUFFER_SIZE - FFT_SIZE) % CIRCULAR_BUFFER_SIZE;
        for (int i = 0; i < FFT_SIZE; ++i) {
            int index = (start_index + i) % CIRCULAR_BUFFER_SIZE;
            out_buffer[i] = circular_buffer[index];
        }

        ApplyWindowFunction(out_buffer, FFT_SIZE);

        arm_rfft_fast_f32(&S, out_buffer, fft_output, 0);

        // Apply band-pass filter to remove unwanted frequencies
        int low_bin = static_cast<int>(30.0f * FFT_SIZE / 48000.0f);
        int high_bin = static_cast<int>(15000.0f * FFT_SIZE / 48000.0f);
        for (int i = 0; i < low_bin * 2; ++i) {
            fft_output[i] = 0.0f; // Zero out bins below the high-pass filter
        }
        for (int i = high_bin * 2 + 1; i < FFT_SIZE; ++i) {
            fft_output[i] = 0.0f; // Zero out bins above the low-pass filter
        }

        // Apply mask using SIMD
        ApplyMaskSIMD(fft_output, mask);

        // Maintain phase continuity
        MaintainPhaseContinuity(fft_output, phase_accum, FFT_SIZE, BUFFER_SIZE);

        // Perform inverse FFT
        arm_rfft_fast_f32(&S, fft_output, ifft_output, 1);

        // Overlap-add to reconstruct the time-domain signal
        static float previous_overlap[OVERLAP_SIZE] = {0}; // Buffer to store the overlap

        // Add the end of the previous window to the start of the new window
        for (int i = 0; i < OVERLAP_SIZE; ++i) {
            ifft_output[i] += previous_overlap[i];
        }

        // Save the end of the current window for the next overlap
        for (int i = 0; i < OVERLAP_SIZE; ++i) {
            previous_overlap[i] = ifft_output[BUFFER_SIZE + i];
        }

        // Output the middle BUFFER_SIZE samples to the audio buffer
        for (size_t i = 0; i < BUFFER_SIZE; ++i) {
            out[0][i] = ifft_output[OVERLAP_SIZE / 2 + i];
        }
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
    GenerateWindowFunction(window, FFT_SIZE);

    hw.StartAudio(AudioCallback);

    while (1) { }
}
