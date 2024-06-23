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
int buffer_count = 0; // Global variable to keep track of the buffer count
float fft_output[FFT_SIZE];
float out_buffer[FFT_SIZE];
float mask_even[FFT_SIZE];
float mask_odd[FFT_SIZE];

void InitFFT() {
    arm_rfft_fast_init_f32(&S, FFT_SIZE);
}

void CreateMasks(int band_size, float offset, float mix) {
    int total_bins = FFT_SIZE / 2;
    int offset_bins = (static_cast<int>(offset * ((float)band_size)) * 2); // loop over 2 times when turning knob from 0 to 1

    // Kill DC component
    mask_even[0] = 0;
    mask_even[1] = 0;
    mask_odd[0] = 0;
    mask_odd[1] = 0;
    for (int i = 2; i < total_bins * 2; i += 2) {
        int bin_index = i / 2;
        int band_index = (bin_index + offset_bins) / band_size; // floor to get the index of the band
        bool even_band = band_index % 2 == 0;
        if (even_band) {
            mask_even[i] = mix;      // Real part
            mask_even[i + 1] = mix;  // Imaginary part
            mask_odd[i] = 1.0f;      // Real part
            mask_odd[i + 1] = 1.0f;  // Imaginary part
        } else {
            mask_odd[i] = 1.0f - mix;      // Real part
            mask_odd[i + 1] = 1.0f - mix;  // Imaginary part
            mask_even[i] = 1.0f;      // Real part
            mask_even[i + 1] = 1.0f;  // Imaginary part
        }
    }
}

// Currently SIMD makes no sense, we could just set the mix level with a single for loop instead of creating any masks at all
void ApplyMaskSIMD(float32_t* fft_data, float32_t* mask) {
    arm_mult_f32(fft_data, mask, fft_data, FFT_SIZE);
}

void DoMask(float32_t* fft_data, float32_t* even_mask, float32_t* odd_mask, float mix) {

}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) {
    hw.ProcessAllControls();

    float band_size = fmap(hw.GetKnobValue(KNOB_BLUR), 2.0, FFT_SIZE / 4, Mapping::LINEAR);
    float offset = hw.GetKnobValue(KNOB_WARP);
    float mix = hw.GetKnobValue(KNOB_MIX);

    CreateMasks(static_cast<int>(band_size), offset, mix);

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

        arm_rfft_fast_f32(&S, out_buffer, fft_output, 0);

        // Apply masks using SIMD
        ApplyMaskSIMD(fft_output, mask_even);
        ApplyMaskSIMD(fft_output, mask_odd);

        // Perform inverse FFT
        arm_rfft_fast_f32(&S, fft_output, out_buffer, 1);

        // Output the last BUFFER_SIZE samples to the audio buffer
        memcpy(out[0], &out_buffer[FFT_SIZE - BUFFER_SIZE], BUFFER_SIZE * sizeof(float)); // Mixed output
    }

    if (buffer_count >= (FFT_SIZE / BUFFER_SIZE) * 10) {
        buffer_count = (FFT_SIZE / BUFFER_SIZE); // don't overflow, ugly af.
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
