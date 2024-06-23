#include "aurora.h"
#include "daisysp.h"
#include "arm_math.h"

using namespace daisy;
using namespace aurora;
using namespace daisysp;

Hardware hw;

#define FFT_SIZE 256
#define FFT_OUTPUT_SIZE (FFT_SIZE) // Corrected size

arm_rfft_fast_instance_f32 S;
float in_buffer[FFT_SIZE];
float out_buffer[FFT_SIZE];
float fft_output[FFT_OUTPUT_SIZE];
float mask_even[FFT_OUTPUT_SIZE];
float mask_odd[FFT_OUTPUT_SIZE];

void InitFFT() {
    arm_rfft_fast_init_f32(&S, FFT_SIZE);
}

void CreateMasks(int band_size, float offset, float mix) {
    int total_bins = FFT_SIZE / 2;
    int offset_bins = (static_cast<int>(offset * total_bins) * 4) % total_bins; // loop over 4 times when turning knob from 0 to 1

    for (int i = 0; i < total_bins * 2; i += 2) {
        int bin_index = i / 2;
        int band_index = static_cast<int>((bin_index + offset_bins) / band_size); // floor to get the index of the band
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

void ApplyMaskSIMD(float32_t* fft_data, float32_t* mask) {
    arm_mult_f32(fft_data, mask, fft_data, FFT_OUTPUT_SIZE);
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) {
    hw.ProcessAllControls();
    // Copy the input to the right output buffer

    memcpy(out[1], in[0], FFT_SIZE * sizeof(float)); // Original input to right channel

    float band_size = fmap(hw.GetKnobValue(KNOB_BLUR), 2.0, 64.0, Mapping::LINEAR);
    float offset = hw.GetKnobValue(KNOB_WARP);
    float mix = hw.GetKnobValue(KNOB_MIX);

    CreateMasks(static_cast<int>(band_size), offset, mix);

    // Copy input to in_buffer
    memcpy(in_buffer, in[0], FFT_SIZE * sizeof(float));

    // Perform FFT on the input buffer
    arm_rfft_fast_f32(&S, in_buffer, fft_output, 0);

    // Apply even and odd masks using SIMD on the same fft_output
    ApplyMaskSIMD(fft_output, mask_even);
    ApplyMaskSIMD(fft_output, mask_odd);

    // Perform inverse FFT on the mixed FFT output
    arm_rfft_fast_f32(&S, fft_output, out_buffer, 1);

    // Copy the result to the left output buffer
    memcpy(out[0], out_buffer, FFT_SIZE * sizeof(float)); // Mixed output
}

int main(void) {
    //hw.SetAudioBlockSize(FFT_SIZE);
    //hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

    hw.Init();

    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    hw.SetAudioBlockSize(FFT_SIZE);
    hw.UpdateHidRates();

    InitFFT();
    hw.StartAudio(AudioCallback);

    while (1) { }
}
