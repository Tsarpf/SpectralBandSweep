#include "aurora.h"
#include "daisysp.h"
#include "arm_math.h"

using namespace daisy;
using namespace aurora;
using namespace daisysp;

Hardware hw;

#define FFT_SIZE 1024
#define FFT_OUTPUT_SIZE (FFT_SIZE / 2 + 1)

arm_rfft_fast_instance_f32 S;
float in_buffer[FFT_SIZE];
float fft_output[FFT_OUTPUT_SIZE];
float out_buffer[FFT_SIZE];
float mask_even[FFT_OUTPUT_SIZE];
float mask_odd[FFT_OUTPUT_SIZE];

void InitFFT() {
    arm_rfft_fast_init_f32(&S, FFT_SIZE);
}

void CreateMasks(float band_size, float offset, float mix) {
    int offset_bins = static_cast<int>(offset * FFT_OUTPUT_SIZE);

    for (int i = 0; i < FFT_OUTPUT_SIZE; ++i) {
        int offset_index = (i + offset_bins) % FFT_OUTPUT_SIZE;
        if ((offset_index / static_cast<int>(band_size)) % 2 == 0) {
            mask_even[offset_index] = mix;
            mask_odd[offset_index] = 0.0f;
        } else {
            mask_even[offset_index] = 0.0f;
            mask_odd[offset_index] = mix;
        }
    }
}

void ApplyMaskSIMD(float32_t* fft_data, float32_t* mask) {
    arm_mult_f32(fft_data, mask, fft_data, FFT_OUTPUT_SIZE);
}

//void ApplyMasks(float32_t* fft_data, float32_t* mask_even, float32_t* mask_odd) {
//    ApplyMasksSIMD(fft_data, mask_even);
//    ApplyMasksSIMD(fft_data, mask_odd);
//    arm_mult_f32(fft_data, mask_even, fft_data, FFT_OUTPUT_SIZE);
//    arm_mult_f32(fft_data, mask_odd, fft_data, FFT_OUTPUT_SIZE);
//}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) {
    hw.ProcessAllControls();

    float band_size = fmap(hw.GetKnobValue(KNOB_BLUR), 500.0, 10000.0, Mapping::LOG);
    float offset = hw.GetKnobValue(KNOB_WARP);
    float mix = hw.GetKnobValue(KNOB_MIX);

    CreateMasks(band_size, offset, mix);

    for (size_t i = 0; i < size; i += FFT_SIZE) {
        // Copy input to in_buffer
        memcpy(in_buffer, &in[0][i], FFT_SIZE * sizeof(float));

        // Perform FFT
        arm_rfft_fast_f32(&S, in_buffer, fft_output, 0);

        // Apply masks using SIMD
        // Eventually save the first result to a separate buffer so it can be output from the other channel
        ApplyMaskSIMD(fft_output, mask_even);
        ApplyMaskSIMD(fft_output, mask_odd);

        // Perform inverse FFT
        arm_rfft_fast_f32(&S, fft_output, out_buffer, 1);

        // Copy output to the audio buffer
        for (size_t j = 0; j < FFT_SIZE; ++j) {
            out[0][i + j] = out_buffer[j]; // Mixed output

            // ugly GPT stuff that makes no sense
            //out[1][i + j] = mask_even[j % FFT_OUTPUT_SIZE] * out_buffer[j]; // Even-masked output
        }
    }
}

int main(void) {
    hw.Init();

    hw.SetAudioBlockSize(FFT_SIZE);
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

    InitFFT();
    hw.StartAudio(AudioCallback);

    while (1) { }
}
