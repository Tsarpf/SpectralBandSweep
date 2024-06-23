#include "aurora.h"
#include "daisysp.h"
#include "kiss_fft.h"
#include "kiss_fftr.h"
#include "arm_math.h"

using namespace daisy;
using namespace aurora;
using namespace daisysp;

Hardware hw;

#define FFT_SIZE 1024
// float32_t magnitudes[FFT_SIZE / 2];
// float32_t mask[FFT_SIZE / 2];
// float32_t result[FFT_SIZE / 2];

kiss_fftr_cfg forward_cfg;
kiss_fftr_cfg inverse_cfg;
float in_buffer[FFT_SIZE];
kiss_fft_cpx out_buffer[FFT_SIZE / 2 + 1];
kiss_fft_cpx processed_buffer[FFT_SIZE / 2 + 1];
float out_time_buffer[FFT_SIZE];

// Mask arrays for even and odd bands
float mask_even[FFT_SIZE / 2 + 1];
float mask_odd[FFT_SIZE / 2 + 1];

void InitFFT() {
    forward_cfg = kiss_fftr_alloc(FFT_SIZE, 0, NULL, NULL);
    inverse_cfg = kiss_fftr_alloc(FFT_SIZE, 1, NULL, NULL);
}

void CreateMasks(float band_size, float offset) {
    int total_bins = FFT_SIZE / 2 + 1;
    int offset_bins = static_cast<int>(offset * total_bins);

    for (int i = 0; i < total_bins; ++i) {
        int index = (i + offset_bins) % total_bins;
        if ((index / static_cast<int>(band_size)) % 2 == 0) {
            mask_even[i] = 1.0f;
            mask_odd[i] = 0.0f;
        } else {
            mask_even[i] = 0.0f;
            mask_odd[i] = 1.0f;
        }
    }
}

void ApplyMasks() {
    for (int i = 0; i < FFT_SIZE / 2 + 1; ++i) {
        processed_buffer[i].r = out_buffer[i].r * (mask_even[i] + mask_odd[i]);
        processed_buffer[i].i = out_buffer[i].i * (mask_even[i] + mask_odd[i]);
    }
}

void ApplyMaskSIMD(float32_t* magnitudes, float32_t* mask, float32_t* result, uint32_t length) {
    arm_mult_f32(magnitudes, mask, result, length);
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) {
    hw.ProcessAllControls();

    float band_size = fmap(hw.GetKnobValue(KNOB_BLUR), 500.0, 10000.0, Mapping::LOG);
    float offset = hw.GetKnobValue(KNOB_WARP);
    float mix = hw.GetKnobValue(KNOB_MIX);

    CreateMasks(band_size, offset);

    for (size_t i = 0; i < size; i += FFT_SIZE) {
        for (size_t j = 0; j < FFT_SIZE; ++j) {
            in_buffer[j] = in[0][i + j];
        }

        kiss_fftr(forward_cfg, in_buffer, out_buffer);
        //ApplyMasks();
        ApplyMaskSIMD(magnitudes, mask, result, FFT_SIZE / 2);
        kiss_fftri(inverse_cfg, processed_buffer, out_time_buffer);

        for (size_t j = 0; j < FFT_SIZE; ++j) {
            float dry_left = in[0][i + j];
            float dry_right = in[1][i + j];
            float wet_left = out_time_buffer[j] * mask_even[j % (FFT_SIZE / 2 + 1)];
            float wet_right = out_time_buffer[j] * mask_odd[j % (FFT_SIZE / 2 + 1)];
            out[0][i + j] = (dry_left * (1.f - mix)) + (wet_left * mix);
            out[1][i + j] = (dry_right * (1.f - mix)) + (wet_right * mix);
        }
    }
}

int main(void) {
    hw.Init();

    InitFFT();
    hw.StartAudio(AudioCallback);

    while (1) { }
}
