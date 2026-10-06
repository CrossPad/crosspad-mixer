#pragma once
/* Host stand-in for the two esp-dsp calls AudioMixerEngine makes, so the
 * tests can run render() down the firmware's route (CROSSPAD_USE_ESP_DSP).
 * Same arithmetic as esp-dsp on the ESP32-S3: dsps_mulc_f32_ae32 is one
 * mul.s per sample and dsps_add_f32_ae32 one add.s (no fused multiply-add),
 * so each result is rounded to float before the next pass reads it. */

typedef int esp_err_t;

static inline esp_err_t dsps_mulc_f32(const float *input, float *output, int len, float C,
                                      int step_in, int step_out)
{
    for (int i = 0; i < len; ++i) output[i * step_out] = input[i * step_in] * C;
    return 0;
}

static inline esp_err_t dsps_add_f32(const float *input1, const float *input2, float *output,
                                     int len, int step1, int step2, int step_out)
{
    for (int i = 0; i < len; ++i) output[i * step_out] = input1[i * step1] + input2[i * step2];
    return 0;
}
