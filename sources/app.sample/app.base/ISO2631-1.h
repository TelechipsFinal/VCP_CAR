// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 7s@100Hz=700; keep some headroom */
#ifndef ISO2631_MAX_SAMPLES
#define ISO2631_MAX_SAMPLES  800U
#endif

typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float z1, z2;
} Biquad_t;

typedef struct {
    Biquad_t sec1;
    Biquad_t sec2;
    Biquad_t sec3;
    Biquad_t sec4;
} WkChain_t;

typedef struct {
    float rms;                 /* m/s^2 */
    float vdv;                 /* m/s^1.75 */
    float peak;                /* m/s^2 */
    float duration_sec;        /* s */
    uint32_t samples;
    float gravity_offset_ms2;  /* m/s^2 */
} ISO2631_Metrics_t;

typedef struct {
    float fs_hz;
    float window_sec;
    uint32_t window_samples;
    float warmup_sec;

    WkChain_t wk;

    uint8_t active;
    uint8_t warmup_done;
    uint32_t start_ms;

    uint32_t warmup_samples;
    float gravity_sum;
    uint32_t gravity_cnt;
    float gravity_offset_ms2;

    float buf[ISO2631_MAX_SAMPLES];
    uint32_t buf_index;
    uint32_t sample_count;

    float rms;
    float vdv;
    float peak;
} ISO2631_State_t;

/* API */
void ISO2631_Init(float fs_hz);
void ISO2631_SetWarmup(float warmup_sec);
void ISO2631_Start(float window_sec, uint32_t now_ms);
uint8_t ISO2631_Update(float az_raw_ms2, uint32_t now_ms);
void ISO2631_Stop(void);

uint8_t ISO2631_IsActive(void);
uint8_t ISO2631_WarmupDone(void);
ISO2631_Metrics_t ISO2631_GetMetrics(void);

#ifdef __cplusplus
}
#endif
