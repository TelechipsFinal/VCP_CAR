// ISO2631-1.h
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float duration_s;
    uint16_t samples;
    float fs_hz;

    float gravity_offset_ms2; // 평균 중력/오프셋(m/s^2)
    float rms;                // a_w (m/s^2)
    float vdv;                // (m/s^1.75)
    float peak;               // |a_w| peak (m/s^2)
} ISO2631_Metrics_t;

// 초기화 (fs_hz=100 권장)
void ISO2631_Init(float fs_hz);

// 워밍업(필터 안정화) 샘플수, 중력 오프셋 샘플수 설정
void ISO2631_SetWarmup(uint16_t warmup_samples, uint16_t gravity_samples);

// 측정 시작 (window_samples 만큼 쌓이면 done=1 반환)
void ISO2631_Start(uint16_t window_samples);

// 샘플 추가: raw_z_ms2 = IMU의 Z축 가속도 (m/s^2)
// return: 0=진행중, 1=윈도우 완료(결과 계산됨)
uint8_t ISO2631_Update(float raw_z_ms2);

// 결과 가져오기 (윈도우 완료 후 호출)
ISO2631_Metrics_t ISO2631_GetMetrics(void);

#ifdef __cplusplus
}
#endif
