// kalman_filter.h
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "sal_com.h"

//temp
#define KALMAN_Q_ANGLE    0.003f    // 작으면 자이로 중심
#define KALMAN_Q_BIAS     0.04f  // 드리프트 천천히 제거
#define KALMAN_R_MEASURE  2.0f      // 작으면 가속도계 중심


typedef struct {
    float angle;
    float bias;
    float P[2][2];
    float Q_angle;
    float Q_bias;
    float R_measure;
} Kalman_t;

void Kalman_Init(Kalman_t *kf);
float Kalman_Update(Kalman_t *kf, float newAngle, float newRate, float dt);
static inline void Kalman_SetR(Kalman_t *kf, float R)
{
    if (R < 0.0001f) R = 0.0001f;
    kf->R_measure = R;
}

// ✅ 추가: Predict-only (가속도 신뢰도 낮을 때 사용)
static inline float Kalman_PredictOnly(Kalman_t *kf, float newRate, float dt)
{
    float rate = newRate - kf->bias;
    kf->angle += dt * rate;

    kf->P[0][0] += dt * (dt*kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    return kf->angle;
}

#endif