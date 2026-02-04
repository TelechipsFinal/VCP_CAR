// kalman_filter.h
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "sal_com.h"

//temp
#define KALMAN_Q_ANGLE    0.003f    // 작으면 자이로 중심
#define KALMAN_Q_BIAS     0.00002f  // 드리프트 천천히 제거
#define KALMAN_R_MEASURE  0.1f      // 작으면 가속도계 중심


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

#endif