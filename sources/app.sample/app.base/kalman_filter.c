#include "kalman_filter.h"
#include <math.h>
#include <ICM_20948.h>




void Kalman_Init(Kalman_t *kf)
{
    kf->angle = 0.0f;
    kf->bias = 0.0f;

    kf->P[0][0] = 1.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;

    kf->Q_angle = KALMAN_Q_ANGLE;
    kf->Q_bias = KALMAN_Q_BIAS;
    kf->R_measure = KALMAN_R_MEASURE;
}

float Kalman_Update(Kalman_t *kf, float newAngle, float newRate, float dt)
{
    // Predict
    float rate = newRate - kf->bias;
    kf->angle += dt * rate;

    kf->P[0][0] += dt * (dt*kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    // Update
    float y = newAngle - kf->angle;
    float S = kf->P[0][0] + kf->R_measure;
    float K[2];
    
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    kf->angle += K[0] * y;
    kf->bias  += K[1] * y;

    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}

