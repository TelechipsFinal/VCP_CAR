#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include "sal_com.h"

// 캘리브레이션 설정
#define CALIB_SAMPLE_COUNT      1000    // 1000 샘플 = 10초 @ 100Hz
#define CALIB_SAMPLE_RATE_MS    10      // 10ms = 100Hz

// 결과 구조체
typedef struct {
    float mean;
    float variance;
    float std_dev;
    float min;
    float max;
} CalibrationResult_t;

typedef struct {
    CalibrationResult_t accel_x;
    CalibrationResult_t accel_y;
    CalibrationResult_t accel_z;
    CalibrationResult_t gyro_x;
    CalibrationResult_t gyro_y;
    CalibrationResult_t gyro_z;
} IMU_CalibrationData_t;

// 함수 선언
void IMU_Calibration_Task(void *pArg);
void Calculate_Statistics(float *data, uint32 count, CalibrationResult_t *result);

#endif /* IMU_CALIBRATION_H */