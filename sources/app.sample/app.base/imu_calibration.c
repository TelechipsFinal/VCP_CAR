#include "imu_calibration.h"
#include <sal_internal.h>
#include <debug.h>
#include <math.h>
#include <ICM_20948.h>
#include "printf_float.h"


// 샘플 버퍼 (전역 변수)
static float accel_x_samples[CALIB_SAMPLE_COUNT];
static float accel_y_samples[CALIB_SAMPLE_COUNT];
static float accel_z_samples[CALIB_SAMPLE_COUNT];
static float gyro_x_samples[CALIB_SAMPLE_COUNT];
static float gyro_y_samples[CALIB_SAMPLE_COUNT];
static float gyro_z_samples[CALIB_SAMPLE_COUNT];


void Calculate_Statistics(float *data, uint32 count, CalibrationResult_t *result)
{
    float sum = 0.0f;
    float sum_sq = 0.0f;
    
    result->min = data[0];
    result->max = data[0];
    
    // 1단계: 평균, 최소/최대값 계산
    for(uint32 i = 0; i < count; i++) {
        sum += data[i];
        
        if(data[i] < result->min) result->min = data[i];
        if(data[i] > result->max) result->max = data[i];
    }
    
    result->mean = sum / count;
    
    // 2단계: 분산 계산
    for(uint32 i = 0; i < count; i++) {
        float diff = data[i] - result->mean;
        sum_sq += diff * diff;
    }
    
    result->variance = sum_sq / count;
    result->std_dev = sqrtf(result->variance);
}

/**
 * @brief IMU 캘리브레이션 태스크
 * 
 * 100Hz로 1000개 샘플 수집 (10초)
 * 센서 노이즈 통계 분석
 * 칼만 필터 파라미터 자동 계산
 */
void IMU_Calibration_Task(void *pArg)
{
    (void)pArg;

    const float dt = CALIB_SAMPLE_RATE_MS / 1000.0f;  // 0.01s
    const float G  = 9.80665f;  // 중력 가속도 (m/s²)

    SALRetCode_t ret;
    uint32 sample_index = 0;
    IMU_CalibrationData_t calib_data;
    
    // 타이밍 제어 변수
    uint32 start_tick;
    uint32 elapsed_tick;
    uint32 sleep_time;
    uint32 timing_violations = 0;
    uint32 max_elapsed = 0;

    /* ================================
     * 시작 안내
     * ================================ */
    mcu_printf("\n========================================\n");
    mcu_printf("   IMU CALIBRATION - PRECISE MODE\n");
    mcu_printf("========================================\n");
    mcu_printf("Sample Count: %d\n", CALIB_SAMPLE_COUNT);
    mcu_printf("Sample Rate : %d ms (100Hz)\n", CALIB_SAMPLE_RATE_MS);
    mcu_printf("Duration    : %d seconds\n\n",
               (CALIB_SAMPLE_COUNT * CALIB_SAMPLE_RATE_MS) / 1000);

    mcu_printf("!!! IMPORTANT !!!\n");
    mcu_printf("Keep IMU STABLE and LEVEL\n");
    mcu_printf("Do NOT move or vibrate\n");
    mcu_printf("Starting in 3 seconds...\n\n");
    
    SAL_TaskSleep(3000);

    mcu_printf("[CALIBRATION] Starting data collection...\n");

    /* ================================
     * 1. DATA COLLECTION (100Hz 정확도 보장)
     * ================================ */
    while (sample_index < CALIB_SAMPLE_COUNT) {
        SAL_GetTickCount(&start_tick);
        
        ret = IMU_Read_Data_DMA();

        if (ret == SAL_RET_SUCCESS) {
            // 샘플 저장
            accel_x_samples[sample_index] = IMU.accel_x;
            accel_y_samples[sample_index] = IMU.accel_y;
            accel_z_samples[sample_index] = IMU.accel_z;
            gyro_x_samples[sample_index]  = IMU.gyro_x;
            gyro_y_samples[sample_index]  = IMU.gyro_y;
            gyro_z_samples[sample_index]  = IMU.gyro_z;

            sample_index++;

            // 진행률 출력 (10%마다)
            if (sample_index % (CALIB_SAMPLE_COUNT / 10) == 0) {
                mcu_printf("[CALIBRATION] Progress: %d%% | Violations: %d | Max: %dms\n",
                           (sample_index * 100) / CALIB_SAMPLE_COUNT,
                           timing_violations,
                           max_elapsed);
            }
        } else {
            mcu_printf("[CALIBRATION] Read error at sample %d, retrying...\n", sample_index);
        }
        
        // 타이밍 측정 및 100Hz 유지
        SAL_GetTickCount(&elapsed_tick);
        elapsed_tick = elapsed_tick - start_tick;
        
        if(elapsed_tick > max_elapsed) {
            max_elapsed = elapsed_tick;
        }
        
        if (elapsed_tick < CALIB_SAMPLE_RATE_MS) {
            sleep_time = CALIB_SAMPLE_RATE_MS - elapsed_tick;
        } else {
            sleep_time = 0;
            timing_violations++;
            mcu_printf("[WARNING] Cycle exceeded %dms: %dms\n", 
                      CALIB_SAMPLE_RATE_MS, elapsed_tick);
        }
        
        SAL_TaskSleep(sleep_time);
    }

    mcu_printf("\n[CALIBRATION] Data collection complete!\n");
    mcu_printf("[CALIBRATION] Timing violations: %d / %d (", 
               timing_violations, CALIB_SAMPLE_COUNT);
    Print_Float_Value((timing_violations * 100.0f) / CALIB_SAMPLE_COUNT, 10);
    mcu_printf("%%)\n");
    mcu_printf("[CALIBRATION] Maximum cycle time: %dms\n\n", max_elapsed);

    /* ================================
     * 2. STATISTICS CALCULATION
     * ================================ */
    mcu_printf("[CALIBRATION] Calculating statistics...\n");
    
    Calculate_Statistics(accel_x_samples, CALIB_SAMPLE_COUNT, &calib_data.accel_x);
    Calculate_Statistics(accel_y_samples, CALIB_SAMPLE_COUNT, &calib_data.accel_y);
    Calculate_Statistics(accel_z_samples, CALIB_SAMPLE_COUNT, &calib_data.accel_z);
    Calculate_Statistics(gyro_x_samples,  CALIB_SAMPLE_COUNT, &calib_data.gyro_x);
    Calculate_Statistics(gyro_y_samples,  CALIB_SAMPLE_COUNT, &calib_data.gyro_y);
    Calculate_Statistics(gyro_z_samples,  CALIB_SAMPLE_COUNT, &calib_data.gyro_z);

    /* ================================
     * 3. RAW DATA SUMMARY
     * ================================ */
    mcu_printf("\n========================================\n");
    mcu_printf("    RAW SENSOR STATISTICS\n");
    mcu_printf("========================================\n\n");
    
    mcu_printf("--- ACCELEROMETER ---\n");
    mcu_printf("X: Mean=");
    Print_Float_Value(calib_data.accel_x.mean, 1000000);
    mcu_printf(" Var=");
    Print_Float_Value(calib_data.accel_x.variance, 1000000);
    mcu_printf("\n");
    
    mcu_printf("Y: Mean=");
    Print_Float_Value(calib_data.accel_y.mean, 1000000);
    mcu_printf(" Var=");
    Print_Float_Value(calib_data.accel_y.variance, 1000000);
    mcu_printf("\n");
    
    mcu_printf("Z: Mean=");
    Print_Float_Value(calib_data.accel_z.mean, 1000000);
    mcu_printf(" Var=");
    Print_Float_Value(calib_data.accel_z.variance, 1000000);
    mcu_printf("\n\n");
    
    mcu_printf("--- GYROSCOPE (deg/s) ---\n");
    mcu_printf("X: Mean=");
    Print_Float_Value(calib_data.gyro_x.mean, 1000000);
    mcu_printf(" Var=");
    Print_Float_Value(calib_data.gyro_x.variance, 1000000);
    mcu_printf("\n");
    
    mcu_printf("Y: Mean=");
    Print_Float_Value(calib_data.gyro_y.mean, 1000000);
    mcu_printf(" Var=");
    Print_Float_Value(calib_data.gyro_y.variance, 1000000);
    mcu_printf("\n");
    
    mcu_printf("Z: Mean=");
    Print_Float_Value(calib_data.gyro_z.mean, 1000000);
    mcu_printf(" Var=");
    Print_Float_Value(calib_data.gyro_z.variance, 1000000);
    mcu_printf("\n\n");

    /* ================================
     * 4. UNIT VERIFICATION
     * ================================ */
    mcu_printf("--- UNIT VERIFICATION ---\n");
    
    float accel_z_mean = calib_data.accel_z.mean;
    if(accel_z_mean > 5.0f) {
        mcu_printf("Accelerometer unit: m/s^2 (Z=");
        Print_Float_Value(accel_z_mean, 1000);
        mcu_printf(" m/s^2)\n");
    } else if(accel_z_mean > 0.8f && accel_z_mean < 1.2f) {
        mcu_printf("Accelerometer unit: g (Z=");
        Print_Float_Value(accel_z_mean, 1000);
        mcu_printf(" g)\n");
    } else {
        mcu_printf("WARNING: Unexpected Z value: ");
        Print_Float_Value(accel_z_mean, 1000);
        mcu_printf("\n");
    }
    mcu_printf("\n");

    /* ================================
     * 5. KALMAN FILTER PARAMETERS
     * ================================ */
    
    // 가속도 분산 (g² 또는 (m/s²)²)
    float ax_var = calib_data.accel_x.variance;
    float ay_var = calib_data.accel_y.variance;
    float az_var = calib_data.accel_z.variance;
    
    // 자이로 분산 (deg/s)²
    float gx_var = calib_data.gyro_x.variance;
    float gy_var = calib_data.gyro_y.variance;
    
    /* R (Measurement Noise)
     * 가속도로 각도 계산 시 노이즈:
     * roll ≈ atan2(ay, az), pitch ≈ atan2(-ax, √(ay²+az²))
     * 
     * 수평 근사: roll ≈ ay/g, pitch ≈ -ax/g
     * var(roll) ≈ var(ay)/g² + var(az)/g²
     */
    float R_roll  = (ay_var + az_var) / (G * G);
    float R_pitch = (ax_var + az_var) / (G * G);
    
    mcu_printf("--- KALMAN PARAMETER CALCULATION ---\n");
    mcu_printf("Accel variances (g^2):\n");
    mcu_printf("  X=");
    Print_Float_Value(ax_var, 1000000);
    mcu_printf(" Y=");
    Print_Float_Value(ay_var, 1000000);
    mcu_printf(" Z=");
    Print_Float_Value(az_var, 1000000);
    mcu_printf("\n\n");
    
    mcu_printf("Gyro variances (deg/s)^2:\n");
    mcu_printf("  X=");
    Print_Float_Value(gx_var, 1000000);
    mcu_printf(" Y=");
    Print_Float_Value(gy_var, 1000000);
    mcu_printf("\n\n");
    
    /* Q_angle (Process Noise - Angle)
     * 자이로 적분으로 인한 각도 불확실성
     * Q_angle = gyro_noise_var × dt²
     */
    const float DEG_TO_RAD = M_PI / 180.0f;
    float gyro_noise_var_rad = (gx_var + gy_var) * 0.5f * DEG_TO_RAD * DEG_TO_RAD;
    
    mcu_printf("Gyro noise variance (rad/s)^2 = ");
    Print_Float_Value(gyro_noise_var_rad, 1000000);
    mcu_printf("\n\n");
    
    float Q_angle = gyro_noise_var_rad * dt * dt;
    
    /* Q_bias (Process Noise - Gyro Bias)
     * 자이로 바이어스 변화율 (보수적 추정)
     * Q_bias = Q_angle × 0.01
     */
    float Q_bias = Q_angle * 0.01f;
    
    /* ================================
     * 6. RESULTS OUTPUT
     * ================================ */
    mcu_printf("\n========================================\n");
    mcu_printf(" KALMAN FILTER PARAMETERS\n");
    mcu_printf("========================================\n\n");

    mcu_printf("R (Measurement Noise, rad^2):\n");
    mcu_printf("  R_roll  = ");
    Print_Float_Value(R_roll, 1000000000);
    mcu_printf("\n");
    
    mcu_printf("  R_pitch = ");
    Print_Float_Value(R_pitch, 1000000000);
    mcu_printf("\n");
    
    mcu_printf("  R_avg   = ");
    Print_Float_Value((R_roll + R_pitch) * 0.5f, 1000000000);
    mcu_printf("\n\n");

    mcu_printf("Q (Process Noise):\n");
    mcu_printf("  Q_angle = ");
    Print_Float_Value(Q_angle, 1000000000);
    mcu_printf(" rad^2\n");
    
    mcu_printf("  Q_bias  = ");
    Print_Float_Value(Q_bias, 1000000000);
    mcu_printf(" rad^2/s^2\n\n");

    /* ================================
     * 7. COPY-PASTE CODE
     * ================================ */
    mcu_printf("========================================\n");
    mcu_printf("RECOMMENDED VALUES (Copy & Paste):\n");
    mcu_printf("========================================\n\n");
    
    // 정지 측정값
    mcu_printf("// Measured (stationary):\n");
    mcu_printf("#define KALMAN_Q_ANGLE   ");
    Print_Float_Value(Q_angle, 1000000000);
    mcu_printf("f\n");
    
    mcu_printf("#define KALMAN_Q_BIAS    ");
    Print_Float_Value(Q_bias, 1000000000);
    mcu_printf("f\n");
    
    mcu_printf("#define KALMAN_R_MEASURE ");
    Print_Float_Value((R_roll + R_pitch) * 0.5f, 1000000000);
    mcu_printf("f\n\n");
    
    // 차량용 권장값 (5배 마진)
    mcu_printf("// Recommended (vehicle, 5x margin):\n");
    mcu_printf("#define KALMAN_Q_ANGLE   ");
    Print_Float_Value(Q_angle * 5.0f, 1000000000);
    mcu_printf("f\n");
    
    mcu_printf("#define KALMAN_Q_BIAS    ");
    Print_Float_Value(Q_bias * 5.0f, 1000000000);
    mcu_printf("f\n");
    
    mcu_printf("#define KALMAN_R_MEASURE ");
    Print_Float_Value((R_roll + R_pitch) * 0.5f * 5.0f, 1000000000);
    mcu_printf("f\n");
    
    mcu_printf("========================================\n\n");
    
    mcu_printf("[CALIBRATION] Completed successfully!\n");
    mcu_printf("[CALIBRATION] Task will now terminate.\n");
    
    SAL_TaskDelete(0);
}