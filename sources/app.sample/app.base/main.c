// SPDX-License-Identifier: Apache-2.0


/*
***************************************************************************************************
*
*   FileName : main.c
*
*   Copyright (c) Telechips Inc.
*
*   Description :
*
*
***************************************************************************************************
*/


#if ( MCU_BSP_SUPPORT_APP_BASE == 1 )

#include <main.h>
#include <stdio.h>
#include <string.h>
#include <sal_api.h>
#include <app_cfg.h>
#include <debug.h>
#include <bsp.h>
#include <math.h>
#include <pdm.h>
#include <gpio.h>
#include <servo_control.h>
#include <ICM_20948.h>
#include <kalman_filter.h>
#include <printf_float.h>
#include <imu_calibration.h>
#include <ADXL345_test.h>
#include <ADXL345.h>
#include <i2c.h>


#if (APLT_LINUX_SUPPORT_SPI_DEMO == 1)
    #include <spi_eccp.h>
#endif
#if (APLT_LINUX_SUPPORT_POWER_CTRL == 1)
    #include <power_app.h>
#endif
#if ( MCU_BSP_SUPPORT_APP_KEY == 1)
    #include <key.h>
#endif  // ( MCU_BSP_SUPPORT_APP_KEY == 1 )

#if ( MCU_BSP_SUPPORT_APP_CONSOLE == 1 )
    #include <console.h>
#endif  // ( MCU_BSP_SUPPORT_APP_CONSOLE == 1 )

#if ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )
    #include <can_demo.h>
#endif  // ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )

#if ( MCU_BSP_SUPPORT_APP_IDLE == 1 )
    #include <idle.h>
#endif  // ( MCU_BSP_SUPPORT_APP_IDLE == 1 )

#if ( MCU_BSP_SUPPORT_APP_SPI_LED == 1 )
    #include <spi_led.h>
#endif  // ( MCU_BSP_SUPPORT_APP_SPI_LED == 1 )

#if ( MCU_BSP_SUPPORT_APP_FW_UPDATE == 1 )
    #include "fwupdate.h"
#elif ( MCU_BSP_SUPPORT_APP_FW_UPDATE_ECCP == 1 )
    #include "fwupdate.h"
#endif
/*
***************************************************************************************************
*                                         TASK CONFIGURATION
***************************************************************************************************
*/

#define SAFETY_TASK_PRIO        (SAL_PRIO_APP_CFG + 0)      // 최고 우선순위
#define CAN_RX_TASK_PRIO        (SAL_PRIO_APP_CFG + 1)      
#define MOTOR_TASK_PRIO         (SAL_PRIO_APP_CFG + 2)
#define ADXL_MONITOR_TASK_PRIO  (SAL_PRIO_APP_CFG + 3)      // ✅ 추가
#define IMU_SUSP_TASK_PRIO      (SAL_PRIO_APP_CFG + 4)      
#define HEIGHT_TASK_PRIO        (SAL_PRIO_APP_CFG + 5)      
#define MONITOR_TASK_PRIO       (SAL_PRIO_APP_CFG + 6)     


// Task Stack Sizes - 극한 최소화
#define SAFETY_TASK_STK_SIZE    (384)   // 1.5KB
#define CAN_RX_TASK_STK_SIZE    (256)   // 1KB
#define MOTOR_TASK_STK_SIZE     (256)   // 1KB
#define IMU_SUSP_TASK_STK_SIZE  (256)   // 2KB (가장 중요) 512로 돌리기@@@@@@@@@@@@@@@
#define HEIGHT_TASK_STK_SIZE    (256)   // 1KB
#define MONITOR_TASK_STK_SIZE   (128)   // 1KB    
#define ADXL_TEST_TASK_STK_SIZE (256)   // 1KB  ✅ 추가
 

// Control Frequencies
#define SAFETY_PERIOD_MS        (10)    // 100Hz 
#define MOTOR_PERIOD_MS         (10)    // 100Hz
#define IMU_SUSP_PERIOD_MS      (10)    // 100Hz
#define HEIGHT_PERIOD_MS        (50)    // 20Hz
#define MONITOR_PERIOD_MS       (100)   // 10Hz

// 측정 설정
#define CALIBRATION_MODE    0  // 0: 정상, 1: 캘리브레이션
#define CALIB_SAMPLE_COUNT      1000    // 측정 샘플 수
#define CALIB_SAMPLE_RATE_MS    10      // 10ms = 100Hz


/*
***************************************************************************************************
*                                         SAFETY CONFIGURATION
***************************************************************************************************
*/

// Safety Thresholds
#define ROLLOVER_GYRO_THRESHOLD     (250.0f)    // °/s
#define ROLLOVER_ANGLE_THRESHOLD    (70.0f)     // °
#define ROLLOVER_RATE_THRESHOLD     (100.0f)    // °/s
#define COLLISION_ACCEL_THRESHOLD   (4.0f)      // G
#define COLLISION_DURATION_MAX      (100)       // ms
#define OVERSPEED_THRESHOLD         (95.0f)     // %
#define IMU_TIMEOUT_MS              (100)       // ms
#define CAN_TIMEOUT_MS              (500)       // ms
#define SLOPE_ANGLE_THRESHOLD       (25.0f)     // °
#define SLOPE_VARIANCE_THRESHOLD    (5.0f)      // °

// Safety Levels
typedef enum {
    SAFETY_NORMAL = 0,
    SAFETY_WARNING,
    SAFETY_EMERGENCY,
    SAFETY_CRITICAL
} SafetyLevel_t;
/*
***************************************************************************************************
*                                         PID CONFIGURATION (정리됨)
***************************************************************************************************
*/

/* ===== 차량 기하학 파라미터 ===== */
#define WHEELBASE_MM            (305.0f)    // 앞뒤 바퀴 간격
#define TRACK_WIDTH_MM          (233.0f)    // 좌우 바퀴 간격

/* ===== 레벨링 PID 게인 (STM32 방식) ===== */
#define LEVELING_GAIN           2.2f
#define INTEGRAL_GAIN           0.30f
#define INTEGRAL_MAX            35.0f
#define DERIVATIVE_GAIN         0.30f
#define DERIVATIVE_FILTER       0.7f
#define DEADBAND                0.5f

/* ===== 댐핑 PID 게인 (속도별) ===== */
#define DAMPING_KP_SOFT         0.0f
#define DAMPING_KD_SOFT         0.0f
#define DAMPING_KP_MEDIUM       0.0f
#define DAMPING_KD_MEDIUM       0.0f
#define DAMPING_KP_HARD         0.0f
#define DAMPING_KD_HARD         0.0f

/* ===== 속도 임계값 ===== */
#define SPEED_THRESHOLD_LOW     30.0f
#define SPEED_THRESHOLD_HIGH    70.0f

/* ===== ADXL Pre-kick 파라미터 ===== */
#define IMPACT_HP_THRESHOLD     (10.0f * 9.81f)  // 0.6G
#define PRE_KICK_MAGNITUDE      0.0f            // 3도
#define PRE_KICK_DECAY          0.85f           // 85% 유지

/* ===== 제어 한계값 ===== */
#define MAX_CORRECTION_DEG      30.0f           // PID 출력 최대 각도

/* ===== Servo follow speed (deg/sec) =====
 * 100Hz(10ms)에서 max_step = rate * 0.01
 * 예) 700deg/s -> 1주기 7deg 이동
 */
#define SERVO_RATE_FLAT_SLOW      300.0f   // 작은 기울기에서의 추종 속도
#define SERVO_RATE_FLAT_FAST      300.0f   // 큰 기울기/급변에서의 추종 속도
#define SERVO_RATE_SLOPE          300.0f   // 언덕에서(출렁 방지) 추종 속도

/* 큰 기울기 기준(이 이상이면 FAST) */
#define TILT_FAST_THRESHOLD_DEG   5.0f
/* ===== LPF 파라미터 ===== */
#define MAX_TILT_ANGLE          ROLLOVER_ANGLE_THRESHOLD

/* ===== Small-tilt 안정화(핵심) ===== */
#define TILT_SOFT_START_DEG      0.5f    // 이 아래는 거의 안 움직이게
#define TILT_SOFT_FULL_DEG       4.0f    // 여기부터 정상 gain(1.0)
#define LEVELING_DEADBAND_SOFT   2.3f    // 작은 기울기에서 더 큰 데드밴드

#define LEVELING_SIGN_ROLL   (1.0f)
#define LEVELING_SIGN_PITCH  (1.0f)

// 히스테리시스 deadzone
#define TGT_DZ_ENTER 0.03f
#define TGT_DZ_EXIT  0.06f

#define LPF_CUTOFF_FREQ         1.5f

// ✅ IMU accel 신뢰도(1g 근처 여부) 기반 칼만 가중치 튜닝
#define ACC_MAG_NORM            (9.81f)
#define ACC_OK_BAND             (1.3f)   // |mag-9.81| < 1.3면 "신뢰"
#define ACC_SOFT_BAND           (3.0f)   // |mag-9.81| < 3.0면 "부분 신뢰"

// ✅ 칼만 R_measure 범위 (작을수록 accel를 더 믿음)
#define KALMAN_R_MIN            (0.10f)  // 기존 값
#define KALMAN_R_MAX            (5.00f)  // 흔들릴 때 accel 거의 무시

/*
***************************************************************************************************
*                                         GLOBAL VARIABLES
***************************************************************************************************
*/
uint32                                  gALiveMsgOnOff;
static uint32                           gALiveCount;


// Task IDs
static uint32 gSafetyTaskID = 0;
static uint32 gCANRxTaskID = 0;
static uint32 gMotorTaskID = 0;
static uint32 gIMUSuspTaskID = 0;
static uint32 gHeightTaskID = 0;
static uint32 gMonitorTaskID = 0;
static uint32 gADXL345TestTaskID = 0;

// Task Stacks
static uint32 gSafetyTaskStk[SAFETY_TASK_STK_SIZE];
static uint32 gCANRxTaskStk[CAN_RX_TASK_STK_SIZE];
static uint32 gMotorTaskStk[MOTOR_TASK_STK_SIZE];
static uint32 gIMUSuspTaskStk[IMU_SUSP_TASK_STK_SIZE];
static uint32 gHeightTaskStk[HEIGHT_TASK_STK_SIZE];
static uint32 gMonitorTaskStk[MONITOR_TASK_STK_SIZE];
static uint32 gADXL345TestTaskStk[ADXL_TEST_TASK_STK_SIZE];

// Shared Data Structures
typedef struct {
    float roll;
    float pitch;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;
    uint32 last_update_time;
} IMU_Data_t;

typedef struct {
    float motor_speed_cmd;
    uint8 suspension_enable;
    uint8 leveling_enable;
    uint32 last_rx_time;
} CAN_Data_t;

typedef struct {
    float current_speed;
    float current_current;
} Motor_Data_t;

typedef struct {
    float position[4];  // FL, FR, RL, RR
} Servo_Data_t;

typedef struct {
    SafetyLevel_t level;
    uint8 suspension_disabled;
    float max_motor_speed;
    float suspension_range_limit;
    uint8 on_slope_detected;
} Safety_Data_t;

typedef struct {
    float height_offset;
} Height_Data_t;

typedef struct {
    float accel_z[4];           // 각 바퀴의 Z축 가속도
    float impact_detected[4];   // 충격 감지 (0~1)
    uint32 last_update_time;
} ADXL_Data_t;

// Global shared variables
static IMU_Data_t g_IMUData;
static CAN_Data_t g_CANData;
static Motor_Data_t g_MotorData;
static Servo_Data_t g_ServoData;
static Safety_Data_t g_SafetyData;
static Height_Data_t g_HeightData;

// Kalman Filters
static Kalman_t kalman_roll;
static Kalman_t kalman_pitch;

static float roll_offset = 0.0f;
static float pitch_offset = 0.0f;

/* ===== PID 컨트롤러 ===== */
typedef struct {
    float error;
    float prev_error;
    float integral;
    float derivative;
    float filtered_derivative;
    float output;
} PID_Leveling_t;

static PID_Leveling_t pid_roll  = {0};
static PID_Leveling_t pid_pitch = {0};

/* ===== 댐핑용 PID (기존 구조체 유지) ===== */
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float dt;
    float integral_max;
} PID_Damping_t;

static volatile uint8 g_IMU_SUSP_INIT_DONE = 0;

/* ✅ 시스템 준비 플래그: 0=준비중(캘리브/안정화), 1=모니터링/제어 시작 */
static volatile uint8 g_SYSTEM_READY = 0;

/* ✅ ADXL bias (캘리브레이션으로 구한 평균값) */
static float g_ADXL_bias_x[4] = {0};
static float g_ADXL_bias_y[4] = {0};
static float g_ADXL_bias_z[4] = {0};

/* ✅ ADXL 캘리브 완료 플래그 */
static volatile uint8 g_ADXL_CAL_DONE = 0;

static ADXL_Data_t g_ADXLData;
static PID_Damping_t pid_damping_wheel[4];  // 각 바퀴별 댐핑 PID

typedef struct {
    float leveling[4];      // FL FR RL RR
    float pid_roll_out_mm;  // pid_roll.output (mm)
    float pid_pitch_out_mm; // pid_pitch.output (mm)
    float soft_w;           // 0~1
    float rate_deg_s;       // deg/sec
    float max_step;         // deg/cycle

    // ✅ debug 추가
    float roll_acc_deg;
    float pitch_acc_deg;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    float pid_roll_integral;
    float pid_pitch_integral;

} SuspDbg_t;

static volatile SuspDbg_t g_SuspDbg;

static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;


/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/

static void Main_StartTask(void *pArg);
static void Safety_Monitor_Task(void *pArg);
static void CAN_RX_Task(void *pArg);
static void Motor_Control_Task(void *pArg);
static void IMU_Suspension_Task(void *pArg);
static void Height_Control_Task(void *pArg);
static void Monitoring_Task(void *pArg);
static void ADXL345_Monitor_Task(void *pArg);

// 댐핑 PID 함수들
static void PID_Damping_Init(PID_Damping_t *pid, float Kp, float Ki, float Kd, float dt_ms);
static float PID_Damping_Update(PID_Damping_t *pid, float setpoint, float measurement);
static void PID_Damping_Reset(PID_Damping_t *pid);

static void Motor_SetSpeed(float speed);
static void AppTaskCreate(void);
static void DisplayAliveLog(void);
static void DisplayOTPInfo(void);


/*
***************************************************************************************************
*                                         PID FUNCTIONS
***************************************************************************************************
*/

/* ===== 댐핑 PID 함수들 ===== */
void PID_Damping_Init(PID_Damping_t *pid, float Kp, float Ki, float Kd, float dt_ms) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = dt_ms / 1000.0f;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->integral_max = 50.0f;
}

float PID_Damping_Update(PID_Damping_t *pid, float setpoint, float measurement) {
    float error = setpoint - measurement;
    
    // Proportional
    float P = pid->Kp * error;
    
    // Integral (Anti-windup)
    pid->integral += error * pid->dt;
    if(pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if(pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    float I = pid->Ki * pid->integral;
    
    // Derivative
    float derivative = (error - pid->prev_error) / pid->dt;
    float D = pid->Kd * derivative;
    
    pid->prev_error = error;
    
    return P + I + D;
}

void PID_Damping_Reset(PID_Damping_t *pid) {
    pid->integral = 0;
    pid->prev_error = 0;
}

void Motor_SetSpeed(float speed) {
    PDMModeConfig_t pwm_cfg;
    uint32 duty_ns;
    
    // Speed to duty cycle (0-100% → 0-100% duty)
    duty_ns = (uint32)(speed * 200000.0f);  // 20kHz PWM (50us period)
    
    if(duty_ns > 20000000) duty_ns = 20000000;
    
    pwm_cfg.mcPortNumber = GPIO_PERICH_CH2;  // Motor channel (GPIO-C)
    pwm_cfg.mcOperationMode = PDM_OUTPUT_MODE_PHASE_1;
    pwm_cfg.mcInversedSignal = 0;
    pwm_cfg.mcOutSignalInIdle = 0;
    pwm_cfg.mcLoopCount = 0;
    pwm_cfg.mcOutputCtrl = 0x05;  // DO_Sel=1, OEN_Sel=1
    pwm_cfg.mcPeriodNanoSec1 = 50000;     // 50us (20kHz)
    pwm_cfg.mcDutyNanoSec1 = duty_ns;
    pwm_cfg.mcDutyNanoSec2 = 0;
    pwm_cfg.mcPeriodNanoSec2 = 0;
    
    PDM_Disable(4, PMM_OFF);
    SAL_TaskSleep(1);
    PDM_SetConfig(4, &pwm_cfg);
    PDM_Enable(4, PMM_OFF);
}

/* ===== 유틸리티 함수 ===== */
static inline float clamp(float v, float lo, float hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* ===== 기하학 변환: 각도 → 높이 변화 (mm) ===== */
static float angle_to_height_mm(float angle_deg, float distance_mm) {
    float rad = angle_deg * M_PI / 180.0f;
    return distance_mm * sinf(rad);
}

/* ===== 높이(mm) → 서보 각도(deg) 변환 ===== */
static float height_to_servo_deg(float height_mm, float max_angle_deg) {
    // 가정: ±70도 기울기 = ±MAX_CORRECTION_DEG 서보 각도
    float max_height_mm = angle_to_height_mm(70.0f, TRACK_WIDTH_MM * 0.5f);
    if (fabsf(max_height_mm) < 0.001f) return 0.0f;
    
    float scale = max_angle_deg / max_height_mm;
    return height_mm * scale;
}
static void compute_leveling_pid(PID_Leveling_t *pid, float error, float dt, uint8 on_slope)
{
    pid->error = error;

    if (!on_slope) {
    if (fabsf(error) < 2.0f) {        /* 소각도/정지 근처 */
        pid->integral *= 0.70f;       /* 기존 0.90보다 강하게 누설 */
        if (fabsf(pid->integral) < 0.05f) {
            pid->integral = 0.0f;
        }
    }
    }

    // 충격 감지
    float error_change = fabsf(error - pid->prev_error);
    if (error_change > 5.0f) {
        pid->integral *= 0.2f;
    }

    float db = DEADBAND;
    float i_enable = 1.0f;

    if (on_slope) {
        db = 0.5f;
        i_enable = 1.0f;
    } else {
        if (fabsf(error) < 3.0f) {
            db = LEVELING_DEADBAND_SOFT;
            i_enable = 0.0f;  
        }
    }

    /* Anti-windup */
    if (fabsf(error) > db && i_enable > 0.5f) {
        float error_reduction = pid->prev_error - error;
        
        if (error_reduction > -0.5f) {
            pid->integral += error * dt;
            pid->integral = clamp(pid->integral, -INTEGRAL_MAX, INTEGRAL_MAX);
        } else if (fabsf(pid->integral) > INTEGRAL_MAX * 0.8f) {
            pid->integral *= 0.98f;
        } else {
            pid->integral += error * dt;
            pid->integral = clamp(pid->integral, -INTEGRAL_MAX, INTEGRAL_MAX);
        }
    } else {
        pid->integral *= 0.90f;
    }

    /* ✅ 수정: 오버슈트 감지 강화 - 에러가 급격히 줄어들 때도 감지 */
    // 방법 1: 부호 반전 (기존)
    if ((pid->prev_error > 1.0f && error < -1.0f) ||
        (pid->prev_error < -1.0f && error > 1.0f)) {
        pid->integral *= 0.3f;
    }
    
    // ✅ 방법 2: 급격한 개선 감지 (새로 추가!)
    // 에러가 1초에 10도 이상 줄어들면 과보정
    float error_rate = (pid->prev_error - error) / dt;  // deg/s
    if (fabsf(error_rate) > 100.0f && fabsf(pid->integral) > 2.0f) {
        // 에러가 너무 빨리 줄어듦 = 과보정 중
        pid->integral *= 0.5f;  // 적분 절반으로
    }

    // Derivative with LPF
    float raw_derivative = (error - pid->prev_error) / dt;
    pid->filtered_derivative = (DERIVATIVE_FILTER * pid->filtered_derivative) +
                               ((1.0f - DERIVATIVE_FILTER) * raw_derivative);
    pid->derivative = pid->filtered_derivative;
    pid->prev_error = error;

    // PID 출력
    pid->output = error * LEVELING_GAIN +
                  pid->integral * INTEGRAL_GAIN +
                  pid->derivative * DERIVATIVE_GAIN;
}

static void reset_leveling_pid(PID_Leveling_t *pid) {
    pid->error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->filtered_derivative = 0.0f;
    pid->output = 0.0f;
}

/*
***************************************************************************************************
*                                          cmain
*
* This is the standard entry point for C code.
*
* Notes
*   It is assumed that your code will call main() once you have performed all necessary
*   initialization.
*
***************************************************************************************************
*/
void cmain (void)
{
    static uint32           AppTaskStartID = 0;
    static uint32           AppTaskStartStk[ACFG_TASK_MEDIUM_STK_SIZE];
    SALRetCode_t            err;
    SALMcuVersionInfo_t     versionInfo = {0,0,0,0};

    (void)SAL_Init();

    BSP_PreInit(); /* Initialize basic BSP functions */

#if ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )
    (void)CAN_DemoInitialize();
#endif  // ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )

    BSP_Init(); /* Initialize BSP functions */

    (void)SAL_GetVersion(&versionInfo);
    mcu_printf("\n===============================\n");
    mcu_printf("    MCU BSP Version: V%d.%d.%d\n",
           versionInfo.viMajorVersion,
           versionInfo.viMinorVersion,
           versionInfo.viPatchVersion);
    mcu_printf("-------------------------------\n");
    DisplayOTPInfo();
    mcu_printf("===============================\n\n");

    // Initialize global data
    memset(&g_IMUData, 0, sizeof(g_IMUData));
    memset(&g_CANData, 0, sizeof(g_CANData));
    memset(&g_MotorData, 0, sizeof(g_MotorData));
    memset(&g_ServoData, 0, sizeof(g_ServoData));
    memset(&g_SafetyData, 0, sizeof(g_SafetyData));
    memset(&g_HeightData, 0, sizeof(g_HeightData));
    memset(&g_ADXLData, 0, sizeof(g_ADXLData));

    g_CANData.suspension_enable = 1;
    g_CANData.leveling_enable = 1;
    
    g_SafetyData.level = SAFETY_NORMAL;
    g_SafetyData.max_motor_speed = 100.0f;
    g_SafetyData.suspension_range_limit = 1.0f; 

    // ✅ 서보 초기값 설정
    for(uint8 i = 0; i < 4; i++){
        g_ServoData.position[i] = 90.0f;
    }
    
    // Initialize Kalman filters
    Kalman_Init(&kalman_roll);
    Kalman_Init(&kalman_pitch);
    
    // create the first app task...
    err = (SALRetCode_t)SAL_TaskCreate(&AppTaskStartID,
                         (const uint8 *)"App Task Start",
                         (SALTaskFunc) &Main_StartTask,
                         &AppTaskStartStk[0],
                         ACFG_TASK_MEDIUM_STK_SIZE,
                         SAL_PRIO_APP_CFG,
                         NULL);

    if (err == SAL_RET_SUCCESS)
    {
        // start woring os.... never return from this function
        (void)SAL_OsStart();
    }
}

/*
***************************************************************************************************
*                                          Main_StartTask
*
* This is an example of a startup task.
*
* Notes
*   As mentioned in the book's text, you MUST initialize the ticker only once multitasking has
*   started.
*
*   1) The first line of code is used to prevent a compiler warning because 'pArg' is not used.
*      The compiler should not generate any code for this statement.
*
***************************************************************************************************
*/


void Main_StartTask(void * pArg)
{
    (void)pArg;
    
    SALRetCode_t err;
    
    mcu_printf("\n[SYSTEM] Initializing...\n");

    //Initialize IMU
    if(ICM_20948_Init() == 0) {
        mcu_printf("[SYSTEM] IMU Initialized\n");
    } else { 
        mcu_printf("[ERROR] IMU Init Failed\n");
        }

    // ✅ I2C 초기화 (I2C2 CH_0)
    if(ADXL345_Test_Init() == SAL_RET_SUCCESS) {
        mcu_printf("[SYSTEM] ADXL345 I2C Initialized\n");
    }
    // Create application tasks
        AppTaskCreate();
        
        mcu_printf("[SYSTEM] Creating Control Tasks...\n\n");
    
    // // Task 0: Safety Monitor (100Hz)
    // err = SAL_TaskCreate(&gSafetyTaskID,
    //                     (const uint8 *)"Safety_Monitor",
    //                     (SALTaskFunc)&Safety_Monitor_Task,
    //                     &gSafetyTaskStk[0],
    //                     SAFETY_TASK_STK_SIZE,
    //                     SAFETY_TASK_PRIO,
    //                     NULL);
    // if(err == SAL_RET_SUCCESS) {
    //     mcu_printf("[SYSTEM] Safety Task Created (Priority: %d, 100Hz)\n", SAFETY_TASK_PRIO);
    // }
    
    // // Task 1: CAN RX (Event-driven) 
    // err = SAL_TaskCreate(&gCANRxTaskID,
    //                     (const uint8 *)"CAN_RX",
    //                     (SALTaskFunc)&CAN_RX_Task,
    //                     &gCANRxTaskStk[0],
    //                     CAN_RX_TASK_STK_SIZE,
    //                     CAN_RX_TASK_PRIO,
    //                     NULL);
    // if(err == SAL_RET_SUCCESS) {
    //     mcu_printf("[SYSTEM] CAN RX Task Created (Priority: %d, Event)\n", CAN_RX_TASK_PRIO);
    // }
    
    // // Task 2: Motor Control (100Hz)                                                                                         
    // err = SAL_TaskCreate(&gMotorTaskID,
    //                     (const uint8 *)"Motor_Control",
    //                     (SALTaskFunc)&Motor_Control_Task,
    //                     &gMotorTaskStk[0],
    //                     MOTOR_TASK_STK_SIZE,
    //                     MOTOR_TASK_PRIO,
    //                     NULL);
    // if(err == SAL_RET_SUCCESS) {
    //     mcu_printf("[SYSTEM] Motor Task Created (Priority: %d, 100Hz)\n", MOTOR_TASK_PRIO);
    // }
    
    // Task 3: IMU + Suspension (100Hz)
    err = SAL_TaskCreate(&gIMUSuspTaskID,
                        (const uint8 *)"IMU_Suspension",
                        (SALTaskFunc)&IMU_Suspension_Task,
                        &gIMUSuspTaskStk[0],
                        IMU_SUSP_TASK_STK_SIZE,
                        IMU_SUSP_TASK_PRIO,
                        NULL);
    if(err == SAL_RET_SUCCESS) {
        mcu_printf("[SYSTEM] IMU+Susp Task Created (Priority: %d, 100Hz)\n", IMU_SUSP_TASK_PRIO);
    }
    
    // // Task 4: Height Control (20Hz)
    // err = SAL_TaskCreate(&gHeightTaskID,
    //                     (const uint8 *)"Height_Control",
    //                     (SALTaskFunc)&Height_Control_Task,
    //                     &gHeightTaskStk[0],
    //                     HEIGHT_TASK_STK_SIZE,
    //                     HEIGHT_TASK_PRIO,
    //                     NULL);
    // if(err == SAL_RET_SUCCESS) {
    //     mcu_printf("[SYSTEM] Height Task Created (Priority: %d, 20Hz)\n", HEIGHT_TASK_PRIO);
    // }
    
    // Task 5: Monitoring (10Hz)
    err = SAL_TaskCreate(&gMonitorTaskID,
                        (const uint8 *)"Monitoring",
                        (SALTaskFunc)&Monitoring_Task,
                        &gMonitorTaskStk[0],
                        MONITOR_TASK_STK_SIZE,
                        MONITOR_TASK_PRIO,
                        NULL);
    if(err == SAL_RET_SUCCESS) {
        mcu_printf("[SYSTEM] Monitor Task Created (Priority: %d, 10Hz)\n\n", MONITOR_TASK_PRIO);
    }

     //Task 6:ADXL345_Monitor
    err = SAL_TaskCreate(&gADXL345TestTaskID,
                        (const uint8 *)"ADXL345_Monitor",
                        (SALTaskFunc)&ADXL345_Monitor_Task, // 수정한 테스트 함수
                        &gADXL345TestTaskStk[0],
                        ADXL_TEST_TASK_STK_SIZE,
                        ADXL_MONITOR_TASK_PRIO,
                        NULL);

    if(err == SAL_RET_SUCCESS) {
        mcu_printf("[SYSTEM] ADXL345 Test Task Created Successfully\n");
    } else {
        mcu_printf("[ERROR] Failed to Create ADXL345 Test Task\n");
    }

    mcu_printf("[SYSTEM] System Initialization Sequence Finished!\n"); 
    mcu_printf("=========================================\n\n");
    
    // Main task finished
    while(1) {
        SAL_TaskSleep(1000);
    }
    
}

/*
***************************************************************************************************
*                                          Safety_Monitor_Task
***************************************************************************************************
*/
void Safety_Monitor_Task(void *pArg) {
    (void)pArg;
    
    uint32 start_tick, current_tick;
    SafetyLevel_t safety_level = SAFETY_NORMAL;
    
    static float avg_roll = 0;
    static float avg_pitch = 0;
    static uint32 sample_count = 0;
    
    float prev_accel[3] = {0};
    uint32 impact_start_time = 0;
    
    mcu_printf("[SAFETY] Task Started (100Hz)\n");
    
    SAL_TaskSleep(50);
    
    while(1) {
        SAL_GetTickCount(&start_tick);
        
        // ========== 1. Read Sensor Data ==========
        SAL_CoreCriticalEnter();
        float roll = g_IMUData.roll;
        float pitch = g_IMUData.pitch;
        float gyro_x = g_IMUData.gyro_x;
        float gyro_y = g_IMUData.gyro_y;
        float accel_x = g_IMUData.accel_x;
        float accel_y = g_IMUData.accel_y;
        float accel_z = g_IMUData.accel_z;
        uint32 last_imu_time = g_IMUData.last_update_time;
        SAL_CoreCriticalExit();
        
        SAL_CoreCriticalEnter();
        float motor_speed = g_MotorData.current_speed;
        uint32 last_can_time = g_CANData.last_rx_time;
        SAL_CoreCriticalExit();
        
        SAL_GetTickCount(&current_tick);
        
        // ========== 2. Safety Checks ==========
        safety_level = SAFETY_NORMAL;
        
        // ✅ 디버깅 플래그
        uint8 rollover_detected = 0;
        uint8 collision_detected = 0;
        uint8 overspeed_detected = 0;
        uint8 imu_timeout_detected = 0;
        uint8 can_timeout_detected = 0;
        
        // 2-1. Rollover Detection
        uint8 fast_rotation = (fabsf(gyro_x) > ROLLOVER_GYRO_THRESHOLD || 
                               fabsf(gyro_y) > ROLLOVER_GYRO_THRESHOLD);
        uint8 sudden_tilt = (fast_rotation && 
                            (fabsf(roll) > ROLLOVER_ANGLE_THRESHOLD || 
                             fabsf(pitch) > ROLLOVER_ANGLE_THRESHOLD));

        if (sudden_tilt) {
            safety_level = SAFETY_CRITICAL;
            rollover_detected = 1;
        }

        // 2-2. Collision Detection
        float accel_delta = sqrtf(
            (accel_x - prev_accel[0]) * (accel_x - prev_accel[0]) +
            (accel_y - prev_accel[1]) * (accel_y - prev_accel[1]) +
            (accel_z - prev_accel[2]) * (accel_z - prev_accel[2])
        );
        
        if(accel_delta > COLLISION_ACCEL_THRESHOLD * 9.81f) {
            if(impact_start_time == 0) {
                impact_start_time = current_tick;
            }
            
            uint32 impact_duration = current_tick - impact_start_time;
            if(impact_duration < COLLISION_DURATION_MAX) {
                safety_level = SAFETY_EMERGENCY;
                collision_detected = 1;
            }
        } else {
            impact_start_time = 0;
        }
        
        prev_accel[0] = accel_x;
        prev_accel[1] = accel_y;
        prev_accel[2] = accel_z;
        
        // 2-3. Overspeed
        if(motor_speed > OVERSPEED_THRESHOLD) {
            if(safety_level < SAFETY_WARNING) {
                safety_level = SAFETY_WARNING;
                overspeed_detected = 1;
            }
        }
        
        // 2-4. IMU Timeout (✅ 시작 시 무시)
        static uint8 imu_first_check = 1;
        if(last_imu_time > 0) {  // ✅ IMU가 한 번이라도 업데이트되었으면
            imu_first_check = 0;
        }
        
        if(!imu_first_check && ((current_tick - last_imu_time) > IMU_TIMEOUT_MS)) {
            safety_level = SAFETY_EMERGENCY;
            imu_timeout_detected = 1;
        }
        
        // 2-5. CAN Timeout (✅ 시작 시 무시)
        static uint8 can_first_check = 1;
        if(last_can_time > 0) {  // ✅ CAN이 한 번이라도 수신되었으면
            can_first_check = 0;
        }
        
        if(!can_first_check && ((current_tick - last_can_time) > CAN_TIMEOUT_MS)) {
            if(safety_level < SAFETY_WARNING) {
                safety_level = SAFETY_WARNING;
                can_timeout_detected = 1;
            }
        }
        
        // ✅ 디버깅: Safety Level 변경 시 원인 출력
        static SafetyLevel_t prev_safety = SAFETY_NORMAL;
        if(safety_level != prev_safety) {
            mcu_printf("\n[SAFETY] Level changed: %d -> %d\n", prev_safety, safety_level);
            if(rollover_detected) mcu_printf("  - Rollover detected\n");
            if(collision_detected) mcu_printf("  - Collision detected\n");
            if(overspeed_detected) mcu_printf("  - Overspeed detected\n");
            if(imu_timeout_detected) {
                mcu_printf("  - IMU timeout (last: %lu, now: %lu, diff: %lu ms)\n", 
                          last_imu_time, current_tick, current_tick - last_imu_time);
            }
            if(can_timeout_detected) {
                mcu_printf("  - CAN timeout (last: %lu, now: %lu, diff: %lu ms)\n",
                          last_can_time, current_tick, current_tick - last_can_time);
            }
            prev_safety = safety_level;
        }
        
        // ========== 3. 경사 감지 ==========
        const float alpha = 0.1f;
        avg_roll = alpha * roll + (1.0f - alpha) * avg_roll;
        avg_pitch = alpha * pitch + (1.0f - alpha) * avg_pitch;
        sample_count++;
        
        uint8 on_slope = 0;
        if(sample_count > 200) {
            float roll_diff = fabs(roll - avg_roll);
            float pitch_diff = fabs(pitch - avg_pitch);
            
            on_slope = ((fabs(avg_roll) > SLOPE_ANGLE_THRESHOLD && roll_diff < 5.0f) ||
                        (fabs(avg_pitch) > SLOPE_ANGLE_THRESHOLD && pitch_diff < 5.0f));
        }
        
        // ========== 4. Apply Safety Actions ==========
        SAL_CoreCriticalEnter();
        g_SafetyData.level = safety_level;
        g_SafetyData.on_slope_detected = on_slope;
        SAL_CoreCriticalExit();
        
        switch(safety_level) {
            case SAFETY_CRITICAL:
                Motor_SetSpeed(0);
                SAL_CoreCriticalEnter();
                g_SafetyData.suspension_disabled = 1;
                SAL_CoreCriticalExit();
                break;
                
            case SAFETY_EMERGENCY:
                SAL_CoreCriticalEnter();
                g_SafetyData.suspension_disabled = 1;
                g_SafetyData.max_motor_speed = 0;
                SAL_CoreCriticalExit();
                break;
                
            case SAFETY_WARNING:
                SAL_CoreCriticalEnter();
                if(motor_speed > OVERSPEED_THRESHOLD) {
                    g_SafetyData.max_motor_speed = 70.0f;
                }
                if(!can_first_check && ((current_tick - last_can_time) > CAN_TIMEOUT_MS)) {
                    g_SafetyData.max_motor_speed = 50.0f;
                }
                g_SafetyData.suspension_range_limit = 0.5f;
                SAL_CoreCriticalExit();
                break;
                
            case SAFETY_NORMAL:
                SAL_CoreCriticalEnter();
                g_SafetyData.suspension_disabled = 0;
                g_SafetyData.max_motor_speed = 100.0f;
                g_SafetyData.suspension_range_limit = 1.0f;
                SAL_CoreCriticalExit();
                break;
        }
        
        // Sleep
        uint32 elapsed = current_tick - start_tick;
        if(elapsed < SAFETY_PERIOD_MS) {
            SAL_TaskSleep(SAFETY_PERIOD_MS - elapsed);
        }
    }
}

/*
***************************************************************************************************
*                                          CAN_RX_Task
***************************************************************************************************
*/

void CAN_RX_Task(void *pArg) {
    (void)pArg;
    
    mcu_printf("[CAN_RX] Task Started (Event-driven)\n");
    
    while(1) {
        // TODO: CAN RX 인터럽트 대기
        // 현재는 폴링 방식으로 시뮬레이션
        SAL_TaskSleep(20);
        
        // Simulate CAN data
        SAL_CoreCriticalEnter();
        g_CANData.motor_speed_cmd = 50.0f;
        g_CANData.suspension_enable = 1;
        g_CANData.leveling_enable = 1;
        SAL_GetTickCount(&g_CANData.last_rx_time);
        SAL_CoreCriticalExit();
    }
}

/*
***************************************************************************************************
*                                          Motor_Control_Task
***************************************************************************************************
*/

void Motor_Control_Task(void *pArg) {
    (void)pArg;
    
    uint32 start_tick, current_tick;
    float current_speed = 0;
    
    mcu_printf("[MOTOR] Task Started (100Hz)\n");
    
    SAL_TaskSleep(100);
    
    while(1) {
        SAL_GetTickCount(&start_tick);
        
        // Read safety and CAN data
        SAL_CoreCriticalEnter();
        SafetyLevel_t safety = g_SafetyData.level;
        float max_speed = g_SafetyData.max_motor_speed;
        SAL_CoreCriticalExit();
        
        SAL_CoreCriticalEnter();
        float target_speed = g_CANData.motor_speed_cmd;
        SAL_CoreCriticalExit();
        
        // Emergency stop
        if(safety >= SAFETY_EMERGENCY) {
            current_speed = 0;
            Motor_SetSpeed(0);
            
            SAL_CoreCriticalEnter();
            g_MotorData.current_speed = 0;
            SAL_CoreCriticalExit();
            
            SAL_TaskSleep(MOTOR_PERIOD_MS);
            continue;
        }
        
        // Limit speed
        if(target_speed > max_speed) {
            target_speed = max_speed;
        }
        
        // Ramping
        float ramp_rate = (safety == SAFETY_WARNING) ? 0.05f : 0.1f;
        if(target_speed > current_speed) {
            current_speed += ramp_rate;
            if(current_speed > target_speed) current_speed = target_speed;
        } else {
            current_speed -= ramp_rate;
            if(current_speed < target_speed) current_speed = target_speed;
        }
        
        // Output
        Motor_SetSpeed(current_speed);
        
        SAL_CoreCriticalEnter();
        g_MotorData.current_speed = current_speed;
        SAL_CoreCriticalExit();
        
        // Sleep
        SAL_GetTickCount(&current_tick);
        uint32 elapsed = current_tick - start_tick;
        if(elapsed < MOTOR_PERIOD_MS) {
            SAL_TaskSleep(MOTOR_PERIOD_MS - elapsed);
        }
    }
}

/*
***************************************************************************************************
*                                          IMU_Suspension_Task
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                          IMU_Suspension_Task
***************************************************************************************************
*/
void IMU_Suspension_Task(void *pArg)
{
    (void)pArg;

    uint32 start_tick, current_tick;
    
    double gx_sum = 0.0, gy_sum = 0.0, gz_sum = 0.0;

    uint32 servo_pulse_ns[4] = {
        SERVO_NEUTRAL_PULSE_NS, SERVO_NEUTRAL_PULSE_NS,
        SERVO_NEUTRAL_PULSE_NS, SERVO_NEUTRAL_PULSE_NS
    };

    static uint32 imu_sample_count = 0;

    /* ✅ Neutral 3초 캘리브 + 안정화 */
    static uint8 calibration_done = 0;

    double icm_roll_sum = 0.0, icm_pitch_sum = 0.0;
    uint32 calib_samples = 0;

    const uint32 CAL_MS = 3000;
    const uint32 PERIOD_MS = IMU_SUSP_PERIOD_MS;     // 10ms
    const uint32 CALIB_SAMPLES = (CAL_MS / PERIOD_MS);  // 300
    const uint32 STABILIZATION_SAMPLES = 200;        // 기존 유지(2초)

    IMU_Data imuRaw;

    // ADXL 관련
    static float wheel_lp_z[4] = {0};       // LPF (중력/경사 성분)
    static float adxl_pre_kick[4] = {0};    // Pre-kick 제어량
    static float wheel_prev_z_damp[4] = {0};

    // ✅ deg/sec 기반 추종용 상태 (서보 목표를 바로 보내지 않고 속도 제한으로 따라감)
    static float servo_current_deg[4] = {0, 0, 0, 0};

    mcu_printf("[IMU_SUSP] Task Started\n");
    mcu_printf("  - Leveling: STM32 PID (geometric)\n");
    mcu_printf("  - Damping: ADXL per-wheel\n");
    mcu_printf("  - Servo follow: deg/sec rate limiting\n\n");

    // 서보 초기화
    Servo_Init();
    Servo_SetPulseAllNs(servo_pulse_ns);

    // 초기값: servo_current_deg도 0(중립 기준 오프셋 0deg)
    for (uint8 i = 0; i < 4; i++) servo_current_deg[i] = 0.0f;

    SAL_CoreCriticalEnter();
    for (uint8 i = 0; i < 4; i++) {
        g_ServoData.position[i] = (float)NS_TO_US(servo_pulse_ns[i]);
    }
    g_IMU_SUSP_INIT_DONE = 1;
    SAL_CoreCriticalExit();

    // 댐핑 PID 초기화
    for (uint8 i = 0; i < 4; i++) {
        PID_Damping_Init(&pid_damping_wheel[i],
                         DAMPING_KP_MEDIUM, 0.0f, DAMPING_KD_MEDIUM,
                         IMU_SUSP_PERIOD_MS);
    }

    mcu_printf("[SERVO] Initialized to neutral\n");
    mcu_printf("[CALIB] Starting calibration (50 samples)...\n");

    while (1) {
        SAL_GetTickCount(&start_tick);

        // ========== 1. IMU 읽기 ==========
        if (IMU_Read_Data_DMA() != SAL_RET_SUCCESS) {
            SAL_TaskSleep(IMU_SUSP_PERIOD_MS);
            continue;
        }
        imuRaw = IMU;

        /* ========== 2. Neutral + 3초 캘리브레이션 ========== */
        if (!calibration_done) {

            /* ✅ 시작 직후/캘리브 중에는 항상 중립 유지 */
            for (uint8 i = 0; i < 4; i++) {
                servo_pulse_ns[i] = SERVO_NEUTRAL_PULSE_NS;
            }
            Servo_SetPulseAllNs(servo_pulse_ns);

            /* IMU 읽기 성공했을 때만 샘플 누적 */
            float r_acc = atan2f(imuRaw.accel_y, imuRaw.accel_z) * 180.0f / M_PI;
            float p_acc = atan2f(-imuRaw.accel_x,
                                sqrtf(imuRaw.accel_y * imuRaw.accel_y +
                                    imuRaw.accel_z * imuRaw.accel_z)) * 180.0f / M_PI;

            icm_roll_sum  += (double)r_acc;
            icm_pitch_sum += (double)p_acc;
            calib_samples++;

            if ((calib_samples % 50) == 0) {
                mcu_printf("  [CALIB] Neutral calib: %lu/%lu\n",
                        (unsigned long)calib_samples, (unsigned long)CALIB_SAMPLES);
            }

            /* ✅ 3초(300샘플) 완료 */
            if (calib_samples >= CALIB_SAMPLES) {
                roll_offset  = (float)(icm_roll_sum  / (double)CALIB_SAMPLES);
                pitch_offset = (float)(icm_pitch_sum / (double)CALIB_SAMPLES);

                mcu_printf("\n[CALIB] ICM offsets done (3s neutral)\n");
                mcu_printf("  ICM Roll offset: ");
                Print_Float_Value(roll_offset, 100);
                mcu_printf(" deg\n  ICM Pitch offset: ");
                Print_Float_Value(pitch_offset, 100);
                mcu_printf(" deg\n");

                /* ✅ ADXL bias 캘리브 완료 대기 (ADXL Task가 3초 동안 수행) */
                mcu_printf("[CALIB] Waiting ADXL bias calibration...\n");
                while (1) {
                    uint8 adxl_ok;
                    SAL_CoreCriticalEnter();
                    adxl_ok = g_ADXL_CAL_DONE;
                    SAL_CoreCriticalExit();
                    if (adxl_ok) break;
                    SAL_TaskSleep(20);
                }

                calibration_done = 1;
                imu_sample_count = 0;   /* ✅ 안정화 카운트는 캘리브 이후부터 다시 */
                mcu_printf("\n[CALIB] Complete! Start stabilization...\n\n");
            }

            /* 캘리브 중에는 IMU 공유값은 0으로 유지 */
            SAL_CoreCriticalEnter();
            g_IMUData.roll  = 0.0f;
            g_IMUData.pitch = 0.0f;
            g_IMUData.gyro_x = imuRaw.gyro_x;
            g_IMUData.gyro_y = imuRaw.gyro_y;
            g_IMUData.gyro_z = imuRaw.gyro_z;
            g_IMUData.accel_x = imuRaw.accel_x;
            g_IMUData.accel_y = imuRaw.accel_y;
            g_IMUData.accel_z = imuRaw.accel_z;
            SAL_GetTickCount(&g_IMUData.last_update_time);
            SAL_CoreCriticalExit();

            SAL_TaskSleep(IMU_SUSP_PERIOD_MS);
            continue;
        }


        // ========== 3. Kalman 필터링 ==========
        imu_sample_count++;

        float roll_acc  = atan2f(imuRaw.accel_y, imuRaw.accel_z) * 180.0f / M_PI - roll_offset;
        float pitch_acc = atan2f(-imuRaw.accel_x,
                                 sqrtf(imuRaw.accel_y * imuRaw.accel_y +
                                       imuRaw.accel_z * imuRaw.accel_z)) * 180.0f / M_PI - pitch_offset;

        float dt = (float)IMU_SUSP_PERIOD_MS / 1000.0f;

        float ax = imuRaw.accel_x;
        float ay = imuRaw.accel_y;
        float az = imuRaw.accel_z;
        float acc_mag = sqrtf(ax*ax + ay*ay + az*az);
        float acc_err = fabsf(acc_mag - ACC_MAG_NORM);
    
        // ✅ acc 신뢰도에 따라 R을 키우면 (S 커짐) accel을 덜 믿게 됨
        float R;
        if (acc_err <= ACC_OK_BAND) {
            R = KALMAN_R_MIN;
        } else if (acc_err <= ACC_SOFT_BAND) {
            float t = (acc_err - ACC_OK_BAND) / (ACC_SOFT_BAND - ACC_OK_BAND); /* 0..1 */
            R = KALMAN_R_MIN + t * (KALMAN_R_MAX - KALMAN_R_MIN);
        } else {
            R = KALMAN_R_MAX;
        }

        Kalman_SetR(&kalman_roll,  R);
        Kalman_SetR(&kalman_pitch, R);

        // ✅ 극단 구간에서는 accel 업데이트 스킵(자이로 적분만)
        uint8 acc_ok = (acc_err <= ACC_SOFT_BAND) ? 1U : 0U;

        float roll, pitch;
        if (acc_ok) {
            roll  = Kalman_Update(&kalman_roll,  roll_acc,  imuRaw.gyro_x, dt);
            pitch = Kalman_Update(&kalman_pitch, pitch_acc, imuRaw.gyro_y, dt);
        } else {
            roll  = Kalman_PredictOnly(&kalman_roll,  imuRaw.gyro_x, dt);
            pitch = Kalman_PredictOnly(&kalman_pitch, imuRaw.gyro_y, dt);
        }

        SAL_CoreCriticalEnter();
        g_IMUData.roll  = clamp(roll,  -70.0f, 70.0f);
        g_IMUData.pitch = clamp(pitch, -70.0f, 70.0f);

        // ✅ gyro raw 저장 (deg/s라고 가정)
        g_IMUData.gyro_x = imuRaw.gyro_x;
        g_IMUData.gyro_y = imuRaw.gyro_y;
        g_IMUData.gyro_z = imuRaw.gyro_z;

        // ✅ accel raw 저장
        g_IMUData.accel_x = imuRaw.accel_x;
        g_IMUData.accel_y = imuRaw.accel_y;
        g_IMUData.accel_z = imuRaw.accel_z;

        SAL_GetTickCount(&g_IMUData.last_update_time);
        SAL_CoreCriticalExit();

        // ========== 3-1. 안정화 대기 ==========
        if (imu_sample_count < STABILIZATION_SAMPLES) {
            if (imu_sample_count % 50 == 0) {
                mcu_printf("[STABIL] %lu/%lu samples...\n",
                        (unsigned long)imu_sample_count, (unsigned long)STABILIZATION_SAMPLES);
            }
            Servo_SetPulseAllNs(servo_pulse_ns);
            SAL_TaskSleep(IMU_SUSP_PERIOD_MS);
            continue;
        }

        if (imu_sample_count == STABILIZATION_SAMPLES) {
            mcu_printf("\n[STABIL] Complete! Starting control...\n\n");

            /* ✅ 이제부터 모니터링 로그 출력 허용 */
            SAL_CoreCriticalEnter();
            g_SYSTEM_READY = 1;
            SAL_CoreCriticalExit();
        }


        // ========== 4. 안전/상태 읽기 ==========
        SAL_CoreCriticalEnter();
        SafetyLevel_t safety   = g_SafetyData.level;
        uint8 susp_disabled    = g_SafetyData.suspension_disabled;
        float range_limit      = g_SafetyData.suspension_range_limit;
        uint8 on_slope         = g_SafetyData.on_slope_detected;
        uint8 leveling_on      = g_CANData.leveling_enable;
        float current_speed    = g_MotorData.current_speed;
        SAL_CoreCriticalExit();

        // ========== 5. 비상 시 중립 복귀 ==========
        if (susp_disabled || safety >= SAFETY_EMERGENCY) {
            reset_leveling_pid(&pid_roll);
            reset_leveling_pid(&pid_pitch);

            for (uint8 i = 0; i < 4; i++) {
                PID_Damping_Reset(&pid_damping_wheel[i]);
                adxl_pre_kick[i] = 0.0f;
                wheel_lp_z[i] = 0.0f;
                wheel_prev_z_damp[i] = 0.0f;

                // ✅ 서보 추종 상태도 중립으로 천천히 복귀(속도 제한)
                servo_current_deg[i] *= 0.95f;
            }

            // 중립 출력
            for (uint8 i = 0; i < 4; i++) {
                servo_pulse_ns[i] = SERVO_NEUTRAL_PULSE_NS;
            }
            Servo_SetPulseAllNs(servo_pulse_ns);

            SAL_CoreCriticalEnter();
            for (uint8 i = 0; i < 4; i++) {
                g_ServoData.position[i] = (float)NS_TO_US(servo_pulse_ns[i]);
            }
            SAL_CoreCriticalExit();

            SAL_TaskSleep(IMU_SUSP_PERIOD_MS);
            continue;
        }

        // ========== 6. ADXL 읽기 + LPF + Pre-kick ==========
        float wheel_accel_z[4];
        SAL_CoreCriticalEnter();
        for (uint8 i = 0; i < 4; i++) {
            wheel_accel_z[i] = g_ADXLData.accel_z[i];
        }
        SAL_CoreCriticalExit();

        // LPF 계수
        float tau   = 1.0f / (2.0f * M_PI * LPF_CUTOFF_FREQ);
        float alpha = dt / (tau + dt);

        // Pre-kick 가중치
        float kick_w;
        if (on_slope) {
            kick_w = 0.0f;
        } else {
            uint32 elapsed_samples = imu_sample_count - STABILIZATION_SAMPLES;
            if (elapsed_samples < 300) {
                kick_w = (float)elapsed_samples / 300.0f;
            } else {
                kick_w = 1.0f;
            }
        }

        for (uint8 i = 0; i < 4; i++) {
            float z = wheel_accel_z[i];

            // LPF(중력/경사)
            wheel_lp_z[i] += alpha * (z - wheel_lp_z[i]);

            // HPF(충격)
            float hp = z - wheel_lp_z[i];

            adxl_pre_kick[i] *= PRE_KICK_DECAY;

            if (fabsf(hp) > IMPACT_HP_THRESHOLD) {
                adxl_pre_kick[i] = (hp > 0.0f) ? -PRE_KICK_MAGNITUDE : PRE_KICK_MAGNITUDE;
            }

            if (fabsf(adxl_pre_kick[i]) < 0.1f) {
                adxl_pre_kick[i] = 0.0f;
            }

            adxl_pre_kick[i] *= kick_w;
        }

        /* ===== 공통: tilt 크기 & soft weight (레벨링/서보레이트 공용) ===== */
        float total_tilt = fabsf(roll) + fabsf(pitch);

        /* small-tilt gain 스케줄링 (0~1) */
        float soft_w = 0.0f;
        if (total_tilt <= TILT_SOFT_START_DEG) {
            soft_w = 0.0f;
        } else if (total_tilt >= TILT_SOFT_FULL_DEG) {
            soft_w = 1.0f;
        } else {
            soft_w = (total_tilt - TILT_SOFT_START_DEG) / (TILT_SOFT_FULL_DEG - TILT_SOFT_START_DEG);
        }


        // ========== 7. 레벨링 PID ==========
        float leveling[4] = {0, 0, 0, 0};

        if (leveling_on) {
            /* ✅ 1) boost 로직 개선 - 언덕에서 더 강하게 */
            float boost = 1.0f;
            if (total_tilt > 12.0f) boost = 2.0f;      // 큰 경사
            else if (total_tilt > 6.0f) boost = 1.6f;  // 중간 경사
            else if (total_tilt > 3.0f) boost = 1.3f;  // 작은 경사

            /* ✅ 2) gain scaling 수정 - 언덕에서는 풀 파워! */
            float leveling_gain_scale;

            // 기존: 0.3 + 0.7*soft_w
            if (on_slope) {
                 // 언덕이면 무조건 100%
                 leveling_gain_scale = 1.0f;
             } else {
                 // 평지에서만 soft scaling 적용
                 leveling_gain_scale = (0.3f + 0.7f * soft_w);  // 최소 30%
            }

            float roll_height_mm  = angle_to_height_mm(roll,  TRACK_WIDTH_MM * 0.5f);
            float pitch_height_mm = angle_to_height_mm(pitch, WHEELBASE_MM * 0.5f);

            // 진동 감지 (기존 로직 유지)
            static float prev_roll_f=0, prev_pitch_f=0;
            static float prev_dr=0, prev_dp=0;
            static int osc_count=0;

            float dr = roll - prev_roll_f;
            float dp = pitch - prev_pitch_f;

            if (fabsf(dr) > 0.5f && (dr * prev_dr < 0)) osc_count = 12;
            else if (fabsf(dp) > 0.5f && (dp * prev_dp < 0)) osc_count = 12;
            else if (osc_count > 0) osc_count--;

            prev_roll_f = roll; prev_pitch_f = pitch;
            prev_dr = dr; prev_dp = dp;

            // ✅ 3) 진동 감쇠 - 평지에서만 적용
            float osc_damping = 1.0f;
            if (!on_slope && osc_count > 0) {
                osc_damping = 0.4f;  // 평지 진동만 감쇠
            }

            /* ✅ 핵심: PID 계산 */
            compute_leveling_pid(&pid_roll,  LEVELING_SIGN_ROLL  * roll_height_mm,  dt, on_slope);
            compute_leveling_pid(&pid_pitch, LEVELING_SIGN_PITCH * pitch_height_mm, dt, on_slope);
            
            float roll_ctrl_mm  = pid_roll.output  * boost * leveling_gain_scale * osc_damping;
            float pitch_ctrl_mm = pid_pitch.output * boost * leveling_gain_scale * osc_damping;

            float roll_ctrl_deg  = height_to_servo_deg(roll_ctrl_mm,  MAX_CORRECTION_DEG);
            float pitch_ctrl_deg = height_to_servo_deg(pitch_ctrl_mm, MAX_CORRECTION_DEG);

            float max_deg = MAX_CORRECTION_DEG * range_limit;
            roll_ctrl_deg  = clamp(roll_ctrl_deg,  -max_deg, max_deg);
            pitch_ctrl_deg = clamp(pitch_ctrl_deg, -max_deg, max_deg);

            // 4바퀴 배분
            
            leveling[0] = -roll_ctrl_deg + pitch_ctrl_deg;  // FL
            leveling[1] = +roll_ctrl_deg + pitch_ctrl_deg;  // FR
            leveling[2] = -roll_ctrl_deg - pitch_ctrl_deg;  // RL
            leveling[3] = +roll_ctrl_deg - pitch_ctrl_deg;  // RR

            for (uint8 i = 0; i < 4; i++) {
                leveling[i] = clamp(leveling[i], -max_deg, max_deg);
            }
            
            // ✅ 새로 추가: 서보 한계 감지
            static uint8 servo_saturated_count = 0;
            uint8 any_saturated = 0;
            
            for (uint8 i = 0; i < 4; i++) {
                if (fabsf(leveling[i]) > max_deg * 0.90f) {  // 90% 이상이면 포화
                    any_saturated = 1;
                    break;
                }
            }
            
            if (any_saturated) {
                servo_saturated_count++;
                if (servo_saturated_count > 30) {  // 0.3초 이상 포화 상태
                    // 적분항 감소
                    pid_roll.integral *= 0.85f;
                    pid_pitch.integral *= 0.85f;
                }
            } else {
                servo_saturated_count = 0;
            }
            
        } else {
            reset_leveling_pid(&pid_roll);
            reset_leveling_pid(&pid_pitch);
        }

        // ========== 8. 독립 댐핑 ==========
        float wheel_damping[4] = {0, 0, 0, 0};

        float damping_kp, damping_kd, max_damping;
        if (current_speed < SPEED_THRESHOLD_LOW) {
            damping_kp = DAMPING_KP_SOFT;
            damping_kd = DAMPING_KD_SOFT;
            max_damping = 8.0f;
        } else if (current_speed < SPEED_THRESHOLD_HIGH) {
            damping_kp = DAMPING_KP_MEDIUM;
            damping_kd = DAMPING_KD_MEDIUM;
            max_damping = 10.0f;
        } else {
            damping_kp = DAMPING_KP_HARD;
            damping_kd = DAMPING_KD_HARD;
            max_damping = 12.0f;
        }

        for (uint8 i = 0; i < 4; i++) {
            pid_damping_wheel[i].Kp = damping_kp;
            pid_damping_wheel[i].Kd = damping_kd;

            float dz = wheel_accel_z[i] - wheel_prev_z_damp[i];
            wheel_prev_z_damp[i] = wheel_accel_z[i];

            dz = clamp(dz, -6.0f * 9.81f, 6.0f * 9.81f);

            wheel_damping[i] = PID_Damping_Update(&pid_damping_wheel[i], 0.0f, dz / 9.81f);

            float max_damp = max_damping * range_limit;
            wheel_damping[i] = clamp(wheel_damping[i], -max_damp, max_damp);
        }

        // ========== 9. 통합 제어 + deg/sec 기반 속도 제한 추종 ==========
        SAL_CoreCriticalEnter();
        float height = g_HeightData.height_offset;
        SAL_CoreCriticalExit();

        float target_deg[4];

        // 9-1) raw target 만들기
        for (uint8 i = 0; i < 4; i++) {
            float raw_target = height + leveling[i] + wheel_damping[i] + adxl_pre_kick[i];

            float max_offset = 70.0f * range_limit;
            raw_target = clamp(raw_target, -max_offset, max_offset);

            target_deg[i] = raw_target;
        }

        // 9-2) 상황별 rate(deg/sec) 선택
        total_tilt = fabsf(roll) + fabsf(pitch);

        float rate_deg_s;
        if (on_slope) {
            rate_deg_s = SERVO_RATE_SLOPE;
        } else {
            /* 소각도(soft_w=0)에서는 very slow, 커질수록 FAST로 */
            float rate_min = 120.0f;
            float rate_max = SERVO_RATE_FLAT_FAST;
            rate_deg_s = rate_min + (rate_max - rate_min) * soft_w;
        }


        float dt_s = (float)IMU_SUSP_PERIOD_MS / 1000.0f;
        float max_step = rate_deg_s * dt_s;   // deg/cycle

        /* ✅ 아주 작은 목표는 움직이지 않게 (헌팅 방지) */
        static uint8 tgt_zero_mode[4] = {1,1,1,1};

        for (uint8 i=0;i<4;i++){
            float a = fabsf(target_deg[i]);
            if (tgt_zero_mode[i]) {
                if (a > TGT_DZ_EXIT) tgt_zero_mode[i] = 0;
            } else {
                if (a < TGT_DZ_ENTER) tgt_zero_mode[i] = 1;
            }
            if (tgt_zero_mode[i]) target_deg[i] = 0.0f;
        }

        /* ===== Debug snapshot (minimal overhead) ===== */
        SAL_CoreCriticalEnter();
        g_SuspDbg.leveling[0] = leveling[0];
        g_SuspDbg.leveling[1] = leveling[1];
        g_SuspDbg.leveling[2] = leveling[2];
        g_SuspDbg.leveling[3] = leveling[3];

        g_SuspDbg.pid_roll_out_mm  = pid_roll.output;
        g_SuspDbg.pid_pitch_out_mm = pid_pitch.output;


        g_SuspDbg.pid_roll_integral  = pid_roll.integral;
        g_SuspDbg.pid_pitch_integral = pid_pitch.integral;

        g_SuspDbg.soft_w    = soft_w;
        g_SuspDbg.rate_deg_s = rate_deg_s;
        g_SuspDbg.max_step  = max_step;

        g_SuspDbg.roll_acc_deg  = roll_acc;
        g_SuspDbg.pitch_acc_deg = pitch_acc;
        g_SuspDbg.gyro_x_dps = imuRaw.gyro_x;
        g_SuspDbg.gyro_y_dps = imuRaw.gyro_y;
        g_SuspDbg.gyro_z_dps = imuRaw.gyro_z;
        SAL_CoreCriticalExit();

        // 9-3) servo_current_deg가 target을 속도 제한으로 따라감
        for (uint8 i = 0; i < 4; i++) {
            float gap = target_deg[i] - servo_current_deg[i];
            float step = clamp(gap, -max_step, max_step);
            servo_current_deg[i] += step;

            // 최종 명령은 servo_current_deg
            target_deg[i] = servo_current_deg[i];
        }

        // ========== 10. 서보 출력 ==========
        float pulse_range_ns = (float)(SERVO_MAX_PULSE_NS - SERVO_MIN_PULSE_NS);
        float deg_range = 2.0f * MAX_CORRECTION_DEG;
        float ns_per_deg = pulse_range_ns / deg_range;

        for (uint8 i = 0; i < 4; i++) {
            float offset_ns = target_deg[i] * ns_per_deg;
            float p = (float)SERVO_NEUTRAL_PULSE_NS + offset_ns;

            servo_pulse_ns[i] = (uint32)clamp(p, (float)SERVO_MIN_PULSE_NS, (float)SERVO_MAX_PULSE_NS);
        }

        Servo_SetPulseAllNs(servo_pulse_ns);

        SAL_CoreCriticalEnter();
        for (uint8 i = 0; i < 4; i++) {
            g_ServoData.position[i] = (float)NS_TO_US(servo_pulse_ns[i]);
        }
        SAL_CoreCriticalExit();

        // ========== 11. Sleep ==========
        SAL_GetTickCount(&current_tick);
        uint32 elapsed = current_tick - start_tick;
        if (elapsed < IMU_SUSP_PERIOD_MS) {
            SAL_TaskSleep(IMU_SUSP_PERIOD_MS - elapsed);
        }
    }
}


/*
***************************************************************************************************
*                                          Monitoring_Task
***************************************************************************************************
*/
void Monitoring_Task(void *pArg) {
    (void)pArg;
    SuspDbg_t dbg;
    uint32 now = 0;

    // ✅ 초기화 대기
    while (1) {
        uint8 ready;
        SAL_CoreCriticalEnter();
        ready = g_IMU_SUSP_INIT_DONE;
        SAL_CoreCriticalExit();

        if (ready != 0U) break;
        SAL_TaskSleep(50);
    }

    /* ✅ 2) "3초 중립 캘리브 + 안정화" 끝날 때까지 대기 (여기까지는 로그 출력 X) */
    while (1) {
        uint8 sys_ready;
        SAL_CoreCriticalEnter();
        sys_ready = g_SYSTEM_READY;
        SAL_CoreCriticalExit();

        if (sys_ready != 0U) break;
        SAL_TaskSleep(50);
    }

    mcu_printf("[MONITOR] Task Started (10Hz)\n\n");
    SAL_TaskSleep(100);
    
    while(1) {
        // ✅ 모든 데이터를 Critical Section 안에서 읽기
        SAL_CoreCriticalEnter();
        SafetyLevel_t safety = g_SafetyData.level;
        uint8 on_slope = g_SafetyData.on_slope_detected;  // ✅ 여기서 읽기!
        float roll = g_IMUData.roll;
        float pitch = g_IMUData.pitch;
        float speed = g_MotorData.current_speed;
        
        float wheel_z[4];
        for(uint8 i = 0; i < 4; i++) {
            wheel_z[i] = g_ADXLData.accel_z[i];
        }
        
        float servo[4];
        for(uint8 i = 0; i < 4; i++) {
            servo[i] = g_ServoData.position[i];
        }
        
        uint32 adxl_t = g_ADXLData.last_update_time;
        dbg = g_SuspDbg;
        SAL_CoreCriticalExit();
        
        SAL_GetTickCount(&now);
        
        // ========== 로그 출력 ==========
        mcu_printf("\n==========================================\n");
        
        // ✅ Safety, Speed, Slope (수정됨)
        mcu_printf("Safety: %d | Speed: ", safety);
        Print_Float_Value(speed, 10);
        mcu_printf("%% | SLOPE: %s\n", on_slope ? "YES" : "NO");
        
        // ICM Roll/Pitch
        mcu_printf("[ICM] Roll: ");
        Print_Float_Value(roll, 10);
        mcu_printf(" Pitch: ");
        Print_Float_Value(pitch, 10);
        mcu_printf("\n");
        
        // ADXL Z-axis
        mcu_printf("[ADXL] FL: ");
        Print_Float_Value(wheel_z[0], 100);
        mcu_printf(" FR: ");
        Print_Float_Value(wheel_z[1], 100);
        mcu_printf(" RL: ");
        Print_Float_Value(wheel_z[2], 100);
        mcu_printf(" RR: ");
        Print_Float_Value(wheel_z[3], 100);
        mcu_printf(" m/s2\n");
        
        // Servo positions
        mcu_printf("[SERVO] FL: ");
        Print_Float_Value(servo[0], 10);
        mcu_printf(" FR: ");
        Print_Float_Value(servo[1], 10);
        mcu_printf(" RL: ");
        Print_Float_Value(servo[2], 10);
        mcu_printf(" RR: ");
        Print_Float_Value(servo[3], 10);
        mcu_printf(" us\n");
        
        // Debug: soft_w, rate, max_step
        mcu_printf("[DBG] soft_w: ");
        Print_Float_Value(dbg.soft_w, 1000);
        mcu_printf(" | rate: ");
        Print_Float_Value(dbg.rate_deg_s, 10);
        mcu_printf(" deg/s | max_step: ");
        Print_Float_Value(dbg.max_step, 1000);
        mcu_printf(" deg/cycle\n");
        
        // PID output (mm)
        mcu_printf("[DBG] PID_out(mm) Roll: ");
        Print_Float_Value(dbg.pid_roll_out_mm, 100);
        mcu_printf(" Pitch: ");
        Print_Float_Value(dbg.pid_pitch_out_mm, 100);
        mcu_printf("\n");
        
        // ✅ PID Integral (추가!)
        mcu_printf("[DBG] PID_integral Roll: ");
        Print_Float_Value(dbg.pid_roll_integral, 100);
        mcu_printf(" Pitch: ");
        Print_Float_Value(dbg.pid_pitch_integral, 100);
        mcu_printf("\n");
        
        // Leveling (deg)
        mcu_printf("[DBG] leveling(deg) FL: ");
        Print_Float_Value(dbg.leveling[0], 100);
        mcu_printf(" FR: ");
        Print_Float_Value(dbg.leveling[1], 100);
        mcu_printf(" RL: ");
        Print_Float_Value(dbg.leveling[2], 100);
        mcu_printf(" RR: ");
        Print_Float_Value(dbg.leveling[3], 100);
        mcu_printf("\n");
        
        // ADXL last update
        mcu_printf("[ADXL] last_update: %u (age %u ms)\n", 
                   (unsigned int)adxl_t, (unsigned int)(now - adxl_t));
        
        // IMU Raw gyro
        mcu_printf("[IMU_RAW] gyro(dps) x: ");
        Print_Float_Value(dbg.gyro_x_dps, 10);
        mcu_printf(" y: ");
        Print_Float_Value(dbg.gyro_y_dps, 10);
        mcu_printf(" z: ");
        Print_Float_Value(dbg.gyro_z_dps, 10);
        mcu_printf("\n");
        
        // IMU Accelerometer angles
        mcu_printf("[IMU_ACC] roll_acc: ");
        Print_Float_Value(dbg.roll_acc_deg, 10);
        mcu_printf(" deg | pitch_acc: ");
        Print_Float_Value(dbg.pitch_acc_deg, 10);
        mcu_printf(" deg\n");
        
        mcu_printf("==========================================\n");
        
        SAL_TaskSleep(MONITOR_PERIOD_MS);
    }
}

/*
***************************************************************************************************
*                                          ADXL345_Monitor_Task
* 
* 100Hz로 4개 ADXL345 센서를 읽어서 각 바퀴의 충격을 감지
***************************************************************************************************
*/
static void ADXL345_Monitor_Task(void *pArg)
{
    (void)pArg;

    uint32 start_tick;

    float prev_accel_z[4] = {0};
    float impact_threshold = 1.6f * 9.81f;  // 1.6G

    /* CH0=RL, CH1=FL, CH2=RR, CH3=FR
       wheel idx: 0=FL,1=FR,2=RL,3=RR */
    const uint8 ADXL_CH_TO_WHEEL[4] = { 2, 0, 3, 1 };

    /* ✅ 3초 캘리브(중립 유지 동안) = 300샘플 @100Hz */
    const uint32 CAL_MS = 3000;
    const uint32 PERIOD_MS = 10;
    const uint32 CAL_SAMPLES = (CAL_MS / PERIOD_MS);

    /* bias 적분용 */
    double sum_x[4] = {0}, sum_y[4] = {0}, sum_z[4] = {0};
    uint32 sample_cnt = 0;

    mcu_printf("[ADXL345] Task Started (100Hz)\n");
    mcu_printf("[ADXL345] Neutral calibration for 3 seconds...\n");

    /* ✅ 시스템 준비 플래그가 1 되기 전까지는 "캘리브 모드"로 bias를 만든다 */
    while (sample_cnt < CAL_SAMPLES) {
        SAL_GetTickCount(&start_tick);

        for (uint8 ch = 0; ch < 4; ch++) {
            float x, y, z;
            uint8 wheel_idx = ADXL_CH_TO_WHEEL[ch];

            /* 캘리브 전이라도 값을 얻기 위해 Calibrated API 그대로 사용 */
            if (ADXL345_ReadAccelCalibrated(ch, &x, &y, &z) == SAL_RET_SUCCESS) {
                sum_x[wheel_idx] += (double)x;
                sum_y[wheel_idx] += (double)y;
                sum_z[wheel_idx] += (double)z;
            }
        }

        sample_cnt++;

        if ((sample_cnt % 50) == 0) {
            mcu_printf("  ADXL calib: %lu/%lu\n", (unsigned long)sample_cnt, (unsigned long)CAL_SAMPLES);
        }

        uint32 now;
        SAL_GetTickCount(&now);
        uint32 elapsed = now - start_tick;
        if (elapsed < PERIOD_MS) SAL_TaskSleep(PERIOD_MS - elapsed);
    }

    /* ✅ bias 확정 */
    for (uint8 i = 0; i < 4; i++) {
        g_ADXL_bias_x[i] = (float)(sum_x[i] / (double)CAL_SAMPLES);
        g_ADXL_bias_y[i] = (float)(sum_y[i] / (double)CAL_SAMPLES);
        g_ADXL_bias_z[i] = (float)(sum_z[i] / (double)CAL_SAMPLES);

        prev_accel_z[i] = g_ADXL_bias_z[i];
    }

    SAL_CoreCriticalEnter();
    g_ADXL_CAL_DONE = 1;
    SAL_CoreCriticalExit();

    mcu_printf("[ADXL345] Bias calibration done.\n");
    mcu_printf("  FL bias z: "); Print_Float_Value(g_ADXL_bias_z[0], 100); mcu_printf("\n");
    mcu_printf("  FR bias z: "); Print_Float_Value(g_ADXL_bias_z[1], 100); mcu_printf("\n");
    mcu_printf("  RL bias z: "); Print_Float_Value(g_ADXL_bias_z[2], 100); mcu_printf("\n");
    mcu_printf("  RR bias z: "); Print_Float_Value(g_ADXL_bias_z[3], 100); mcu_printf("\n");

    SAL_TaskSleep(100);

    /* ✅ 이후부터는 정상 모니터링(impact 계산 + bias 제거 후 저장) */
    while(1) {
        SAL_GetTickCount(&start_tick);

        for(uint8 ch = 0; ch < 4; ch++) {
            float x, y, z;
            uint8 wheel_idx = ADXL_CH_TO_WHEEL[ch];

            if(ADXL345_ReadAccelCalibrated(ch, &x, &y, &z) == SAL_RET_SUCCESS) {

                /* ✅ bias 제거 */
                x -= g_ADXL_bias_x[wheel_idx];
                y -= g_ADXL_bias_y[wheel_idx];
                z -= g_ADXL_bias_z[wheel_idx];

                float delta_z = fabsf(z - prev_accel_z[wheel_idx]);
                prev_accel_z[wheel_idx] = z;

                float impact = 0.0f;
                if(delta_z > impact_threshold) {
                    impact = fminf((delta_z / impact_threshold) - 1.0f, 1.0f);
                }

                SAL_CoreCriticalEnter();
                g_ADXLData.accel_z[wheel_idx] = z;
                g_ADXLData.impact_detected[wheel_idx] = impact;
                SAL_GetTickCount(&g_ADXLData.last_update_time);
                SAL_CoreCriticalExit();
            } else {
                /* 실패시 impact=0 */
                SAL_CoreCriticalEnter();
                g_ADXLData.impact_detected[wheel_idx] = 0.0f;
                SAL_CoreCriticalExit();
            }
        }

        uint32 now;
        SAL_GetTickCount(&now);
        uint32 elapsed = now - start_tick;
        if(elapsed < PERIOD_MS) {
            SAL_TaskSleep(PERIOD_MS - elapsed);
        }
    }
}



static void AppTaskCreate(void)
{
#if (APLT_LINUX_SUPPORT_SPI_DEMO == 1)
    ECCP_InitSPIManager();
#endif  
#if (APLT_LINUX_SUPPORT_POWER_CTRL == 1)
    POWER_APP_StartDemo();
#endif

  
#if ( MCU_BSP_SUPPORT_APP_CONSOLE == 1 )
    CreateConsoleTask();
#endif  // ( MCU_BSP_SUPPORT_APP_CONSOLE == 1 )

#if ( MCU_BSP_SUPPORT_APP_KEY == 1 )
    KEY_AppCreate();
#endif  // ( MCU_BSP_SUPPORT_APP_KEY == 1 )

#if ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )
    CAN_DemoCreateApp();
#endif  // ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )

#if ( MCU_BSP_SUPPORT_APP_FW_UPDATE == 1 )
    CreateFWUDTask();
#elif ( MCU_BSP_SUPPORT_APP_FW_UPDATE_ECCP == 1 )
    CreateFWUDTask();
#endif

#if ( MCU_BSP_SUPPORT_APP_IDLE == 1 )
    IDLE_CreateTask();
#endif  // ( MCU_BSP_SUPPORT_APP_IDLE == 1 )

#if ( MCU_BSP_SUPPORT_APP_SPI_LED == 1)
    SPILED_CreateAppTask();
#endif  // ( MCU_BSP_SUPPORT_APP_SPI_LED == 1 )

}

static void DisplayAliveLog(void)
{
    if (gALiveMsgOnOff != 0U)
    {
        mcu_printf("\n %d", gALiveCount);

        gALiveCount++;

        if(gALiveCount >= MAIN_UINT_MAX_NUM)
        {
            gALiveCount = 0;
        }
    }
    else
    {
        gALiveCount = 0;
    }
}

#define LDT1_AREA_ADDR  0xA1011800U
#define PMU_REG_ADDR    0xA0F28000U

static void DisplayOTPInfo(void)
{
    volatile uint32 *ldt1Addr;
    volatile uint32 *chipNameAddr;
    volatile uint32 *remapAddr;
    volatile uint32 *hsmStatusAddr;
    uint32          chipName = 0;
    uint32          dualBankVal = 0;
    uint32          dual_bank = 0;
    uint32          expandFlashVal = 0;
    uint32          expand_flash = 0;
    uint32          remap_mode = 0;
    uint32          hsm_ready = 0;

    //----------------------------------------------------------------
    // OTP LDT1 Read
    // [11:0]Dual_Bank_Selection, [59:48]EXPAND_FLASH
    // Dual_Bank_Sel: [0xC0][11: 0] & [0xD0][11: 0] & [0xE0][11: 0] & [0xF0][11: 0]
    // EXPAND_FLASH : [0xC4][27:16] & [0xD4][27:16] & [0xE4][27:16] & [0xF4][27:16]
    // HwMC_PRG_FLS_LDT1: 0xA1011800

    ldt1Addr = (volatile uint32 *)(LDT1_AREA_ADDR + 0x00C0);
    chipNameAddr = (volatile uint32 *)(LDT1_AREA_ADDR + 0x0300);
    remapAddr = (volatile uint32 *)(PMU_REG_ADDR);
    hsmStatusAddr = (volatile uint32 *)(PMU_REG_ADDR + 0x0020);

    chipName = *chipNameAddr;
    chipName &= 0x000FFFFF;

    dualBankVal = ldt1Addr[ 0];
    expandFlashVal = ldt1Addr[ 1];

    dualBankVal &= ldt1Addr[ 4];
    expandFlashVal &= ldt1Addr[ 5];

    dualBankVal &= ldt1Addr[ 8];
    expandFlashVal &= ldt1Addr[ 9];

    dualBankVal &= ldt1Addr[12];
    expandFlashVal &= ldt1Addr[13];

    dualBankVal = (dualBankVal >> 0) & 0x0FFF;
    expandFlashVal  = (expandFlashVal >> 16) & 0x0FFF;

    dual_bank = (dualBankVal == 0x0FFF) ? 0 : 1;            // (single_bank : dual_bank)
    expand_flash  = (expandFlashVal  == 0x0000) ? 0 : 1;    // (only_eFlash : use_extSNOR)

    remap_mode = remapAddr[ 0];

    mcu_printf("    CHIP   NAME  : %x\n",    chipName);
    mcu_printf("    DUAL   BANK  : %d\n",    dual_bank);
    mcu_printf("    EXPAND FLASH : %d\n",    expand_flash);
    mcu_printf("    REMAP  MODE  : %d\n",    (remap_mode >> 16));

    hsm_ready = hsmStatusAddr[ 0];
    hsm_ready = (hsm_ready >> 2) & 0x0001;
#if 0
    if(hsm_ready)
    {
        mcu_printf("    HSM    READY : %d\n",    hsm_ready);
    }
    else
    {
        while(hsm_ready != 1)
        {
            mcu_printf("    HSM    READY : %d\n",    hsm_ready);
            mcu_printf("    wait...\n");
            hsm_ready = (hsm_ready >> 2) & 0x0001;
        }
    }
#else
    mcu_printf("    HSM    READY : %d\n",    hsm_ready);
#endif
}

#endif  // ( MCU_BSP_SUPPORT_APP_BASE == 1 )