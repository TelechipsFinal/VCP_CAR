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
#include "ISO2631-1.h"



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

#define CAN_RX_TASK_PRIO        (SAL_PRIO_APP_CFG + 1)      
#define MOTOR_TASK_PRIO         (SAL_PRIO_APP_CFG + 2)
#define ADXL_MONITOR_TASK_PRIO  (SAL_PRIO_APP_CFG + 3)      // ✅ 추가
#define IMU_SUSP_TASK_PRIO      (SAL_PRIO_APP_CFG + 4)      
#define HEIGHT_TASK_PRIO        (SAL_PRIO_APP_CFG + 5)      
#define MONITOR_TASK_PRIO       (SAL_PRIO_APP_CFG + 6)
#define ISO_TASK_PRIO          (SAL_PRIO_APP_CFG + 7)





// Task Stack Sizes - 극한 최소화
#define CAN_RX_TASK_STK_SIZE    (128)   // 1KB
#define MOTOR_TASK_STK_SIZE     (128)   // 1KB
#define IMU_SUSP_TASK_STK_SIZE  (512)   // 2KB (가장 중요) 512로 돌리기@@@@@@@@@@@@@@@
#define HEIGHT_TASK_STK_SIZE    (128)   // 1KB
#define MONITOR_TASK_STK_SIZE   (128)   // 1KB    
#define ADXL_TEST_TASK_STK_SIZE (512)   // 1KB  ✅ 추가
 #define ISO_TASK_STK_SIZE      (256)   // 1KB 정도면 충분

// Control Frequencies
#define MOTOR_PERIOD_MS         (10)    // 100Hz
#define IMU_SUSP_PERIOD_MS      (10)    // 100Hz
#define HEIGHT_PERIOD_MS        (50)    // 20Hz
#define MONITOR_PERIOD_MS       (100)   // 10Hz
#define ISO_PERIOD_MS          (10)    // ✅ 100Hz (ISO 측정은 100Hz 권장)

/*
***************************************************************************************************
*                                         PID CONFIGURATION (정리됨)
***************************************************************************************************
*/

/* ===== 차량 기하학 파라미터 ===== */
#define WHEELBASE_MM            (305.0f)    // 앞뒤 바퀴 간격
#define TRACK_WIDTH_MM          (233.0f)    // 좌우 바퀴 간격

/* ===== 레벨링 PID 게인 (STM32 방식) ===== */
#define LEVELING_GAIN           1.5f
#define INTEGRAL_GAIN           0.30f
#define INTEGRAL_MAX            25.0f
#define DERIVATIVE_GAIN         0.25f
#define DERIVATIVE_FILTER       0.7f
#define DEADBAND                0.5f

/* ===== 속도 임계값 ===== */
#define SPEED_THRESHOLD_LOW     30.0f
#define SPEED_THRESHOLD_HIGH    70.0f

/* ===== ADXL Pre-kick 파라미터 ===== */
#define PREKICK_HP_THRESHOLD     (0.6f * 9.81f)  // 0.6G
#define PRE_KICK_MAGNITUDE      3.5f            // 6도
#define PRE_KICK_DECAY          0.85f           // 85% 유지

/* ===== 제어 한계값 ===== */
#define MAX_CORRECTION_DEG      70.0f           // PID 출력 최대 각도

/* ===== Servo follow speed (deg/sec) =====
 * 100Hz(10ms)에서 max_step = rate * 0.01
 * 예) 700deg/s -> 1주기 7deg 이동
 */
#define SERVO_RATE_FLAT_FAST      300.0f   // 큰 기울기/급변에서의 추종 속도
#define SERVO_RATE_SLOPE          300.0f   // 언덕에서(출렁 방지) 추종 속도

/* ===== LPF 파라미터 ===== */
#define MAX_TILT_ANGLE            70.0f

/* ===== Small-tilt 안정화(핵심) ===== */
#define SMALL_TILT_MIN      0.25f    // 이 아래는 거의 안 움직이게
#define SMALL_TILT_MAX       2.5f    // 여기부터 정상 gain(1.0)

#define LEVELING_SIGN_ROLL   (1.0f)
#define LEVELING_SIGN_PITCH  (1.0f)

#define LPF_CUTOFF_FREQ         1.5f

// ✅ IMU accel 신뢰도(1g 근처 여부) 기반 칼만 가중치 튜닝
#define ACC_MAG_NORM            (9.81f)
#define ACC_OK_BAND             (1.3f)   // |mag-9.81| < 1.3면 "신뢰"
#define ACC_SOFT_BAND           (3.0f)   // |mag-9.81| < 3.0면 "부분 신뢰"

// ✅ 칼만 R_measure 범위 (작을수록 accel를 더 믿음)
#define KALMAN_R_MIN            (0.10f)  // 기존 값
#define KALMAN_R_MAX            (5.00f)  // 흔들릴 때 accel 거의 무시

/* ===== 수평제어 OFF 테스트 모드 ===== */
#define INDEPENDENT_WHEEL_TEST_MODE   (1)

/* ===== Impact-based pre-kick tuning ===== */
#define PREKICK_GAIN_BASE        (2.0f)   // 기본 1.0
#define PREKICK_GAIN_MAX_ADD     (1.5f)   // impact=1일 때 추가 배수 (총 3.5배)
#define PREKICK_IMPACT_DEADBAND  (0.02f)  // 이 이하 impact는 무시
#define PREKICK_IMPACT_LPF       (0.60f)  // impact 필터 (0~1), 클수록 빠름

// ✅ 추가: 내부 목표 각도 범위(너 코드가 실제로 쓰는 clamp 범위)
#define SERVO_CMD_DEG_LIMIT   (35.0f)   // 지금 raw_target clamp에 맞춤


#define IMPACT_THRESHOLD_G (0.8f)
#define G_TO_MS2               (9.80665f)
#define IMPACT_THRESHOLD_MS2   (IMPACT_THRESHOLD_G * G_TO_MS2)
#define IMPACT_COOLDOWN_MS     (200U)   // 같은 충격 중복 감지 방지(필요시 300~500으로)

/* ===== Damping velocity estimator ===== */
#define DAMP_VEL_LEAK        (0.98f)   // 0.95~0.995 (클수록 오래 유지, 작을수록 빨리 감쇠)
#define DAMP_VEL_LIMIT_MS    (3.0f)    // 추정 속도 제한 (m/s)

#define IMPACT_PRINT_LOCK_MS   (150U)
#define IMPACT_LATCH_HOLD_MS   (400U)
#define IMPACT_PRINT_THRESH    (0.05f)

#define DAMPING_VEL_TO_DEG   (6.0f)   // 4~20 사이 튜닝

/* impact gate tuning */
#define IMP_DOM_THR        (0.18f)   // "충격 있다" 판정 (0~1)
#define IMP_COIN_THR       (0.10f)   // 동시충격 인정 임계
#define IMP_COIN_WIN_MS    (40U)     // 동시충격 윈도우(10ms 주기면 4tick 정도)
#define INHIBIT_MS_SINGLE  (120U)    // 단일휠일 때 다른휠 inhibit 시간
#define INHIBIT_MS_AXLE    (60U)     // 앞2개 같이 맞으면 뒤쪽 잠깐만 약화(선택)

// Define 추가 (Line 100)
#define SLOPE_ALPHA             0.05f
#define SLOPE_ANGLE_THRESHOLD   6.0f
#define SLOPE_VARIANCE_MAX      1.5f
#define SLOPE_WARMUP_SAMPLES    200

/* ===== 서보 중립 위치 ===== */
#define SERVO_NEUTRAL_DEG       0.0f    // 서보 중립 각도 (deg)

static volatile uint8  g_lead_axle = 0;   // 0:none, 1:front, 2:rear
static volatile uint32 g_lead_until_ms = 0;

/* ===== FAST BOOT ===== */
#define FAST_BOOT_MODE             1

#if FAST_BOOT_MODE
  #define SERVO_SETTLE_MS          (200U)   // 기존 1000ms → 200ms
  #define IMU_CAL_MS               (600U)   // 기존 3000ms → 600ms
  #define IMU_STAB_SAMPLES         (40U)    // 기존 200 → 40 (10ms 주기면 0.4s)
  #define ADXL_CAL_MS              (600U)   // 기존 3000ms → 600ms
#else
  #define SERVO_SETTLE_MS          (1000U)
  #define IMU_CAL_MS               (3000U)
  #define IMU_STAB_SAMPLES         (200U)
  #define ADXL_CAL_MS              (3000U)
#endif



/*
***************************************************************************************************
*                                         GLOBAL VARIABLES
***************************************************************************************************
*/
uint32                                  gALiveMsgOnOff;
static uint32                           gALiveCount;


// Task IDs
static uint32 gCANRxTaskID = 0;
static uint32 gMotorTaskID = 0;
static uint32 gIMUSuspTaskID = 0;
static uint32 gHeightTaskID = 0;
static uint32 gMonitorTaskID = 0;
static uint32 gADXL345TestTaskID = 0;
static uint32 gISOTaskID = 0;

// Task Stacks
static uint32 gCANRxTaskStk[CAN_RX_TASK_STK_SIZE];
static uint32 gMotorTaskStk[MOTOR_TASK_STK_SIZE];
static uint32 gIMUSuspTaskStk[IMU_SUSP_TASK_STK_SIZE];
static uint32 gHeightTaskStk[HEIGHT_TASK_STK_SIZE];
static uint32 gMonitorTaskStk[MONITOR_TASK_STK_SIZE];
static uint32 gADXL345TestTaskStk[ADXL_TEST_TASK_STK_SIZE];
static uint32 gISOTaskStk[ISO_TASK_STK_SIZE];


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
    float height_offset;
} Height_Data_t;



// Global shared variables
static IMU_Data_t g_IMUData;
static CAN_Data_t g_CANData;
static Motor_Data_t g_MotorData;
static Servo_Data_t g_ServoData;
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

typedef struct {
    float accel_x[4];
    float accel_y[4];
    float accel_z[4];
    float impact_detected[4];
    uint32 last_update_time;
} ADXL_Data_t;

static ADXL_Data_t g_ADXLData;

typedef struct {
    float leveling[4];
    float pid_roll_out_mm;
    float pid_pitch_out_mm;
    float soft_w;
    float rate_deg_s;
    float max_step;
    float roll_acc_deg;
    float pitch_acc_deg;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float pid_roll_integral;
    float pid_pitch_integral;
    float shock_absorb[4];  // ✅ 수정: pre_kick_deg + damp_out 대신 이것만
    float target_deg[4];
} SuspDbg_t;

static volatile SuspDbg_t g_SuspDbg;

static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;


typedef enum {
    WHEEL_FL = 0,
    WHEEL_FR = 1,
    WHEEL_RL = 2,
    WHEEL_RR = 3,
    WHEEL_MAX = 4
} WheelIdx_t;

static const char *g_wheel_name[WHEEL_MAX] = { "FL", "FR", "RL", "RR" };

/* dev(=mux채널/센서번호) -> wheel index 매핑
 * 네 테스트 기준(예시): dev0=RL, dev1=FL, dev2=RR, dev3=FR
 * (너가 실제로 쓰는 매핑이 이거 맞으면 그대로)
 */
static const uint8 g_ADXL_DEV_TO_WHEEL[ADXL_COUNT] = { WHEEL_RL, WHEEL_FL, WHEEL_RR, WHEEL_FR };


/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/

static void Main_StartTask(void *pArg);
static void CAN_RX_Task(void *pArg);
static void Motor_Control_Task(void *pArg);
static void IMU_Suspension_Task(void *pArg);
static void Height_Control_Task(void *pArg);
static void Monitoring_Task(void *pArg);
static void ADXL345_Monitor_Task(void *pArg);
static void ISO2631_Task(void *pArg);

// 댐핑 PID 함수들

static void Motor_SetSpeed(float speed);
static void AppTaskCreate(void);
static void DisplayAliveLog(void);
static void DisplayOTPInfo(void);


/*
***************************************************************************************************
*                                         PID FUNCTIONS
***************************************************************************************************
*/


static void Motor_SetSpeed(float speed) {
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


static inline float clamp01(float x) { return (x < 0.0f) ? 0.0f : ((x > 1.0f) ? 1.0f : x); }

static inline uint32 pct_to_pulse_ns(float pct, uint32 min_ns, uint32 max_ns)
{
    // pct: 0~100
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    float t = pct / 100.0f;
    return (uint32)((float)min_ns + ((float)(max_ns - min_ns) * t));
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
    
    // ✅ 추가: 장시간 카운터
    static uint32 small_error_cycles = 0;
    
    // 정지/소각도에서 적분 리셋
    if (!on_slope) {
        if (fabsf(error) < 3.0f) {  // 3mm = 약 1.4도
            pid->integral *= 0.50f;
            
            if (fabsf(pid->integral) < 0.5f) {
                pid->integral = 0.0f;
            }
        }
        
        // ✅ 추가: 2도 이하가 오래 지속되면 강제 감소
        if (fabsf(error) < 4.0f) {  // 2도 이하
            small_error_cycles++;
            
            // 10초 이상 (1000 cycles @ 100Hz)
            if (small_error_cycles > 1000) {
                pid->integral *= 0.95f;  // 천천히 감소
                
                if (fabsf(pid->integral) > 15.0f) {
                    mcu_printf("[WARN] Long-term integral high: ");
                    Print_Float_Value(pid->integral, 10);
                    mcu_printf(" | Forcing decay\n");
                    pid->integral *= 0.80f;
                }
            }
        } else {
            small_error_cycles = 0;  // 리셋
        }
    }

    // 충격 감지
    float error_change = fabsf(error - pid->prev_error);
    if (error_change > 8.0f) {
        pid->integral *= 0.2f;
        small_error_cycles = 0;
    }

    // 적분 활성화
    float i_enable = on_slope ? 3.0f : 5.0f;  // 4.0 → 5.0
    
    if (fabsf(error) > i_enable) {
        pid->integral += error * dt;
        pid->integral = clamp(pid->integral, -INTEGRAL_MAX, INTEGRAL_MAX);
        small_error_cycles = 0;
    } else {
        pid->integral *= 0.95f;
        if (fabsf(pid->integral) < 0.2f) {
            pid->integral = 0.0f;
        }
    }

    // ✅ 추가: 복귀 감지 (기울기 감소 중 + 적분 많이 쌓임)
    if (fabsf(error) < 10.0f && fabsf(pid->integral) > 15.0f) {
        // 에러 감소 중인데 적분이 많이 남아있으면
        float error_rate = (pid->prev_error - error) / dt;
        
        // 복귀 중이면 (에러가 빠르게 줄어들면)
        if (fabsf(error_rate) > 50.0f) {
            pid->integral *= 0.70f;  // 적분 30% 감소
        }
    }

    // 오버슈트 감지
    if ((pid->prev_error >  2.0f && error < -2.0f) ||
        (pid->prev_error < -2.0f && error >  2.0f)) {
        pid->integral *= 0.3f;
        small_error_cycles = 0;
    }

    // 급격한 개선
    float error_rate = (pid->prev_error - error) / dt;
    if (fabsf(error_rate) > 100.0f && fabsf(pid->integral) > 2.0f) {
        pid->integral *= 0.5f;
        small_error_cycles = 0;
    }

    // ✅ 적분 상한 경고
    if (fabsf(pid->integral) > INTEGRAL_MAX * 0.8f) {
        mcu_printf("[WARN] PID integral high: ");
        Print_Float_Value(pid->integral, 10);
        mcu_printf("\n");
        pid->integral *= 0.75f;
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

/* =========================================================
 * Anti-windup (back-calculation)
 * - u_cmd : controller output before clamp
 * - u_lim : saturation limit (abs)
 * - dt    : seconds
 * ========================================================= */
static inline void awu_backcalc(PID_Leveling_t *pid, float u_cmd, float u_lim, float dt)
{
    /* u_cmd는 clamp 전 값이 들어와야 함.
       너는 지금 roll_ctrl_deg/pitch_ctrl_deg를 clamp 후에 넘기고 있으니,
       호출부도 아래 “2) 호출 위치”대로 바꿔야 효과가 맞음. */

    if (!pid) return;
    if (u_lim <= 0.0f) return;

    /* saturation 적용 */
    float u_sat = clamp(u_cmd, -u_lim, u_lim);

    /* 포화 차이 */
    float e_sat = (u_sat - u_cmd);

    /* back-calc gain: 너무 크면 튐, 너무 작으면 효과 없음 */
    const float Kb = 6.0f;   // 3~10 튜닝

    /* 적분항 보정 (너 PID가 integral을 output에 더하는 구조라는 가정) */
    pid->integral += (Kb * e_sat) * dt;
}

static inline float apply_deadband(float x, float db)
{
    if (fabsf(x) <= db) return 0.0f;
    return (x > 0) ? (x - db) : (x + db);
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
    memset(&g_HeightData, 0, sizeof(g_HeightData));
    memset(&g_ADXLData, 0, sizeof(g_ADXLData));

    g_CANData.suspension_enable = 1;
    g_CANData.leveling_enable = 1;

    // ✅ 서보 초기값 설정
    for(uint8 i = 0; i < 4; i++){
        g_ServoData.position[i] = (float)NS_TO_US(SERVO_NEUTRAL_PULSE_NS);
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

    // ✅ 1. 서보 먼저 중립으로 (모든 태스크 생성 전)
    mcu_printf("[SERVO] Initializing to neutral...\n");
    uint32 neutral_pulse[4] = {
        SERVO_NEUTRAL_PULSE_NS, SERVO_NEUTRAL_PULSE_NS,
        SERVO_NEUTRAL_PULSE_NS, SERVO_NEUTRAL_PULSE_NS
    };
    
    Servo_Init();
    Servo_SetPulseAllNs(neutral_pulse);
    
    SAL_CoreCriticalEnter();
    for(uint8 i = 0; i < 4; i++){
        g_ServoData.position[i] = (float)NS_TO_US(SERVO_NEUTRAL_PULSE_NS);
    }
    SAL_CoreCriticalExit();
    
    mcu_printf("[SERVO] Stabilizing...\n");
    SAL_TaskSleep(SERVO_SETTLE_MS);


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

    //      //Task 6:ADXL345_Monitor
    // err = SAL_TaskCreate(&gADXL345TestTaskID,
    //                     (const uint8 *)"ADXL345_Monito_Test",
    //                     (SALTaskFunc)&ADXL345_Monitor_Task, // 수정한 테스트 함수
    //                     &gADXL345TestTaskStk[0],
    //                     ADXL_TEST_TASK_STK_SIZE,
    //                     ADXL_MONITOR_TASK_PRIO,
    //                     NULL);



    // if(err == SAL_RET_SUCCESS) {
    //     mcu_printf("[SYSTEM] ADXL345 Test Task Created Successfully\n");
    // } else {
    //     mcu_printf("[ERROR] Failed to Create ADXL345 Test Task\n");
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
    // err = SAL_TaskCreate(&gMonitorTaskID,
    //                     (const uint8 *)"Monitoring",
    //                     (SALTaskFunc)&Monitoring_Task,
    //                     &gMonitorTaskStk[0],
    //                     MONITOR_TASK_STK_SIZE,
    //                     MONITOR_TASK_PRIO,
    //                     NULL);
    // if(err == SAL_RET_SUCCESS) {
    //     mcu_printf("[SYSTEM] Monitor Task Created (Priority: %d, 10Hz)\n\n", MONITOR_TASK_PRIO);
    // }

    // Task X: ISO2631 (100Hz)
err = SAL_TaskCreate(&gISOTaskID,
                    (const uint8 *)"ISO2631",
                    (SALTaskFunc)&ISO2631_Task,
                    &gISOTaskStk[0],
                    ISO_TASK_STK_SIZE,
                    ISO_TASK_PRIO,
                    NULL);

if(err == SAL_RET_SUCCESS) {
    mcu_printf("[SYSTEM] ISO2631 Task Created (Priority: %d, 100Hz)\n", ISO_TASK_PRIO);
} else {
    mcu_printf("[ERROR] Failed to Create ISO2631 Task\n");
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
        
        // Read CAN data (Safety 제거)
        SAL_CoreCriticalEnter();
        float target_speed = g_CANData.motor_speed_cmd;
        SAL_CoreCriticalExit();
        
        // 최대 속도는 항상 100%
        float max_speed = 100.0f;
        
        // Limit speed
        if(target_speed > max_speed) {
            target_speed = max_speed;
        }
        
        // Ramping (항상 빠른 응답)
        float ramp_rate = 0.1f;  // Safety에 따른 조건 제거
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
    
    // ========== IMU 캘리브 변수 ==========
    double gx_sum = 0.0, gy_sum = 0.0, gz_sum = 0.0;
    double icm_roll_sum = 0.0, icm_pitch_sum = 0.0;
    static uint32 imu_sample_count = 0;
    static uint8 imu_calibration_done = 0;
    uint32 imu_calib_samples = 0;
    const uint32 IMU_CAL_SAMPLES = (IMU_CAL_MS / IMU_SUSP_PERIOD_MS);
    const uint32 STABILIZATION_SAMPLES = IMU_STAB_SAMPLES;
    
    // ========== ADXL 캘리브 변수 ==========
    double adxl_sum_x[4] = {0}, adxl_sum_y[4] = {0}, adxl_sum_z[4] = {0};
    static uint8 adxl_calibration_done = 0;
    uint32 adxl_calib_samples = 0;
    const uint32 ADXL_CAL_SAMPLES = (ADXL_CAL_MS / IMU_SUSP_PERIOD_MS);
    
    // ========== 서보 ==========
    uint32 servo_pulse_ns[4] = {
        SERVO_NEUTRAL_PULSE_NS, SERVO_NEUTRAL_PULSE_NS,
        SERVO_NEUTRAL_PULSE_NS, SERVO_NEUTRAL_PULSE_NS
    };
    
    // ========== 제어 변수 ==========
    static float wheel_lp_z[4] = {0};
    static float shock_absorb[4] = {0};
    static float impact_filt[4] = {0};
    static float servo_current_deg[4] = {0, 0, 0, 0};
    
    // ========== ADXL impact 감지용 ==========
    static float adxl_prev_mag[4] = {0};
    static uint32 adxl_last_hit_ms[4] = {0};
    static float adxl_impact_hold[4] = {0};
    
    // ========== 경사 감지 ==========
    static float avg_roll = 0.0f;
    static float avg_pitch = 0.0f;
    static uint32 slope_sample_count = 0;
    
    IMU_Data imuRaw;

    mcu_printf("[IMU_SUSP] Task Started (ADXL integrated)\n");
    mcu_printf("  - Leveling: STM32 PID (geometric)\n");
    mcu_printf("  - Damping: ADXL per-wheel (integrated)\n");
    mcu_printf("  - Servo follow: deg/sec rate limiting\n\n");
    
   // ========== ADXL 센서 초기화 (테스트 코드 방식) ==========
    mcu_printf("[ADXL] Initializing sensors (verified writes)...\n");
    
    for (uint8 dev = 0; dev < ADXL_COUNT; dev++) {
        uint8 reg_before = 0, reg_after = 0;
        
        // ✅ 1. 현재 상태 확인
        if (ADXL_MuxSelectByDev(dev) != SAL_RET_SUCCESS) {
            mcu_printf("  [WARN] ADXL%d MUX select failed\n", dev);
            continue;
        }
        
        // ✅ 2. DATA_FORMAT 현재값 읽기
        (void)ADXL345_ReadReg(dev, ADXL345_REG_DATA_FORMAT, &reg_before);
        
        // ✅ 3. Write + Verify (테스트 코드 방식)
        // BW_RATE = 0x0A (100Hz)
        for (uint8 retry = 0; retry < 3; retry++) {
            (void)ADXL345_WriteReg(dev, ADXL345_REG_BW_RATE, 0x0A);
            SAL_TaskSleep(10);
            
            uint8 readback = 0;
            if (ADXL345_ReadReg(dev, ADXL345_REG_BW_RATE, &readback) == SAL_RET_SUCCESS) {
                if (readback == 0x0A) break;
            }
            SAL_TaskSleep(5);
        }
        
        // DATA_FORMAT = 0x09 (±4g, Full Resolution)
        for (uint8 retry = 0; retry < 3; retry++) {
            (void)ADXL345_WriteReg(dev, ADXL345_REG_DATA_FORMAT, 0x09);
            SAL_TaskSleep(10);
            
            uint8 readback = 0;
            if (ADXL345_ReadReg(dev, ADXL345_REG_DATA_FORMAT, &readback) == SAL_RET_SUCCESS) {
                if (readback == 0x09) break;
            }
            SAL_TaskSleep(5);
        }
        
        // POWER_CTL = 0x08 (Measurement mode)
        for (uint8 retry = 0; retry < 3; retry++) {
            (void)ADXL345_WriteReg(dev, ADXL345_REG_POWER_CTL, 0x08);
            SAL_TaskSleep(20);
            
            uint8 readback = 0;
            if (ADXL345_ReadReg(dev, ADXL345_REG_POWER_CTL, &readback) == SAL_RET_SUCCESS) {
                if (readback == 0x08) break;
            }
            SAL_TaskSleep(5);
        }
        
        // ✅ 4. 최종 확인
        (void)ADXL345_ReadReg(dev, ADXL345_REG_DATA_FORMAT, &reg_after);
        
        mcu_printf("  [ADXL%d] DATA_FORMAT: 0x%02X -> 0x%02X (exp 0x09)\n",
                   dev, reg_before, reg_after);
    }
    
    mcu_printf("[ADXL] Sensors configured\n\n");
    
    for (uint8 i = 0; i < 4; i++) {
        servo_current_deg[i] = 0.0f;
        adxl_prev_mag[i] = 0.0f;
        adxl_last_hit_ms[i] = 0;
        adxl_impact_hold[i] = 0.0f;
    }

    SAL_CoreCriticalEnter();
    for (uint8 i = 0; i < 4; i++) {
        g_ServoData.position[i] = (float)NS_TO_US(servo_pulse_ns[i]);
    }
    g_IMU_SUSP_INIT_DONE = 1;
    SAL_CoreCriticalExit();

    mcu_printf("[SERVO] Waiting for servo stabilization...\n");
    SAL_TaskSleep(1000);
    
    mcu_printf("[CALIB] Starting calibration (IMU + ADXL)...\n");
    mcu_printf("        Keep the vehicle still and level!\n\n");

    while (1) {
        SAL_GetTickCount(&start_tick);

        // ========== 1. IMU 읽기 ==========
        if (IMU_Read_Data_DMA() != SAL_RET_SUCCESS) {
            SAL_TaskSleep(IMU_SUSP_PERIOD_MS);
            continue;
        }
        imuRaw = IMU;

        // ========== 2. 캘리브레이션 단계 ==========
        
        /* ===== Phase 1: IMU 캘리브 ===== */
        if (!imu_calibration_done) {
            for (uint8 i = 0; i < 4; i++) {
                servo_pulse_ns[i] = SERVO_NEUTRAL_PULSE_NS;
            }
            Servo_SetPulseAllNs(servo_pulse_ns);

            gx_sum += (double)imuRaw.gyro_x;
            gy_sum += (double)imuRaw.gyro_y;
            gz_sum += (double)imuRaw.gyro_z;

            float r_acc = atan2f(imuRaw.accel_y, imuRaw.accel_z) * 180.0f / M_PI;
            float p_acc = atan2f(-imuRaw.accel_x,
                                sqrtf(imuRaw.accel_y * imuRaw.accel_y +
                                    imuRaw.accel_z * imuRaw.accel_z)) * 180.0f / M_PI;

            icm_roll_sum  += (double)r_acc;
            icm_pitch_sum += (double)p_acc;
            imu_calib_samples++;

            if ((imu_calib_samples % 50) == 0) {
                mcu_printf("  [IMU %d/%d] R/P: ",
                           (int)imu_calib_samples, (int)IMU_CAL_SAMPLES);
                Print_Float_Value(r_acc, 10);
                mcu_printf("/");
                Print_Float_Value(p_acc, 10);
                mcu_printf("\n");
            }

            if (imu_calib_samples >= IMU_CAL_SAMPLES) {
                roll_offset  = (float)(icm_roll_sum  / (double)IMU_CAL_SAMPLES);
                pitch_offset = (float)(icm_pitch_sum / (double)IMU_CAL_SAMPLES);
                gyro_bias_x = (float)(gx_sum / (double)IMU_CAL_SAMPLES);
                gyro_bias_y = (float)(gy_sum / (double)IMU_CAL_SAMPLES);
                gyro_bias_z = (float)(gz_sum / (double)IMU_CAL_SAMPLES);

                mcu_printf("\n[CALIB] IMU complete!\n");
                mcu_printf("  Roll offset: ");
                Print_Float_Value(roll_offset, 100);
                mcu_printf(" | Pitch offset: ");
                Print_Float_Value(pitch_offset, 100);
                mcu_printf("\n");
                
                imu_calibration_done = 1;
                mcu_printf("\n[CALIB] Starting ADXL calibration...\n");
            }

            SAL_CoreCriticalEnter();
            g_IMUData.roll  = 0.0f;
            g_IMUData.pitch = 0.0f;
            SAL_CoreCriticalExit();

            SAL_TaskSleep(IMU_SUSP_PERIOD_MS);
            continue;
        }

        /* ===== Phase 2: ADXL 캘리브 ===== */
        if (!adxl_calibration_done) {
            for (uint8 i = 0; i < 4; i++) {
                servo_pulse_ns[i] = SERVO_NEUTRAL_PULSE_NS;
            }
            Servo_SetPulseAllNs(servo_pulse_ns);

            for (uint8 dev = 0; dev < ADXL_COUNT; dev++) {
                float x, y, z;
                uint8 wheel = g_ADXL_DEV_TO_WHEEL[dev];

                if (ADXL_MuxSelectByDev(dev) != SAL_RET_SUCCESS) continue;
                
                if (ADXL345_ReadAccelCalibrated(dev, &x, &y, &z) == SAL_RET_SUCCESS) {
                    adxl_sum_x[wheel] += (double)x;
                    adxl_sum_y[wheel] += (double)y;
                    adxl_sum_z[wheel] += (double)z;
                }
            }
            adxl_calib_samples++;

            if ((adxl_calib_samples % 50) == 0) {
                float avg_z[4];
                for (uint8 w = 0; w < 4; w++) {
                    avg_z[w] = (float)(adxl_sum_z[w] / (double)adxl_calib_samples);
                }
                
                mcu_printf("  [ADXL %d/%d] Z: ",
                           (int)adxl_calib_samples, (int)ADXL_CAL_SAMPLES);
                Print_Float_Value(avg_z[0], 10); mcu_printf("/");
                Print_Float_Value(avg_z[1], 10); mcu_printf("/");
                Print_Float_Value(avg_z[2], 10); mcu_printf("/");
                Print_Float_Value(avg_z[3], 10);
                mcu_printf(" m/s2\n");
            }

            if (adxl_calib_samples >= ADXL_CAL_SAMPLES) {
                for (uint8 w = 0; w < 4; w++) {
                    g_ADXL_bias_x[w] = (float)(adxl_sum_x[w] / (double)ADXL_CAL_SAMPLES);
                    g_ADXL_bias_y[w] = (float)(adxl_sum_y[w] / (double)ADXL_CAL_SAMPLES);
                    g_ADXL_bias_z[w] = (float)(adxl_sum_z[w] / (double)ADXL_CAL_SAMPLES);
                }

                SAL_CoreCriticalEnter();
                g_ADXL_CAL_DONE = 1;
                SAL_CoreCriticalExit();

                adxl_calibration_done = 1;
                imu_sample_count = 0;
                
                mcu_printf("\n[CALIB] ADXL complete!\n");
                mcu_printf("[STABIL] Starting stabilization...\n\n");
            }

            SAL_CoreCriticalEnter();
            g_IMUData.roll  = 0.0f;
            g_IMUData.pitch = 0.0f;
            SAL_CoreCriticalExit();

            SAL_TaskSleep(IMU_SUSP_PERIOD_MS);
            continue;
        }

        // ========== 3. Kalman 필터링 ==========
        imu_sample_count++;
        
        float gx = imuRaw.gyro_x - gyro_bias_x;
        float gy = imuRaw.gyro_y - gyro_bias_y;

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
    
        float R;
        if (acc_err <= ACC_OK_BAND) {
            R = KALMAN_R_MIN;
        } else if (acc_err <= ACC_SOFT_BAND) {
            float t = (acc_err - ACC_OK_BAND) / (ACC_SOFT_BAND - ACC_OK_BAND);
            R = KALMAN_R_MIN + t * (KALMAN_R_MAX - KALMAN_R_MIN);
        } else {
            R = KALMAN_R_MAX;
        }

        Kalman_SetR(&kalman_roll,  R);
        Kalman_SetR(&kalman_pitch, R);

        uint8 acc_ok = (acc_err <= ACC_SOFT_BAND) ? 1U : 0U;

        float roll, pitch;
        if (acc_ok) {
            roll  = Kalman_Update(&kalman_roll,  roll_acc,  gx, dt);
            pitch = Kalman_Update(&kalman_pitch, pitch_acc, gy, dt);
        } else {
            roll  = Kalman_PredictOnly(&kalman_roll,  gx, dt);
            pitch = Kalman_PredictOnly(&kalman_pitch, gy, dt);
        }

        SAL_CoreCriticalEnter();
        g_IMUData.roll  = clamp(roll,  -70.0f, 70.0f);
        g_IMUData.pitch = clamp(pitch, -70.0f, 70.0f);
        g_IMUData.gyro_x = imuRaw.gyro_x;
        g_IMUData.gyro_y = imuRaw.gyro_y;
        g_IMUData.gyro_z = imuRaw.gyro_z;
        g_IMUData.accel_x = imuRaw.accel_x;
        g_IMUData.accel_y = imuRaw.accel_y;
        g_IMUData.accel_z = imuRaw.accel_z;
        SAL_GetTickCount(&g_IMUData.last_update_time);
        SAL_CoreCriticalExit();

        // ========== 3-1. 안정화 대기 ==========
        if (imu_sample_count < STABILIZATION_SAMPLES) {
            if (imu_sample_count % 50 == 0) {
                mcu_printf("  [STABIL] %d/%d | Roll: ",
                        (int)imu_sample_count, (int)STABILIZATION_SAMPLES);
                Print_Float_Value(roll, 10);
                mcu_printf(" Pitch: ");
                Print_Float_Value(pitch, 10);
                mcu_printf("\n");
            }
            Servo_SetPulseAllNs(servo_pulse_ns);
            SAL_TaskSleep(IMU_SUSP_PERIOD_MS);
            continue;
        }

        if (imu_sample_count == STABILIZATION_SAMPLES) {
            mcu_printf("\n[STABIL] Complete! Control starting...\n\n");

            SAL_CoreCriticalEnter();
            g_SYSTEM_READY = 1;
            SAL_CoreCriticalExit();
        }

        // ========== 4. ADXL 읽기 (2개씩 교대) ==========
        
        static uint8 adxl_batch = 0;  // 0 or 1
        
        uint32 now_ms;
        SAL_GetTickCount(&now_ms);
        
        float wheel_accel_z[4];
        
        // 짝수 주기: 센서 0, 1 / 홀수 주기: 센서 2, 3
        uint8 start_dev = adxl_batch * 2;
        uint8 end_dev = start_dev + 2;
        
        for (uint8 dev = start_dev; dev < end_dev; dev++) {
            float x, y, z;
            uint8 wheel = g_ADXL_DEV_TO_WHEEL[dev];

            if (ADXL_MuxSelectByDev(dev) != SAL_RET_SUCCESS) {
                wheel_accel_z[wheel] = 0.0f;
                continue;
            }

            SAL_TaskSleep(2);  // 2ms

            SALRetCode_t ret = ADXL345_ReadAccelCalibrated(dev, &x, &y, &z);

            if (ret == SAL_RET_SUCCESS) {
                x -= g_ADXL_bias_x[wheel];
                y -= g_ADXL_bias_y[wheel];
                z -= g_ADXL_bias_z[wheel];

                wheel_accel_z[wheel] = z;

                // Impact 감지 (동일)
                float mag   = sqrtf(x*x + y*y + z*z);
                float delta = fabsf(mag - adxl_prev_mag[wheel]);
                adxl_prev_mag[wheel] = mag;

                float impact = 0.0f;
                if (delta >= IMPACT_THRESHOLD_MS2) {
                    if ((now_ms - adxl_last_hit_ms[wheel]) >= IMPACT_COOLDOWN_MS) {
                        adxl_last_hit_ms[wheel] = now_ms;
                        impact = (delta / IMPACT_THRESHOLD_MS2) - 1.0f;
                        impact = clamp(impact, 0.0f, 1.0f);
                    }
                }

                adxl_impact_hold[wheel] *= 0.85f;
                if (impact > adxl_impact_hold[wheel]) {
                    adxl_impact_hold[wheel] = impact;
                }
                impact = adxl_impact_hold[wheel];

                SAL_CoreCriticalEnter();
                g_ADXLData.accel_x[wheel] = x;
                g_ADXLData.accel_y[wheel] = y;
                g_ADXLData.accel_z[wheel] = z;
                g_ADXLData.impact_detected[wheel] = impact;
                SAL_GetTickCount(&g_ADXLData.last_update_time);
                SAL_CoreCriticalExit();
                
            } else {
                SAL_CoreCriticalEnter();
                g_ADXLData.impact_detected[wheel] *= 0.85f;
                SAL_CoreCriticalExit();
            }
            
            SAL_TaskSleep(2);  // 2ms
        }
        
        // ✅ 다음 주기에는 다른 배치
        adxl_batch = 1 - adxl_batch;  // 0 ↔ 1 토글

        // ========== 5. 경사 감지 ==========
        
        const float alpha_slope = SLOPE_ALPHA;
        avg_roll = alpha_slope * roll + (1.0f - alpha_slope) * avg_roll;
        avg_pitch = alpha_slope * pitch + (1.0f - alpha_slope) * avg_pitch;
        slope_sample_count++;

        uint8 on_slope = 0;
        if (slope_sample_count > SLOPE_WARMUP_SAMPLES) {
            float roll_diff = fabsf(roll - avg_roll);
            float pitch_diff = fabsf(pitch - avg_pitch);
            
            on_slope = ((fabsf(avg_roll) > SLOPE_ANGLE_THRESHOLD && roll_diff < SLOPE_VARIANCE_MAX) ||
                        (fabsf(avg_pitch) > SLOPE_ANGLE_THRESHOLD && pitch_diff < SLOPE_VARIANCE_MAX));
        }

        // ========== 6. 제어 파라미터 ==========
        
        float total_tilt = fabsf(roll) + fabsf(pitch);
        
        static float tilt_hold = 0.0f;
        tilt_hold = 0.98f * tilt_hold + 0.02f * total_tilt;
        uint8 pseudo_slope = (tilt_hold > 3.0f) ? 1U : 0U;
        
        uint8 slope_like = (on_slope || pseudo_slope) ? 1U : 0U;
        
        float soft_w = 0.0f;
        if (total_tilt <= SMALL_TILT_MIN) soft_w = 0.0f;
        else if (total_tilt >= SMALL_TILT_MAX) soft_w = 1.0f;
        else soft_w = (total_tilt - SMALL_TILT_MIN) / (SMALL_TILT_MAX - SMALL_TILT_MIN);

        // ========== 7. ADXL 댐핑 (HPF 기반) ==========
        
        float tau   = 1.0f / (2.0f * M_PI * LPF_CUTOFF_FREQ);
        float alpha = dt / (tau + dt);

        float shock_raw[4];
        float shock_abs_[4];

        uint8 top1 = 0;
        float a1 = 0.0f;

        const float SHOCK_DEADBAND = 15.0f;

        /* HPF 계산 + top1 찾기 */
        for (uint8 i = 0; i < 4; i++) {
            float z = wheel_accel_z[i];

            wheel_lp_z[i] += alpha * (z - wheel_lp_z[i]);

            float s = z - wheel_lp_z[i];
            if (fabsf(s) < SHOCK_DEADBAND) s = 0.0f;
            shock_raw[i]  = s;
            shock_abs_[i] = fabsf(s);
            if (shock_abs_[i] > a1) {
                a1 = shock_abs_[i];
                top1 = i;
            }
        }

        /* top1 축 결정 */
        uint8 lead_axle = 0;
        if (a1 > 0.0f) {
            lead_axle = (top1 == WHEEL_FL || top1 == WHEEL_FR) ? 1U : 2U;
        }

        /* 같은 축에서 top2 찾기 */
        uint8 top2 = top1;
        float a2 = 0.0f;

        if (lead_axle == 1U) {
            uint8 a = WHEEL_FL, b = WHEEL_FR;
            top2 = (top1 == a) ? b : a;
            a2 = shock_abs_[top2];
        } else if (lead_axle == 2U) {
            uint8 a = WHEEL_RL, b = WHEEL_RR;
            top2 = (top1 == a) ? b : a;
            a2 = shock_abs_[top2];
        }

        /* 동시 충격 인정 */
        uint8 allow_any = (lead_axle != 0U) ? 1U : 0U;
        uint8 allow2 = 0U;
        if (allow_any) {
            allow2 = (a2 >= a1 * 0.70f) ? 1U : 0U;
        }

        /* 댐핑 계산 */
        for (uint8 i = 0; i < 4; i++) {
            uint8 selected = 0U;
            if (allow_any) {
                if (i == top1) selected = 1U;
                else if (allow2 && (i == top2)) selected = 1U;
            }

            if (!selected) {
                shock_absorb[i] = 0.0f;
                continue;
            }

            float shock = shock_raw[i];
            float mag = fabsf(shock);
            
            const float DAMP_GAIN = 5.0f;
            float response = -mag * DAMP_GAIN;
            
            shock_absorb[i] *= 0.80f;
            if (fabsf(response) > fabsf(shock_absorb[i])) {
                shock_absorb[i] = response;
            }
            if (fabsf(shock_absorb[i]) > 20.0f) {
                shock_absorb[i] *= 0.50f;
            }
            if (fabsf(shock_absorb[i]) < 0.5f) {
                shock_absorb[i] = 0.0f;
            }
            shock_absorb[i] = clamp(shock_absorb[i], -30.0f, 30.0f);
        }

        // ========== 8. 레벨링 PID ==========
        
        float leveling[4] = {0, 0, 0, 0};

        #if INDEPENDENT_WHEEL_TEST_MODE
        /* 테스트 모드: 레벨링 OFF */
        reset_leveling_pid(&pid_roll);
        reset_leveling_pid(&pid_pitch);
        #else
        /* 정상 레벨링 */
        SAL_CoreCriticalEnter();
        uint8 leveling_on = g_CANData.leveling_enable;
        SAL_CoreCriticalExit();

        uint8 slope_state = slope_like;

        if (leveling_on) {
            float boost = 1.0f;
            if (total_tilt > 12.0f) boost = 2.0f;
            else if (total_tilt > 6.0f) boost = 1.6f;
            else if (total_tilt > 3.0f) boost = 1.3f;

            float leveling_gain_scale;
            if (slope_like) {
                leveling_gain_scale = 1.0f;
            } else {
                leveling_gain_scale = (0.6f + 0.4f * soft_w);
            }

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

            float osc_damping = 1.0f;
            if (!on_slope && osc_count > 0) {
                osc_damping = 0.4f;
            }

            float roll_for_level  = slope_state ? avg_roll  : roll;
            float pitch_for_level = slope_state ? avg_pitch : pitch;

            float roll_height_mm  = angle_to_height_mm(roll_for_level,  TRACK_WIDTH_MM * 0.5f);
            float pitch_height_mm = angle_to_height_mm(pitch_for_level, WHEELBASE_MM   * 0.5f);

            roll_height_mm  = apply_deadband(roll_height_mm,  3.0f);
            pitch_height_mm = apply_deadband(pitch_height_mm, 3.0f);

            if (fabsf(roll_for_level) < 0.2f) roll_height_mm = 0.0f;
            if (fabsf(pitch_for_level) < 0.2f) pitch_height_mm = 0.0f;

            compute_leveling_pid(&pid_roll,  LEVELING_SIGN_ROLL  * roll_height_mm,  dt, slope_state);
            compute_leveling_pid(&pid_pitch, LEVELING_SIGN_PITCH * pitch_height_mm, dt, slope_state);
                        
            float roll_ctrl_mm  = pid_roll.output  * boost * leveling_gain_scale * osc_damping;
            float pitch_ctrl_mm = pid_pitch.output * boost * leveling_gain_scale * osc_damping;

            float roll_ctrl_deg  = height_to_servo_deg(roll_ctrl_mm,  MAX_CORRECTION_DEG);
            float pitch_ctrl_deg = height_to_servo_deg(pitch_ctrl_mm, MAX_CORRECTION_DEG);

            float max_deg = MAX_CORRECTION_DEG;

            float roll_cmd_deg_unc  = roll_ctrl_deg;
            float pitch_cmd_deg_unc = pitch_ctrl_deg;

            roll_ctrl_deg  = clamp(roll_ctrl_deg,  -max_deg, max_deg);
            pitch_ctrl_deg = clamp(pitch_ctrl_deg, -max_deg, max_deg);
            
            awu_backcalc(&pid_roll,  roll_cmd_deg_unc,  max_deg, dt);
            awu_backcalc(&pid_pitch, pitch_cmd_deg_unc, max_deg, dt);

            leveling[WHEEL_FL] = -roll_ctrl_deg + pitch_ctrl_deg;
            leveling[WHEEL_FR] = +roll_ctrl_deg + pitch_ctrl_deg;
            leveling[WHEEL_RL] = -roll_ctrl_deg - pitch_ctrl_deg;
            leveling[WHEEL_RR] = +roll_ctrl_deg - pitch_ctrl_deg;

            for (uint8 i = 0; i < 4; i++) {
                leveling[i] = clamp(leveling[i], -max_deg, max_deg);
            }
            
            static uint8 servo_saturated_count = 0;
            uint8 any_saturated = 0;
            
            for (uint8 i = 0; i < 4; i++) {
                if (fabsf(leveling[i]) > max_deg * 0.90f) {
                    any_saturated = 1;
                    break;
                }
            }
            
            if (any_saturated) {
                servo_saturated_count++;
                if (servo_saturated_count > 30) {
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
        #endif

        // ========== 9. 통합 제어 ==========
        
        SAL_CoreCriticalEnter();
        float height = g_HeightData.height_offset;
        SAL_CoreCriticalExit();

        float target_deg[4];

        for (uint8 i = 0; i < 4; i++) {
            float raw_target = height + leveling[i] + shock_absorb[i];
            float max_offset = SERVO_CMD_DEG_LIMIT;
            raw_target = clamp(raw_target, -max_offset, max_offset);
            target_deg[i] = raw_target;
        }

        total_tilt = fabsf(roll) + fabsf(pitch);

        float rate_deg_s;
        if (slope_like) {
            rate_deg_s = SERVO_RATE_SLOPE;
        } else {
            float rate_min = 120.0f;
            float rate_max = SERVO_RATE_FLAT_FAST;
            rate_deg_s = rate_min + (rate_max - rate_min) * soft_w;
        }

        float dt_s = (float)IMU_SUSP_PERIOD_MS / 1000.0f;
        float max_step = rate_deg_s * dt_s;

        for (uint8 i=0;i<4;i++){
            float local_step = max_step;
            if (!on_slope && impact_filt[i] > 0.10f) {
                float extra = 1.5f * impact_filt[i];
                local_step = clamp(max_step + extra, 0.5f, 3.0f);
            }
            float gap = target_deg[i] - servo_current_deg[i];
            float step = clamp(gap, -local_step, local_step);
            servo_current_deg[i] += step;
            target_deg[i] = servo_current_deg[i];
        }

        // ========== 10. Debug 스냅샷 ==========
        
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

        for (uint8 i = 0; i < 4; i++) {
            g_SuspDbg.shock_absorb[i] = shock_absorb[i];
            g_SuspDbg.target_deg[i]   = target_deg[i];
        }
        SAL_CoreCriticalExit();

        // ========== 11. 서보 출력 ==========
        
        for (uint8 i = 0; i < 4; i++) {
            float pct = 50.0f + (target_deg[i] / SERVO_CMD_DEG_LIMIT) * 50.0f;
            servo_pulse_ns[i] = pct_to_pulse_ns(pct, SERVO_MIN_PULSE_NS, SERVO_MAX_PULSE_NS);
        }
        Servo_SetPulseAllNs(servo_pulse_ns);

        SAL_CoreCriticalEnter();
        for (uint8 i = 0; i < 4; i++) {
            g_ServoData.position[i] = (float)NS_TO_US(servo_pulse_ns[i]);
        }
        SAL_CoreCriticalExit();

        // ========== 12. Sleep ==========
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
void Monitoring_Task(void *pArg)
{
    (void)pArg;
    SuspDbg_t dbg;
    uint32 now = 0;

    /* ===== Impact event latch (10Hz에서도 안 놓치게) ===== */
    static uint32 last_hit_ms[WHEEL_MAX]    = {0,0,0,0};
    static float  last_hit_val[WHEEL_MAX]   = {0,0,0,0};
    static uint8  last_hit_valid[WHEEL_MAX] = {0,0,0,0};

    /* "이벤트 한 줄"이 너무 도배되는 것 방지용 */
    static uint32 last_print_ms[WHEEL_MAX]  = {0,0,0,0};

    // ✅ 초기화 대기
    while (1) {
        uint8 ready;
        SAL_CoreCriticalEnter();
        ready = g_IMU_SUSP_INIT_DONE;
        SAL_CoreCriticalExit();

        if (ready != 0U) break;
        SAL_TaskSleep(50);
    }

    /* ✅ "3초 중립 캘리브 + 안정화" 끝날 때까지 대기 (여기까지는 로그 출력 X) */
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

    while (1) {

        /* ===== 스냅샷 ===== */
        float roll, pitch, speed;
        float wheel_z[WHEEL_MAX];
        float servo[WHEEL_MAX];
        uint32 adxl_t;
        float impact[WHEEL_MAX];
        float ax[WHEEL_MAX], ay[WHEEL_MAX]; /* (선택) one-line 이벤트에 쓰기 위해 */

        SAL_CoreCriticalEnter();
        roll     = g_IMUData.roll;
        pitch    = g_IMUData.pitch;
        speed    = g_MotorData.current_speed;

        for (uint8 i = 0; i < WHEEL_MAX; i++) {
            wheel_z[i] = g_ADXLData.accel_z[i];
            ax[i]      = g_ADXLData.accel_x[i];
            ay[i]      = g_ADXLData.accel_y[i];
            impact[i]  = g_ADXLData.impact_detected[i];
            servo[i]   = g_ServoData.position[i];
        }

        adxl_t = g_ADXLData.last_update_time;
        dbg = g_SuspDbg;
        SAL_CoreCriticalExit();

        SAL_GetTickCount(&now);

        /* =========================================================
         * 1) IMPACT 이벤트가 뜨면 "한 줄 로그" 먼저 출력 (핵심)
         * ========================================================= */
        for (uint8 i = 0; i < WHEEL_MAX; i++) {
            if (impact[i] >= IMPACT_PRINT_THRESH) {
                if ((now - last_print_ms[i]) >= IMPACT_PRINT_LOCK_MS) {
                    last_print_ms[i] = now;

                    last_hit_ms[i]    = now;
                    last_hit_val[i]   = impact[i];
                    last_hit_valid[i] = 1U;

                    mcu_printf("[IMPACT][%d ms][%s] imp:",
                            (int)now, g_wheel_name[i]);   // ✅ 여기만 수정
                    Print_Float_Value(impact[i], 1000);

                    mcu_printf(" | Z:");
                    Print_Float_Value(wheel_z[i], 100);

                    mcu_printf(" X:");
                    Print_Float_Value(ax[i], 100);
                    mcu_printf(" Y:");
                    Print_Float_Value(ay[i], 100);

                    mcu_printf("  adxl_age:%dms\n", (int)(now - adxl_t));
                }
            }
        }


        /* =========================================================
         * 2) 기존 모니터링 블록(너가 올린 그대로)
         * ========================================================= */
        mcu_printf("\n==========================================\n");

        mcu_printf("Speed: ");
        Print_Float_Value(speed, 10);
        mcu_printf("%%\n");

        mcu_printf("[ICM] Roll: ");
        Print_Float_Value(roll, 10);
        mcu_printf(" Pitch: ");
        Print_Float_Value(pitch, 10);
        mcu_printf("\n");

        mcu_printf("[ADXL] FL: ");
        Print_Float_Value(wheel_z[WHEEL_FL], 100);
        mcu_printf(" FR: ");
        Print_Float_Value(wheel_z[WHEEL_FR], 100);
        mcu_printf(" RL: ");
        Print_Float_Value(wheel_z[WHEEL_RL], 100);
        mcu_printf(" RR: ");
        Print_Float_Value(wheel_z[WHEEL_RR], 100);
        mcu_printf(" m/s2\n");

        mcu_printf("[SERVO] FL: ");
        Print_Float_Value(servo[WHEEL_FL], 10);
        mcu_printf(" FR: ");
        Print_Float_Value(servo[WHEEL_FR], 10);
        mcu_printf(" RL: ");
        Print_Float_Value(servo[WHEEL_RL], 10);
        mcu_printf(" RR: ");
        Print_Float_Value(servo[WHEEL_RR], 10);
        mcu_printf(" us\n");

        mcu_printf("[DBG] soft_w: ");
        Print_Float_Value(dbg.soft_w, 1000);
        mcu_printf(" | rate: ");
        Print_Float_Value(dbg.rate_deg_s, 10);
        mcu_printf(" deg/s | max_step: ");
        Print_Float_Value(dbg.max_step, 1000);
        mcu_printf(" deg/cycle\n");

        mcu_printf("[DBG] PID_out(mm) Roll: ");
        Print_Float_Value(dbg.pid_roll_out_mm, 100);
        mcu_printf(" Pitch: ");
        Print_Float_Value(dbg.pid_pitch_out_mm, 100);
        mcu_printf("\n");

        mcu_printf("[DBG] PID_integral Roll: ");
        Print_Float_Value(dbg.pid_roll_integral, 100);
        mcu_printf(" Pitch: ");
        Print_Float_Value(dbg.pid_pitch_integral, 100);
        mcu_printf("\n");

        mcu_printf("[DBG] leveling(deg) FL: ");
        Print_Float_Value(dbg.leveling[0], 100);
        mcu_printf(" FR: ");
        Print_Float_Value(dbg.leveling[1], 100);
        mcu_printf(" RL: ");
        Print_Float_Value(dbg.leveling[2], 100);
        mcu_printf(" RR: ");
        Print_Float_Value(dbg.leveling[3], 100);
        mcu_printf("\n");

        mcu_printf("[ADXL] last_update: %d (age %d ms)\n",
                   (int)adxl_t, (int)(now - adxl_t));

        mcu_printf("[IMU_RAW] gyro(dps) x: ");
        Print_Float_Value(dbg.gyro_x_dps, 10);
        mcu_printf(" y: ");
        Print_Float_Value(dbg.gyro_y_dps, 10);
        mcu_printf(" z: ");
        Print_Float_Value(dbg.gyro_z_dps, 10);
        mcu_printf("\n");

        mcu_printf("[IMU_ACC] roll_acc: ");
        Print_Float_Value(dbg.roll_acc_deg, 10);
        mcu_printf(" deg | pitch_acc: ");
        Print_Float_Value(dbg.pitch_acc_deg, 10);
        mcu_printf(" deg\n");

        mcu_printf("Mode: %s\n",
        #if INDEPENDENT_WHEEL_TEST_MODE
                "INDEPENDENT_WHEEL_TEST (NO LEVELING)"
        #else
                "NORMAL (LEVELING + ADXL)"
        #endif
        );

        {
            uint8 any = 0;
            mcu_printf("[IMPACT_LAST] ");

            for (uint8 i = 0; i < WHEEL_MAX; i++) {
                if (last_hit_valid[i] && ((now - last_hit_ms[i]) <= IMPACT_LATCH_HOLD_MS)) {
                    any = 1;
                    mcu_printf("%s:", g_wheel_name[i]);
                    Print_Float_Value(last_hit_val[i], 1000);
                    mcu_printf("(%dms) ", (int)(now - last_hit_ms[i]));
                }
            }

            if (!any) {
                mcu_printf("none");
            }
            mcu_printf("\n");
        }

        mcu_printf("[DBG] shock_abs(deg) FL:");
        Print_Float_Value(dbg.shock_absorb[WHEEL_FL], 100);
        mcu_printf(" FR:");
        Print_Float_Value(dbg.shock_absorb[WHEEL_FR], 100);
        mcu_printf(" RL:");
        Print_Float_Value(dbg.shock_absorb[WHEEL_RL], 100);
        mcu_printf(" RR:");
        Print_Float_Value(dbg.shock_absorb[WHEEL_RR], 100);
        mcu_printf("\n");

        mcu_printf("[DBG] target(deg)  FL:");
        Print_Float_Value(dbg.target_deg[WHEEL_FL], 100);
        mcu_printf(" FR:");
        Print_Float_Value(dbg.target_deg[WHEEL_FR], 100);
        mcu_printf(" RL:");
        Print_Float_Value(dbg.target_deg[WHEEL_RL], 100);
        mcu_printf(" RR:");
        Print_Float_Value(dbg.target_deg[WHEEL_RR], 100);
        mcu_printf("\n");

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

    const uint32 CAL_MS = ADXL_CAL_MS;
    const uint32 PERIOD_MS = 10;
    const uint32 CAL_SAMPLES = (CAL_MS / PERIOD_MS);

    double sum_x[WHEEL_MAX] = {0}, sum_y[WHEEL_MAX] = {0}, sum_z[WHEEL_MAX] = {0};
    uint32 sample_cnt = 0;

    static uint32 adxl_ok_cnt = 0;
    static uint32 adxl_fail_cnt = 0;

    float prev_mag[WHEEL_MAX] = {0};
    uint32 last_hit_ms[WHEEL_MAX] = {0};
    static float impact_hold[WHEEL_MAX] = {0};

    mcu_printf("[ADXL345] Task Started (100Hz)\n");

    /* ====== 센서 설정 ====== */
    for (uint8 dev = 0; dev < ADXL_COUNT; dev++) {
        if (ADXL_MuxSelectByDev(dev) != SAL_RET_SUCCESS) {
            mcu_printf("[ADXL345] Mux select failed for dev %d\n", dev);
            continue;
        }
        
        // ✅ Mux 안정화 대기 (마이크로초 단위면 충분)
        // 하드웨어에 따라 1-5us 필요할 수 있음
        for (volatile uint32 i = 0; i < 100; i++); // ~1us delay
        
        (void)ADXL345_WriteReg(dev, ADXL345_REG_BW_RATE,     0x0A);
        SAL_TaskSleep(10);
        (void)ADXL345_WriteReg(dev, ADXL345_REG_DATA_FORMAT, 0x09);
        SAL_TaskSleep(10);
        (void)ADXL345_WriteReg(dev, ADXL345_REG_POWER_CTL,   0x08);
        SAL_TaskSleep(20);
    }

    mcu_printf("[ADXL345] Neutral calibration...\n");

    /* ===== 1) 캘리브레이션 루프 ===== */
    while (sample_cnt < CAL_SAMPLES) {
        SAL_GetTickCount(&start_tick);

        for (uint8 dev = 0; dev < ADXL_COUNT; dev++) {
            float x, y, z;
            uint8 wheel = g_ADXL_DEV_TO_WHEEL[dev];

            if (ADXL_MuxSelectByDev(dev) != SAL_RET_SUCCESS) {
                adxl_fail_cnt++;
                continue;
            }
            
            // ✅ Mux 안정화: 마이크로초 대기 (Sleep 제거!)
            for (volatile uint32 i = 0; i < 100; i++);
            
            if (ADXL345_ReadAccelCalibrated(dev, &x, &y, &z) == SAL_RET_SUCCESS) {
                sum_x[wheel] += (double)x;
                sum_y[wheel] += (double)y;
                sum_z[wheel] += (double)z;
                adxl_ok_cnt++;
            } else {
                adxl_fail_cnt++;
            }
        }

        sample_cnt++;

        if ((sample_cnt % 50) == 0) {
            float avg_z[WHEEL_MAX];
            for (uint8 w = 0; w < WHEEL_MAX; w++) {
                avg_z[w] = (float)(sum_z[w] / (double)sample_cnt);
            }

            mcu_printf("  [ADXL %d/%d] AVG_Z: ",
                       (int)sample_cnt, (int)CAL_SAMPLES);
            Print_Float_Value(avg_z[WHEEL_FL], 10); mcu_printf("/");
            Print_Float_Value(avg_z[WHEEL_FR], 10); mcu_printf("/");
            Print_Float_Value(avg_z[WHEEL_RL], 10); mcu_printf("/");
            Print_Float_Value(avg_z[WHEEL_RR], 10);
            mcu_printf(" m/s2 (ok:%d fail:%d)\n", (int)adxl_ok_cnt, (int)adxl_fail_cnt);
        }

        uint32 now;
        SAL_GetTickCount(&now);
        uint32 elapsed = now - start_tick;
        if (elapsed < PERIOD_MS) {
            SAL_TaskSleep(PERIOD_MS - elapsed);
        }
    }

    /* ===== bias 확정 ===== */
    for (uint8 w = 0; w < WHEEL_MAX; w++) {
        g_ADXL_bias_x[w] = (float)(sum_x[w] / (double)CAL_SAMPLES);
        g_ADXL_bias_y[w] = (float)(sum_y[w] / (double)CAL_SAMPLES);
        g_ADXL_bias_z[w] = (float)(sum_z[w] / (double)CAL_SAMPLES);

        prev_mag[w] = 0.0f;
        last_hit_ms[w] = 0;
        impact_hold[w] = 0.0f;
    }

    SAL_CoreCriticalEnter();
    g_ADXL_CAL_DONE = 1;
    SAL_CoreCriticalExit();

    mcu_printf("\n[ADXL345] Calibration complete!\n");
    mcu_printf("  Bias Z: ");
    Print_Float_Value(g_ADXL_bias_z[WHEEL_FL], 10); mcu_printf("/");
    Print_Float_Value(g_ADXL_bias_z[WHEEL_FR], 10); mcu_printf("/");
    Print_Float_Value(g_ADXL_bias_z[WHEEL_RL], 10); mcu_printf("/");
    Print_Float_Value(g_ADXL_bias_z[WHEEL_RR], 10);
    mcu_printf(" m/s2\n");
    mcu_printf("  Success: %d, Fail: %d\n", (int)adxl_ok_cnt, (int)adxl_fail_cnt);

    /* ===== 2) 정상 모니터링 루프 ===== */
    while (1) {
        SAL_GetTickCount(&start_tick);

        uint32 now;
        SAL_GetTickCount(&now);
        uint32 tms = now;

        for (uint8 dev = 0; dev < ADXL_COUNT; dev++) {
            float x, y, z;
            uint8 wheel = g_ADXL_DEV_TO_WHEEL[dev];

            if (ADXL_MuxSelectByDev(dev) != SAL_RET_SUCCESS) {
                adxl_fail_cnt++;
                
                SAL_CoreCriticalEnter();
                g_ADXLData.impact_detected[wheel] = 0.0f;
                SAL_CoreCriticalExit();
                continue;
            }

            // ✅ Mux 안정화: 마이크로초 대기
            for (volatile uint32 i = 0; i < 100; i++);

            SALRetCode_t ret = ADXL345_ReadAccelCalibrated(dev, &x, &y, &z);

            if (ret == SAL_RET_SUCCESS) {
                adxl_ok_cnt++;

                /* bias 제거 */
                x -= g_ADXL_bias_x[wheel];
                y -= g_ADXL_bias_y[wheel];
                z -= g_ADXL_bias_z[wheel];

                float mag   = sqrtf(x*x + y*y + z*z);
                float delta = fabsf(mag - prev_mag[wheel]);
                prev_mag[wheel] = mag;

                float impact = 0.0f;
                if (delta >= IMPACT_THRESHOLD_MS2) {
                    if ((tms - last_hit_ms[wheel]) >= IMPACT_COOLDOWN_MS) {
                        last_hit_ms[wheel] = tms;
                        impact = (delta / IMPACT_THRESHOLD_MS2) - 1.0f;
                        impact = clamp(impact, 0.0f, 1.0f);
                    }
                }

                if (impact > 0.0f) {
                    uint8 axle = (wheel == WHEEL_FL || wheel == WHEEL_FR) ? 1U : 2U;

                    SAL_CoreCriticalEnter();
                    uint32 now_ms = tms;
                    if (g_lead_axle == 0U || g_lead_axle == axle) {
                        g_lead_axle = axle;
                        g_lead_until_ms = now_ms + 220U;
                    }
                    SAL_CoreCriticalExit();
                }

                /* hold/decay */
                impact_hold[wheel] *= 0.85f;
                if (impact > impact_hold[wheel]) impact_hold[wheel] = impact;
                impact = impact_hold[wheel];

                SAL_CoreCriticalEnter();
                g_ADXLData.accel_x[wheel] = x;
                g_ADXLData.accel_y[wheel] = y;
                g_ADXLData.accel_z[wheel] = z;
                g_ADXLData.impact_detected[wheel] = impact;
                SAL_GetTickCount(&g_ADXLData.last_update_time);
                SAL_CoreCriticalExit();

            } else {
                adxl_fail_cnt++;

                SAL_CoreCriticalEnter();
                g_ADXLData.impact_detected[wheel] = 0.0f;
                SAL_CoreCriticalExit();
            }
        }

        uint32 end;
        SAL_GetTickCount(&end);
        uint32 elapsed = end - start_tick;
        if (elapsed < PERIOD_MS) {
            SAL_TaskSleep(PERIOD_MS - elapsed);
        }
    }
}

static void ISO2631_Print_Box(const ISO2631_Metrics_t *m)
{
    mcu_printf("\r\n");
    mcu_printf("=============================================\r\n");
    mcu_printf(" ISO 2631-1 VIBRATION MEASUREMENT RESULTS\r\n");
    mcu_printf("=============================================\r\n");

    mcu_printf("Duration: ");
    Print_Float_Value(m->duration_s, 100);
    mcu_printf(" seconds\r\n");

    mcu_printf("Samples: %d\r\n", (int)m->samples);

    mcu_printf("Sample Rate: ");
    Print_Float_Value(m->fs_hz, 10);
    mcu_printf(" Hz\r\n");

    mcu_printf("Gravity Offset: ");
    Print_Float_Value(m->gravity_offset_ms2 / 9.81f, 10000);
    mcu_printf(" g\r\n");

    mcu_printf("---------------------------------------------\r\n");

    mcu_printf("RMS (a_w): ");
    Print_Float_Value(m->rms, 10000);
    mcu_printf(" m/s^2\r\n");

    mcu_printf("VDV: ");
    Print_Float_Value(m->vdv, 10000);
    mcu_printf(" m/s^1.75\r\n");

    mcu_printf("Peak Accel: ");
    Print_Float_Value(m->peak, 10000);
    mcu_printf(" m/s^2\r\n");

    mcu_printf("---------------------------------------------\r\n");

    mcu_printf("Comfort Assessment (8-hour exposure):\r\n");
    if (m->rms < 0.315f)      mcu_printf("Status: NOT UNCOMFORTABLE\r\n");
    else if (m->rms < 0.63f)  mcu_printf("Status: A LITTLE UNCOMFORTABLE\r\n");
    else if (m->rms < 1.0f)   mcu_printf("Status: FAIRLY UNCOMFORTABLE\r\n");
    else if (m->rms < 1.6f)   mcu_printf("Status: UNCOMFORTABLE\r\n");
    else if (m->rms < 2.5f)   mcu_printf("Status: VERY UNCOMFORTABLE\r\n");
    else                      mcu_printf("Status: EXTREMELY UNCOMFORTABLE\r\n");

    mcu_printf("=============================================\r\n\r\n");
}

static void ISO2631_Task(void *pArg)
{
    (void)pArg;

    // ✅ 시스템 준비 완료(캘리브/안정화 끝) 기다림
    while (1) {
        uint8 ready;
        SAL_CoreCriticalEnter();
        ready = g_SYSTEM_READY;
        SAL_CoreCriticalExit();

        if (ready) break;
        SAL_TaskSleep(50);
    }

    mcu_printf("[ISO2631] Task Started (100Hz)\n");
    mcu_printf("[ISO2631] Logging IMU data for Excel...\n\n");

    // ✅ ISO 모듈 초기화 + 1회 측정
    ISO2631_Init(100.0f);            // 100Hz
    ISO2631_SetWarmup(200, 100);     // warmup 2s, gravity 1s
    ISO2631_Start(700);              // 7s window @100Hz (측정구간)

    // ✅ CSV 헤더 출력 (엑셀에서 복사-붙여넣기용)
    mcu_printf("=== CSV DATA START ===\n");
    mcu_printf("Time_ms,Roll_deg,Pitch_deg,AccelZ_ms2\n");

    uint32 start_tick = 0;
    uint32 log_start_ms = 0;
    SAL_GetTickCount(&log_start_ms);

    while (1) {
        SAL_GetTickCount(&start_tick);

        // ✅ IMU 데이터 스냅샷
        float roll, pitch, az;
        uint32 now_ms;
        
        SAL_CoreCriticalEnter();
        roll = g_IMUData.roll;
        pitch = g_IMUData.pitch;
        az = g_IMUData.accel_z;
        SAL_CoreCriticalExit();

        SAL_GetTickCount(&now_ms);
        uint32 elapsed_ms = now_ms - log_start_ms;

        // ✅ CSV 형식으로 출력 (띄어쓰기 없이!)
        mcu_printf("%d,", (int)elapsed_ms);
        Print_Float_Value(roll, 100);
        mcu_printf(",");
        Print_Float_Value(pitch, 100);
        mcu_printf(",");
        Print_Float_Value(az, 1000);
        mcu_printf("\n");

        // ✅ ISO2631 업데이트 (az는 이미 m/s^2 단위)
        uint8 done = ISO2631_Update(az);

        // ✅ 1번만 출력하고 태스크 종료
        if (done == 1U) {
            mcu_printf("=== CSV DATA END ===\n\n");
            
            ISO2631_Metrics_t m = ISO2631_GetMetrics();
            ISO2631_Print_Box(&m);

            mcu_printf("\n[ISO2631] Done. Task idle.\n");
            mcu_printf("[EXCEL] Copy CSV data and paste into Excel\n");
            mcu_printf("        Data > Text to Columns > Delimited > Comma\n\n");

            while (1) {
                SAL_TaskSleep(1000);
            }
        }

        // 정확히 100Hz 유지
        uint32 end_tick;
        SAL_GetTickCount(&end_tick);
        uint32 elapsed = end_tick - start_tick;
        if (elapsed < ISO_PERIOD_MS) {
            SAL_TaskSleep(ISO_PERIOD_MS - elapsed);
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