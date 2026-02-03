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
#define MONITOR_TASK_PRIO       (SAL_PRIO_APP_CFG + 5)      


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
*                                         PID CONFIGURATION
***************************************************************************************************
*/

// PID Controller Structure
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float dt;
    float integral_max;
} PID_Controller_t;

// PID Gains
#define LEVELING_KP     (2.0f)
#define LEVELING_KI     (0.0f)
#define LEVELING_KD     (0.0f)

#define DAMPING_KP_SOFT     (0.3f)  // 저속: 부드러운 댐핑
#define DAMPING_KD_SOFT     (0.0f)

#define DAMPING_KP_MEDIUM   (0.5f)  // 중속: 밸런스
#define DAMPING_KD_MEDIUM   (0.0f)

#define DAMPING_KP_HARD     (0.8f)  // 고속: 강한 댐핑
#define DAMPING_KD_HARD     (0.0f)

// 속도 임계값 (%)
#define SPEED_THRESHOLD_LOW     (30.0f)   // 30% 이하: 저속
#define SPEED_THRESHOLD_HIGH    (70.0f)   // 70% 이상: 고속

#define MAX_TILT_ANGLE ROLLOVER_ANGLE_THRESHOLD   
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

// PID Controllers
static PID_Controller_t pid_roll;
static PID_Controller_t pid_pitch;
static PID_Controller_t pid_damping;

static volatile uint8 g_IMU_SUSP_INIT_DONE = 0;

static ADXL_Data_t g_ADXLData;
static PID_Controller_t pid_damping_wheel[4];  // 각 바퀴별 댐핑 PID

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
static void ADXL345_Monitor_Task(void *pArg);  // ✅ 추가

static void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd, float dt_ms);
static float PID_Update(PID_Controller_t *pid, float setpoint, float measurement);
static void PID_Reset(PID_Controller_t *pid);

static void Motor_SetSpeed(float speed);

static void AppTaskCreate(void);
static void DisplayAliveLog(void);
static void DisplayOTPInfo(void);


/*
***************************************************************************************************
*                                         PID FUNCTIONS
***************************************************************************************************
*/

void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd, float dt_ms) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = dt_ms / 1000.0f;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->integral_max = 50.0f;
}

float PID_Update(PID_Controller_t *pid, float setpoint, float measurement) {
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

void PID_Reset(PID_Controller_t *pid) {
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
    memset(&g_ADXLData, 0, sizeof(g_ADXLData));  // ✅ 추가

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
    
    // Initialize PID controllers
    PID_Init(&pid_roll, LEVELING_KP, LEVELING_KI, LEVELING_KD, IMU_SUSP_PERIOD_MS);
    PID_Init(&pid_pitch, LEVELING_KP, LEVELING_KI, LEVELING_KD, IMU_SUSP_PERIOD_MS);
    
    // ✅ 댐핑 PID는 중간값으로 초기화 (런타임에 동적 변경)
    PID_Init(&pid_damping, DAMPING_KP_MEDIUM, 0.0f, DAMPING_KD_MEDIUM, IMU_SUSP_PERIOD_MS);

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
        
        // ✅ I2C 클럭 상태 확인
        uint32 i2c_clk = CLOCK_GetPeriRate((sint32)CLOCK_PERI_I2C2);
        mcu_printf("[DEBUG] I2C Clock: %d Hz\n", i2c_clk);
        
        // ✅ GPIO 기능 확인
        mcu_printf("[DEBUG] Checking GPIO settings...\n");
        mcu_printf("  (SCL): Expected=INPUT+FUNC2\n");
        mcu_printf("  (SDA): Expected=INPUT+FUNC2\n");
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

     //Task 6:ADXL345
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

void IMU_Suspension_Task(void *pArg)
{
    (void)pArg;

    uint32 start_tick, current_tick;

    uint32 servo_pulse_ns[4] = {
        SERVO_NEUTRAL_PULSE_NS,
        SERVO_NEUTRAL_PULSE_NS,
        SERVO_NEUTRAL_PULSE_NS,
        SERVO_NEUTRAL_PULSE_NS
    };

    static uint32 imu_sample_count = 0;

    // 캘리브레이션 변수
    static uint8 calibration_done = 0;
    float roll_sum  = 0.0f;
    float pitch_sum = 0.0f;
    uint32 calib_samples = 0;
    const uint32 CALIB_SAMPLES = 100;

    IMU_Data imuRaw;
    static uint8 servo_div2 = 0;

    // ✅ ADXL Pre-kick (static으로 유지)
    static float adxl_pre_kick[4] = {0, 0, 0, 0};

    mcu_printf("[IMU_SUSP] Task Started\n");
    mcu_printf("  - IMU Sampling: 100Hz (10ms)\n");
    mcu_printf("  - PID Control:  100Hz (10ms)\n");
    mcu_printf("  - Servo Output: 50Hz (20ms) via PDM\n");
    mcu_printf("  - Adaptive Damping: Speed-Dependent\n");
    mcu_printf("  - ADXL Pre-kick: Enabled\n");
    mcu_printf("  - Calibrating Roll/Pitch offset...\n\n");

    // 서보 초기화
    Servo_Init();

    mcu_printf("\n╔════════════════════════════╗\n");
    mcu_printf("║   SERVO RANGE TEST START   ║\n");
    mcu_printf("╚════════════════════════════╝\n\n");

    // Test 1: 최소 높이
    mcu_printf("▶ Test 1: MIN Position (pulse=%d us)\n", (int)NS_TO_US(SERVO_MIN_PULSE_NS));
    uint32 min_pulse[4] = {
        SERVO_MIN_PULSE_NS,
        SERVO_MIN_PULSE_NS,
        SERVO_MIN_PULSE_NS,
        SERVO_MIN_PULSE_NS
    };
    Servo_SetPulseAllNs(min_pulse);
    SAL_TaskSleep(1500);

    // Test 2: 최대 높이
    mcu_printf("▶ Test 2: MAX Position (pulse=%d us)\n", (int)NS_TO_US(SERVO_MAX_PULSE_NS));
    uint32 max_pulse[4] = {
        SERVO_MAX_PULSE_NS,
        SERVO_MAX_PULSE_NS,
        SERVO_MAX_PULSE_NS,
        SERVO_MAX_PULSE_NS
    };
    Servo_SetPulseAllNs(max_pulse);
    SAL_TaskSleep(1500);

    // Test 3: 중립
    mcu_printf("▶ Test 3: Neutral Position (pulse=%d us)\n", (int)NS_TO_US(SERVO_NEUTRAL_PULSE_NS));
    Servo_SetNeutral_All();
    SAL_TaskSleep(1000);

    mcu_printf("\n✅ SERVO TEST COMPLETE!\n\n");

    for (uint8 i = 0; i < 4; i++) servo_pulse_ns[i] = SERVO_NEUTRAL_PULSE_NS;
    Servo_SetPulseAllNs(servo_pulse_ns);

    SAL_CoreCriticalEnter();
    for (uint8 i = 0; i < 4; i++) {
        g_ServoData.position[i] = (float)NS_TO_US(servo_pulse_ns[i]);
    }
    g_IMU_SUSP_INIT_DONE = 1;
    SAL_CoreCriticalExit();

    mcu_printf("[IMU_SUSP] Servos initialized to neutral pulse\n");
    SAL_TaskSleep(200);

    // PID 초기화
    for(uint8 i = 0; i < 4; i++) {
        PID_Init(&pid_damping_wheel[i], DAMPING_KP_MEDIUM, 0.0f, DAMPING_KD_MEDIUM, IMU_SUSP_PERIOD_MS);
    }
    
    while(1) {
        SAL_GetTickCount(&start_tick);

        // ========== 1. IMU 읽기 ==========
        if (IMU_Read_Data_DMA() != SAL_RET_SUCCESS) {
            SAL_TaskSleep(IMU_SUSP_PERIOD_MS);
            continue;
        }

        imuRaw = IMU;

        // ========== 2. 캘리브레이션 단계 ==========
        if (!calibration_done)
        {
            float roll_acc  = atan2f(imuRaw.accel_y, imuRaw.accel_z) * 180.0f / M_PI;
            float pitch_acc = atan2f(-imuRaw.accel_x,
                               sqrtf(imuRaw.accel_y * imuRaw.accel_y +
                                     imuRaw.accel_z * imuRaw.accel_z)) * 180.0f / M_PI;

            roll_sum  += roll_acc;
            pitch_sum += pitch_acc;
            calib_samples++;

            if (calib_samples >= CALIB_SAMPLES)
            {
                roll_offset  = roll_sum  / (float)CALIB_SAMPLES;
                pitch_offset = pitch_sum / (float)CALIB_SAMPLES;
                calibration_done = 1;

                mcu_printf("  [CALIB] Complete!\n");
                mcu_printf("  Roll offset:  ");
                Print_Float_Value(roll_offset, 100);
                mcu_printf(" deg\n");
                mcu_printf("  Pitch offset: ");
                Print_Float_Value(pitch_offset, 100);
                mcu_printf(" deg\n\n");
            }

            SAL_CoreCriticalEnter();
            g_IMUData.roll   = 0.0f;
            g_IMUData.pitch  = 0.0f;
            g_IMUData.gyro_x = imuRaw.gyro_x;
            g_IMUData.gyro_y = imuRaw.gyro_y;
            g_IMUData.gyro_z = imuRaw.gyro_z;
            g_IMUData.accel_x = imuRaw.accel_x;
            g_IMUData.accel_y = imuRaw.accel_y;
            g_IMUData.accel_z = imuRaw.accel_z;
            SAL_GetTickCount(&g_IMUData.last_update_time);
            SAL_CoreCriticalExit();

            for (uint8 i = 0; i < 4; i++) servo_pulse_ns[i] = SERVO_NEUTRAL_PULSE_NS;

            servo_div2 ^= 1U;
            if (servo_div2 == 0U) {
                Servo_SetPulseAllNs(servo_pulse_ns);
            }

            SAL_TaskSleep(IMU_SUSP_PERIOD_MS);
            continue;
        }

        // ========== 3. Kalman 필터링 ==========
        imu_sample_count++;

        float roll_acc  = atan2f(imuRaw.accel_y, imuRaw.accel_z) * 180.0f / M_PI;
        float pitch_acc = atan2f(-imuRaw.accel_x,
                          sqrtf(imuRaw.accel_y * imuRaw.accel_y +
                                imuRaw.accel_z * imuRaw.accel_z)) * 180.0f / M_PI;

        roll_acc  -= roll_offset;
        pitch_acc -= pitch_offset;

        float dt = IMU_SUSP_PERIOD_MS / 1000.0f;

        float roll  = Kalman_Update(&kalman_roll,  roll_acc,  imuRaw.gyro_x, dt);
        float pitch = Kalman_Update(&kalman_pitch, pitch_acc, imuRaw.gyro_y, dt);

        SAL_CoreCriticalEnter();
        g_IMUData.roll  = fmaxf(fminf(roll,  MAX_TILT_ANGLE), -MAX_TILT_ANGLE);
        g_IMUData.pitch = fmaxf(fminf(pitch, MAX_TILT_ANGLE), -MAX_TILT_ANGLE);
        g_IMUData.gyro_x = imuRaw.gyro_x;
        g_IMUData.gyro_y = imuRaw.gyro_y;
        g_IMUData.gyro_z = imuRaw.gyro_z;
        g_IMUData.accel_x = imuRaw.accel_x;
        g_IMUData.accel_y = imuRaw.accel_y;
        g_IMUData.accel_z = imuRaw.accel_z;
        SAL_GetTickCount(&g_IMUData.last_update_time);
        SAL_CoreCriticalExit();

        // 안정화 대기
        if (imu_sample_count < 100)
        {
            for (uint8 i = 0; i < 4; i++) servo_pulse_ns[i] = SERVO_NEUTRAL_PULSE_NS;

            servo_div2 ^= 1U;
            if (servo_div2 == 0U) {
                Servo_SetPulseAllNs(servo_pulse_ns);
            }

            SAL_CoreCriticalEnter();
            for (uint8 i = 0; i < 4; i++) {
                g_ServoData.position[i] = (float)NS_TO_US(servo_pulse_ns[i]);
            }
            SAL_CoreCriticalExit();

            SAL_TaskSleep(IMU_SUSP_PERIOD_MS);
            continue;
        }

        // ========== 4. 안전 레벨 및 속도 읽기 ==========
        SAL_CoreCriticalEnter();
        SafetyLevel_t safety = g_SafetyData.level;
        uint8 susp_disabled  = g_SafetyData.suspension_disabled;
        float range_limit    = g_SafetyData.suspension_range_limit;
        uint8 on_slope       = g_SafetyData.on_slope_detected;
        SAL_CoreCriticalExit();

        SAL_CoreCriticalEnter();
        float current_speed = g_MotorData.current_speed;
        SAL_CoreCriticalExit();

        // ========== 5. 비상 상황: 중립 복귀 ==========
        if (susp_disabled || safety >= SAFETY_EMERGENCY)
        {
            PID_Reset(&pid_roll);
            PID_Reset(&pid_pitch);
            PID_Reset(&pid_damping);
            
            for(uint8 i = 0; i < 4; i++) {
                PID_Reset(&pid_damping_wheel[i]);
                adxl_pre_kick[i] = 0.0f;  // ✅ Pre-kick 리셋
            }

            for (uint8 i = 0; i < 4; i++) {
                float cur = (float)servo_pulse_ns[i];
                float neu = (float)SERVO_NEUTRAL_PULSE_NS;
                cur += (neu - cur) * 0.05f;
                servo_pulse_ns[i] = (uint32)cur;
            }

            servo_div2 ^= 1U;
            if (servo_div2 == 0U) {
                Servo_SetPulseAllNs(servo_pulse_ns);
            }

            SAL_CoreCriticalEnter();
            for (uint8 i = 0; i < 4; i++) {
                g_ServoData.position[i] = (float)NS_TO_US(servo_pulse_ns[i]);
            }
            SAL_CoreCriticalExit();

            SAL_TaskSleep(IMU_SUSP_PERIOD_MS);
            continue;
        }

        // ========== 6. CAN 명령 읽기 ==========
        SAL_CoreCriticalEnter();
        uint8 leveling_on = g_CANData.leveling_enable;
        SAL_CoreCriticalExit();

        // ========================================================================
        // ========== ✅ ADXL Pre-kick: 충격 감지 시 3도만 시작 ==========
        // ========================================================================
        
        float wheel_accel_z[4];
        SAL_CoreCriticalEnter();
        for(uint8 i = 0; i < 4; i++) {
            wheel_accel_z[i] = g_ADXLData.accel_z[i];
        }
        SAL_CoreCriticalExit();

        static float prev_wheel_accel_z[4] = {0};
        const float IMPACT_THRESHOLD = 2.0f * 9.81f;  // 2G
        const float PRE_KICK_MAGNITUDE = 3.0f;         // 3도
        const float DECAY_RATE = 0.9f;                 // 90% 감쇠
        
        for(uint8 i = 0; i < 4; i++) {
            float accel_change = wheel_accel_z[i] - prev_wheel_accel_z[i];
            prev_wheel_accel_z[i] = wheel_accel_z[i];
            
            // ✅ 1. 기존 pre-kick 감쇠
            adxl_pre_kick[i] *= DECAY_RATE;
            
            // ✅ 2. 새로운 충격 감지
            if(fabsf(accel_change) > IMPACT_THRESHOLD) {
                if(accel_change > 0) {
                    adxl_pre_kick[i] = -PRE_KICK_MAGNITUDE;  // 위로 충격 → 수축
                } else {
                    adxl_pre_kick[i] = +PRE_KICK_MAGNITUDE;  // 아래로 충격 → 확장
                }
            }
            
            // ✅ 3. 매우 작아지면 0으로
            if(fabsf(adxl_pre_kick[i]) < 0.1f) {
                adxl_pre_kick[i] = 0.0f;
            }
        }

        // ========== 7. Auto-Leveling PID ==========
        float leveling[4] = {0, 0, 0, 0};

        static uint8 pid_initialized = 0;
        if (!pid_initialized) {
            PID_Reset(&pid_roll);
            PID_Reset(&pid_pitch);
            PID_Reset(&pid_damping);
            pid_initialized = 1;
        }

        if (leveling_on)
        {
            float gain_multiplier = on_slope ? 1.3f : 1.0f;

            float roll_control  = - PID_Update(&pid_roll,  0.0f, roll)  * gain_multiplier;
            float pitch_control = PID_Update(&pid_pitch, 0.0f, pitch) * gain_multiplier;

            // Roll/Pitch 기반 대각선 제어
            leveling[0] = -roll_control - pitch_control;  // FL
            leveling[1] =  roll_control - pitch_control;  // FR
            leveling[2] = -roll_control + pitch_control;  // RL
            leveling[3] =  roll_control + pitch_control;  // RR

            for (uint8 i = 0; i < 4; i++) {
                float max_offset = (float)ROLLOVER_ANGLE_THRESHOLD * range_limit;
                if (leveling[i] >  max_offset) leveling[i] =  max_offset;
                if (leveling[i] < -max_offset) leveling[i] = -max_offset;
            }
        }
        else
        {
            PID_Reset(&pid_roll);
            PID_Reset(&pid_pitch);
        }

        // ========== 8. 속도 의존 댐핑 ==========
        static float prev_accel_z = 0.0f;
        float damping[4] = {0, 0, 0, 0};

        float accel_change = imuRaw.accel_z - prev_accel_z;
        prev_accel_z = imuRaw.accel_z;

        float damping_kp, damping_kd, max_damping;
        if (current_speed < SPEED_THRESHOLD_LOW) {
            damping_kp = DAMPING_KP_SOFT;
            damping_kd = DAMPING_KD_SOFT;
            max_damping = 10.0f;
        }
        else if (current_speed < SPEED_THRESHOLD_HIGH) {
            damping_kp = DAMPING_KP_MEDIUM;
            damping_kd = DAMPING_KD_MEDIUM;
            max_damping = 12.0f;
        }
        else {
            damping_kp = DAMPING_KP_HARD;
            damping_kd = DAMPING_KD_HARD;
            max_damping = 15.0f;
        }

        pid_damping.Kp = damping_kp;
        pid_damping.Kd = damping_kd;

        float damping_control = PID_Update(&pid_damping, 0.0f, accel_change / 9.81f);

        for (uint8 i = 0; i < 4; i++) {
            damping[i] = damping_control;

            float max_damp = max_damping * range_limit;
            if (damping[i] >  max_damp) damping[i] =  max_damp;
            if (damping[i] < -max_damp) damping[i] = -max_damp;
        }

        // ========== 9. Height Offset 읽기 ==========
        SAL_CoreCriticalEnter();
        float height = g_HeightData.height_offset;
        SAL_CoreCriticalExit();

        // ========== 10. 통합 제어 + 슬루레이트 리미터 ==========
        static float prev_target_deg[4] = {0, 0, 0, 0};
        float target_deg[4];

        for (uint8 i = 0; i < 4; i++)
        {
            // ✅ 최종 = 차고 + 레벨링(ICM) + 댐핑(ICM) + Pre-kick(ADXL)
            float t = height + leveling[i] + damping[i] + adxl_pre_kick[i];

            float max_offset = (float)ROLLOVER_ANGLE_THRESHOLD * range_limit;
            if (t >  max_offset) t =  max_offset;
            if (t < -max_offset) t = -max_offset;

            float max_change = 5.0f;
            float diff = t - prev_target_deg[i];

            if (diff >  max_change) t = prev_target_deg[i] + max_change;
            if (diff < -max_change) t = prev_target_deg[i] - max_change;

            prev_target_deg[i] = t;
            target_deg[i] = t;
        }

        // ========== 11. 서보 출력(펄스로만) ==========
        const float PULSE_RANGE_NS = (float)((int32)SERVO_MAX_PULSE_NS - (int32)SERVO_MIN_PULSE_NS);
        const float DEG_RANGE = (float)(2.0f * ROLLOVER_ANGLE_THRESHOLD);
        const float NS_PER_DEG = PULSE_RANGE_NS / DEG_RANGE;

        for (uint8 i = 0; i < 4; i++) {
            float offset_ns = target_deg[i] * NS_PER_DEG;
            float p = (float)SERVO_NEUTRAL_PULSE_NS + offset_ns;
            servo_pulse_ns[i] = (uint32)p;
        }

        servo_div2 ^= 1U;
        if (servo_div2 == 0U) {
            Servo_SetPulseAllNs(servo_pulse_ns);
        }

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
*                                          Height_Control_Task
***************************************************************************************************
*/
void Height_Control_Task(void *pArg) {
    (void)pArg;
    
    uint32 start_tick, current_tick;
    float current_height = 0;
    
    static uint8 height_enable = 0;  // ✅ 경고 제거용
    static uint32 height_delay = 0;
    
    mcu_printf("[HEIGHT] Task Started (20Hz)\n");
    
    SAL_TaskSleep(250);
    
    while(1) {  // ✅ 루프 시작
        SAL_GetTickCount(&start_tick);
        
        // ✅ 초기 딜레이 처리 (루프 내부로 이동)
        if(!height_enable) {
            height_delay++;
            if(height_delay < 40) {  // 2초 (20Hz)
                g_HeightData.height_offset = 0;
                SAL_TaskSleep(HEIGHT_PERIOD_MS);
                continue;
            }
            height_enable = 1;
        }
        
        // Read safety
        SAL_CoreCriticalEnter();
        SafetyLevel_t safety = g_SafetyData.level;
        uint8 susp_disabled = g_SafetyData.suspension_disabled;
        SAL_CoreCriticalExit();
        
        if(susp_disabled || safety >= SAFETY_EMERGENCY) {
            current_height = 0;
            
            SAL_CoreCriticalEnter();
            g_HeightData.height_offset = 0;
            SAL_CoreCriticalExit();
            
            SAL_TaskSleep(HEIGHT_PERIOD_MS);
            continue;
        }
        
        // Read motor speed
        SAL_CoreCriticalEnter();
        float speed = g_MotorData.current_speed;
        SAL_CoreCriticalExit();
        
        // Calculate target height
        float target_height = 0;
        if(speed >= 70.0f) {
            target_height = -10.0f;
        } else if(speed >= 50.0f) {
            target_height = -5.0f;
        } else if(speed <= 10.0f) {
            target_height = 5.0f;
        }
        
        // Ramping
        float ramp_rate = 0.1f;
        if(target_height > current_height) {
            current_height += ramp_rate;
            if(current_height > target_height) current_height = target_height;
        } else if(target_height < current_height) {
            current_height -= ramp_rate;
            if(current_height < target_height) current_height = target_height;
        }
        
        SAL_CoreCriticalEnter();
        g_HeightData.height_offset = current_height;
        SAL_CoreCriticalExit();
        
        // Sleep
        SAL_GetTickCount(&current_tick);
        uint32 elapsed = current_tick - start_tick;
        if(elapsed < HEIGHT_PERIOD_MS) {
            SAL_TaskSleep(HEIGHT_PERIOD_MS - elapsed);
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

    while (1) {
        uint8 ready;
        SAL_CoreCriticalEnter();
        ready = g_IMU_SUSP_INIT_DONE;
        SAL_CoreCriticalExit();

        if (ready != 0U) break;
        SAL_TaskSleep(50);   // 50ms polling
    }

    mcu_printf("[MONITOR] Task Started (10Hz)\n\n");

    /* optional: small delay to avoid immediate collision with last init prints */
    SAL_TaskSleep(100);
    
    
    while(1) {
        SAL_CoreCriticalEnter();
        SafetyLevel_t safety = g_SafetyData.level;
        float roll = g_IMUData.roll;
        float pitch = g_IMUData.pitch;
        float speed = g_MotorData.current_speed;

        
        // ✅ ADXL 데이터
        float wheel_z[4];
        for(uint8 i = 0; i < 4; i++) {
            wheel_z[i] = g_ADXLData.accel_z[i];
        }
        
        // ✅ 서보 위치
        float servo[4];
        for(uint8 i = 0; i < 4; i++) {
            servo[i] = g_ServoData.position[i];
        }
        SAL_CoreCriticalExit();
        
        mcu_printf("==========================================\n");
        mcu_printf("Safety: %d | Speed: ", safety);
        Print_Float_Value(speed, 10);
        mcu_printf("%%\n");
        
        // ✅ ICM 차체 자세
        mcu_printf("[ICM] Roll: ");
        Print_Float_Value(roll, 10);
        mcu_printf(" Pitch: ");
        Print_Float_Value(pitch, 10);
        mcu_printf("\n");
        
        // ✅ ADXL 각 바퀴 Z축
        mcu_printf("[ADXL] FL: ");
        Print_Float_Value(wheel_z[0], 10);
        mcu_printf(" FR: ");
        Print_Float_Value(wheel_z[1], 10);
        mcu_printf(" RL: ");
        Print_Float_Value(wheel_z[2], 10);
        mcu_printf(" RR: ");
        Print_Float_Value(wheel_z[3], 10);
        mcu_printf(" m/s²\n");
        
        // ✅ 서보 최종 위치
        mcu_printf("[SERVO] FL: ");
        Print_Float_Value(servo[0], 10);
        mcu_printf(" FR: ");
        Print_Float_Value(servo[1], 10);
        mcu_printf(" RL: ");
        Print_Float_Value(servo[2], 10);
        mcu_printf(" RR: ");
        Print_Float_Value(servo[3], 10);
        mcu_printf(" us\n");
        mcu_printf("==========================================\n\n");
        
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
    float impact_threshold = 2.0f * 9.81f;  // 2G
    
    mcu_printf("[ADXL345] Monitor Task Started (100Hz)\n");
    
    // 캘리브레이션
    mcu_printf("[ADXL345] Calibrating all 4 sensors...\n");
    ADXL345_CalibrateAll(200);  // 2초
    
    SAL_TaskSleep(100);
    
    while(1) {
        SAL_GetTickCount(&start_tick);
        
        // 4개 센서 순차 읽기
        for(uint8 dev = 0; dev < 4; dev++) {
            float x, y, z;
            
            if(ADXL345_ReadAccelCalibrated(dev, &x, &y, &z) == SAL_RET_SUCCESS) {
                // Z축 변화량 계산
                float delta_z = fabsf(z - prev_accel_z[dev]);
                prev_accel_z[dev] = z;
                
                // 충격 강도 (0~1 정규화)
                float impact = 0.0f;
                if(delta_z > impact_threshold) {
                    impact = fminf((delta_z / impact_threshold) - 1.0f, 1.0f);
                }
                
                // 공유 데이터 업데이트
                SAL_CoreCriticalEnter();
                g_ADXLData.accel_z[dev] = z;
                g_ADXLData.impact_detected[dev] = impact;
                SAL_GetTickCount(&g_ADXLData.last_update_time);
                SAL_CoreCriticalExit();
            }
        }
        
        // 10ms 주기 유지
        uint32 current_tick;
        SAL_GetTickCount(&current_tick);
        uint32 elapsed = current_tick - start_tick;
        if(elapsed < 10) {
            SAL_TaskSleep(10 - elapsed);
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
