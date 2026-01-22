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
#include <ICM_20948.h>
#include <kalman_filter.h>
#include <printf_float.h>
#include <imu_calibration.h>

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
// IMU Task Configuration
#define IMU_TASK_PRIO           (SAL_PRIO_APP_CFG + 1)      // 최고 우선순위
#define IMU_TASK_STK_SIZE       ACFG_TASK_MEDIUM_STK_SIZE

// Suspension Control Task Configuration  
#define SUSPENSION_TASK_PRIO    (SAL_PRIO_APP_CFG + 2)      // 2번째 우선순위
#define SUSPENSION_TASK_STK_SIZE ACFG_TASK_MEDIUM_STK_SIZE

#define CALIBRATION_MODE    0  // 0: 정상, 1: 캘리브레이션

/*
***************************************************************************************************
*                                         GLOBAL VARIABLES
***************************************************************************************************
*/
uint32                                  gALiveMsgOnOff;
static uint32                           gALiveCount;

// Task IDs
static uint32                           gCalibTaskID = 0;
static uint32                           gIMUTaskID = 0;
static uint32                           gSuspensionTaskID = 0;

// Task Stacks
static uint32                           gCalibTaskStk[IMU_TASK_STK_SIZE];
static uint32                           gIMUTaskStk[IMU_TASK_STK_SIZE];
static uint32                           gSuspensionTaskStk[SUSPENSION_TASK_STK_SIZE];

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/

static void Main_StartTask
(
    void *                              pArg
);

static void IMU_Task
(   
    void *                              pArg
);

static void Suspension_Task
(   
    void *                              pArg
);


static void AppTaskCreate
(
    void
);

static void DisplayAliveLog
(
    void
);

static void DisplayOTPInfo
(
    void
);


/*
***************************************************************************************************
*                                         FUNCTIONS
***************************************************************************************************
*/


// 측정 설정
#define CALIB_SAMPLE_COUNT      1000    // 측정 샘플 수
#define CALIB_SAMPLE_RATE_MS    10      // 10ms = 100Hz

static Kalman_t kalman_roll;
static Kalman_t kalman_pitch;

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

#include <gpio.h>
#include <uart_example.h>

static void Main_StartTask(void *pArg)
{
    (void)pArg;
    SAL_OsInitFuncs();
    SALRetCode_t ret;

    SAL_TaskSleep(100);

    mcu_printf("[INIT] Initializing ICM-20948 IMU...\n");
    ret = ICM_20948_Init();
    
    if(ret != SAL_RET_SUCCESS) {
        mcu_printf("[ERROR] ICM-20948 initialization FAILED!\n");
        mcu_printf("[ERROR] System halted.\n");
        while(1) { SAL_TaskSleep(1000); }
    }


    mcu_printf("[SUCCESS] ICM-20948 initialized!\n");

    Kalman_Init(&kalman_roll);
    Kalman_Init(&kalman_pitch);
    mcu_printf("[INIT] Kalman filters initialized\n");

#if CALIBRATION_MODE
    // ===== 캘리브레이션 모드 =====
    ret = SAL_TaskCreate(&gCalibTaskID,
                         (const uint8 *)"IMU_Calib",
                         (SALTaskFunc)&IMU_Calibration_Task,
                         &gCalibTaskStk[0],
                         IMU_TASK_STK_SIZE,
                         5,
                         NULL);
    
    if(ret == SAL_RET_SUCCESS) {
        mcu_printf("[TASK] Calibration Task created\n\n");
    }else {
        mcu_printf("[ERROR] Calibration Task creation failed!\n");
    }
#endif

    ret = SAL_TaskCreate(&gIMUTaskID,
                         (const uint8 *)"IMU_100Hz",
                         (SALTaskFunc)&IMU_Task,
                         &gIMUTaskStk[0],
                         IMU_TASK_STK_SIZE,
                         IMU_TASK_PRIO,
                         NULL);
    
    if(ret == SAL_RET_SUCCESS) {
        mcu_printf("[TASK] IMU Task created (Priority: %d, 100Hz)\n", IMU_TASK_PRIO);
    } else {
        mcu_printf("[ERROR] IMU Task creation failed!\n");
    }

    // ret = SAL_TaskCreate(&gSuspensionTaskID,
    //                      (const uint8 *)"Suspension_100Hz",
    //                      (SALTaskFunc)&Suspension_Task,
    //                      &gSuspensionTaskStk[0],
    //                      SUSPENSION_TASK_STK_SIZE,
    //                      SUSPENSION_TASK_PRIO,
    //                      NULL);
    
    // if(ret == SAL_RET_SUCCESS) {
    //     mcu_printf("[TASK] Suspension Task created (Priority: %d, 100Hz)\n", SUSPENSION_TASK_PRIO);
    // } else {
    //     mcu_printf("[ERROR] Suspension Task creation failed!\n");
    // }

    // mcu_printf("\n[SYSTEM] All tasks created successfully!\n");
    // mcu_printf("[SYSTEM] Starting real-time operation...\n\n");

    AppTaskCreate();

    while(1) {
        DisplayAliveLog();
        SAL_TaskSleep(5000);
    }
}

static void IMU_Task(void *pArg)
{
    (void)pArg;
    
    SALRetCode_t ret;
    uint32 start_tick;
    uint32 elapsed_tick;
    uint32 sleep_time;
    uint32 cycle_counter = 0;
    uint32 error_counter = 0;
    uint32 max_elapsed = 0;

    float roll_raw, pitch_raw;
    float roll_filtered, pitch_filtered;
    const float dt = 0.01f;

    
    mcu_printf("[IMU_TASK] Started\n");
    mcu_printf("[IMU_TASK] Frequency: 100Hz (10ms period)\n");
    mcu_printf("[IMU_TASK] Priority: %d (Highest)\n", IMU_TASK_PRIO);
    mcu_printf("[IMU_TASK] Communication: Lock-Free (Critical Section)\n\n");
    
    SAL_TaskSleep(100);

    float accel_x_offset = 0.0f;
    float accel_y_offset = 0.0f;
    float gyro_x_offset = 0.0f;
    float gyro_y_offset = 0.0f;
    float gyro_z_offset = 0.0f;
    
    mcu_printf("[IMU_TASK] Auto-calibration starting...\n");
    mcu_printf("[IMU_TASK] Keep sensor LEVEL for 2 seconds!\n");
    
    for(uint32 i = 0; i < 200; i++) {
        ret = IMU_Read_Data_DMA();
        if(ret == SAL_RET_SUCCESS) {
            accel_x_offset += IMU.accel_x;
            accel_y_offset += IMU.accel_y;
            gyro_x_offset += IMU.gyro_x;
            gyro_y_offset += IMU.gyro_y;
            gyro_z_offset += IMU.gyro_z;
        }
        SAL_TaskSleep(10);
    }
    
    accel_x_offset /= 200.0f;
    accel_y_offset /= 200.0f;
    gyro_x_offset /= 200.0f;
    gyro_y_offset /= 200.0f;
    gyro_z_offset /= 200.0f;
    
    mcu_printf("[IMU_TASK] Offsets: AccX=");
    Print_Float_Value(accel_x_offset, 1000);
    mcu_printf(" AccY=");
    Print_Float_Value(accel_y_offset, 1000);
    mcu_printf("\n");
    
    mcu_printf("[IMU_TASK] Auto-calibration complete!\n\n");


    while(1) {
        SAL_GetTickCount(&start_tick);
        
        ret = IMU_Read_Data_DMA();
        
        if(ret == SAL_RET_SUCCESS) {

            float accel_x_corrected = IMU.accel_x - accel_x_offset;
            float accel_y_corrected = IMU.accel_y - accel_y_offset;
            float accel_z_corrected = IMU.accel_z;
            
            float gyro_x_corrected = IMU.gyro_x - gyro_x_offset;
            float gyro_y_corrected = IMU.gyro_y - gyro_y_offset;

            roll_raw = atan2f(accel_y_corrected, accel_z_corrected);
            pitch_raw = atan2f(-accel_x_corrected, sqrtf(accel_y_corrected * accel_y_corrected + accel_z_corrected * accel_z_corrected));
            
            float gyro_x_rad = IMU.gyro_x * (M_PI / 180.0f);
            float gyro_y_rad = IMU.gyro_y * (M_PI / 180.0f);
            
            roll_filtered = Kalman_Update(&kalman_roll, roll_raw, gyro_x_rad, dt);
            pitch_filtered = Kalman_Update(&kalman_pitch, pitch_raw, gyro_y_rad, dt);

            cycle_counter++;
            
            if(cycle_counter >= 10) {
                mcu_printf("[IMU_TASK] 10 cycles | Accel: X= ");
                Print_Float_Value(IMU.accel_x, 1000);
                mcu_printf(" Accel: Y=");
                Print_Float_Value(IMU.accel_y, 1000);
                mcu_printf(" Accel: Z=");
                Print_Float_Value(IMU.accel_z, 1000);
                mcu_printf("\n");

                mcu_printf("[IMU_TASK] Roll: ");
                Print_Float_Value(roll_filtered * 180.0f / M_PI, 100);
                mcu_printf(" deg, Pitch: ");
                Print_Float_Value(pitch_filtered * 180.0f / M_PI, 100);
                mcu_printf(" deg\n");

            
                if(error_counter > 0) {
                    mcu_printf("[IMU_TASK] Errors in last second: %d\n", error_counter);
                    error_counter = 0;
                }
                
                cycle_counter = 0;
                max_elapsed = 0;
            }
        } else {
            error_counter++;
            mcu_printf("[IMU_TASK] Read failed! (Error count: %d)\n", error_counter);
        }    
        
        
        SAL_GetTickCount(&elapsed_tick);
        elapsed_tick = elapsed_tick - start_tick;
        
        if(elapsed_tick > max_elapsed) {
            max_elapsed = elapsed_tick;
        }

        if(elapsed_tick < 10) {
            sleep_time = 10 - elapsed_tick;
        } else {
            sleep_time = 0;
            mcu_printf("[WARNING] IMU processing exceeded 10ms: %dms\n", elapsed_tick);
        }
        
        SAL_TaskSleep(sleep_time);
    }
}


static void Suspension_Task(void *pArg)
{
    (void)pArg;
    
    uint32 start_tick;
    uint32 elapsed_tick;
    uint32 sleep_time;
    
    IMU_Data local_imu;
    
    uint32 damping_level = 1;
    uint32 prev_level = 1;
    uint32 control_counter = 0;
    uint32 max_elapsed = 0;
    
    mcu_printf("[SUSPENSION_TASK] Started\n");
    mcu_printf("[SUSPENSION_TASK] Frequency: 100Hz (10ms period)\n");
    mcu_printf("[SUSPENSION_TASK] Priority: %d\n", SUSPENSION_TASK_PRIO);
    mcu_printf("[SUSPENSION_TASK] Data Access: Lock-Free Copy\n\n");
    
    SAL_TaskSleep(150);

    while(1) {
         SAL_GetTickCount(&start_tick);
        
        SAL_CoreCriticalEnter();
        local_imu = IMU;
        SAL_CoreCriticalExit();
        
        if(local_imu.accel_z > 1.5f || local_imu.accel_z < 0.5f) {
            damping_level = 3;
        }

        else if(local_imu.gyro_x > 30.0f || local_imu.gyro_x < -30.0f) {
            damping_level = 3;
        }

        else if(local_imu.accel_y > 0.3f || local_imu.accel_y < -0.3f) {
            damping_level = 2;
        }

        else if(local_imu.gyro_x > 15.0f || local_imu.gyro_x < -15.0f) {
            damping_level = 2;
        }

        else {
            damping_level = 1;
        }
        
        /*
        PWM_SetDutyCycle(DAMPER_FRONT_LEFT,  damping_level * 33);
        PWM_SetDutyCycle(DAMPER_FRONT_RIGHT, damping_level * 33);
        PWM_SetDutyCycle(DAMPER_REAR_LEFT,   damping_level * 33);
        PWM_SetDutyCycle(DAMPER_REAR_RIGHT,  damping_level * 33);
        */
        
        control_counter++;
        
        // if(damping_level != prev_level) {
        //     mcu_printf("[SUSPENSION] Level: %d→%d | AccZ:%.2f AccY:%.2f GyroX:%.2f\n",
        //               prev_level, damping_level, 
        //               local_imu.accel_z, local_imu.accel_y, local_imu.gyro_x);
        //     prev_level = damping_level;
        // }
        
         if(control_counter >= 100) {
        //     mcu_printf("[SUSPENSION] 100 cycles | Level:%d | Max:%dms\n", 
        //               damping_level, max_elapsed);
             control_counter = 0;
             max_elapsed = 0;
        }
        
        SAL_GetTickCount(&elapsed_tick);
        elapsed_tick = elapsed_tick - start_tick;
        
        if(elapsed_tick > max_elapsed) {
            max_elapsed = elapsed_tick;
        }
        
        if(elapsed_tick < 10) {
            sleep_time = 10 - elapsed_tick;
        } else {
            sleep_time = 0;
            mcu_printf("[WARNING] Suspension control exceeded 10ms: %dms\n", elapsed_tick);
        }
        
        SAL_TaskSleep(sleep_time);
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

