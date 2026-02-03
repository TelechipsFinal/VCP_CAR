// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
*   FileName : motor_drive.c
*
*   Copyright (c) Telechips Inc.
*
*   Description :
*       L298N motor driver sample using PDM (PWM) and GPIO.
*
***************************************************************************************************
*/

#if ( MCU_BSP_SUPPORT_TEST_APP_PDM == 1 )

#include "motor_drive.h"
#include "pdm.h"
#include "gpio.h"
#include "debug.h"
#include "sal_internal.h"

static void MotorDrive_SleepForSec
(
    uint32                              uiSec
)
{
    (void)SAL_TaskSleep(uiSec * (1000UL));
}

void MotorDrive_L298NTest
(
    void
)
{
    SALRetCode_t    ret             = SAL_RET_FAILED;
    PDMModeConfig_t sModeConfigInfo = {0U,};
    uint32          uiDutyPermille  = 0U;
    uint32          uiCnt           = 0U;

    mcu_printf("\n== Start L298N Motor Sample ==\n");

    /* IN1/IN2 as GPIO outputs */
    (void)GPIO_Config(L298N_IN1_PIN, (uint32)(GPIO_FUNC(0U) | GPIO_OUTPUT));
    (void)GPIO_Config(L298N_IN2_PIN, (uint32)(GPIO_FUNC(0U) | GPIO_OUTPUT));
    (void)GPIO_Set(L298N_IN1_PIN, 0UL);
    (void)GPIO_Set(L298N_IN2_PIN, 0UL);

    (void)PDM_Init();
    MotorDrive_SleepForSec(1UL);

    sModeConfigInfo.mcOperationMode     = PDM_OUTPUT_MODE_PHASE_1;
    sModeConfigInfo.mcPortNumber        = L298N_PDM_OUT_SEL_CH;
    sModeConfigInfo.mcClockDivide       = 0UL;
    sModeConfigInfo.mcLoopCount         = 0UL;
    sModeConfigInfo.mcInversedSignal    = 0UL;

    /* 20 kHz PWM: period = 50,000 ns */
    sModeConfigInfo.mcPeriodNanoSec1    = 50000UL;
    sModeConfigInfo.mcPeriodNanoSec2    = 0UL;
    sModeConfigInfo.mcDutyNanoSec2      = 0UL;

    /* Forward: IN1=1, IN2=0 */
    (void)GPIO_Set(L298N_IN1_PIN, 1UL);
    (void)GPIO_Set(L298N_IN2_PIN, 0UL);

    for(uiDutyPermille = 200U; uiDutyPermille <= 800U; uiDutyPermille += 200U)
    {
        uint64 dutyNs = ((uint64)sModeConfigInfo.mcPeriodNanoSec1 * (uint64)uiDutyPermille) / 1000ULL;
        sModeConfigInfo.mcDutyNanoSec1 = (uint32)dutyNs;

        (void)PDM_Disable(L298N_PDM_CH, PMM_ON);
        uiCnt = 0U;
        while(PDM_GetChannelStatus(L298N_PDM_CH))
        {
            SAL_TaskSleep(1U);
            uiCnt++;
            if(3000U < uiCnt)
            {
                mcu_printf("[%s:%d] ERROR! PDM is not disabled\n", __func__, __LINE__);
                break;
            }
        }

        ret = PDM_SetConfig((uint32)L298N_PDM_CH, &sModeConfigInfo);
        if(ret == SAL_RET_SUCCESS)
        {
            PDM_Enable((uint32)L298N_PDM_CH, PMM_ON);
        }

        mcu_printf("\n\tForward duty: %d %%\n", uiDutyPermille / 10U);
        MotorDrive_SleepForSec(3UL);
    }

    /* Stop (coast): IN1=0, IN2=0 */
    (void)PDM_Disable(L298N_PDM_CH, PMM_ON);
    (void)GPIO_Set(L298N_IN1_PIN, 0UL);
    (void)GPIO_Set(L298N_IN2_PIN, 0UL);
    MotorDrive_SleepForSec(2UL);

    /* Reverse: IN1=0, IN2=1 */
    (void)GPIO_Set(L298N_IN1_PIN, 0UL);
    (void)GPIO_Set(L298N_IN2_PIN, 1UL);

    for(uiDutyPermille = 300U; uiDutyPermille <= 700U; uiDutyPermille += 200U)
    {
        uint64 dutyNs = ((uint64)sModeConfigInfo.mcPeriodNanoSec1 * (uint64)uiDutyPermille) / 1000ULL;
        sModeConfigInfo.mcDutyNanoSec1 = (uint32)dutyNs;

        (void)PDM_Disable(L298N_PDM_CH, PMM_ON);
        uiCnt = 0U;
        while(PDM_GetChannelStatus(L298N_PDM_CH))
        {
            SAL_TaskSleep(1U);
            uiCnt++;
            if(3000U < uiCnt)
            {
                mcu_printf("[%s:%d] ERROR! PDM is not disabled\n", __func__, __LINE__);
                break;
            }
        }

        ret = PDM_SetConfig((uint32)L298N_PDM_CH, &sModeConfigInfo);
        if(ret == SAL_RET_SUCCESS)
        {
            PDM_Enable((uint32)L298N_PDM_CH, PMM_ON);
        }

        mcu_printf("\n\tReverse duty: %d %%\n", uiDutyPermille / 10U);
        MotorDrive_SleepForSec(3UL);
    }

    /* Brake (fast): IN1=1, IN2=1 */
    (void)PDM_Disable(L298N_PDM_CH, PMM_ON);
    (void)GPIO_Set(L298N_IN1_PIN, 1UL);
    (void)GPIO_Set(L298N_IN2_PIN, 1UL);
    MotorDrive_SleepForSec(1UL);

    /* All stop */
    (void)GPIO_Set(L298N_IN1_PIN, 0UL);
    (void)GPIO_Set(L298N_IN2_PIN, 0UL);

    mcu_printf("\n== End L298N Motor Sample ==\n");
}

#endif  // ( MCU_BSP_SUPPORT_TEST_APP_PDM == 1 )
