// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
*   FileName : motor_control.c
*
*   Copyright (c) Telechips Inc.
*
*   Description :
*       L298N motor control using PDM (PWM) and GPIO.
*
***************************************************************************************************
*/

#if ( MCU_BSP_SUPPORT_MOTOR_PDM == 1 )

#include "motor_control.h"
#include "pdm.h"
#include "gpio.h"
#include "debug.h"
#include "sal_internal.h"

static uint8                            gMotorControlInited = 0U;
static PDMModeConfig_t                  gMotorPdmConfigA;
static PDMModeConfig_t                  gMotorPdmConfigB;

static void MotorControl_WaitPdmDisabled
(
    uint32                              uiPdmCh
)
{
    uint32 uiCnt = 0U;

    while(PDM_GetChannelStatus(uiPdmCh))
    {
        SAL_TaskSleep(1U);
        uiCnt++;
        if(3000U < uiCnt)
        {
            mcu_printf("[MotorControl] ERROR! PDM ch%d is not disabled\n", (int)uiPdmCh);
            break;
        }
    }
}

static void MotorControl_ApplyDutyPermille
(
    uint32                              uiPdmCh,
    PDMModeConfig_t *                   pCfg,
    uint32                              uiDutyPermille
)
{
    uint64 dutyNs = ((uint64)pCfg->mcPeriodNanoSec1 * (uint64)uiDutyPermille) / 1000ULL;
    pCfg->mcDutyNanoSec1 = (uint32)dutyNs;

    (void)PDM_Disable(uiPdmCh, PMM_ON);
    MotorControl_WaitPdmDisabled(uiPdmCh);
    (void)PDM_SetConfig(uiPdmCh, pCfg);
    (void)PDM_Enable(uiPdmCh, PMM_ON);
}

static void MotorControl_SetSingleMotorSpeed
(
    uint32                              uiPdmCh,
    PDMModeConfig_t *                   pCfg,
    uint32                              uiInHighPin,
    uint32                              uiInLowPin,
    uint32                              uiSpeed
)
{
    uint32 dutyPermille = uiSpeed;

    if(dutyPermille > 1000U)
    {
        dutyPermille = 1000U;
    }

    if(dutyPermille == 0U)
    {
        (void)PDM_Disable(uiPdmCh, PMM_ON);
        MotorControl_WaitPdmDisabled(uiPdmCh);
        (void)GPIO_Set(uiInHighPin, 0UL);
        (void)GPIO_Set(uiInLowPin, 0UL);
    }
    else
    {
        (void)GPIO_Set(uiInHighPin, 1UL);
        (void)GPIO_Set(uiInLowPin, 0UL);
        MotorControl_ApplyDutyPermille(uiPdmCh, pCfg, dutyPermille);
    }
}

void MotorControl_Init
(
    void
)
{
    if(gMotorControlInited != 0U)
    {
        return;
    }

    /* IN1/IN2/IN3/IN4 as GPIO outputs */
    (void)GPIO_Config(L298N_IN1_PIN, (uint32)(GPIO_FUNC(0U) | GPIO_OUTPUT));
    (void)GPIO_Config(L298N_IN2_PIN, (uint32)(GPIO_FUNC(0U) | GPIO_OUTPUT));
    (void)GPIO_Config(L298N_IN3_PIN, (uint32)(GPIO_FUNC(0U) | GPIO_OUTPUT));
    (void)GPIO_Config(L298N_IN4_PIN, (uint32)(GPIO_FUNC(0U) | GPIO_OUTPUT));
    (void)GPIO_Set(L298N_IN1_PIN, 0UL);
    (void)GPIO_Set(L298N_IN2_PIN, 0UL);
    (void)GPIO_Set(L298N_IN3_PIN, 0UL);
    (void)GPIO_Set(L298N_IN4_PIN, 0UL);

    (void)PDM_Init();
    (void)SAL_TaskSleep(1000UL);

    gMotorPdmConfigA.mcOperationMode  = PDM_OUTPUT_MODE_PHASE_1;
    gMotorPdmConfigA.mcPortNumber     = L298N_PDM_OUT_SEL_CH;
    gMotorPdmConfigA.mcClockDivide    = 0UL;
    gMotorPdmConfigA.mcLoopCount      = 0UL;
    gMotorPdmConfigA.mcInversedSignal = 0UL;

    /* 20 kHz PWM: period = 50,000 ns */
    gMotorPdmConfigA.mcPeriodNanoSec1 = 50000UL;
    gMotorPdmConfigA.mcPeriodNanoSec2 = 0UL;
    gMotorPdmConfigA.mcDutyNanoSec2   = 0UL;

    (void)SAL_MemCopy(&gMotorPdmConfigB, &gMotorPdmConfigA, sizeof(PDMModeConfig_t));

    /* Forward: Motor A(IN1=1, IN2=0), Motor B(IN3=1, IN4=0) */
    (void)GPIO_Set(L298N_IN1_PIN, 1UL);
    (void)GPIO_Set(L298N_IN2_PIN, 0UL);
    (void)GPIO_Set(L298N_IN3_PIN, 1UL);
    (void)GPIO_Set(L298N_IN4_PIN, 0UL);

    MotorControl_ApplyDutyPermille(L298N_PDM_CH_A, &gMotorPdmConfigA, 0U);
    MotorControl_ApplyDutyPermille(L298N_PDM_CH_B, &gMotorPdmConfigB, 0U);

    gMotorControlInited = 1U;
}

void MotorControl_SetSpeed
(
    uint32                              uiSpeed
)
{
    if(gMotorControlInited == 0U)
    {
        MotorControl_Init();
    }

    MotorControl_SetDualSpeed(uiSpeed, uiSpeed);
}

void MotorControl_SetDualSpeed
(
    uint32                              uiLeftSpeed,
    uint32                              uiRightSpeed
)
{
    if(gMotorControlInited == 0U)
    {
        MotorControl_Init();
    }

    MotorControl_SetSingleMotorSpeed(L298N_PDM_CH_A, &gMotorPdmConfigA, L298N_IN1_PIN, L298N_IN2_PIN, uiLeftSpeed);
    MotorControl_SetSingleMotorSpeed(L298N_PDM_CH_B, &gMotorPdmConfigB, L298N_IN3_PIN, L298N_IN4_PIN, uiRightSpeed);
}

#endif  // ( MCU_BSP_SUPPORT_MOTOR_PDM == 1 )
