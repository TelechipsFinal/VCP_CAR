// SPDX-License-Identifier: Apache-2.0

#include "servo_control.h"
#include "debug.h"
#include <math.h>

/*
***************************************************************************************************
*                                         LOCAL VARIABLES
***************************************************************************************************
*/

static uint32 g_current_pulse[4] = {0, 0, 0, 0};

/*
***************************************************************************************************
*                                         SERVO INITIALIZATION
***************************************************************************************************
*/

void Servo_Init(void)
{
    SALRetCode_t ret;
    PDMModeConfig_t pwm_config;

    mcu_printf("[SERVO] Initializing PDM for 4-channel servo control...\n");

    PDM_Init();

    // ✅ 각 채널 개별 초기화
    for (uint8 ch = 0; ch < 4; ch++) {
        pwm_config.mcPortNumber      = GPIO_PERICH_CH0;
        pwm_config.mcOperationMode   = PDM_OUTPUT_MODE_PHASE_1;
        pwm_config.mcClockDivide     = 0;
        pwm_config.mcOutSignalInIdle = 0;
        pwm_config.mcInversedSignal  = 0;
        pwm_config.mcOutputCtrl      = 0x05;
        pwm_config.mcLoopCount       = 0;
        pwm_config.mcPeriodNanoSec1  = SERVO_PWM_PERIOD_NS;
        pwm_config.mcDutyNanoSec1    = SERVO_NEUTRAL_PULSE_NS;
        pwm_config.mcPeriodNanoSec2  = 0;
        pwm_config.mcDutyNanoSec2    = 0;

        ret = PDM_SetConfig(ch, &pwm_config);
        if (ret != SAL_RET_SUCCESS) {
            mcu_printf("[ERROR] PDM CH%d config failed (ret=%d)\n", ch, ret);
            continue;
        }

        ret = PDM_Enable(ch, PMM_OFF);
        if (ret != SAL_RET_SUCCESS) {
            mcu_printf("[ERROR] PDM CH%d enable failed (ret=%d)\n", ch, ret);
            continue;
        }

        g_current_pulse[ch] = SERVO_NEUTRAL_PULSE_NS;
        mcu_printf("[SERVO] CH%d initialized (1500us)\n", ch);
    }

    mcu_printf("[SERVO] All channels initialized\n\n");
}

/*
***************************************************************************************************
*                                         SERVO CONTROL
***************************************************************************************************
*/

void Servo_SetPulse(uint8 channel, uint32 pulse_ns)
{
    SALRetCode_t ret;
    PDMModeConfig_t pwm_config;
    uint32 retry = 0;

    if (channel > 3) return;

    if (pulse_ns < SERVO_MIN_PULSE_NS) pulse_ns = SERVO_MIN_PULSE_NS;
    if (pulse_ns > SERVO_MAX_PULSE_NS) pulse_ns = SERVO_MAX_PULSE_NS;

    // ✅ 값이 같으면 스킵 (불필요한 재설정 방지)
    if (g_current_pulse[channel] == pulse_ns) return;

    // ✅ PDM 설정 구조체
    pwm_config.mcPortNumber      = GPIO_PERICH_CH0;
    pwm_config.mcOperationMode   = PDM_OUTPUT_MODE_PHASE_1;
    pwm_config.mcClockDivide     = 0;
    pwm_config.mcOutSignalInIdle = 0;
    pwm_config.mcInversedSignal  = 0;
    pwm_config.mcOutputCtrl      = 0x05;
    pwm_config.mcLoopCount       = 0;
    pwm_config.mcPeriodNanoSec1  = SERVO_PWM_PERIOD_NS;
    pwm_config.mcDutyNanoSec1    = pulse_ns;
    pwm_config.mcPeriodNanoSec2  = 0;
    pwm_config.mcDutyNanoSec2    = 0;

    // ✅ STEP 1: Disable
    ret = PDM_Disable(channel, PMM_OFF);
    if (ret != SAL_RET_SUCCESS) {
        return;
    }

    // ✅ STEP 2: IDLE 대기 (최대 1000회 = ~100us)
    retry = 1000;
    while (retry > 0) {
        if (PDM_GetChannelStatus(channel) == PDM_OFF) {
            break;
        }
        retry--;
        for(volatile uint32 i = 0; i < 10; i++);
    }

    if (retry == 0) {
        // IDLE 도달 실패 - 강제 재활성화
        (void)PDM_Enable(channel, PMM_OFF);
        return;
    }

    // ✅ STEP 3: SetConfig
    ret = PDM_SetConfig(channel, &pwm_config);
    if (ret != SAL_RET_SUCCESS) {
        (void)PDM_Enable(channel, PMM_OFF);
        return;
    }

    // ✅ STEP 4: Enable
    ret = PDM_Enable(channel, PMM_OFF);
    if (ret != SAL_RET_SUCCESS) {
        return;
    }

    g_current_pulse[channel] = pulse_ns;
}

void Servo_SetAngle(uint8 channel, float angle)
{
    uint32 pulse_ns;

    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;

    pulse_ns = SERVO_MIN_PULSE_NS +
               (uint32)((angle / SERVO_MAX_ANGLE) *
                        (SERVO_MAX_PULSE_NS - SERVO_MIN_PULSE_NS));

    Servo_SetPulse(channel, pulse_ns);
}

void Servo_SetPosition_All(float angles[4])
{
    for (uint8 i = 0; i < 4; i++) {
        Servo_SetAngle(i, angles[i]);
    }
}

void Servo_SetNeutral_All(void)
{
    float neutral[4] = {
        SERVO_NEUTRAL_ANGLE,
        SERVO_NEUTRAL_ANGLE,
        SERVO_NEUTRAL_ANGLE,
        SERVO_NEUTRAL_ANGLE
    };

    Servo_SetPosition_All(neutral);
}