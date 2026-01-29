// SPDX-License-Identifier: Apache-2.0

#include "servo_control.h"
#include "debug.h"
#include <math.h>

static uint32 g_current_pulse[4] = {0, 0, 0, 0};
static uint8 g_servo_initialized = 0;

// ✅ PDM 채널 정의 (인접 GPIO-A 핀: GPA10~13)
static const uint8 SERVO_PDM_CHANNEL[4] = {
    SERVO_FL_CHANNEL,  // 0: Front Left  = PDM0-A (GPA10)
    SERVO_FR_CHANNEL,  // 1: Front Right = PDM1-A (GPA11)
    SERVO_RL_CHANNEL,  // 2: Rear Left   = PDM2-A (GPA12)
    SERVO_RR_CHANNEL   // 3: Rear Right  = PDM3-A (GPA13)
};

static const uint32 SERVO_GPIO_PORT[4] = {
    SERVO_FL_PORT,  // GPIO-A
    SERVO_FR_PORT,  // GPIO-A
    SERVO_RL_PORT,  // GPIO-A
    SERVO_RR_PORT   // GPIO-A
};

void Servo_Init(void)
{
    SALRetCode_t ret;
    PDMModeConfig_t pwm_config;

    mcu_printf("\n[SERVO] Initializing with adjacent GPIO pins (GPA10~13)...\n");
    mcu_printf("  FL: PDM%d-A (GPA10)\n", SERVO_FL_CHANNEL);
    mcu_printf("  FR: PDM%d-A (GPA11)\n", SERVO_FR_CHANNEL);
    mcu_printf("  RL: PDM%d-A (GPA12)\n", SERVO_RL_CHANNEL);
    mcu_printf("  RR: PDM%d-A (GPA13)\n", SERVO_RR_CHANNEL);

    PDM_Init();

    for (uint8 i = 0; i < 4; i++) {
        uint8 pdm_ch = SERVO_PDM_CHANNEL[i];
        
        pwm_config.mcPortNumber      = SERVO_GPIO_PORT[i];
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

        ret = PDM_SetConfig(pdm_ch, &pwm_config);
        if (ret != SAL_RET_SUCCESS) {
            mcu_printf("[SERVO] Servo%d SetConfig failed: %d\n", i, ret);
            continue;
        }

        ret = PDM_Enable(pdm_ch, PMM_OFF);
        if (ret != SAL_RET_SUCCESS) {
            mcu_printf("[SERVO] Servo%d Enable failed: %d\n", i, ret);
            continue;
        }

        g_current_pulse[i] = SERVO_NEUTRAL_PULSE_NS;
        mcu_printf("[SERVO] Servo%d OK (PDM%d, GPA1%d, 1500us)\n", i, pdm_ch, 10+i);
    }

    g_servo_initialized = 1;
    mcu_printf("[SERVO] All servos initialized!\n\n");
}

void Servo_SetPulse(uint8 servo_idx, uint32 pulse_ns)
{
    SALRetCode_t ret;
    PDMModeConfig_t pwm_config;
    uint32 retry;

    if (servo_idx > 3) return;
    if (!g_servo_initialized) return;

    if (pulse_ns < SERVO_MIN_PULSE_NS) pulse_ns = SERVO_MIN_PULSE_NS;
    if (pulse_ns > SERVO_MAX_PULSE_NS) pulse_ns = SERVO_MAX_PULSE_NS;
    if (g_current_pulse[servo_idx] == pulse_ns) return;

    uint8 pdm_ch = SERVO_PDM_CHANNEL[servo_idx];

    pwm_config.mcPortNumber      = SERVO_GPIO_PORT[servo_idx];
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

    ret = PDM_Disable(pdm_ch, PMM_OFF);
    if (ret != SAL_RET_SUCCESS) return;

    retry = 10000;
    while (retry > 0) {
        if (PDM_GetChannelStatus(pdm_ch) == PDM_OFF) break;
        retry--;
        for(volatile uint32 i = 0; i < 10; i++);
    }
    if (retry == 0) {
        (void)PDM_Enable(pdm_ch, PMM_OFF);
        return;
    }

    ret = PDM_SetConfig(pdm_ch, &pwm_config);
    if (ret != SAL_RET_SUCCESS) {
        (void)PDM_Enable(pdm_ch, PMM_OFF);
        return;
    }

    ret = PDM_Enable(pdm_ch, PMM_OFF);
    if (ret == SAL_RET_SUCCESS) {
        g_current_pulse[servo_idx] = pulse_ns;
    }
}

void Servo_SetAngle(uint8 servo_idx, float angle)
{
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;

    uint32 pulse_ns = SERVO_MIN_PULSE_NS +
                      (uint32)((angle / SERVO_MAX_ANGLE) *
                               (SERVO_MAX_PULSE_NS - SERVO_MIN_PULSE_NS));

    Servo_SetPulse(servo_idx, pulse_ns);
}

// servo_control.c에 추가

void Servo_SetPosition_All(float angles[4])
{
    PDMModeConfig_t pwm_config;
    uint32 pulse_ns[4];
    uint8 pdm_ch[4];
    
    // ✅ 각도 → 펄스 변환
    for (uint8 i = 0; i < 4; i++) {
        if (angles[i] < SERVO_MIN_ANGLE) angles[i] = SERVO_MIN_ANGLE;
        if (angles[i] > SERVO_MAX_ANGLE) angles[i] = SERVO_MAX_ANGLE;
        
        pulse_ns[i] = SERVO_MIN_PULSE_NS +
                      (uint32)((angles[i] / SERVO_MAX_ANGLE) *
                               (SERVO_MAX_PULSE_NS - SERVO_MIN_PULSE_NS));
        
        pdm_ch[i] = SERVO_PDM_CHANNEL[i];
        
        // 변화 없으면 스킵
        if (g_current_pulse[i] == pulse_ns[i]) {
            pdm_ch[i] = 0xFF;  // 마크
        }
    }
    
    // ✅ Disable all (변경된 채널만)
    for (uint8 i = 0; i < 4; i++) {
        if (pdm_ch[i] != 0xFF) {
            PDM_Disable(pdm_ch[i], PMM_OFF);
        }
    }
    
    // ✅ Wait IDLE
    for (uint8 i = 0; i < 4; i++) {
        if (pdm_ch[i] != 0xFF) {
            uint32 retry = 10000;
            while (retry > 0) {
                if (PDM_GetChannelStatus(pdm_ch[i]) == PDM_OFF) break;
                retry--;
                for(volatile uint32 j = 0; j < 10; j++);
            }
        }
    }
    
    // ✅ SetConfig all
    for (uint8 i = 0; i < 4; i++) {
        if (pdm_ch[i] != 0xFF) {
            pwm_config.mcPortNumber      = SERVO_GPIO_PORT[i];
            pwm_config.mcOperationMode   = PDM_OUTPUT_MODE_PHASE_1;
            pwm_config.mcClockDivide     = 0;
            pwm_config.mcOutSignalInIdle = 0;
            pwm_config.mcInversedSignal  = 0;
            pwm_config.mcOutputCtrl      = 0x05;
            pwm_config.mcLoopCount       = 0;
            pwm_config.mcPeriodNanoSec1  = SERVO_PWM_PERIOD_NS;
            pwm_config.mcDutyNanoSec1    = pulse_ns[i];
            pwm_config.mcPeriodNanoSec2  = 0;
            pwm_config.mcDutyNanoSec2    = 0;
            
            PDM_SetConfig(pdm_ch[i], &pwm_config);
        }
    }
    
    // ✅ Enable all (동시!)
    for (uint8 i = 0; i < 4; i++) {
        if (pdm_ch[i] != 0xFF) {
            PDM_Enable(pdm_ch[i], PMM_OFF);
            g_current_pulse[i] = pulse_ns[i];
        }
    }
}

void Servo_SetNeutral_All(void)
{
    float neutral[4] = {90.0f, 90.0f, 90.0f, 90.0f};
    Servo_SetPosition_All(neutral);
}