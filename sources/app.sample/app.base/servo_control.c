// SPDX-License-Identifier: Apache-2.0
#include "servo_control.h"
#include "debug.h"

/* IMPORTANT:
 * - debug printf does NOT support %u/%lu/%f.
 * - Only use %d/%x/%X/%s/%c here.
 */

/* Register access for glitch-free update */
#include "bsp.h"        /* MCU_BSP_PWM_BASE (or equivalent) */
#include "pdm_dev.h"    /* PDM_BASE, offsets, PDM_GetPSTN1Reg(), ... */

static uint32 g_current_pulse[4] = {0, 0, 0, 0};
static uint8  g_servo_initialized = 0;

static const uint8 SERVO_PDM_CHANNEL[4] = {
    SERVO_FL_CHANNEL,
    SERVO_FR_CHANNEL,
    SERVO_RL_CHANNEL,
    SERVO_RR_CHANNEL
};

static const uint32 SERVO_GPIO_PORT[4] = {
    SERVO_FL_PORT,
    SERVO_FR_PORT,
    SERVO_RL_PORT,
    SERVO_RR_PORT
};

static inline uint32 clamp_pulse_ns(uint32 p)
{
    if (p < SERVO_MIN_PULSE_NS) return SERVO_MIN_PULSE_NS;
    if (p > SERVO_MAX_PULSE_NS) return SERVO_MAX_PULSE_NS;
    return p;
}

/*
 * Phase1 glitch-free duty update:
 * - Keep output ENABLED always (no Disable/Enable)
 * - Write PSTN1/PSTN2 (LOW/HIGH)
 * - Set VUP
 * - Toggle TRIG
 *
 * NOTE:
 * - SERVO_CONTROL_H defines PDM_TICK_NS = 16ns for 125MHz/2 with clkdiv=0
 * - This code assumes PDM_CLKDIV == 0 and PDM_TICK_NS matches HW
 */
static inline void PDM_UpdateDuty_Phase1_NoGlitch(uint32 uiChannel, uint32 duty_ns, uint32 period_ns)
{
    uint32 moduleId  = uiChannel / PDM_TOTAL_CH_PER_MODULE;  /* 0~2 */
    uint32 channelId = uiChannel % PDM_TOTAL_CH_PER_MODULE;  /* 0~3 */

    uint32 base = PDM_BASE + (moduleId * PDM_MODULE_OFFSET);

    uint32 pstn1_reg = base + PDM_GetPSTN1Reg(channelId);
    uint32 pstn2_reg = base + PDM_GetPSTN2Reg(channelId);

    /* ns -> tick */
    uint32 total_cnt = period_ns / PDM_TICK_NS;
    uint32 high_cnt  = duty_ns   / PDM_TICK_NS;

    if (high_cnt > total_cnt) high_cnt = total_cnt;

    uint32 low_cnt   = total_cnt - high_cnt;

    /* HW limit adjust (-2) to match driver behavior, prevent underflow */
    if (low_cnt  > PDM_HW_LIMIT_VALUE_2)  low_cnt  -= PDM_HW_LIMIT_VALUE_2; else low_cnt  = 0;
    if (high_cnt > PDM_HW_LIMIT_VALUE_2)  high_cnt -= PDM_HW_LIMIT_VALUE_2; else high_cnt = 0;

    /* write new low/high */
    SAL_WriteReg(low_cnt,  pstn1_reg);
    SAL_WriteReg(high_cnt, pstn2_reg);

    /* VUP + TRIG on OP_EN */
    uint32 op_en_reg = PDM_BASE + PDM_OP_EN_REG_OFFSET + (moduleId * PDM_MODULE_OFFSET);
    uint32 v;

    /* 1) VUP set */
    v = SAL_ReadReg(op_en_reg);
    v |= (1UL << PDM_GetOPENValueUpReg(channelId));
    SAL_WriteReg(v, op_en_reg);

    /* tiny delay (avoid back-to-back bus writes edge case) */
    for (volatile uint32 d = 0; d < 50UL; d++) { ; }

    /* 2) TRIG toggle (clear -> set) */
    v = SAL_ReadReg(op_en_reg);
    v &= ~(1UL << PDM_GetOPENTrigReg(channelId));
    SAL_WriteReg(v, op_en_reg);

    v |= (1UL << PDM_GetOPENTrigReg(channelId));
    SAL_WriteReg(v, op_en_reg);

    /* tiny delay */
    for (volatile uint32 d2 = 0; d2 < 50UL; d2++) { ; }
}

uint32 Servo_GetPulseNs(uint8 servo_idx)
{
    if (servo_idx > 3) return 0;
    return g_current_pulse[servo_idx];
}

void Servo_Init(void)
{
    SALRetCode_t ret;
    PDMModeConfig_t cfg;

    mcu_printf("\n[SERVO] Init (Phase1, NO disable/enable in runtime)\n");

    PDM_Init();

    for (uint8 i = 0; i < 4; i++)
    {
        uint8 ch = SERVO_PDM_CHANNEL[i];

        cfg.mcPortNumber      = SERVO_GPIO_PORT[i];
        cfg.mcOperationMode   = PDM_OUTPUT_MODE_PHASE_1;
        cfg.mcClockDivide     = PDM_CLKDIV;      /* fixed */
        cfg.mcOutSignalInIdle = 0;
        cfg.mcInversedSignal  = 0;
        cfg.mcOutputCtrl      = 0x05;
        cfg.mcLoopCount       = 0;
        cfg.mcPeriodNanoSec1  = SERVO_PWM_PERIOD_NS;
        cfg.mcDutyNanoSec1    = SERVO_NEUTRAL_PULSE_NS;
        cfg.mcPeriodNanoSec2  = 0;
        cfg.mcDutyNanoSec2    = 0;

        ret = PDM_SetConfig(ch, &cfg);
        if (ret != SAL_RET_SUCCESS)
        {
            mcu_printf("[SERVO] ch%d SetConfig fail:%d\n", (int)ch, (int)ret);
            continue;
        }

        ret = PDM_Enable(ch, PMM_OFF);
        if (ret != SAL_RET_SUCCESS)
        {
            mcu_printf("[SERVO] ch%d Enable fail:%d\n", (int)ch, (int)ret);
            continue;
        }

        g_current_pulse[i] = SERVO_NEUTRAL_PULSE_NS;

        mcu_printf("[SERVO] %d OK (PDM%d duty=%d ns)\n",
                   (int)i, (int)ch, (int)g_current_pulse[i]);
    }

    g_servo_initialized = 1;
    mcu_printf("[SERVO] Init done\n\n");
}

void Servo_SetPulseNs(uint8 servo_idx, uint32 pulse_ns)
{
    if (servo_idx > 3) return;
    if (!g_servo_initialized) return;

    pulse_ns = clamp_pulse_ns(pulse_ns);
    if (g_current_pulse[servo_idx] == pulse_ns) return;

    /* ✅ NO Disable/Enable, NO SetConfig */
    PDM_UpdateDuty_Phase1_NoGlitch((uint32)SERVO_PDM_CHANNEL[servo_idx],
                                  pulse_ns,
                                  SERVO_PWM_PERIOD_NS);

    g_current_pulse[servo_idx] = pulse_ns;
}

void Servo_SetPulseAllNs(const uint32 pulse_ns_in[4])
{
    if (!g_servo_initialized) return;

    for (uint8 i = 0; i < 4; i++)
    {
        uint32 p = clamp_pulse_ns(pulse_ns_in[i]);

        if (g_current_pulse[i] == p) continue;

        /* ✅ NO Disable/Enable, NO SetConfig */
        PDM_UpdateDuty_Phase1_NoGlitch((uint32)SERVO_PDM_CHANNEL[i],
                                      p,
                                      SERVO_PWM_PERIOD_NS);

        g_current_pulse[i] = p;
    }
}

void Servo_SetNeutral_All(void)
{
    uint32 p[4] = {
        SERVO_NEUTRAL_PULSE_NS,
        SERVO_NEUTRAL_PULSE_NS,
        SERVO_NEUTRAL_PULSE_NS,
        SERVO_NEUTRAL_PULSE_NS
    };
    Servo_SetPulseAllNs(p);
}
