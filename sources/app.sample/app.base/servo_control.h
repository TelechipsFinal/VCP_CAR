// SPDX-License-Identifier: Apache-2.0
#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <sal_api.h>
#include <pdm.h>
#include <gpio.h>

/*
***************************************************************************************************
*                                         SERVO CONFIGURATION
***************************************************************************************************
*/

// PDM 채널 (PDM0-A~PDM3-A → GPA10~13)
#define SERVO_FL_CHANNEL        0
#define SERVO_FR_CHANNEL        1
#define SERVO_RL_CHANNEL        2
#define SERVO_RR_CHANNEL        3

// 포트 (GPIO-A)
#define SERVO_FL_PORT           GPIO_PERICH_CH0
#define SERVO_FR_PORT           GPIO_PERICH_CH0
#define SERVO_RL_PORT           GPIO_PERICH_CH0
#define SERVO_RR_PORT           GPIO_PERICH_CH0

// 50Hz
#define SERVO_PWM_PERIOD_NS     (20000000UL)

// 펄스 범위 (너가 쓰던 널널한 범위 유지)
#define SERVO_MIN_PULSE_NS      (1200000UL)
#define SERVO_MAX_PULSE_NS      (2450000UL)
#define SERVO_NEUTRAL_PULSE_NS  (1825000UL)

// 내부: PDM tick 계산 (클럭/분주 고정)
#define PDM_PERI_CLK_HZ         (125000000UL)
#define PDM_CLKDIV              (0UL)      // 0 => 내부에서 /2
#define PDM_TICK_NS             (16UL)     // 1e9 / (125MHz/2) = 16ns (정확)

/* 편의 매크로 */
#define US_TO_NS(us)   ((uint32)((us) * 1000UL))
#define NS_TO_US(ns)   ((uint32)((ns) / 1000UL))

void   Servo_Init(void);

void   Servo_SetPulseNs(uint8 servo_idx, uint32 pulse_ns);
void   Servo_SetPulseAllNs(const uint32 pulse_ns_in[4]);
void   Servo_SetNeutral_All(void);

uint32 Servo_GetPulseNs(uint8 servo_idx);

#endif // SERVO_CONTROL_H
