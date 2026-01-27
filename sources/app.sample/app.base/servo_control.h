// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
*   FileName : servo_control.h
*
*   Description : VCP-G PDM-based Servo Motor Control for Active Suspension
*
***************************************************************************************************
*/

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

// 서보 채널 정의 (PDM 채널)
#define SERVO_FL_CHANNEL        PDM_CHANNEL_0    // Front Left
#define SERVO_FR_CHANNEL        PDM_CHANNEL_1    // Front Right
#define SERVO_RL_CHANNEL        PDM_CHANNEL_2    // Rear Left
#define SERVO_RR_CHANNEL        PDM_CHANNEL_3    // Rear Right

// GPIO 핀 매핑 (GPIO-A 포트 사용)
#define SERVO_FL_GPIO           GPIO_GPA(10)     // PDM0 → GPA10
#define SERVO_FR_GPIO           GPIO_GPA(11)     // PDM1 → GPA11
#define SERVO_RL_GPIO           GPIO_GPA(16)     // PDM2 → GPA16
#define SERVO_RR_GPIO           GPIO_GPA(17)     // PDM3 → GPA17

// 서보 PWM 파라미터 (MG996R 기준)
#define SERVO_PWM_PERIOD_NS     (20000000UL)     // 20ms = 50Hz
#define SERVO_MIN_PULSE_NS      (1000000UL)       // 1ms (0도)
#define SERVO_MAX_PULSE_NS      (2000000UL)      // 2ms (180도)
#define SERVO_NEUTRAL_PULSE_NS  (1500000UL)      // 1.5ms (90도)

// 서보 각도 범위
#define SERVO_MIN_ANGLE         (0.0f)
#define SERVO_MAX_ANGLE         (180.0f)
#define SERVO_NEUTRAL_ANGLE     (90.0f)

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/

/**
 * @brief 서보 시스템 초기화 (PDM 기반)
 */
void Servo_Init(void);

/**
 * @brief 서보모터 PWM 펄스 설정
 * @param channel 채널 번호 (0~3)
 * @param pulse_ns 펄스 폭 (나노초)
 */
void Servo_SetPulse(uint8 channel, uint32 pulse_ns);

/**
 * @brief 서보모터 각도 설정
 * @param channel 채널 번호 (0~3)
 * @param angle 각도 (0~180도)
 */
void Servo_SetAngle(uint8 channel, float angle);

/**
 * @brief 모든 서보 각도 한번에 설정
 * @param angles 4개 서보 각도 배열 [FL, FR, RL, RR]
 */
void Servo_SetPosition_All(float angles[4]);

/**
 * @brief 모든 서보를 중립 위치로 설정
 */
void Servo_SetNeutral_All(void);

#endif // SERVO_CONTROL_H