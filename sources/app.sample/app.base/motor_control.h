// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
*   FileName : motor_control.h
*
*   Copyright (c) Telechips Inc.
*
*   Description :
*       L298N motor control using PDM (PWM) and GPIO.
*
***************************************************************************************************
*/

#ifndef MCU_BSP_MOTOR_CONTROL_HEADER
#define MCU_BSP_MOTOR_CONTROL_HEADER

#if ( MCU_BSP_SUPPORT_MOTOR_PDM == 1 )

#include "gpio.h"

#if ( MCU_BSP_SUPPORT_MOTOR_PDM != 1 )
    #error MCU_BSP_SUPPORT_MOTOR_PDM value must be 1.
#endif  // ( MCU_BSP_SUPPORT_MOTOR_PDM != 1 )

/* L298N motor driver pins (A0~A11 header) */
#define L298N_IN1_PIN                   (GPIO_GPC(2UL))   /* A2 (Motor A IN1) */
#define L298N_IN2_PIN                   (GPIO_GPC(3UL))   /* A3 (Motor A IN2) */
#define L298N_IN3_PIN                   (GPIO_GPC(5UL))   /* A4 (Motor B IN3) */
#define L298N_IN4_PIN                   (GPIO_GPC(4UL))   /* A5 (Motor B IN4) */

/* L298N ENA/ENB use PDM output (header 39/38) */
#define L298N_PDM_CH_A                  (4UL)             /* CH3 + ch4 -> GPK12 (header 39), ENA */
#define L298N_PDM_CH_B                  (5UL)             /* CH3 + ch5 -> GPK13 (header 38), ENB */
#define L298N_PDM_OUT_SEL_CH            (GPIO_PERICH_CH3) /* GPIO_K group */

void MotorControl_Init
(
    void
);

void MotorControl_SetSpeed
(
    uint32                              uiSpeed
);

void MotorControl_SetDualSpeed
(
    uint32                              uiLeftSpeed,
    uint32                              uiRightSpeed
);

#endif  // ( MCU_BSP_SUPPORT_MOTOR_PDM == 1 )

#endif  // MCU_BSP_MOTOR_CONTROL_HEADER
