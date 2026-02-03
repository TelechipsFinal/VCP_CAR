// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
*   FileName : motor_drive.h
*
*   Copyright (c) Telechips Inc.
*
*   Description :
*       L298N motor driver sample using PDM (PWM) and GPIO.
*
***************************************************************************************************
*/

#ifndef MCU_BSP_MOTOR_DRIVE_HEADER
#define MCU_BSP_MOTOR_DRIVE_HEADER

#if ( MCU_BSP_SUPPORT_TEST_APP_PDM == 1 )

#include "gpio.h"

#if ( MCU_BSP_SUPPORT_DRIVER_PDM != 1 )
    #error MCU_BSP_SUPPORT_DRIVER_PDM value must be 1.
#endif  // ( MCU_BSP_SUPPORT_DRIVER_PDM != 1 )

/* L298N motor driver pins (set to match your board wiring) */
#define L298N_IN1_PIN                   (GPIO_GPC(12UL))
#define L298N_IN2_PIN                   (GPIO_GPC(13UL))

/* L298N ENA uses PDM output */
#define L298N_PDM_CH                    (0UL)
#define L298N_PDM_OUT_SEL_CH            (GPIO_PERICH_CH0)

void MotorDrive_L298NTest
(
    void
);

#endif  // ( MCU_BSP_SUPPORT_TEST_APP_PDM == 1 )

#endif  // MCU_BSP_MOTOR_DRIVE_HEADER
