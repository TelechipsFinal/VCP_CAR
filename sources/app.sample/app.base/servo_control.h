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


#define SERVO_FL_CHANNEL        0    // PDM0-A → GPA(10)
#define SERVO_FR_CHANNEL        1    // PDM1-A → GPA(11)
#define SERVO_RL_CHANNEL        2    // PDM2-A → GPA(12)
#define SERVO_RR_CHANNEL        3    // PDM3-A → GPA(13)


#define SERVO_FL_PORT           GPIO_PERICH_CH0   // GPIO-A
#define SERVO_FR_PORT           GPIO_PERICH_CH0   // GPIO-A
#define SERVO_RL_PORT           GPIO_PERICH_CH0   // GPIO-A
#define SERVO_RR_PORT           GPIO_PERICH_CH0   // GPIO-A


#define SERVO_PWM_PERIOD_NS      (20000000UL)
#define SERVO_MIN_PULSE_NS       (500000UL)
#define SERVO_MAX_PULSE_NS       (2500000UL)
#define SERVO_NEUTRAL_PULSE_NS   (1500000UL)

#define SERVO_MIN_ANGLE          (0.0f)
#define SERVO_MAX_ANGLE          (180.0f)
#define SERVO_NEUTRAL_ANGLE      (90.0f)

void Servo_Init(void);
void Servo_SetPulse(uint8 channel, uint32 pulse_ns);
void Servo_SetAngle(uint8 channel, float angle);
void Servo_SetPosition_All(float angles[4]);
void Servo_SetNeutral_All(void);

#endif // SERVO_CONTROL_H