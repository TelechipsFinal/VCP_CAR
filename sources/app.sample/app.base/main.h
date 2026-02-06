// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
*   FileName : main.h
*
*   Copyright (c) Telechips Inc.
*
*   Description :
*
*
***************************************************************************************************
*/

#ifndef MCU_BSP_MAIN_HEADER
#define MCU_BSP_MAIN_HEADER

#if ( MCU_BSP_SUPPORT_APP_BASE == 1 )

/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************
*/
#include <sal_internal.h>

/*
***************************************************************************************************
*                                             DEFINITIONS
***************************************************************************************************
*/
#define MAIN_UINT_MAX_NUM               (4294967295U)

/*
***************************************************************************************************
*                                         DRIVING MODE
***************************************************************************************************
*/
typedef enum {
    DRIVE_MODE_COMFORT = 0,
    DRIVE_MODE_NORMAL  = 1,
    DRIVE_MODE_SPORT   = 2
} DriveMode_t;

/* Driving mode control (구조만 제공) */
extern void DriveMode_Set(DriveMode_t mode);
extern DriveMode_t DriveMode_Get(void);
extern const char *DriveMode_ToString(DriveMode_t mode);
extern void DriveMode_SetAutoEnabled(uint8 enable);
extern uint8 DriveMode_IsAutoEnabled(void);
extern void DriveMode_UpdateAutoBySpeed(uint32 speed);


/*
***************************************************************************************************
*                                             GLOBAL VARIABLES
***************************************************************************************************
*/
extern uint32                           gALiveMsgOnOff;

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
extern void cmain
(
    void
);

#endif  // ( MCU_BSP_SUPPORT_APP_BASE == 1 )

#endif  //  MCU_BSP_MAIN_HEADER
