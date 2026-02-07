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

typedef struct {
    uint32 speed;        // latest speed command
    uint8  steering;     // 0=LEFT,1=RIGHT, etc
    uint8  drivemode;    // 0=AUTO,1=COMFORT,2=NORMAL,3=SPORT
    uint8  speed_valid;
    uint8  steering_valid;
    uint8  drivemode_valid;
    uint32 last_rx_tick;
} DriveCmd_t;

/* Driving mode control (구조만 제공) */
extern void DriveMode_Set(DriveMode_t mode);
extern DriveMode_t DriveMode_Get(void);
extern const char *DriveMode_ToString(DriveMode_t mode);
extern void DriveMode_SetAutoEnabled(uint8 enable);
extern uint8 DriveMode_IsAutoEnabled(void);
extern void DriveMode_UpdateAutoBySpeed(uint32 speed);

extern DriveCmd_t gDriveCmd;


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
