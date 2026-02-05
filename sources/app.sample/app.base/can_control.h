// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
*   FileName : can_control.h
*
*   Copyright (c) Telechips Inc.
*
*   Description :
*       CAN control for speed and steering (left/right).
*
***************************************************************************************************
*/

#ifndef MCU_BSP_CAN_CONTROL_HEADER
#define MCU_BSP_CAN_CONTROL_HEADER

#if ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )

#include "sal_internal.h"

/* CAN ID Definitions */
#define CAN_CTRL_ID_SPEED               (0x20U)
#define CAN_CTRL_ID_STEERING            (0x21U)

/* Steering Data Values */
#define CAN_CTRL_DATA_LEFT              (0x00U)
#define CAN_CTRL_DATA_RIGHT             (0x01U)

void CAN_ControlInit
(
    uint8                               ucChannel
);

void CAN_ControlPoll
(
    void
);

void CAN_ControlSendSpeed
(
    void
);

#endif  // ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )

#endif  // MCU_BSP_CAN_CONTROL_HEADER
