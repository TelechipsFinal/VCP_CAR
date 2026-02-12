// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
*   FileName : can_control.c
*
*   Copyright (c) Telechips Inc.
*
*   Description :
*       CAN control for speed and steering (left/right).
*
***************************************************************************************************
*/

#if ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )

#include "can_control.h"
#include "can_config.h"
#include "can.h"
#include "can_reg.h"
#include "can_drv.h"
#include "debug.h"
#include "main.h"

#if ( MCU_BSP_SUPPORT_MOTOR_PDM == 1 )
#include "motor_control.h"
#endif

static uint8 gTargetChannel = 0U;
static uint32 gLastSpeed = 0U;
static uint32 gRxSemId = 0U;
static uint8 gRxSemReady = 0U;

static void CAN_ControlCallbackRx
(
    uint8                               ucCh,
    uint32                              uiRxIndex,
    CANMessageBufferType_t              uiRxBufferType,
    CANErrorType_t                      uiError
)
{
    if( ( uiError == CAN_ERROR_NONE ) && ( ucCh == gTargetChannel ) )
    {
        if( gRxSemReady != 0U )
        {
            (void)SAL_SemaphoreRelease(gRxSemId);
        }
    }

    ( void ) uiRxIndex;
    ( void ) uiRxBufferType;
}

static void CAN_ControlProcessMessage
(
    const CANMessage_t *                psRxMsg
)
{
    uint32 idxDump;

    if( psRxMsg == NULL_PTR )
    {
        return;
    }

    mcu_printf( "[CAN CTRL] RX len=%d data=", (int)psRxMsg->mDataLength );
    for( idxDump = 0U; idxDump < psRxMsg->mDataLength; idxDump++ )
    {
        mcu_printf( "%02X", psRxMsg->mData[idxDump] );
        if( (idxDump + 1U) < psRxMsg->mDataLength )
        {
            mcu_printf( " " );
        }
    }
    mcu_printf( "\n" );

    if( psRxMsg->mDataLength == 0U )
    {
        mcu_printf( "[CAN CTRL] Empty data (ID: 0x%X)\n", psRxMsg->mId );
        return;
    }

    switch( psRxMsg->mId )
    {
        case CAN_CTRL_ID_SPEED:
        {
            uint32 speed = 0U;

            if( psRxMsg->mDataLength > 0U )
            {
                uint32 idx;
                uint32 parsed = 0U;
                uint8 isAscii = 1U;

                for( idx = 0U; idx < psRxMsg->mDataLength; idx++ )
                {
                    uint8 ch = psRxMsg->mData[idx];
                    if( (ch < (uint8)'0') || (ch > (uint8)'9') )
                    {
                        isAscii = 0U;
                        break;
                    }
                }

                if( isAscii != 0U )
                {
                    for( idx = 0U; idx < psRxMsg->mDataLength; idx++ )
                    {
                        parsed = (parsed * 10U) + (uint32)(psRxMsg->mData[idx] - (uint8)'0');
                    }
                    speed = parsed;
                }
                else
                {
                    if( psRxMsg->mDataLength >= 2U )
                    {
                        speed = (uint32)psRxMsg->mData[0] | ((uint32)psRxMsg->mData[1] << 8U);
                    }
                    else
                    {
                        speed = (uint32)psRxMsg->mData[0];
                    }
                }
            }

            if( speed > 1000U )
            {
                speed = 1000U;
            }

            SAL_CoreCriticalEnter();
            gDriveCmd.speed = speed;
            gDriveCmd.speed_valid = 1U;
            SAL_GetTickCount(&gDriveCmd.last_rx_tick);
            SAL_CoreCriticalExit();

            gLastSpeed = speed;
            mcu_printf( "[CAN CTRL] Speed: %d\n", (int)speed );
            break;
        }

        case CAN_CTRL_ID_STEERING:
        {
            uint8 steer = psRxMsg->mData[0];

            if( (steer >= (uint8)'0') && (steer <= (uint8)'9') )
            {
                steer = (uint8)(steer - (uint8)'0');
            }

            SAL_CoreCriticalEnter();
            gDriveCmd.steering = steer;
            gDriveCmd.steering_valid = 1U;
            SAL_GetTickCount(&gDriveCmd.last_rx_tick);
            SAL_CoreCriticalExit();

            mcu_printf( "[CAN CTRL] Steering: 0x%X\n", steer );
            break;
        }
        case CAN_CTRL_ID_SPEED_BOOST:
        {
            if (psRxMsg->mDataLength == 0U) {
                mcu_printf("[CAN CTRL] SpeedBoost empty data (ID: 0x%X)\n", psRxMsg->mId);
                break;
            }

            uint8 boost = psRxMsg->mData[0];
            if ((boost >= (uint8)'0') && (boost <= (uint8)'9')) {
                boost = (uint8)(boost - (uint8)'0');
            }

            if (boost == 1U) {
                SAL_CoreCriticalEnter();
                gDriveCmd.speed_boost_req = 1U;
                gDriveCmd.speed_boost_valid = 1U;
                SAL_GetTickCount(&gDriveCmd.last_rx_tick);
                SAL_CoreCriticalExit();
                mcu_printf("[CAN CTRL] SpeedBoost: 1\n");
            } else {
                mcu_printf("[CAN CTRL] SpeedBoost ignored: 0x%X\n", boost);
            }
            break;
        }
        case CAN_CTRL_ID_DRIVEMODE:
        {
            if (psRxMsg->mDataLength == 0U) {
                mcu_printf("[CAN CTRL] DriveMode empty data (ID: 0x%X)\n", psRxMsg->mId);
                break;
            }

            uint8 mode = psRxMsg->mData[0];

            SAL_CoreCriticalEnter();
            gDriveCmd.drivemode = mode;
            gDriveCmd.drivemode_valid = 1U;
            SAL_GetTickCount(&gDriveCmd.last_rx_tick);
            SAL_CoreCriticalExit();

            mcu_printf("[CAN CTRL] DriveMode: 0x%X\n", mode);
            break;
        }
        case CAN_CTRL_ID_IW_TEST_MODE:
        {
            if (psRxMsg->mDataLength == 0U) {
                mcu_printf("[CAN CTRL] IW Test empty data (ID: 0x%X)\n", psRxMsg->mId);
                break;
            }

            uint8 mode = psRxMsg->mData[0];
            if ((mode >= (uint8)'0') && (mode <= (uint8)'9')) {
                mode = (uint8)(mode - (uint8)'0');
            }

            SAL_CoreCriticalEnter();
            gDriveCmd.iw_test_mode = (mode != 0U) ? 1U : 0U;
            gDriveCmd.iw_test_valid = 1U;
            SAL_GetTickCount(&gDriveCmd.last_rx_tick);
            SAL_CoreCriticalExit();

            mcu_printf("[CAN CTRL] IW Test Mode: %d\n", (int)((mode != 0U) ? 1 : 0));
            break;
        }
        case CAN_CTRL_ID_SPEED_LIMIT:
        {
            if (psRxMsg->mDataLength == 0U) {
                mcu_printf("[CAN CTRL] SpeedLimit empty data (ID: 0x%X)\n", psRxMsg->mId);
                break;
            }

            mcu_printf("[CAN CTRL] Speed limit cmd ignored in task-based mode\n");
            break;
        }

        default:
        {
            mcu_printf( "[CAN CTRL] Unknown ID: 0x%X\n", psRxMsg->mId );
            break;
        }
    }
}

void CAN_ControlInit
(
    uint8                               ucChannel
)
{
    gTargetChannel = ucChannel;
    gRxSemReady = 0U;

    if( SAL_SemaphoreCreate(&gRxSemId, (const uint8 *)"CAN_RX_SEM", 0UL, SAL_OPT_BLOCKING) == SAL_RET_SUCCESS )
    {
        gRxSemReady = 1U;
    }
    else
    {
        mcu_printf("[CAN CTRL] RX semaphore create failed, fallback to polling mode\n");
    }

#if ( MCU_BSP_SUPPORT_MOTOR_PDM == 1 )
    MotorControl_Init();
#endif

    ( void ) CAN_RegisterCallbackFunctionRx( &CAN_ControlCallbackRx );

    mcu_printf( "[CAN CTRL] Initialized for Channel %d\n", ucChannel );
}

void CAN_ControlPoll
(
    void
)
{
    uint32          uiRxMsgNum;
    CANMessage_t    sRxMsg;

    uiRxMsgNum = CAN_CheckNewRxMessage( gTargetChannel );
    if (uiRxMsgNum > 0UL) {
        mcu_printf("[CAN CTRL] RX pending: %lu\n", (unsigned long)uiRxMsgNum);
    }

    while( uiRxMsgNum > 0UL )
    {
        ( void ) SAL_MemSet( &sRxMsg, 0, sizeof( CANMessage_t ) );

        if( CAN_GetNewRxMessage( gTargetChannel, &sRxMsg ) == CAN_ERROR_NONE )
        {
            CAN_ControlProcessMessage( &sRxMsg );
        }

        uiRxMsgNum = CAN_CheckNewRxMessage( gTargetChannel );
    }
}

uint8 CAN_ControlIsRxSemaphoreReady
(
    void
)
{
    return gRxSemReady;
}

uint32 CAN_ControlGetRxSemaphoreId
(
    void
)
{
    return gRxSemId;
}

void CAN_ControlSendSpeed
(
    void
)
{
    CANMessage_t    sTxMsg;
    uint8           ucTxBufferIndex;

    (void)SAL_MemSet(&sTxMsg, 0, sizeof(CANMessage_t));

    sTxMsg.mBufferType           = CAN_TX_BUFFER_TYPE_FIFO;
    sTxMsg.mBufferIndex          = 0U;
    sTxMsg.mErrorStateIndicator  = 0U;
    sTxMsg.mExtendedId           = 0U;
    sTxMsg.mRemoteTransmitRequest= 0U;
    sTxMsg.mId                   = 0x30U;
    sTxMsg.mFDFormat             = 0U;
    sTxMsg.mBitRateSwitching     = 1U;
    sTxMsg.mMessageMarker        = 0xFFU;
    sTxMsg.mEventFIFOControl     = 1U;
    sTxMsg.mDataLength           = 2U;

    sTxMsg.mData[0] = (uint8)(gLastSpeed & 0xFFU);
    sTxMsg.mData[1] = (uint8)((gLastSpeed >> 8U) & 0xFFU);

    (void)CAN_SendMessage(gTargetChannel, &sTxMsg, &ucTxBufferIndex);
}

#endif  // ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )
