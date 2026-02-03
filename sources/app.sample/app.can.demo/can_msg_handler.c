// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
*   FileName : can_msg_handler.c
*
*   Copyright (c) Telechips Inc.
*
*   Description : CAN Message Handler for STM32 commands
*
***************************************************************************************************
*/

#if ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )

/**************************************************************************************************
*                                           INCLUDE FILES
**************************************************************************************************/

#include <app_cfg.h>
#include "bsp.h"
#include "debug.h"
#include "can_config.h"
#include "can_reg.h"
#include "can.h"
#include "can_drv.h"
#include "can_msg_handler.h"


/**************************************************************************************************
*                                            DEFINITIONS
**************************************************************************************************/

#define CAN_MSG_HANDLER_TASK_STK_SIZE   (512U)
#define CAN_MSG_HANDLER_TASK_PRIO       (SAL_PRIO_CAN_DEMO)

/* CAN ID Definitions from STM32 */
#define CAN_ID_START        0x20    // 'q' key - Start command
#define CAN_ID_CONTROL      0x21    // 'w'/'s' keys - ACC/DEC command
#define CAN_ID_DIRECTION    0x22    // 'a'/'d' keys - LEFT/RIGHT command

/* CAN Data Values */
#define CAN_DATA_START      0x01
#define CAN_DATA_ACC        0x01
#define CAN_DATA_DEC        0x00
#define CAN_DATA_LEFT       0x00
#define CAN_DATA_RIGHT      0x01


/**************************************************************************************************
*                                          LOCAL VARIABLES
**************************************************************************************************/

static CANFlagValue_t gRxFlag = CAN_FLAG_FALSE;
static uint8 gTargetChannel = 0;  // CAN Channel to monitor


/**************************************************************************************************
*                                        FUNCTION PROTOTYPES
**************************************************************************************************/

static void CAN_MsgHandlerCallbackRx
(
    uint8                               ucCh,
    uint32                              uiRxIndex,
    CANMessageBufferType_t              uiRxBufferType,
    CANErrorType_t                      uiError
);

static void CAN_MsgHandlerTask
(
    void *                              pArg
);

static void CAN_ProcessReceivedMessage
(
    const CANMessage_t *                psRxMsg
);


/**************************************************************************************************
*                                             FUNCTIONS
**************************************************************************************************/

/*
***************************************************************************************************
*                                   CAN_MsgHandlerCallbackRx
*
* Description : Callback function for CAN reception event
*
* Arguments   : ucCh            - CAN channel number
*               uiRxIndex       - Receive buffer index
*               uiRxBufferType  - Type of receive buffer
*               uiError         - Error code
*
* Returns     : None
***************************************************************************************************
*/
static void CAN_MsgHandlerCallbackRx
(
    uint8                               ucCh,
    uint32                              uiRxIndex,
    CANMessageBufferType_t              uiRxBufferType,
    CANErrorType_t                      uiError
)
{
    if( ( uiError == CAN_ERROR_NONE ) && ( ucCh == gTargetChannel ) )
    {
        gRxFlag = CAN_FLAG_TRUE;
    }

    ( void ) uiRxIndex;
    ( void ) uiRxBufferType;
}

/*
***************************************************************************************************
*                                   CAN_ProcessReceivedMessage
*
* Description : Process received CAN message and display appropriate message
*
* Arguments   : psRxMsg - Pointer to received CAN message
*
* Returns     : None
***************************************************************************************************
*/
static void CAN_ProcessReceivedMessage
(
    const CANMessage_t *                psRxMsg
)
{
    if( psRxMsg == NULL_PTR )
    {
        return;
    }

    /* Check if message has data */
    if( psRxMsg->mDataLength == 0U )
    {
        CAN_D( "[CAN MSG] Received message with no data (ID: 0x%X)\n", psRxMsg->mId );
        return;
    }

    /* Process message based on CAN ID */
    switch( psRxMsg->mId )
    {
        case CAN_ID_START:
        {
            if( psRxMsg->mData[0] == CAN_DATA_START )
            {
                mcu_printf( "\n========================================\n" );
                mcu_printf( "[CAN MSG] START Command Received!\n" );
                mcu_printf( "  -> System Starting...\n" );
                mcu_printf( "========================================\n\n" );
            }
            break;
        }

        case CAN_ID_CONTROL:
        {
            if( psRxMsg->mData[0] == CAN_DATA_ACC )
            {
                mcu_printf( "\n========================================\n" );
                mcu_printf( "[CAN MSG] ACCELERATE Command Received!\n" );
                mcu_printf( "  -> Speed Up\n" );
                mcu_printf( "========================================\n\n" );
            }
            else if( psRxMsg->mData[0] == CAN_DATA_DEC )
            {
                mcu_printf( "\n========================================\n" );
                mcu_printf( "[CAN MSG] DECELERATE Command Received!\n" );
                mcu_printf( "  -> Speed Down\n" );
                mcu_printf( "========================================\n\n" );
            }
            break;
        }

        case CAN_ID_DIRECTION:
        {
            if( psRxMsg->mData[0] == CAN_DATA_LEFT )
            {
                mcu_printf( "\n========================================\n" );
                mcu_printf( "[CAN MSG] LEFT Turn Command Received!\n" );
                mcu_printf( "  -> Turning Left\n" );
                mcu_printf( "========================================\n\n" );
            }
            else if( psRxMsg->mData[0] == CAN_DATA_RIGHT )
            {
                mcu_printf( "\n========================================\n" );
                mcu_printf( "[CAN MSG] RIGHT Turn Command Received!\n" );
                mcu_printf( "  -> Turning Right\n" );
                mcu_printf( "========================================\n\n" );
            }
            break;
        }

        default:
        {
            mcu_printf( "[CAN MSG] Unknown CAN ID received: 0x%X, Data: 0x%X\n", 
                       psRxMsg->mId, psRxMsg->mData[0] );
            break;
        }
    }
}

/*
***************************************************************************************************
*                                   CAN_MsgHandlerTask
*
* Description : Main task for handling CAN messages
*
* Arguments   : pArg - Task argument (not used)
*
* Returns     : None
***************************************************************************************************
*/
static void CAN_MsgHandlerTask
(
    void *                              pArg
)
{
    uint32          uiRxMsgNum;
    CANMessage_t    sRxMsg;

    ( void ) pArg;

    mcu_printf( "[CAN MSG HANDLER] Task Started - Monitoring Channel %d\n", gTargetChannel );

    while( 1 )
    {
        if( gRxFlag == CAN_FLAG_TRUE )
        {
            /* Check if there are new messages */
            uiRxMsgNum = CAN_CheckNewRxMessage( gTargetChannel );

            while( uiRxMsgNum > 0UL )
            {
                /* Get the message */
                ( void ) SAL_MemSet( &sRxMsg, 0, sizeof( CANMessage_t ) );
                
                if( CAN_GetNewRxMessage( gTargetChannel, &sRxMsg ) == CAN_ERROR_NONE )
                {
                    /* Process the received message */
                    CAN_ProcessReceivedMessage( &sRxMsg );
                }

                /* Check for more messages */
                uiRxMsgNum = CAN_CheckNewRxMessage( gTargetChannel );
            }

            gRxFlag = CAN_FLAG_FALSE;
        }

        /* Sleep for 10ms to prevent busy waiting */
        ( void ) SAL_TaskSleep( 10 );
    }
}

/*
***************************************************************************************************
*                                   CAN_MsgHandlerInit
*
* Description : Initialize CAN message handler
*
* Arguments   : ucChannel - CAN channel to monitor
*
* Returns     : 0 on success, -1 on failure
***************************************************************************************************
*/
sint32 CAN_MsgHandlerInit
(
    uint8                               ucChannel
)
{
    gTargetChannel = ucChannel;
    gRxFlag = CAN_FLAG_FALSE;

    /* Register callback function */
    ( void ) CAN_RegisterCallbackFunctionRx( &CAN_MsgHandlerCallbackRx );

    mcu_printf( "[CAN MSG HANDLER] Initialized for Channel %d\n", ucChannel );

    return 0;
}

/*
***************************************************************************************************
*                                   CAN_MsgHandlerCreateTask
*
* Description : Create CAN message handler task
*
* Arguments   : None
*
* Returns     : None
***************************************************************************************************
*/
void CAN_MsgHandlerCreateTask
(
    void
)
{
    static uint32 uiTaskID;
    static uint32 uiTaskStk[CAN_MSG_HANDLER_TASK_STK_SIZE];

    ( void ) SAL_TaskCreate( 
        &uiTaskID,
        ( const uint8 * ) "CAN Msg Handler",
        ( SALTaskFunc ) &CAN_MsgHandlerTask,
        ( uint32 * const ) &uiTaskStk[0],
        CAN_MSG_HANDLER_TASK_STK_SIZE,
        CAN_MSG_HANDLER_TASK_PRIO,
        NULL_PTR
    );

    mcu_printf( "[CAN MSG HANDLER] Task Created\n" );
}

#endif  // ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )