
/*
***************************************************************************************************
*
*   FileName : ICM-20948.c
*
*   Copyright (c) Telechips Inc.
*
*   Description : AJH
*
*
***************************************************************************************************
*/

#include <sal_internal.h>
#include <stdint.h>
#include <string.h>
#include <gpsb.h>
#include <gpio.h>
#include <ICM_20948.h>


uint32 DMA_Semaphore_ID;
const uint8 DMA_Semaphore[] = "DMA_SEM";

IMU_Data IMU = {0};

static uint32 __attribute__((aligned(32))) tx_dma_buf[ICM20948_DMA_BUF_SIZE / 4];
static uint32 __attribute__((aligned(32))) rx_dma_buf[ICM20948_DMA_BUF_SIZE / 4];

static volatile uint8 DMA_ERROR = 0;

void ICM_20948_SelectBank(uint8 bank) {
    uint8 tx_buf[2] = {ICM20948_REG_BANK_SEL & 0x7F, bank << 4};
    uint8 rx_buf[2] = { 0 };
    
    GPSB_CsActivate(ICM_20948_CHANNEL, ICM_20948_CS_GPIO, FALSE);
    GPSB_Xfer(ICM_20948_CHANNEL, tx_buf, rx_buf, 2, GPSB_XFER_MODE_WITHOUT_INTERRUPT);
    GPSB_CsDeactivate(ICM_20948_CHANNEL, ICM_20948_CS_GPIO, FALSE);
    SAL_TaskSleep(1);
}

void ICM_20948_Write(uint8 addr, uint8 data)
{
    uint8 tx_buf[2] = { addr & 0x7F, data };
    uint8 rx_buf[2] = { 0 };

    GPSB_CsActivate(ICM_20948_CHANNEL, ICM_20948_CS_GPIO, FALSE);
    GPSB_Xfer(ICM_20948_CHANNEL, tx_buf, rx_buf, 2, GPSB_XFER_MODE_WITHOUT_INTERRUPT);
    GPSB_CsDeactivate(ICM_20948_CHANNEL, ICM_20948_CS_GPIO, FALSE);
    SAL_TaskSleep(1);
}

uint8 ICM_20948_Read(uint8 addr){
    uint8 tx_buf[2] = {addr | 0x80, 0x00};
    uint8 rx_buf[2] = {0};

    GPSB_CsActivate(ICM_20948_CHANNEL, ICM_20948_CS_GPIO, FALSE);
    GPSB_Xfer(ICM_20948_CHANNEL, tx_buf, rx_buf, 2, GPSB_XFER_MODE_WITHOUT_INTERRUPT);
    GPSB_CsDeactivate(ICM_20948_CHANNEL, ICM_20948_CS_GPIO, FALSE);
    SAL_TaskSleep(1);

    return rx_buf[1];
}

SALRetCode_t ICM_20948_ReadMultiple(uint8 addr, uint8 *data, uint8 len){
    uint8 *tx_buf = (uint8 *)tx_dma_buf;
    uint8 *rx_buf = (uint8 *)rx_dma_buf;
    SALRetCode_t ret;

    tx_buf[0] = addr | 0x80;
    memset(&tx_buf[1], 0, len);

    DMA_ERROR = 0;

    GPSB_CsActivate(ICM_20948_CHANNEL, ICM_20948_CS_GPIO, FALSE);

    ret = GPSB_AsyncXfer(ICM_20948_CHANNEL, (uint32 *)tx_buf, (uint32 *)rx_buf, len + 1, GPSB_XFER_MODE_WITH_INTR_WITH_DMA);

    if(ret != SAL_RET_SUCCESS) {
        GPSB_CsDeactivate(ICM_20948_CHANNEL, ICM_20948_CS_GPIO, FALSE);
        mcu_printf("[IMU] AsyncXfer failed\n");  // 디버그 추가
        return SAL_RET_FAILED;
    }
    
     ret = SAL_SemaphoreWait(DMA_Semaphore_ID, 200, SAL_OPT_BLOCKING);  // 200ms timeout

    if(ret != SAL_RET_SUCCESS || DMA_ERROR) {
        mcu_printf("[IMU] DMA transfer failed\n");
        return SAL_RET_FAILED;
    }
    
    memcpy(data, &rx_buf[1], len);
    
    return SAL_RET_SUCCESS;
}

SALRetCode_t ICM_20948_Init(void)
{
    uint8 who_am_i;
    SALRetCode_t ret;

    GPIO_Config(ICM_20948_CS_GPIO, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Set(ICM_20948_CS_GPIO, 1);
    GPIO_Config(ICM_20948_SCLK_GPIO, GPIO_FUNC(ICM_20948_GPIO_FUNC));
    GPIO_Config(ICM_20948_MOSI_GPIO, GPIO_FUNC(ICM_20948_GPIO_FUNC));
    GPIO_Config(ICM_20948_MISO_GPIO, GPIO_FUNC(ICM_20948_GPIO_FUNC) | GPIO_INPUT | GPIO_INPUTBUF_EN);

    GPSBOpenParam_t param = {
        .uiSdo = ICM_20948_MOSI_GPIO,
        .uiSdi = ICM_20948_MISO_GPIO,
        .uiSclk = ICM_20948_SCLK_GPIO,
        .uiIsSlave = GPSB_MASTER_MODE,
        .uiDmaBufSize = sizeof(tx_dma_buf),
        .pDmaAddrTx = tx_dma_buf,
        .pDmaAddrRx = rx_dma_buf,
        .fbCallback = IMU_Callback,
        .pArg = NULL
    };

    if (GPSB_Open(ICM_20948_CHANNEL, param) != SAL_RET_SUCCESS) {
        mcu_printf("[ICM_20948] GPSB open failed\n");
        return SAL_RET_FAILED;
    }

    GPSB_SetMode(ICM_20948_CHANNEL, GPSB_MODE_3);// idle High, LOW -> Edge
    GPSB_SetBpw(ICM_20948_CHANNEL, 8);
    GPSB_SetSpeed(ICM_20948_CHANNEL, 7000000); // ICM_20948 max speed
    GPSB_CsInit(ICM_20948_CHANNEL, ICM_20948_CS_GPIO, FALSE);

    ret = SAL_SemaphoreCreate(&DMA_Semaphore_ID, DMA_Semaphore, 1, SAL_OPT_BLOCKING);
    if(ret != SAL_RET_SUCCESS) {
        mcu_printf("[ICM_20948] SemaphoreCreate failed\n");
        GPSB_Close(ICM_20948_CHANNEL);
        return SAL_RET_FAILED;
    }

    ret = SAL_SemaphoreWait(DMA_Semaphore_ID, 10, SAL_OPT_BLOCKING);
    if(ret != SAL_RET_SUCCESS) {
        mcu_printf("[ICM_20948] Initial semaphore take failed\n");
        SAL_SemaphoreDelete(DMA_Semaphore_ID);
        GPSB_Close(ICM_20948_CHANNEL);
        return SAL_RET_FAILED;
    }

    SAL_TaskSleep(100);
    ICM_20948_SelectBank(0);
    SAL_TaskSleep(10);

    who_am_i = ICM_20948_Read(ICM20948_WHO_AM_I);

    if (who_am_i != ICM20948_DEVICE_ID) {
        mcu_printf("[ICM-20948] Wrong device ID!\n");
        GPSB_Close(ICM_20948_CHANNEL);
        SAL_SemaphoreDelete(DMA_Semaphore_ID);
        return SAL_RET_FAILED;
    }

    ICM_20948_SelectBank(0);
    ICM_20948_Write(ICM20948_PWR_MGMT_1, 0x80);
    SAL_TaskSleep(100);

    ICM_20948_Write(ICM20948_PWR_MGMT_1, 0x01);
    SAL_TaskSleep(10);
    ICM_20948_Write(ICM20948_PWR_MGMT_2, 0x00);
    SAL_TaskSleep(10);
    ICM_20948_Write(ICM20948_USER_CTRL, 0x10);
    SAL_TaskSleep(10);

    ICM_20948_SelectBank(2);
    ICM_20948_Write( ICM20948_GYRO_SMPLRT_DIV, 0x0A); // ~ 102 Hz output
    SAL_TaskSleep(10);
    ICM_20948_Write( ACCEL_SMPLRT_DIV_2, 0x0A); // ~ 102 Hz output
    SAL_TaskSleep(10);
    ICM_20948_Write(ICM20948_GYRO_CONFIG, 0x21); // ±250 dps, DLPF=4 (3 dB BW ≈ 23.9 Hz)
    SAL_TaskSleep(10);
    ICM_20948_Write(ICM20948_ACCEL_CONFIG_1, 0x23); // ±4 g, DLPF=4 (3 dB BW ≈ 23.9 Hz)
    SAL_TaskSleep(10);

    ICM_20948_SelectBank(0);
    SAL_TaskSleep(10);

    mcu_printf("IMU initialized with DMA\n");

    return SAL_RET_SUCCESS;

}

// ICM_20948.c에 추가
SALRetCode_t IMU_Read_Data_Polling(void)
{
    uint8 data[12];
    int16_t raw[6];
    uint8 i;
    
    // 폴링 방식: 한 바이트씩 12번 읽기
    for(i = 0; i < 12; i++) {
        data[i] = ICM_20948_Read(ICM20948_ACCEL_XOUT_H + i);
    }
    
    raw[0] = (int16_t)((data[0] << 8) | data[1]);
    raw[1] = (int16_t)((data[2] << 8) | data[3]);
    raw[2] = (int16_t)((data[4] << 8) | data[5]);
    raw[3] = (int16_t)((data[6] << 8) | data[7]);
    raw[4] = (int16_t)((data[8] << 8) | data[9]);
    raw[5] = (int16_t)((data[10] << 8) | data[11]);

    IMU.accel_x = raw[0] / 8192.0f;
    IMU.accel_y = raw[1] / 8192.0f;
    IMU.accel_z = raw[2] / 8192.0f;

    IMU.gyro_x = raw[3] / 131.0f;
    IMU.gyro_y = raw[4] / 131.0f;
    IMU.gyro_z = raw[5] / 131.0f;

    return SAL_RET_SUCCESS;
}

SALRetCode_t IMU_Read_Data_DMA(void)
{
    SALRetCode_t ret;
    
    uint8 data[12];
    int16_t raw[6];
    ret = ICM_20948_ReadMultiple(ICM20948_ACCEL_XOUT_H, data, 12);
    
    if(ret != SAL_RET_SUCCESS) {
        return SAL_RET_FAILED;
    }
    raw[0] = (int16_t)((data[0] << 8) | data[1]);   // Accel X
    raw[1] = (int16_t)((data[2] << 8) | data[3]);   // Accel Y
    raw[2] = (int16_t)((data[4] << 8) | data[5]);   // Accel Z
    raw[3] = (int16_t)((data[6] << 8) | data[7]);   // Gyro X
    raw[4] = (int16_t)((data[8] << 8) | data[9]);   // Gyro Y
    raw[5] = (int16_t)((data[10] << 8) | data[11]); // Gyro Z

    IMU.accel_x = raw[0] / 8192.0f;
    IMU.accel_y = raw[1] / 8192.0f;
    IMU.accel_z = raw[2] / 8192.0f;

    IMU.gyro_x = raw[3] / 131.0f;
    IMU.gyro_y = raw[4] / 131.0f;
    IMU.gyro_z = raw[5] / 131.0f;

    return SAL_RET_SUCCESS;
}


void IMU_Callback(uint32 uiCh, uint32 uiEvent, void *pArg)
{
    (void)uiCh;
    (void)pArg;

    GPSB_CsDeactivate(ICM_20948_CHANNEL, ICM_20948_CS_GPIO, FALSE);
    
    if(uiEvent == GPSB_EVENT_COMPLETE || uiEvent == GPSB_EVENT_TXCOMPLETE) {
        DMA_ERROR = 0;
        SAL_SemaphoreRelease(DMA_Semaphore_ID);
    }
    else if(uiEvent & (GPSB_EVENT_ERR_ROR | GPSB_EVENT_ERR_WUR | 
                       GPSB_EVENT_ERR_RUR | GPSB_EVENT_ERR_WOR)) {
        mcu_printf("[IMU] DMA Error: 0x%X\n", uiEvent);
        DMA_ERROR = 1;
        SAL_SemaphoreRelease(DMA_Semaphore_ID);
    }
    else {
        mcu_printf("[IMU] Unexpected event: 0x%X\n", uiEvent);
        DMA_ERROR = 1;
        SAL_SemaphoreRelease(DMA_Semaphore_ID);
    }
}
