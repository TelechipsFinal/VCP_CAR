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
#ifndef GPSB_ICM_20948_H_
#define GPSB_ICM_20948_H_

#define ICM_20948_CHANNEL     0
#define ICM_20948_CS_GPIO     GPIO_GPB(5)
#define ICM_20948_SCLK_GPIO   GPIO_GPB(4)
#define ICM_20948_MOSI_GPIO   GPIO_GPB(6)
#define ICM_20948_MISO_GPIO   GPIO_GPB(7)
#define ICM_20948_GPIO_FUNC   1

// DMA Buffer Size
#define ICM20948_DMA_BUF_SIZE   64

/* ICM-20948 Bank register */
#define ICM20948_REG_BANK_SEL       0x7F

/* Bank 0 register */
#define ICM20948_WHO_AM_I           0x00
#define ICM20948_USER_CTRL          0x03
#define ICM20948_PWR_MGMT_1         0x06
#define ICM20948_PWR_MGMT_2         0x07
#define ICM20948_ACCEL_XOUT_H       0x2D
#define ICM20948_GYRO_XOUT_H        0x33


/* Bank 2 register */
#define ICM20948_GYRO_SMPLRT_DIV    0x00
#define ACCEL_SMPLRT_DIV_2          0x11
#define ICM20948_GYRO_CONFIG      0x01
#define ICM20948_ACCEL_CONFIG_1       0x14

/* ICM-20948 Device-ID */
#define ICM20948_DEVICE_ID          0xEA

#include <sal_com.h>

void ICM_20948_SelectBank(uint8 bank);
void ICM_20948_Write(uint8 addr, uint8 data);
uint8 ICM_20948_Read(uint8 addr);
SALRetCode_t ICM_20948_ReadMultiple(uint8 addr, uint8 *data, uint8 len);
SALRetCode_t IMU_Read_Data_Polling(void);
SALRetCode_t IMU_Read_Data_DMA(void);
SALRetCode_t ICM_20948_Init(void);
void IMU_Callback(uint32 uiCh, uint32 uiEvent, void *pArg);

typedef struct {
    float accel_x;  // float로 변경
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float roll;
    float pitch;
    float filtered_roll;
    float filtered_pitch;
} IMU_Data;


extern uint32 DMA_Semaphore_ID;
extern IMU_Data IMU;

#endif
