#ifndef GPSB_ADXL345_H_
#define GPSB_ADXL345_H_

#include <sal_com.h>
#include <gpio.h>

/*
 * I2C0 CH_0 설정 (GPB0/1)
 * - 가장 안정적인 I2C 포트
 */
#define ADXL345_I2C_CH          (0U)              // I2C0
#define ADXL345_I2C_PORT        (0U)              // Port 0
#define ADXL345_I2C_SPEED_KHZ   (100U)             // 400kHz (diagnostic 안정성)
#define ADXL345_I2C_SCL_GPIO    GPIO_GPB(0)       // SCL
#define ADXL345_I2C_SDA_GPIO    GPIO_GPB(1)       // SDA

#define ADXL345_I2C_ADDR_7BIT   (0x53U)

/* 레지스터 */
#define ADXL345_REG_DEVID       0x00
#define ADXL345_REG_BW_RATE     0x2C
#define ADXL345_REG_POWER_CTL   0x2D
#define ADXL345_REG_DATA_FORMAT 0x31
#define ADXL345_REG_DATAX0      0x32

#define ADXL345_DEVICE_ID       0xE5
#define ADXL345_MEASURE         0x08

/* ===== TCA9548A (I2C Mux) ===== */
#define TCA9548A_ADDR_7BIT   (0x70)   // A0~A2에 따라 0x70~0x77
#define TCA9548A_ADDR_8BIT   (TCA9548A_ADDR_7BIT << 1U)
#define ADXL_COUNT  (4)

// Calibration structure
typedef struct {
    float offset_x;
    float offset_y;
    float offset_z;
    float scale_x;
    float scale_y;
    float scale_z;
    uint8 calibrated;
} ADXL345_Calibration_t;

/* ===== MUX 관련 (추가) ===== */
extern const uint8 g_adxl_mux_ch[ADXL_COUNT];  // MUX 채널 맵핑
SALRetCode_t ADXL_MuxSelectByDev(uint8 dev);   // MUX 채널 선택

// Calibration functions
SALRetCode_t ADXL345_GetCalibration(uint8 sensor_id, ADXL345_Calibration_t *cal);
SALRetCode_t ADXL345_SetCalibration(uint8 sensor_id, const ADXL345_Calibration_t *cal);
SALRetCode_t ADXL345_ReadAccelCalibrated(uint8 sensor_id, float *x, float *y, float *z);
SALRetCode_t ADXL345_QuickCalibrateAll(uint32 samples);

/* 함수 */
SALRetCode_t ADXL345_Test_Init(void);
SALRetCode_t ADXL345_Test_ReadID(uint8 *devid);
SALRetCode_t ADXL345_ReadReg(uint8 sensor_id, uint8 reg, uint8 *val);
SALRetCode_t ADXL345_ReadRegsBurst(uint8 sensor_id, uint8 start_reg, uint8 *buf, uint8 len);
SALRetCode_t ADXL345_WriteReg(uint8 sensor_id, uint8 reg, uint8 val);
SALRetCode_t ADXL345_ReadAccel(uint8 sensor_id, float *x, float *y, float *z);
SALRetCode_t ADXL345_ReadRaw(uint8 sensor_id, int16 *x, int16 *y, int16 *z);

#endif