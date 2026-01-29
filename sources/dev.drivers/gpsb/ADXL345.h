#ifndef GPSB_ADXL345_H_
#define GPSB_ADXL345_H_

#include <sal_com.h>

/* 포트 설정: ICM이 사용하던 Port 1 (GPB 4,5,6,7) 그대로 사용 */
#define ADXL345_CHANNEL         0               // Channel 0
#define ADXL345_SCLK_GPIO       GPIO_GPB(4)
#define ADXL345_CS0_GPIO        GPIO_GPB(5)
#define ADXL345_MOSI_GPIO       GPIO_GPB(6)
#define ADXL345_MISO_GPIO       GPIO_GPB(7)
#define ADXL345_GPIO_FUNC       1                // 일반 GPIO 포트는 FUNC 1

/* ADXL345 레지스터 */
#define ADXL345_REG_DEVID       0x00
#define ADXL345_REG_POWER_CTL   0x2D
#define ADXL345_REG_DATA_FORMAT 0x31
#define ADXL345_REG_DATAX0      0x32

/* SPI 명령 및 상수 */
#define ADXL345_DEVICE_ID       0xE5
#define ADXL345_SPI_READ        0x80
#define ADXL345_SPI_MB          0x40
#define ADXL345_MEASURE         0x08

/* 함수 프로토타입 */
SALRetCode_t ADXL345_Test_Init(void);
SALRetCode_t ADXL345_Test_ReadID(uint8 *devid);
void ADXL345_Test_Task(void *pArg);

#endif