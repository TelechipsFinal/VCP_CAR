#include <sal_internal.h>
#include <debug.h>
#include <stdint.h>
#include <gpsb.h>
#include <gpio.h>
#include <ADXL345.h>

SALRetCode_t ADXL345_Test_Init(void) {
    mcu_printf("[ADXL345] Initializing SPI Port 1 (Ch 0)...\n");

    GPIO_Config(GPIO_GPC(8), GPIO_INPUT); 

    /* 1. GPIO 설정 (ICM 설정 복제) */
    GPIO_Config(ADXL345_CS0_GPIO, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Set(ADXL345_CS0_GPIO, 1); // CS High로 시작 (SPI 모드 강제)

    GPIO_Config(ADXL345_SCLK_GPIO, GPIO_FUNC(ADXL345_GPIO_FUNC));
    GPIO_Config(ADXL345_MOSI_GPIO, GPIO_FUNC(ADXL345_GPIO_FUNC));
    // MISO는 반드시 입력 버퍼 활성화
    GPIO_Config(ADXL345_MISO_GPIO, GPIO_FUNC(ADXL345_GPIO_FUNC) | GPIO_INPUT | GPIO_INPUTBUF_EN);

    GPIO_Set(ADXL345_MISO_GPIO, GPIO_PULLUP);
    /* 2. GPSB Open */
    GPSBOpenParam_t param = {
        .uiSdo = ADXL345_MOSI_GPIO,
        .uiSdi = ADXL345_MISO_GPIO,
        .uiSclk = ADXL345_SCLK_GPIO,
        .uiIsSlave = GPSB_MASTER_MODE,
        .uiDmaBufSize = 0,
        .pDmaAddrTx = NULL,
        .pDmaAddrRx = NULL,
        .fbCallback = NULL,
        .pArg = NULL
    };

    GPSB_Close(ADXL345_CHANNEL); 
    
    if(GPSB_Open(ADXL345_CHANNEL, param) != SAL_RET_SUCCESS) {
        // 만약 여기서 실패한다면, Main_StartTask에서 이미 열린 것입니다.
        // 강제로 진행하려면 Close 후 재오픈이 가장 확실합니다.
        mcu_printf("[ADXL345] GPSB Open Fail - Already in use?\n");
        return SAL_RET_FAILED; 
    }

    GPSB_SetMode(ADXL345_CHANNEL, GPSB_MODE_3); // CPOL=1, CPHA=1
    GPSB_SetBpw(ADXL345_CHANNEL, 8);
    GPSB_SetSpeed(ADXL345_CHANNEL, 100000); 

    GPSB_CsInit(ADXL345_CHANNEL, ADXL345_CS0_GPIO, FALSE);

    SAL_TaskSleep(10); 
    return SAL_RET_SUCCESS;
}
SALRetCode_t ADXL345_Test_ReadID(uint8 *devid) {
    uint8 tx_buf[2] = { ADXL345_SPI_READ | ADXL345_REG_DEVID, 0x00 };
    uint8 rx_buf[2] = { 0 };
    SALRetCode_t ret;

    GPSB_CsActivate(ADXL345_CHANNEL, ADXL345_CS0_GPIO, FALSE);
    ret = GPSB_Xfer(ADXL345_CHANNEL, tx_buf, rx_buf, 2, GPSB_XFER_MODE_WITHOUT_INTERRUPT);
    GPSB_CsDeactivate(ADXL345_CHANNEL, ADXL345_CS0_GPIO, FALSE);
    SAL_TaskSleep(1);

    mcu_printf("[ADXL345] ReadID TX:%02X %02X RX:%02X %02X\n",
               tx_buf[0], tx_buf[1], rx_buf[0], rx_buf[1]);

    *devid = rx_buf[1];
    return ret;
}

/* 가속도계 레지스터 쓰기 */
SALRetCode_t ADXL345_WriteReg(uint8 sensor_id, uint8 reg, uint8 val) {
    uint8 tx[2];
    SALRetCode_t ret;

    tx[0] = reg; // Write 모드는 MSB가 0
    tx[1] = val;

    GPSB_CsActivate(ADXL345_CHANNEL, ADXL345_CS0_GPIO, FALSE);
    ret = GPSB_Xfer(ADXL345_CHANNEL, tx, NULL, 2, GPSB_XFER_MODE_WITHOUT_INTERRUPT);
    GPSB_CsDeactivate(ADXL345_CHANNEL, ADXL345_CS0_GPIO, FALSE);
    SAL_TaskSleep(1);

    return ret;
}

/* 가속도 데이터 읽기 및 단위 변환 */
SALRetCode_t ADXL345_ReadAccel(uint8 sensor_id, float *x, float *y, float *z) {
    uint8 tx[7] = {0,};
    uint8 rx[7] = {0,};
    int16 raw_x, raw_y, raw_z;
    SALRetCode_t ret;

    // Multi-byte Read 비트(0x40)와 Read 비트(0x80)를 함께 설정
    tx[0] = ADXL345_SPI_READ | ADXL345_SPI_MB | ADXL345_REG_DATAX0;

    GPSB_CsActivate(ADXL345_CHANNEL, ADXL345_CS0_GPIO, FALSE);
    // 주소 1바이트 + 데이터 6바이트 = 총 7바이트 트랜잭션
    ret = GPSB_Xfer(ADXL345_CHANNEL, tx, rx, 7, GPSB_XFER_MODE_WITHOUT_INTERRUPT);
    GPSB_CsDeactivate(ADXL345_CHANNEL, ADXL345_CS0_GPIO, FALSE);
    SAL_TaskSleep(1);

    mcu_printf("[ADXL345] ReadAccel TX:%02X %02X %02X %02X %02X %02X %02X RX:%02X %02X %02X %02X %02X %02X %02X\n",
               tx[0], tx[1], tx[2], tx[3], tx[4], tx[5], tx[6],
               rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6]);

    if(ret == SAL_RET_SUCCESS) {
        // ADXL345 데이터는 Little Endian (LSB, MSB 순)
        // rx[0]은 주소 전송 시 받은 쓰레기값이므로 rx[1]부터 처리
        raw_x = (int16)((rx[2] << 8) | rx[1]);
        raw_y = (int16)((rx[4] << 8) | rx[3]);
        raw_z = (int16)((rx[6] << 8) | rx[5]);

        // Full-res 모드 scale factor: 3.9mg/LSB
        const float factor = 0.0039f * 9.80665f;
        if(x) *x = (float)raw_x * factor;
        if(y) *y = (float)raw_y * factor;
        if(z) *z = (float)raw_z * factor;
    }

    return ret;
}
