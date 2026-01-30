#include <sal_internal.h>
#include <debug.h>
#include <stdint.h>
#include <i2c.h>
#include <gpio.h>
#include <ADXL345.h>

static uint8 ADXL345_I2C_Addr(void)
{
    return (uint8)(ADXL345_I2C_ADDR_7BIT << 1U);
}

SALRetCode_t ADXL345_Test_Init(void)
{
    static uint8 initialized = 0;
    SALRetCode_t ret;
    
    if (initialized) {
        mcu_printf("[ADXL345] Already initialized\n");
        return SAL_RET_SUCCESS;
    }
    
    mcu_printf("[ADXL345] Initializing I2C%d Port%d (GPB0/1)...\n",
               ADXL345_I2C_CH, ADXL345_I2C_PORT);

    // I2C 초기화
    ret = I2C_Open((uint8)ADXL345_I2C_CH, 
                   (uint32)ADXL345_I2C_PORT,
                   (uint32)ADXL345_I2C_SPEED_KHZ, 
                   NULL, NULL);
    
    if (ret != SAL_RET_SUCCESS) {
        mcu_printf("[ADXL345] I2C Open Failed (ret=%d)\n", ret);
        return SAL_RET_FAILED;
    }

    SAL_TaskSleep(50);
    mcu_printf("[ADXL345] I2C Initialized\n");
    
    // 전체 I2C 스캔
    mcu_printf("[ADXL345] Scanning I2C bus...\n");
    uint8 found = 0;
    
    for(uint8 addr = 0x08; addr <= 0x77; addr++) {
        uint8 dummy = 0x00;
        I2CXfer_t xfer = {0};
        xfer.xCmdBuf = &dummy;
        xfer.xCmdLen = 1;
        
        if(I2C_XferCmd((uint8)ADXL345_I2C_CH, (uint8)(addr << 1), xfer, 0) == SAL_RET_SUCCESS) {
            mcu_printf("  [I2C] Device found at 0x%02X\n", addr);
            found++;
        }
    }
    
    if(found == 0) {
        mcu_printf("  [WARNING] No I2C devices detected!\n");
        mcu_printf("  Hardware Check:\n");
        mcu_printf("    1. ADXL345 VCC → 3.3V (NOT 5V!)\n");
        mcu_printf("    2. ADXL345 GND → GND\n");
        mcu_printf("    3. ADXL345 SCL → GPB0 (left side of board)\n");
        mcu_printf("    4. ADXL345 SDA → GPB1 (left side of board)\n");
        mcu_printf("    5. ADXL345 CS  → 3.3V (I2C mode)\n");
        mcu_printf("    6. ADXL345 SDO → GND (address 0x53)\n");
    }

    initialized = 1;
    return SAL_RET_SUCCESS;
}

SALRetCode_t ADXL345_Test_ReadID(uint8 *devid)
{
    uint8 cmd = ADXL345_REG_DEVID;
    uint8 data = 0;
    I2CXfer_t xfer = {0};

    xfer.xCmdBuf = &cmd;
    xfer.xCmdLen = 1;
    xfer.xInBuf = &data;
    xfer.xInLen = 1;

    SALRetCode_t ret = I2C_XferCmd((uint8)ADXL345_I2C_CH, ADXL345_I2C_Addr(), xfer, 0);

    *devid = data;
    return ret;
}

SALRetCode_t ADXL345_ReadReg(uint8 sensor_id, uint8 reg, uint8 *val)
{
    uint8 cmd = reg;
    uint8 data = 0;
    I2CXfer_t xfer = {0};

    xfer.xCmdBuf = &cmd;
    xfer.xCmdLen = 1;
    xfer.xInBuf = &data;
    xfer.xInLen = 1;

    SALRetCode_t ret = I2C_XferCmd((uint8)ADXL345_I2C_CH, ADXL345_I2C_Addr(), xfer, 0);
    if (ret != SAL_RET_SUCCESS) {
        mcu_printf("[ADXL345] ReadReg fail reg=0x%02X ret=%d\n", reg, ret);
    }
    if (val) {
        *val = data;
    }

    (void)sensor_id;
    return ret;
}

SALRetCode_t ADXL345_WriteReg(uint8 sensor_id, uint8 reg, uint8 val)
{
    uint8 cmd = reg;
    uint8 data = val;
    I2CXfer_t xfer = {0};

    xfer.xCmdBuf = &cmd;
    xfer.xCmdLen = 1;
    xfer.xOutBuf = &data;
    xfer.xOutLen = 1;

    SALRetCode_t ret = I2C_XferCmd((uint8)ADXL345_I2C_CH, ADXL345_I2C_Addr(), xfer, 0);
    if (ret != SAL_RET_SUCCESS) {
        mcu_printf("[ADXL345] WriteReg fail reg=0x%02X val=0x%02X ret=%d\n", reg, val, ret);
    }

    (void)sensor_id;
    return ret;
}

SALRetCode_t ADXL345_ReadAccel(uint8 sensor_id, float *x, float *y, float *z)
{
    uint8 cmd = ADXL345_REG_DATAX0;
    uint8 rx[6] = {0};
    I2CXfer_t xfer = {0};

    xfer.xCmdBuf = &cmd;
    xfer.xCmdLen = 1;
    xfer.xInBuf = rx;
    xfer.xInLen = 6;

    SALRetCode_t ret = I2C_XferCmd((uint8)ADXL345_I2C_CH, ADXL345_I2C_Addr(), xfer, 0);

    if (ret == SAL_RET_SUCCESS) {
        int16 raw_x = (int16)((rx[1] << 8) | rx[0]);
        int16 raw_y = (int16)((rx[3] << 8) | rx[2]);
        int16 raw_z = (int16)((rx[5] << 8) | rx[4]);

        const float factor = 0.0039f * 9.80665f;
        if (x) *x = (float)raw_x * factor;
        if (y) *y = (float)raw_y * factor;
        if (z) *z = (float)raw_z * factor;
    }

    } else {
        mcu_printf("[ADXL345] ReadAccel fail ret=%d\n", ret);
    }

    (void)sensor_id;
    return ret;
}
