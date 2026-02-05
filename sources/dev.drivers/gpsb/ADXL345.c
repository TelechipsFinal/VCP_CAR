#include <sal_internal.h>
#include <debug.h>
#include <stdint.h>
#include <i2c.h>
#include <i2c_reg.h>
#include <gpio.h>
#include <ADXL345.h>

static uint32 g_adxl345_i2c_sem;
static uint8 g_adxl345_i2c_sem_init = 0;
static const uint8 g_adxl345_i2c_sem_name[] = "ADXL345_I2C";
ADXL345_Calibration_t g_adxl345_cal[4] = {0};  // 최대 4개 센서 지원
const uint8 g_adxl_mux_ch[ADXL_COUNT] = { 0, 1, 2, 3 }; // ADXL0~3 -> MUX CH0~3

static SALRetCode_t ADXL345_SelectBySensor(uint8 sensor_id);


static uint8 ADXL345_I2C_Addr(void)
{
    return (uint8)(ADXL345_I2C_ADDR_7BIT << 1U);
}

static SALRetCode_t ADXL345_I2C_Xfer(I2CXfer_t xfer)
{
    SALRetCode_t ret = SAL_RET_FAILED;
    uint8 retry;

    for (retry = 0; retry < 3; retry++) {
        if (g_adxl345_i2c_sem_init != 0U) {
            ret = SAL_SemaphoreWait(g_adxl345_i2c_sem, 50, SAL_OPT_BLOCKING);
            if (ret != SAL_RET_SUCCESS) {
                return ret;
            }
        }

        // ✅ I2C_Xfer 사용 (I2C_XferCmd 대신)
        ret = I2C_Xfer((uint8)ADXL345_I2C_CH, ADXL345_I2C_Addr(), xfer, 0);

        if (g_adxl345_i2c_sem_init != 0U) {
            (void)SAL_SemaphoreRelease(g_adxl345_i2c_sem);
        }

        if (ret == SAL_RET_SUCCESS) {
            break;
        }
        SAL_TaskSleep(2);
    }

    return ret;
}
SALRetCode_t ADXL345_Test_Init(void)
{
    static uint8 initialized = 0;
    SALRetCode_t ret;
    
    if (initialized) {
        return SAL_RET_SUCCESS;
    }
    
    mcu_printf("[ADXL345] Initializing I2C%d Port%d (GPB0/1)...\n",
               ADXL345_I2C_CH, ADXL345_I2C_PORT);

    ret = I2C_Open((uint8)ADXL345_I2C_CH, 
                   (uint32)ADXL345_I2C_PORT,
                   (uint32)ADXL345_I2C_SPEED_KHZ, 
                   NULL, NULL);
    
    if (ret != SAL_RET_SUCCESS) {
        mcu_printf("[ADXL345] I2C Open Failed (ret=%d)\n", ret);
        return SAL_RET_FAILED;
    }

    if (g_adxl345_i2c_sem_init == 0U) {
        if (SAL_SemaphoreCreate(&g_adxl345_i2c_sem, g_adxl345_i2c_sem_name, 1, SAL_OPT_BLOCKING) == SAL_RET_SUCCESS) {
            g_adxl345_i2c_sem_init = 1;
        }
    }

    SAL_TaskSleep(50);
    mcu_printf("[ADXL345] I2C Initialized\n\n");

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
    xfer.xOpt = 0;

    SALRetCode_t ret = ADXL345_I2C_Xfer(xfer);

    *devid = data;
    return ret;
}
SALRetCode_t ADXL345_ReadReg(uint8 sensor_id, uint8 reg, uint8 *val)
{
    I2CXfer_t xfer = {0};
    uint8 data = 0;
    SALRetCode_t ret;

    // MUX 선택
    ret = ADXL345_SelectBySensor(sensor_id);
    if (ret != SAL_RET_SUCCESS) {
        return ret;
    }

    // Step 1: Write register address
    xfer.xCmdBuf = NULL;
    xfer.xCmdLen = 0;
    xfer.xOutBuf = &reg;
    xfer.xOutLen = 1;
    xfer.xInBuf = NULL;
    xfer.xInLen = 0;
    xfer.xOpt = 0;

    ret = ADXL345_I2C_Xfer(xfer);
    if (ret != SAL_RET_SUCCESS) {
        return ret;
    }

    SAL_TaskSleep(2);

    // Step 2: Read data
    xfer.xCmdBuf = NULL;
    xfer.xCmdLen = 0;
    xfer.xOutBuf = NULL;
    xfer.xOutLen = 0;
    xfer.xInBuf = &data;
    xfer.xInLen = 1;
    xfer.xOpt = 0;

    ret = ADXL345_I2C_Xfer(xfer);
    
    if (val) {
        *val = data;
    }

    return ret;
}
SALRetCode_t ADXL345_ReadRegsBurst(uint8 sensor_id, uint8 start_reg, uint8 *buf, uint8 len)
{
    I2CXfer_t xfer = {0};
    SALRetCode_t ret;

    // MUX 선택
    ret = ADXL345_SelectBySensor(sensor_id);
    if (ret != SAL_RET_SUCCESS) {
        return ret;
    }

    // Step 1: Write register address
    xfer.xCmdBuf = NULL;
    xfer.xCmdLen = 0;
    xfer.xOutBuf = &start_reg;
    xfer.xOutLen = 1;
    xfer.xInBuf = NULL;
    xfer.xInLen = 0;
    xfer.xOpt = 0;

    ret = ADXL345_I2C_Xfer(xfer);
    if (ret != SAL_RET_SUCCESS) {
        return ret;
    }

    SAL_TaskSleep(2);

    // Step 2: Read burst
    xfer.xCmdBuf = NULL;
    xfer.xCmdLen = 0;
    xfer.xOutBuf = NULL;
    xfer.xOutLen = 0;
    xfer.xInBuf = buf;
    xfer.xInLen = len;
    xfer.xOpt = 0;

    ret = ADXL345_I2C_Xfer(xfer);
    if (ret != SAL_RET_SUCCESS) {
        mcu_printf("[ADXL345] ReadBurst fail reg=0x%02X len=%d ret=%d\n", start_reg, len, ret);
    }

    return ret;
}

SALRetCode_t ADXL345_WriteReg(uint8 sensor_id, uint8 reg, uint8 val)
{
    I2CXfer_t xfer = {0};
    uint8 buf[2];
    SALRetCode_t ret;

    // MUX 선택
    ret = ADXL345_SelectBySensor(sensor_id);
    if (ret != SAL_RET_SUCCESS) {
        return ret;
    }

    buf[0] = reg;
    buf[1] = val;

    xfer.xCmdBuf = NULL;
    xfer.xCmdLen = 0;
    xfer.xOutBuf = buf;
    xfer.xOutLen = 2;
    xfer.xInBuf = NULL;
    xfer.xInLen = 0;
    xfer.xOpt = 0;

    ret = ADXL345_I2C_Xfer(xfer);
    if (ret != SAL_RET_SUCCESS) {
        mcu_printf("[ADXL345] WriteReg fail reg=0x%02X val=0x%02X ret=%d\n", reg, val, ret);
    }

    return ret;
}
SALRetCode_t ADXL345_ReadAccel(uint8 sensor_id, float *x, float *y, float *z)
{
    I2CXfer_t xfer = {0};
    uint8 cmd = ADXL345_REG_DATAX0;
    uint8 rx[6] = {0};
    SALRetCode_t ret;

    // MUX 선택
    ret = ADXL345_SelectBySensor(sensor_id);
    if (ret != SAL_RET_SUCCESS) {
        return ret;
    }

    // Step 1: Write register address
    xfer.xCmdBuf = NULL;
    xfer.xCmdLen = 0;
    xfer.xOutBuf = &cmd;
    xfer.xOutLen = 1;
    xfer.xInBuf = NULL;
    xfer.xInLen = 0;
    xfer.xOpt = 0;

    ret = ADXL345_I2C_Xfer(xfer);
    if (ret != SAL_RET_SUCCESS) {
        return ret;
    }

    SAL_TaskSleep(2);

    // Step 2: Read 6 bytes
    xfer.xCmdBuf = NULL;
    xfer.xCmdLen = 0;
    xfer.xOutBuf = NULL;
    xfer.xOutLen = 0;
    xfer.xInBuf = rx;
    xfer.xInLen = 6;
    xfer.xOpt = 0;

    ret = ADXL345_I2C_Xfer(xfer);

    if (ret == SAL_RET_SUCCESS) {
        int16 raw_x = (int16)((rx[1] << 8) | rx[0]);
        int16 raw_y = (int16)((rx[3] << 8) | rx[2]);
        int16 raw_z = (int16)((rx[5] << 8) | rx[4]);

        const float factor = 0.0039f * 9.80665f;
        if (x) *x = (float)raw_x * factor;
        if (y) *y = (float)raw_y * factor;
        if (z) *z = (float)raw_z * factor;
    }

    return ret;
}
SALRetCode_t ADXL345_ReadRaw(uint8 sensor_id, int16 *x, int16 *y, int16 *z)
{
    I2CXfer_t xfer = {0};
    uint8 cmd = ADXL345_REG_DATAX0;
    uint8 rx[6] = {0};
    SALRetCode_t ret;

    // MUX 선택
    ret = ADXL345_SelectBySensor(sensor_id);
    if (ret != SAL_RET_SUCCESS) {
        return ret;
    }

    // Step 1: Write register address
    xfer.xCmdBuf = NULL;
    xfer.xCmdLen = 0;
    xfer.xOutBuf = &cmd;
    xfer.xOutLen = 1;
    xfer.xInBuf = NULL;
    xfer.xInLen = 0;
    xfer.xOpt = 0;

    ret = ADXL345_I2C_Xfer(xfer);
    if (ret != SAL_RET_SUCCESS) {
        return ret;
    }

    SAL_TaskSleep(2);

    // Step 2: Read 6 bytes
    xfer.xCmdBuf = NULL;
    xfer.xCmdLen = 0;
    xfer.xOutBuf = NULL;
    xfer.xOutLen = 0;
    xfer.xInBuf = rx;
    xfer.xInLen = 6;
    xfer.xOpt = 0;

    ret = ADXL345_I2C_Xfer(xfer);
    if (ret == SAL_RET_SUCCESS) {
        if (x) *x = (int16)((rx[1] << 8) | rx[0]);
        if (y) *y = (int16)((rx[3] << 8) | rx[2]);
        if (z) *z = (int16)((rx[5] << 8) | rx[4]);
    }

    return ret;
}

/*
***************************************************************************************************
*                                          ADXL345_GetCalibration
*
* Function to get current calibration parameters
*
* @param    sensor_id [in]  : Sensor ID (0-3)
* @param    cal [out]       : Calibration structure
* @return   SAL_RET_SUCCESS or SAL_RET_FAILED
*
***************************************************************************************************
*/
SALRetCode_t ADXL345_GetCalibration(uint8 sensor_id, ADXL345_Calibration_t *cal)
{
    if (sensor_id >= 4 || cal == NULL) {
        return SAL_RET_FAILED;
    }

    *cal = g_adxl345_cal[sensor_id];
    return SAL_RET_SUCCESS;
}

/*
***************************************************************************************************
*                                          ADXL345_SetCalibration
*
* Function to set calibration parameters manually
*
* @param    sensor_id [in]  : Sensor ID (0-3)
* @param    cal [in]        : Calibration structure
* @return   SAL_RET_SUCCESS or SAL_RET_FAILED
*
***************************************************************************************************
*/
SALRetCode_t ADXL345_SetCalibration(uint8 sensor_id, const ADXL345_Calibration_t *cal)
{
    if (sensor_id >= 4 || cal == NULL) {
        return SAL_RET_FAILED;
    }

    g_adxl345_cal[sensor_id] = *cal;
    return SAL_RET_SUCCESS;
}

/*
***************************************************************************************************
*                                          ADXL345_ReadAccelCalibrated
*
* Function to read calibrated acceleration data
*
* @param    sensor_id [in]  : Sensor ID (0-3)
* @param    x [out]         : X-axis acceleration (m/s²)
* @param    y [out]         : Y-axis acceleration (m/s²)
* @param    z [out]         : Z-axis acceleration (m/s²)
* @return   SAL_RET_SUCCESS or SAL_RET_FAILED
*
***************************************************************************************************
*/
SALRetCode_t ADXL345_ReadAccelCalibrated(uint8 sensor_id, float *x, float *y, float *z)
{
    float raw_x, raw_y, raw_z;
    SALRetCode_t ret;

    if (sensor_id >= 4) {
        return SAL_RET_FAILED;
    }

    ret = ADXL345_ReadAccel(sensor_id, &raw_x, &raw_y, &raw_z);
    
    if (ret != SAL_RET_SUCCESS) {
        return ret;
    }

    // Apply calibration if available
    if (g_adxl345_cal[sensor_id].calibrated) {
        if (x) *x = (raw_x + g_adxl345_cal[sensor_id].offset_x) * g_adxl345_cal[sensor_id].scale_x;
        if (y) *y = (raw_y + g_adxl345_cal[sensor_id].offset_y) * g_adxl345_cal[sensor_id].scale_y;
        if (z) *z = (raw_z + g_adxl345_cal[sensor_id].offset_z) * g_adxl345_cal[sensor_id].scale_z;
    } else {
        // No calibration, return raw values
        if (x) *x = raw_x;
        if (y) *y = raw_y;
        if (z) *z = raw_z;
    }

    return SAL_RET_SUCCESS;
}

static SALRetCode_t TCA9548A_SelectChannel(uint8 ch)
{
    I2CXfer_t xfer = {0};
    uint8 mask = (uint8)(1U << ch);

    xfer.xCmdBuf = NULL;
    xfer.xCmdLen = 0;
    xfer.xOutBuf = &mask;
    xfer.xOutLen = 1;
    xfer.xInBuf  = NULL;
    xfer.xInLen  = 0;
    xfer.xOpt    = 0;

    /* 같은 I2C 채널/포트 사용 */
    return I2C_Xfer((uint8)ADXL345_I2C_CH, (uint8)TCA9548A_ADDR_8BIT, xfer, 0);
}

static SALRetCode_t ADXL345_SelectBySensor(uint8 sensor_id)
{
    if (sensor_id >= ADXL_COUNT) return SAL_RET_FAILED;
    return TCA9548A_SelectChannel(sensor_id);  // dev 0->ch0, 1->ch1...
}


SALRetCode_t ADXL_MuxSelectByDev(uint8 dev)
{
    if (dev >= ADXL_COUNT) return SAL_RET_FAILED;
    return TCA9548A_SelectChannel(g_adxl_mux_ch[dev]);
}

SALRetCode_t ADXL345_QuickCalibrateAll(uint32 samples)
{
    float sum_x[4] = {0}, sum_y[4] = {0}, sum_z[4] = {0};
    uint32 valid_samples[4] = {0};

    for(uint32 i = 0; i < samples; i++) {
        for(uint8 dev = 0; dev < 4; dev++) {
            float x, y, z;
            if(ADXL345_ReadAccel(dev, &x, &y, &z) == SAL_RET_SUCCESS) {
                sum_x[dev] += x;
                sum_y[dev] += y;
                sum_z[dev] += z;
                valid_samples[dev]++;
            }
        }
        SAL_TaskSleep(10);  // 100Hz
    }

    for(uint8 dev = 0; dev < 4; dev++) {
        if(valid_samples[dev] < (samples / 2U)) {
            mcu_printf("  [WARN] ADXL%d: Low sample count (%lu/%lu)\n",
                       dev, (unsigned long)valid_samples[dev], (unsigned long)samples);
            g_adxl345_cal[dev].calibrated = 0;
            continue;
        }

        float avg_x = sum_x[dev] / (float)valid_samples[dev];
        float avg_y = sum_y[dev] / (float)valid_samples[dev];
        float avg_z = sum_z[dev] / (float)valid_samples[dev];

        // ✅ CalibrateAll()과 동일한 정의로 맞추기 (raw + offset 방식)
        g_adxl345_cal[dev].offset_x = -avg_x;
        g_adxl345_cal[dev].offset_y = -avg_y;
        g_adxl345_cal[dev].offset_z = -(avg_z - 9.80665f);

        g_adxl345_cal[dev].scale_x = 1.0f;
        g_adxl345_cal[dev].scale_y = 1.0f;
        g_adxl345_cal[dev].scale_z = 1.0f;
        g_adxl345_cal[dev].calibrated = 1;
    }

    return SAL_RET_SUCCESS;
}