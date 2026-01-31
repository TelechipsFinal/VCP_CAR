#include <sal_internal.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <debug.h>
#include <ADXL345.h>
#include <ADXL345_test.h>
#include <printf_float.h>
#include <i2c.h>

#define IMPACT_THRESHOLD_G  (2.0f)

static SALRetCode_t ADXL345_ReadRegStable(uint8 reg, uint8 *val);
static SALRetCode_t ADXL345_WriteVerify(uint8 reg, uint8 val, uint8 retries, uint32 delay_ms);
static void ADXL_Test_Calibration(void);
static SALRetCode_t ADXL345_ReadRegStable(uint8 reg, uint8 *val)
{
    uint8 a = 0;
    uint8 b = 0;
    uint8 retry;

    for (retry = 0; retry < 5; retry++) {
        SALRetCode_t ra = ADXL345_ReadReg(0, reg, &a);
        if (ra != SAL_RET_SUCCESS) {
            SAL_TaskSleep(2);
            continue;
        }
        SALRetCode_t rb = ADXL345_ReadReg(0, reg, &b);
        if (rb != SAL_RET_SUCCESS) {
            SAL_TaskSleep(2);
            continue;
        }
        if (a == b) {
            if (val) {
                *val = a;
            }
            return SAL_RET_SUCCESS;
        }
        SAL_TaskSleep(2);
    }

    if (val) {
        *val = b;
    }
    mcu_printf("  [WARN] Unstable reg 0x%02X (last=0x%02X)\n", reg, b);
    return SAL_RET_SUCCESS;
}
static SALRetCode_t ADXL345_WriteVerify(uint8 reg, uint8 val, uint8 retries, uint32 delay_ms)
{
    uint8 readback = 0;
    uint8 i;

    for (i = 0; i < retries; i++) {
        ADXL345_WriteReg(0, reg, val);
        SAL_TaskSleep(delay_ms);
        if (ADXL345_ReadRegStable(reg, &readback) == SAL_RET_SUCCESS && readback == val) {
            return SAL_RET_SUCCESS;
        }
        SAL_TaskSleep(5);
    }

    return SAL_RET_FAILED;
}

/*
 * ADXL345 검증 메인 태스크
 */void ADXL345_Test_Task(void *pArg) 
{
    (void)pArg;
    uint8 devid = 0;
    uint8 retry = 0;
    uint8 reg = 0;
    uint8 reg_before = 0;
    uint8 reg_after = 0;
    uint8 reg_late1 = 0;
    uint8 reg_late2 = 0;
    uint8 burst[6] = {0};
    
    mcu_printf("\n╔════════════════════════════════════╗\n");
    mcu_printf("║   ADXL345 Test Task Started        ║\n");
    mcu_printf("╚════════════════════════════════════╝\n\n");
    
    SAL_TaskSleep(100);

    /* ========== TEST 1: Device ID ========== */
    mcu_printf("[TEST 1] Device ID Check\n");
    mcu_printf("=========================================\n");

    for(retry = 0; retry < 5; retry++) {
        if(ADXL345_Test_ReadID(&devid) == SAL_RET_SUCCESS) {
            if(devid == ADXL345_DEVICE_ID) {
                break;
            }
        }
        mcu_printf("  Retry %d/5... (Read: 0x%02X)\n", retry + 1, devid);
        SAL_TaskSleep(200);
    }

    if(devid != ADXL345_DEVICE_ID) {
        mcu_printf("  [FATAL] Sensor Not Responding!\n");
        mcu_printf("  Expected: 0xE5, Got: 0x%02X\n", devid);
        mcu_printf("  Check:\n");
        mcu_printf("    - I2C Pins: SCL=GPB0, SDA=GPB1\n");
        mcu_printf("    - I2C Channel: %d, Port: %d\n", ADXL345_I2C_CH, ADXL345_I2C_PORT);
        mcu_printf("    - I2C Addr: 0x%02X (7-bit)\n", ADXL345_I2C_ADDR_7BIT);
        mcu_printf("    - Hardware connections\n");
        SAL_TaskDelete(0);
        return;
    }

    mcu_printf("  [SUCCESS] ADXL345 Verified (ID: 0xE5)\n\n");

    /* ========== 센서 초기화 ========== */
    mcu_printf("[ADXL345] Configuring Sensor...\n");
    (void)ADXL345_ReadRegStable(ADXL345_REG_DATA_FORMAT, &reg_before);
    mcu_printf("  DATA_FORMAT before write: 0x%02X\n", reg_before);

    (void)ADXL345_WriteVerify(ADXL345_REG_BW_RATE, 0x0A, 3, 10);
    (void)ADXL345_WriteVerify(ADXL345_REG_DATA_FORMAT, 0x09, 3, 10);
    (void)ADXL345_WriteVerify(ADXL345_REG_POWER_CTL, 0x08, 3, 20);
    SAL_TaskSleep(50);

    (void)ADXL345_ReadRegStable(ADXL345_REG_DATA_FORMAT, &reg_after);
    mcu_printf("  DATA_FORMAT after write:  0x%02X (expected 0x09)\n", reg_after);

    SAL_TaskSleep(100);
    (void)ADXL345_ReadRegStable(ADXL345_REG_DATA_FORMAT, &reg_late1);
    SAL_TaskSleep(900);
    (void)ADXL345_ReadRegStable(ADXL345_REG_DATA_FORMAT, &reg_late2);
    mcu_printf("  DATA_FORMAT +100ms:       0x%02X\n", reg_late1);
    mcu_printf("  DATA_FORMAT +1000ms:      0x%02X\n", reg_late2);
    if ((reg_after != 0x09) || (reg_late1 != reg_after) || (reg_late2 != reg_after)) {
        mcu_printf("  [WARN] DATA_FORMAT changed after config. Possible overwrite.\n");
    }
    mcu_printf("[ADXL345] Configuration Complete\n\n");
    
    /* ========== 테스트 실행 ========== */


    ADXL_Test_Calibration();

    mcu_printf("[TEST 2] Register Readback\n");
    mcu_printf("=========================================\n");
    if (ADXL345_ReadRegsBurst(0, ADXL345_REG_BW_RATE, burst, sizeof(burst)) == SAL_RET_SUCCESS) {
        mcu_printf("  BURST 0x2C-0x31: %02X %02X %02X %02X %02X %02X\n",
                   burst[0], burst[1], burst[2], burst[3], burst[4], burst[5]);
    } else {
        mcu_printf("  [ERROR] BURST read failed\n");
    }
    if (ADXL345_ReadRegStable(ADXL345_REG_BW_RATE, &reg) == SAL_RET_SUCCESS) {
        mcu_printf("  BW_RATE: 0x%02X (expected 0x0A)\n", reg);
    }
    for (retry = 0; retry < 5; retry++) {
        if (ADXL345_ReadRegStable(ADXL345_REG_DATA_FORMAT, &reg) == SAL_RET_SUCCESS) {
            mcu_printf("  DATA_FORMAT: 0x%02X (expected 0x09)\n", reg);
            if (reg == 0x09) {
                break;
            }
        }
        SAL_TaskSleep(5);
    }
    if (ADXL345_ReadRegStable(ADXL345_REG_POWER_CTL, &reg) == SAL_RET_SUCCESS) {
        mcu_printf("  POWER_CTL: 0x%02X (expected 0x08)\n", reg);
    }

    mcu_printf("\n[TEST 3] Single Read (with retry)\n");
    mcu_printf("=========================================\n");
    for (retry = 0; retry < 3; retry++) {
        if (ADXL_Test_SingleRead() == SAL_RET_SUCCESS) {
            break;
        }
        mcu_printf("  Retry %d/3...\n", retry + 1);
        SAL_TaskSleep(100);
    }

    ADXL_Test_Continuous(5000);

    ADXL_Test_ImpactDetection(10000);

    mcu_printf("\n╔════════════════════════════════════╗\n");
    mcu_printf("║   All Tests Finished!              ║\n");
    mcu_printf("╚════════════════════════════════════╝\n");

    SAL_TaskDelete(0);
}

/* 단일 읽기 테스트 */
SALRetCode_t ADXL_Test_SingleRead(void) 
{
    float x, y, z;
    int16 rx, ry, rz;
    
    SALRetCode_t ret = ADXL345_ReadAccel(0, &x, &y, &z);
    
    if(ret == SAL_RET_SUCCESS) {
        mcu_printf("  X: ");
        Print_Float_Value(x, 100);
        mcu_printf(" m/s² | Y: ");
        Print_Float_Value(y, 100);
        mcu_printf(" m/s² | Z: ");
        Print_Float_Value(z, 100);
        mcu_printf(" m/s²\n");
        if (ADXL345_ReadRaw(0, &rx, &ry, &rz) == SAL_RET_SUCCESS) {
            mcu_printf("  RAW: X=%d Y=%d Z=%d\n", rx, ry, rz);
        }
    } else {
        mcu_printf("  [ERROR] Read Failed\n");
    }
    return ret;
}

/* 연속 읽기 테스트 */SALRetCode_t ADXL_Test_Continuous(uint32 duration_ms) 
{
    uint32 start, current, count = 0;
    float x, y, z;

    mcu_printf("\n[TEST 4] Continuous Read (5sec)\n");
    mcu_printf("=========================================\n");
    
    SAL_GetTickCount(&start);
    
    while(1) {
        SAL_GetTickCount(&current);
        if((current - start) >= duration_ms) break;

        if(ADXL345_ReadAccel(0, &x, &y, &z) == SAL_RET_SUCCESS) {
            if(count % 5 == 0) {
                float mag = sqrtf((x * x) + (y * y) + (z * z));
                mcu_printf("  [%5d ms] X:", (current - start));
                Print_Float_Value(x, 100);
                mcu_printf(" Y:");
                Print_Float_Value(y, 100);
                mcu_printf(" Z:");
                Print_Float_Value(z, 100);
                mcu_printf(" |A|:");
                Print_Float_Value(mag, 100);
                mcu_printf(" m/s²\n");
            }
            count++;
        }
        SAL_TaskSleep(50);
    }
    mcu_printf("  Total samples: %d\n", count);
    return SAL_RET_SUCCESS;
}

/* 충격 감지 테스트 */SALRetCode_t ADXL_Test_ImpactDetection(uint32 duration_ms) 
{
    uint32 start, current, impacts = 0;
    float x, y, z;
    float prev_mag = 0.0f;

    mcu_printf("\n[TEST 5] Impact Detection (10sec, >2.0G)\n");
    mcu_printf("=========================================\n");
    
    SAL_GetTickCount(&start);
    if (ADXL345_ReadAccel(0, &x, &y, &z) == SAL_RET_SUCCESS) {
        prev_mag = sqrtf((x * x) + (y * y) + (z * z));
    }

    while(1) {
        SAL_GetTickCount(&current);
        if((current - start) >= duration_ms) break;

        if(ADXL345_ReadAccel(0, &x, &y, &z) == SAL_RET_SUCCESS) {
            float mag = sqrtf((x * x) + (y * y) + (z * z));
            float delta = fabsf(mag - prev_mag);
            if(delta > (IMPACT_THRESHOLD_G * 9.80665f)) {
                impacts++;
                mcu_printf("  [IMPACT #%d] at %d ms, Δ|A|: ", impacts, (current - start));
                Print_Float_Value(delta / 9.80665f, 100);
                mcu_printf(" G\n");
                SAL_TaskSleep(100);
            }
            prev_mag = mag;
        }
        SAL_TaskSleep(50);
    }
    
    mcu_printf("  Total Impacts: %d\n", impacts);
    return SAL_RET_SUCCESS;
}static void ADXL_Test_Calibration(void)
{
    SALRetCode_t ret;
    ADXL345_Calibration_t cal;
    float x, y, z, magnitude;
    uint8 i;

    mcu_printf("\n");
    mcu_printf("lqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqk\n");
    mcu_printf("x   CALIBRATION TEST                 x\n");
    mcu_printf("mqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqj\n");
    mcu_printf("\n");

    mcu_printf("[BEFORE CALIBRATION]\n");
    mcu_printf("=========================================\n");
    
    for (i = 0; i < 5; i++) {
        ret = ADXL345_ReadAccel(0, &x, &y, &z);
        if (ret == SAL_RET_SUCCESS) {
            magnitude = sqrtf(x*x + y*y + z*z);
            
            // ✅ Print_Float_Value 사용
            mcu_printf("  Raw: X:");
            Print_Float_Value(x, 100);
            mcu_printf(" Y:");
            Print_Float_Value(y, 100);
            mcu_printf(" Z:");
            Print_Float_Value(z, 100);
            mcu_printf(" |A|:");
            Print_Float_Value(magnitude, 100);
            mcu_printf(" m/s²\n");
        }
        SAL_TaskSleep(200);
    }

    mcu_printf("\n[CALIBRATION]\n");
    mcu_printf("=========================================\n");
    ret = ADXL345_CalibrateOffset(0, 500);
    
    if (ret != SAL_RET_SUCCESS) {
        mcu_printf("[ERROR] Calibration failed!\n");
        return;
    }
 ADXL345_GetCalibration(0, &cal);
    mcu_printf("\n[CALIBRATION PARAMETERS]\n");
    mcu_printf("=========================================\n");
    
    // ✅ multiplier를 1000으로 증가
    mcu_printf("  Offset X: ");
    Print_Float_Value(cal.offset_x, 1000);
    mcu_printf(" m/s²\n");
    
    mcu_printf("  Offset Y: ");
    Print_Float_Value(cal.offset_y, 1000);
    mcu_printf(" m/s²\n");
    
    mcu_printf("  Offset Z: ");
    Print_Float_Value(cal.offset_z, 1000);
    mcu_printf(" m/s²\n");
    
    mcu_printf("  Scale  X: ");
    Print_Float_Value(cal.scale_x, 1000);
    mcu_printf("\n");
    
    mcu_printf("  Scale  Y: ");
    Print_Float_Value(cal.scale_y, 1000);
    mcu_printf("\n");
    
    mcu_printf("  Scale  Z: ");
    Print_Float_Value(cal.scale_z, 1000);
    mcu_printf("\n");
    
    mcu_printf("\n[AFTER CALIBRATION]\n");
    mcu_printf("=========================================\n");
    
    for (i = 0; i < 5; i++) {
        ret = ADXL345_ReadAccelCalibrated(0, &x, &y, &z);
        if (ret == SAL_RET_SUCCESS) {
            magnitude = sqrtf(x*x + y*y + z*z);
            
            // ✅ Print_Float_Value 사용
            mcu_printf("  Cal: X:");
            Print_Float_Value(x, 100);
            mcu_printf(" Y:");
            Print_Float_Value(y, 100);
            mcu_printf(" Z:");
            Print_Float_Value(z, 100);
            mcu_printf(" |A|:");
            Print_Float_Value(magnitude, 100);
            mcu_printf(" m/s²\n");
        }
        SAL_TaskSleep(200);
    }

    mcu_printf("\n[EXPECTED] When flat: X≈0, Y≈0, Z≈9.81 m/s²\n");
}