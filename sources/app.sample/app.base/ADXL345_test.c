#include <sal_internal.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <debug.h>
#include <ADXL345.h>
#include <ADXL345_test.h>
#include <printf_float.h>

#define IMPACT_THRESHOLD_G  (2.0f)

/*
 * ADXL345 검증 메인 태스크
 */
void ADXL345_Test_Task(void *pArg) 
{
    (void)pArg;
    uint8 devid = 0;
    uint8 retry = 0;
    uint8 reg = 0;
    
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
        mcu_printf("    - I2C Pins: SCL= , SDA= \n");
        mcu_printf("    - I2C Channel: %d, Port: %d\n", ADXL345_I2C_CH, ADXL345_I2C_PORT);
        mcu_printf("    - I2C Addr: 0x%02X (7-bit)\n", ADXL345_I2C_ADDR_7BIT);
        mcu_printf("    - Hardware connections\n");
        SAL_TaskDelete(0);
        return;
    }

    mcu_printf("  [SUCCESS] ADXL345 Verified (ID: 0xE5)\n\n");

    /* ========== 센서 초기화 ========== */
    mcu_printf("[ADXL345] Configuring Sensor...\n");
    ADXL345_WriteReg(0, ADXL345_REG_BW_RATE, 0x0A);     // 100Hz
    SAL_TaskSleep(10);
    ADXL345_WriteReg(0, ADXL345_REG_DATA_FORMAT, 0x09); // ±4g Full-res
    SAL_TaskSleep(10);
    ADXL345_WriteReg(0, ADXL345_REG_POWER_CTL, 0x08);   // Measure mode
    SAL_TaskSleep(50);
    mcu_printf("[ADXL345] Configuration Complete\n\n");
    
    /* ========== 테스트 실행 ========== */
    mcu_printf("[TEST 2] Register Readback\n");
    mcu_printf("=========================================\n");
    if (ADXL345_ReadReg(0, ADXL345_REG_BW_RATE, &reg) == SAL_RET_SUCCESS) {
        mcu_printf("  BW_RATE: 0x%02X (expected 0x0A)\n", reg);
    } else {
        mcu_printf("  [ERROR] BW_RATE read failed\n");
    }
    if (ADXL345_ReadReg(0, ADXL345_REG_DATA_FORMAT, &reg) == SAL_RET_SUCCESS) {
        mcu_printf("  DATA_FORMAT: 0x%02X (expected 0x09)\n", reg);
    } else {
        mcu_printf("  [ERROR] DATA_FORMAT read failed\n");
    }
    if (ADXL345_ReadReg(0, ADXL345_REG_POWER_CTL, &reg) == SAL_RET_SUCCESS) {
        mcu_printf("  POWER_CTL: 0x%02X (expected 0x08)\n", reg);
    } else {
        mcu_printf("  [ERROR] POWER_CTL read failed\n");
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
    
    SALRetCode_t ret = ADXL345_ReadAccel(0, &x, &y, &z);
    
    if(ret == SAL_RET_SUCCESS) {
        mcu_printf("  X: ");
        Print_Float_Value(x, 100);
        mcu_printf(" m/s² | Y: ");
        Print_Float_Value(y, 100);
        mcu_printf(" m/s² | Z: ");
        Print_Float_Value(z, 100);
        mcu_printf(" m/s²\n");
    } else {
        mcu_printf("  [ERROR] Read Failed\n");
    }
    return ret;
}

/* 연속 읽기 테스트 */
SALRetCode_t ADXL_Test_Continuous(uint32 duration_ms) 
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
            if(count % 20 == 0) {
                mcu_printf("  [%5d ms] Z: ", (current - start));
                Print_Float_Value(z, 100);
                mcu_printf(" m/s²\n");
            }
            count++;
        }
        SAL_TaskSleep(50); // 20Hz
    }
    mcu_printf("  Total samples: %d\n", count);
    return SAL_RET_SUCCESS;
}

/* 충격 감지 테스트 */
SALRetCode_t ADXL_Test_ImpactDetection(uint32 duration_ms) 
{
    uint32 start, current, impacts = 0;
    float x, y, z, prev_z = 0;

    mcu_printf("\n[TEST 5] Impact Detection (10sec, >2.0G)\n");
    mcu_printf("=========================================\n");
    
    SAL_GetTickCount(&start);
    ADXL345_ReadAccel(0, NULL, NULL, &prev_z);

    while(1) {
        SAL_GetTickCount(&current);
        if((current - start) >= duration_ms) break;

        if(ADXL345_ReadAccel(0, &x, &y, &z) == SAL_RET_SUCCESS) {
            float delta = fabsf(z - prev_z);
            if(delta > (IMPACT_THRESHOLD_G * 9.80665f)) {
                impacts++;
                mcu_printf("  [IMPACT #%d] at %d ms, ΔZ: ", impacts, (current - start));
                Print_Float_Value(delta / 9.80665f, 100);
                mcu_printf(" G\n");
                SAL_TaskSleep(100);
            }
            prev_z = z;
        }
        SAL_TaskSleep(50);
    }
    
    mcu_printf("  Total Impacts: %d\n", impacts);
    return SAL_RET_SUCCESS;
}
