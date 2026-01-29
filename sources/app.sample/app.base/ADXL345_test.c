#include <sal_internal.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <gpsb.h>
#include <gpio.h>
#include <debug.h>
#include <ADXL345.h>
#include <ADXL345_test.h>

/* Impact detection threshold (2.0G) */
#define IMPACT_THRESHOLD_G          (2.0f)

/* TEST TASK: 가속도계 검증 메인 루틴 */
void ADXL345_Test_Task(void *pArg) {
    (void)pArg;
    uint8 devid = 0;
    uint8 retry = 0;
    
    mcu_printf("\n[ADXL345] Starting Verification Task...\n");
    SAL_TaskSleep(100);

    // TEST 1: Device ID Check
    mcu_printf("\n[TEST 1] Device ID Check\n");
    mcu_printf("=========================================\n");

    for(retry = 0; retry < 5; retry++) {
        if(ADXL345_Test_ReadID(&devid) == SAL_RET_SUCCESS) {
            if(devid == ADXL345_DEVICE_ID) break; 
        }
        
        mcu_printf("  Attempt %d: ID Read = %d\n", (retry + 1), devid);
        SAL_TaskSleep(100);
    }

    if(devid == ADXL345_DEVICE_ID) {
        mcu_printf("  [SUCCESS] ADXL345 Verified (ID: 0xE5)\n");
    } else {
        mcu_printf("  [FATAL] Sensor Not Responding! (Read: 0x%X)\n", devid);
        mcu_printf("  Check: CS(GPB5), SCLK(GPB4), MOSI(GPB6), MISO(GPB7)\n");
        SAL_TaskDelete(0);
        return;
    }

    // 센서 초기 설정
    ADXL345_WriteReg(0, ADXL345_REG_DATA_FORMAT, 0x0B); 
    ADXL345_WriteReg(0, ADXL345_REG_POWER_CTL, ADXL345_MEASURE);
    
    // 테스트 항목 실행
    ADXL_Test_SingleRead();
    ADXL_Test_Continuous(5000);
    ADXL_Test_ImpactDetection(10000);

    mcu_printf("\n[ADXL345] All Tests Finished!\n");
    SAL_TaskDelete(0);
}

/* 단일 데이터 읽기 테스트 */
SALRetCode_t ADXL_Test_SingleRead(void) {
    float x, y, z;
    SALRetCode_t ret = ADXL345_ReadAccel(0, &x, &y, &z);
    
    if(ret == SAL_RET_SUCCESS) {
        mcu_printf("  [Single Read] X:");
        Print_Float_Value(x, 100);
        mcu_printf("  Y:");
        Print_Float_Value(y, 100);
        mcu_printf("  Z:");
        Print_Float_Value(z, 100);
        mcu_printf(" m/s^2\n");
    } else {
        mcu_printf("  [ERROR] Read Failed\n");
    }
    return ret;
}

/* 연속 출력 테스트 (5초간) */
SALRetCode_t ADXL_Test_Continuous(uint32 duration_ms) {
    uint32 start, current, count = 0;
    float x, y, z;

    mcu_printf("\n[TEST 2] Continuous Read (5sec)\n");
    SAL_GetTickCount(&start);
    
    while(1) {
        SAL_GetTickCount(&current);
        if((current - start) >= duration_ms) break;

        if(ADXL345_ReadAccel(0, &x, &y, &z) == SAL_RET_SUCCESS) {
            if(count % 20 == 0) { // 20번에 한 번씩 출력 (부하 감소)
                mcu_printf("  Time: %d ms | Z-Axis: ", (current - start));
                Print_Float_Value(z, 100);
                mcu_printf(" m/s^2\n");
            }
            count++;
        }
        SAL_TaskSleep(5);
    }
    return SAL_RET_SUCCESS;
}

/* 충격 감지 테스트 (10초간) */
SALRetCode_t ADXL_Test_ImpactDetection(uint32 duration_ms) {
    uint32 start, current, impacts = 0;
    float x, y, z, prev_z = 0;

    mcu_printf("\n[TEST 3] Impact Detection (10sec, Threshold: 2.0G)\n");
    SAL_GetTickCount(&start);
    ADXL345_ReadAccel(0, NULL, NULL, &prev_z);

    while(1) {
        SAL_GetTickCount(&current);
        if((current - start) >= duration_ms) break;

        if(ADXL345_ReadAccel(0, &x, &y, &z) == SAL_RET_SUCCESS) {
            float delta = fabsf(z - prev_z);
            if(delta > (IMPACT_THRESHOLD_G * 9.80665f)) {
                impacts++;
                mcu_printf("  [ALERT] IMPACT! Delta: ");
                Print_Float_Value(delta / 9.80665f, 100);
                mcu_printf(" G (at %d ms)\n", (current - start));
                SAL_TaskSleep(100); // 중복 감지 방지
            }
            prev_z = z;
        }
        SAL_TaskSleep(5);
    }
    mcu_printf("  Total Impacts Detected: %d\n", impacts);
    return SAL_RET_SUCCESS;
}