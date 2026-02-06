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

static SALRetCode_t ADXL345_ReadRegStable(uint8 dev, uint8 reg, uint8 *val);
static SALRetCode_t ADXL345_WriteVerify(uint8 dev, uint8 reg, uint8 val, uint8 retries, uint32 delay_ms);
static const char *g_wheel_name[ADXL_COUNT] = {
    "FL", "RL", "RR", "FR"   // 네가 말한 순서대로 매핑 (필요하면 바꿔)
};


static SALRetCode_t ADXL345_ReadRegStable(uint8 dev, uint8 reg, uint8 *val)
{
    uint8 a=0,b=0;
    for (uint8 retry=0; retry<5; retry++) {
        if (ADXL345_ReadReg(dev, reg, &a) != SAL_RET_SUCCESS) { SAL_TaskSleep(2); continue; }
        if (ADXL345_ReadReg(dev, reg, &b) != SAL_RET_SUCCESS) { SAL_TaskSleep(2); continue; }
        if (a==b) { if(val) *val=a; return SAL_RET_SUCCESS; }
        SAL_TaskSleep(2);
    }
    if(val) *val=b;
    mcu_printf("  [WARN] ADXL%d unstable reg 0x%02X (last=0x%02X)\n", dev, reg, b);
    return SAL_RET_SUCCESS;
}

static SALRetCode_t ADXL345_WriteVerify(uint8 dev, uint8 reg, uint8 val, uint8 retries, uint32 delay_ms)
{
    uint8 readback=0;
    for (uint8 i=0; i<retries; i++) {
        (void)ADXL345_WriteReg(dev, reg, val);
        SAL_TaskSleep(delay_ms);
        if (ADXL345_ReadRegStable(dev, reg, &readback)==SAL_RET_SUCCESS && readback==val) {
            return SAL_RET_SUCCESS;
        }
        SAL_TaskSleep(5);
    }
    return SAL_RET_FAILED;
}

void ADXL345_Test_Task(void *pArg)
{
    (void)pArg;

    uint8 devid = 0;
    uint8 dev, retry;
    uint8 reg_before = 0, reg_after = 0;
    uint8 burst[6] = {0};

    mcu_printf("\n╔════════════════════════════════════╗\n");
    mcu_printf("║   ADXL345 x4 (TCA9548A) Test Start ║\n");
    mcu_printf("╚════════════════════════════════════╝\n\n");

    SAL_TaskSleep(100);

    /* ========== TEST 0: MUX sanity (optional) ========== */
    mcu_printf("[MUX] Selecting CH0..CH3\n");
    for (dev = 0; dev < ADXL_COUNT; dev++) {
        if (ADXL_MuxSelectByDev(dev) != SAL_RET_SUCCESS) {
            mcu_printf("  [FATAL] MUX select failed for ADXL%d (ch=%d)\n", dev, g_adxl_mux_ch[dev]);
            SAL_TaskDelete(0);
            return;
        }
        SAL_TaskSleep(5);
    }
    mcu_printf("  [OK] MUX channel switch OK\n\n");

    /* ========== TEST 1: Device ID for all sensors ========== */
    mcu_printf("[TEST 1] Device ID Check (ADXL0~3)\n");
    mcu_printf("=========================================\n");

    for (dev = 0; dev < ADXL_COUNT; dev++) {
        devid = 0;

        for (retry = 0; retry < 5; retry++) {
            /* 채널 선택 후 ID 읽기 */
            if (ADXL_MuxSelectByDev(dev) == SAL_RET_SUCCESS) {
                if (ADXL345_Test_ReadID(&devid) == SAL_RET_SUCCESS && devid == ADXL345_DEVICE_ID) {
                    break;
                }
            }
            mcu_printf("  [ADXL%d] Retry %d/5... (Read: 0x%02X)\n", dev, retry + 1, devid);
            SAL_TaskSleep(100);
        }

        if (devid != ADXL345_DEVICE_ID) {
            mcu_printf("  [FATAL][ADXL%d] Not Responding! (Got 0x%02X)\n", dev, devid);
            mcu_printf("    - Check wiring on MUX CH%d\n", g_adxl_mux_ch[dev]);
            SAL_TaskDelete(0);
            return;
        }

        mcu_printf("  [OK][ADXL%d] ID=0x%02X\n", dev, devid);
    }

    mcu_printf("\n");

    /* ========== TEST 2: Configure all sensors ========== */
    mcu_printf("[TEST 2] Configure (BW_RATE/DATA_FORMAT/POWER_CTL)\n");
    mcu_printf("=========================================\n");

    for (dev = 0; dev < ADXL_COUNT; dev++) {
        (void)ADXL345_ReadRegStable(dev, ADXL345_REG_DATA_FORMAT, &reg_before);

        (void)ADXL345_WriteVerify(dev, ADXL345_REG_BW_RATE,      0x0A, 3, 10);
        (void)ADXL345_WriteVerify(dev, ADXL345_REG_DATA_FORMAT,  0x09, 3, 10);
        (void)ADXL345_WriteVerify(dev, ADXL345_REG_POWER_CTL,    0x08, 3, 20);
        SAL_TaskSleep(20);

        (void)ADXL345_ReadRegStable(dev, ADXL345_REG_DATA_FORMAT, &reg_after);

        mcu_printf("  [ADXL%d] DATA_FORMAT: 0x%02X -> 0x%02X (exp 0x09)\n",
                   dev, reg_before, reg_after);

        if (ADXL345_ReadRegsBurst(dev, ADXL345_REG_BW_RATE, burst, sizeof(burst)) == SAL_RET_SUCCESS) {
            mcu_printf("  [ADXL%d] BURST 0x2C-0x31: %02X %02X %02X %02X %02X %02X\n",
                       dev, burst[0], burst[1], burst[2], burst[3], burst[4], burst[5]);
        } else {
            mcu_printf("  [ADXL%d] [ERROR] BURST read failed\n", dev);
        }
    }

    mcu_printf("\n");

    /* ========== TEST 3: Impact-only read all sensors for 10 seconds ========== */
    mcu_printf("[TEST 3] Impact Event Only (10 sec, > %.2fG)\n", IMPACT_THRESHOLD_G);
    mcu_printf("=========================================\n");

    {
        uint32 start, now;
        float x, y, z;
        float prev_mag[ADXL_COUNT] = {0};
        uint32 last_hit_ms[ADXL_COUNT] = {0};

        SAL_GetTickCount(&start);

        /* 초기 prev_mag 세팅 */
        for (dev = 0; dev < ADXL_COUNT; dev++) {
            if (ADXL_MuxSelectByDev(dev) == SAL_RET_SUCCESS) {
                if (ADXL345_ReadAccelCalibrated(dev, &x, &y, &z) == SAL_RET_SUCCESS) {
                    prev_mag[dev] = sqrtf(x*x + y*y + z*z);
                }
            }
            SAL_TaskSleep(5);
        }

        while (1) {
            SAL_GetTickCount(&now);
            if ((now - start) >= 10000U) break;   // 10초

            for (dev = 0; dev < ADXL_COUNT; dev++) {

                if (ADXL_MuxSelectByDev(dev) != SAL_RET_SUCCESS) {
                    continue;
                }

                if (ADXL345_ReadAccelCalibrated(dev, &x, &y, &z) == SAL_RET_SUCCESS) {

                    float mag   = sqrtf(x*x + y*y + z*z);
                    float delta = fabsf(mag - prev_mag[dev]);

                    /* 임팩트 판정 + 쿨다운 */
                    if (delta >= IMPACT_THRESHOLD_MS2) {
                        uint32 tms = (now - start);

                        if ((tms - last_hit_ms[dev]) >= IMPACT_COOLDOWN_MS) {
                            last_hit_ms[dev] = tms;

                            mcu_printf("  [IMPACT][%4d ms][ADXL%d", (int)tms, dev);
#ifdef ADXL_COUNT
                            mcu_printf("/%s", g_wheel_name[dev]);   // 휠명 출력 (선택)
#endif
                            mcu_printf("] Δ|A|:");
                            Print_Float_Value(delta / G_TO_MS2, 100);
                            mcu_printf(" G  |A|:");
                            Print_Float_Value(mag / G_TO_MS2, 100);
                            mcu_printf(" G  X:");
                            Print_Float_Value(x / G_TO_MS2, 100);
                            mcu_printf(" Y:");
                            Print_Float_Value(y / G_TO_MS2, 100);
                            mcu_printf(" Z:");
                            Print_Float_Value(z / G_TO_MS2, 100);
                            mcu_printf(" (G)\n");
                        }
                    }

                    prev_mag[dev] = mag;
                }
            }

            SAL_TaskSleep(20); // 50ms보다 더 촘촘히 보고 싶으면 20ms 권장
        }
    }

    mcu_printf("\n╔════════════════════════════════════╗\n");
    mcu_printf("║   ADXL345 x4 Test Finished!        ║\n");
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
}