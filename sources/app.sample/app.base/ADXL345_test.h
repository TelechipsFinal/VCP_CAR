#ifndef ADXL345_TEST_H_
#define ADXL345_TEST_H_

#include <sal_com.h>

// Main test task
void ADXL345_Test_Task(void *pArg);

// Public test functions (called from other modules)
SALRetCode_t ADXL_Test_SingleRead(void);
SALRetCode_t ADXL_Test_Continuous(uint32 duration_ms);
SALRetCode_t ADXL_Test_ImpactDetection(uint32 duration_ms);
void ADXL_Test_Calibration(uint8 dev);


#endif // ADXL345_TEST_H_