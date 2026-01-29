#ifndef ADXL345_TEST_H_
#define ADXL345_TEST_H_

#include <sal_com.h>

void ADXL345_Test_Task(void *pArg);
SALRetCode_t ADXL_Test_SingleRead(void);
SALRetCode_t ADXL_Test_Continuous(uint32 duration_ms);
SALRetCode_t ADXL_Test_ImpactDetection(uint32 duration_ms);

#endif