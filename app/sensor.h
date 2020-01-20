#ifndef _sensor_h_
#define _sensor_h_

#include "stm32f4xx.h"
#include "DxlRegister.h"


/* 映射数据定义 ------------------------------------------------------------------------*/
#define DATA_LED_STATUS               HALFWORD_CAST(gbpControlTable[P_LED_HEAD])

#define DATA_KEY1_STATUS              gbpControlTable[P_BUTTON]
#define DATA_KEY2_STATUS              gbpControlTable[P_BUTTON+1]
#define DATA_KEY3_STATUS              gbpControlTable[P_BUTTON+2]
#define DATA_KEY4_STATUS              gbpControlTable[P_BUTTON+3]
#define DATA_KEY5_STATUS              gbpControlTable[P_BUTTON+4]

#define DATA_GYRO_X                   HALFWORD_CAST(gbpControlTable[P_GYRO_X])
#define DATA_GYRO_Y                   HALFWORD_CAST(gbpControlTable[P_GYRO_Y])
#define DATA_GYRO_Z                   HALFWORD_CAST(gbpControlTable[P_GYRO_Z])
#define DATA_ACCEL_X                  HALFWORD_CAST(gbpControlTable[P_ACC_X])
#define DATA_ACCEL_Y                  HALFWORD_CAST(gbpControlTable[P_ACC_Y])
#define DATA_ACCEL_Z                  HALFWORD_CAST(gbpControlTable[P_ACC_Z])
#define DATA_MAGNETIC_X               HALFWORD_CAST(gbpControlTable[P_LEFT_MIC])
#define DATA_MAGNETIC_Y               HALFWORD_CAST(gbpControlTable[P_ADC_CH2])
#define DATA_MAGNETIC_Z               HALFWORD_CAST(gbpControlTable[P_ADC_CH3])

#define DATA_VBAT_ADC                 gbpControlTable[P_PRESENT_VOLTAGE]

#define DATA_DISTANCE                 HALFWORD_CAST(gbpControlTable[P_ADC_CH5])
/*-------------------------------------------------------------------------------------*/


#define POWER_ENLARGE_NUM    (39.2+6.19)/6.19

void SensorInit(void);
void SensorGet(void);
void SensorSet(void);

#endif

