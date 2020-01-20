#include "sensor.h"
#include "DXL.h"
#include "tim/tim.h"
#include "adc/adc.h"
#include "led/led.h"
#include "key/key.h"
#include "APL_IMU/IMU.h"
#include "VL53L0/VL53L0.h"
#include "misrophone/misrophone.h"
#include "Buzzer/Buzzer.h"

/* Sensor get -----------------------------------------------------------------------*/

void SensorGetAdc()
{
  uint16_t adcValue = Get_Adc_Average(11, 10);
  DATA_VBAT_ADC = ((double)adcValue)/4096*3.3*POWER_ENLARGE_NUM*10;
}


void SensorGetKey()
{
  uint8_t keyStatus = 0;
  
  keyStatus = KeyStatusAllGet();
  
  DATA_KEY1_STATUS = (keyStatus>>0)&(0x01);
  DATA_KEY2_STATUS = (keyStatus>>1)&(0x01);
  DATA_KEY3_STATUS = (keyStatus>>2)&(0x01);
  DATA_KEY4_STATUS = (keyStatus>>3)&(0x01);
  DATA_KEY5_STATUS = (keyStatus>>4)&(0x01);
}


void SensorGetImu()
{
  ImuData_t imuData = {0};
  
  IMU_GetData(&imuData);
  
  DATA_GYRO_X = imuData.gray.x;
  DATA_GYRO_Y = imuData.gray.y;
  DATA_GYRO_Z = imuData.gray.z;
  DATA_ACCEL_X = imuData.accel.x;
  DATA_ACCEL_Y = imuData.accel.y;
  DATA_ACCEL_Z = imuData.accel.z;
  DATA_MAGNETIC_X = imuData.magnetic.x;
  DATA_MAGNETIC_Y = imuData.magnetic.y;
  DATA_MAGNETIC_Z = imuData.magnetic.z;
}

void SensorGetDistance()
{
	DATA_DISTANCE = VL53L0_Get();
}


void DXL_PowerControl()
{
  static uint8_t flag = 0;
  if(DATA_KEY1_STATUS && !flag)
  {
    if(DXL_GetPower())
    {
      DXL_SetPower(0);
      DATA_DYNAMIXEL_POWER = 0;
    }
    else
    {
      DXL_SetPower(1);
      DATA_DYNAMIXEL_POWER = 1;
    }
    flag = 1;
  }
  else if(!DATA_KEY1_STATUS && flag)
    flag = 0;
}

void BuzzerWarning()
{
	float PowerBattary = DATA_VBAT_ADC/10.0;
	if(PowerBattary < 11)
		BuzzerRing();
	else 
    BuzzerRingStop();
}

void SensorGet()
{
  SensorGetAdc();
  SensorGetImu();
  SensorGetKey();
	SensorGetDistance();
	BuzzerWarning();
	DXL_PowerControl();
}

/* Sensor set -----------------------------------------------------------------------*/

void SensorSetLed()
{
  for(uint8_t i=0;i<5;i++)
  {
    if(DATA_LED_STATUS & (1<<i))
      LedOn(i+1);
    else
      LedOff(i+1);
  }
}


//void SensorSetRgb()
//{
//  for(uint8_t i=0;i<5;i++)
//  {
//    if(DATA_RGB_STATUS & (1<<(i*3)))
//      LedOn(i+1);
//    else
//      LedOff(i+1);
//    
//    if(DATA_RGB_STATUS & (1<<(i*3+1)))
//    {
//    }
//    
//    if(DATA_RGB_STATUS & (1<<(i*3+2)))
//    {
//    }
//  }
//}


void SensorSet()
{
  SensorSetLed();
}

/* Sensor --------------------------------------------------------------------------*/

void SensorIrq()
{
  SensorGet();
}


void SensorInit()
{
  delay_ms(10);
  Adc_Init();
  LedInit();
  KeyInit();
  IMU_Init();
  BuzzerInit();
  VL53L0_ALL_Init();
  Misrophone_Init();
  Tim2Irq_Set(SensorIrq);
  Tim2IntInit(20000-1,672-1); // 40ms
}



