#include "sensor.h"
#include "DXL.h"
#include "tim/tim.h"
#include "adc/adc.h"
#include "led/led.h"
#include "key/key.h"
#include "imu.h"
#include "VL53L0/VL53L0.h"
#include "Buzzer/Buzzer.h"

/* Sensor get -----------------------------------------------------------------------*/

void sensor_getADC()
{
  DATA_VBAT_ADC = (uint8_t)((double)ADC_GetValue() / 4096 * 3.3 * POWER_ENLARGE_NUM * 10);
}

void sensor_getKey()
{
  uint8_t keyStatus = 0;

  keyStatus = KeyStatusAllGet();

  DATA_KEY1_STATUS = (keyStatus >> 0) & (0x01);
  DATA_KEY2_STATUS = (keyStatus >> 1) & (0x01);
  DATA_KEY3_STATUS = (keyStatus >> 2) & (0x01);
  DATA_KEY4_STATUS = (keyStatus >> 3) & (0x01);
  DATA_KEY5_STATUS = (keyStatus >> 4) & (0x01);
}

void sensor_getImu()
{
  int16_t imuData[9];
  imu_getData(imuData);
  DATA_GYRO_X = imuData[0];
  DATA_GYRO_Y = imuData[1];
  DATA_GYRO_Z = imuData[2];
  DATA_ACCEL_X = imuData[3];
  DATA_ACCEL_Y = imuData[4];
  DATA_ACCEL_Z = imuData[5];
  DATA_MAGNETIC_X = imuData[6];
  DATA_MAGNETIC_Y = imuData[7];
  DATA_MAGNETIC_Z = imuData[8];
}

void sensor_getDistance()
{
  DATA_DISTANCE = VL53L0_Get();
}

/* Sensor set -----------------------------------------------------------------------*/

void sensor_setLed()
{
  for (uint8_t i = 0; i < 5; i++)
  {
    if (DATA_LED_STATUS & (1 << i))
      led_on(i + 1);
    else
      led_off(i + 1);
  }
}

void sensor_set()
{
  sensor_setLed();
}

/* Sensor --------------------------------------------------------------------------*/

void dxl_powerCtl()
{
  static uint8_t flag = 0;
  if (DATA_KEY1_STATUS && !flag)
  {
    if (dxl_getPower())
    {
      dxl_setPower(0);
      DATA_DYNAMIXEL_POWER = 0;
      key_buzzer(1);
    }
    else
    {
      dxl_setPower(1);
      DATA_DYNAMIXEL_POWER = 1;
      key_buzzer(2);
    }
    flag = 1;
  }
  else if (!DATA_KEY1_STATUS && flag)
    flag = 0;
}

void buzzer_warning()
{
  float PowerBattary = DATA_VBAT_ADC / 10.0;
  Buzzer(PowerBattary);
}

#define TIMER_TIME_MS (5.0) // ms

void sensor_poll()
{
  static uint32_t count = 0;
  sensor_getADC();
  sensor_getImu();
  if (count % (uint32_t)(100.0 / TIMER_TIME_MS) == 0)
  {
    sensor_getKey();
    sensor_getDistance();
    buzzer_warning();
    dxl_powerCtl();
  }
  count++;
  if (count >= 1000)
    count = 0;
}

static void irqHandle()
{
  sensor_poll();
}

void sensor_init()
{
  ADC_Config();
  IIC_Init();
  key_init();

  uint32_t count = 0;
  while (imu_init())
  {
    led_on(1);
    delay_ms(50);
    led_off(1);
    delay_ms(50);
    count++;
    if (count >= 20)
    {
      break;
    }
  }
  VL53L0_ALL_Init();

  Tim2Irq_Set(irqHandle);
  Tim2IntInit(TIMER_TIME_MS * 1000 - 1, 84 - 1);
}
