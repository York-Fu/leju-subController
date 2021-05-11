#ifndef _Imu_h_
#define _Imu_h_

#include "stdio.h"
#include "string.h"
#include "iic/s_iic.h"
#include "lsm6ds3tr/lsm6ds3tr_c_reg.h"
#include "lsm303ah/lsm303ah_reg.h"
#include "lsm6ds3tr/lsm6ds3.h"
#include "lsm303ah/LSM303C_MAG_driver.h"
#include "system/delay/delay.h"

#define LS6DS3RT_I2C      I2C3
#define LS6303AH_I2C      I2C3

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
}Axis_t;

typedef struct
{
  Axis_t gyro;
  Axis_t accel;
  Axis_t magnetic;
}ImuData_t;

uint8_t imu_init(void);
uint8_t imu_getData(int16_t *imuData);
void imu_convertData(void);

#endif
