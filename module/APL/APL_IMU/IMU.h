#ifndef _IMU_h_
#define _IMU_h_

#include "stm32f4xx.h"


typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
}GyroData_t;

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
}AccelData_t;

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
}MagneticData_t;


typedef struct
{
  GyroData_t gray;
  AccelData_t accel;
  MagneticData_t magnetic;
}ImuData_t;


void IMU_GetRawData(ImuData_t *data);
void IMU_GetData(ImuData_t *data);

void IMU_Init(void);


#endif

