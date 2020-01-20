#include "APL_IMU/IMU.h"
#include "iic/myiic.h"
#include "EPL_IMU/lsm6ds3.h"
#include "EPL_IMU/LSM303C_MAG_driver.h"


// raw data -32000~32000
GyroData_t GyroRawData;
AccelData_t AccelRawData;
MagneticData_t MagneticRawData;

// convert data 0 ~ 1023
GyroData_t GyroData;
AccelData_t AccelData;
MagneticData_t MagneticData;



void IMU_GetRawValue(ImuData_t *data)
{
  data->gray = GyroRawData;
  data->accel = AccelRawData;
  data->magnetic = MagneticRawData;
}


void IMU_GetValue(ImuData_t *data)
{
  data->gray = GyroData;
  data->accel = AccelData;
  data->magnetic = MagneticData;
}


void IMU_Init()
{
  IIC_Init();
  Lsm6d3s_Configuration();
  Lsm303c_Configuration();
}


void IMU_GetData(ImuData_t *data)
{
  uint8_t gryo_data[6]= {0};
  uint8_t acc_data[6]= {0};
  uint8_t mag_data[6]= {0};

  //gyro read
  if((Lsm6d3s_ReadByte(LSM6DS3TR_C_ACC_GYRO_STATUS_REG)&0x02)!=0)
  {
    Lsm6d3s_ReadLenByte(LSM6DS3TR_C_ACC_GYRO_OUTX_L_G,gryo_data,6);
    data->gray.x = (gryo_data[1]<<8)|(gryo_data[0]);
    data->gray.y = (gryo_data[3]<<8)|(gryo_data[2]);
    data->gray.z = (gryo_data[5]<<8)|(gryo_data[4]);
  }

  //acc read
  if((Lsm6d3s_ReadByte(LSM6DS3TR_C_ACC_GYRO_STATUS_REG)&0x01)!=0)
  {
    Lsm6d3s_ReadLenByte(LSM6DS3TR_C_ACC_GYRO_OUTX_L_XL,acc_data,6);

    data->accel.x = (acc_data[1]<<8)|(acc_data[0]);
    data->accel.y = (acc_data[3]<<8)|(acc_data[2]);
    data->accel.z = (acc_data[5]<<8)|(acc_data[4]);
  }
  
  //mag read
  if(LSM303C_MAG_R_NewXYZData(0, (LSM303C_MAG_ZYXDA_t*) &flag_LSM303C_MAG_XYZDA)!=0)
  {
    LSM303C_MAG_Get_Magnetic(0, mag_data);
    data->magnetic.x = (mag_data[1]<<8)|(mag_data[0]);  
    data->magnetic.y = (mag_data[3]<<8)|(mag_data[2]);
    data->magnetic.z = (mag_data[5]<<8)|(mag_data[4]);
  }
}

