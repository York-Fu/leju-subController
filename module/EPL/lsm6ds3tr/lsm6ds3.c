/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
* File Name          : gyro_acc.c
* Author             : danceww
* Version            : V0.0.1
* Date               : 2011/01/15
* Description        : functions about sensor control
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "lsm6ds3.h"
#include "iic/s_iic.h"
#include "system/delay/delay.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

u8 Lsm6d3s_ReadByte(u8 addr)
{
  u8 temp = 0;
  IIC_ReadByte(0XD4, addr, &temp);
  return temp;
}

void Lsm6d3s_ReadLenByte(u8 ReadAddr, u8 *data, u8 Len)
{
  for (uint16_t i = 0; i < Len; i++)
  {
    IIC_ReadByte(0XD4, ReadAddr++, data++);
  }
}

void Lsm6d3s_WriteByte(u8 regaddr, u8 data)
{
  IIC_WriteByte(0XD4, regaddr, &data);
  delay_ms(10);
}

// Lsm6ds3
void Lsm6d3s_Configuration(void)
{
  //while(Lsm6d3s_ReadByte(0x0F)!=0x69);//²âÊÔÊ¹ÓÃ

  Lsm6d3s_WriteByte(LSM6DS3TR_C_ACC_GYRO_CTRL2_G, 0x68);
  //ENABLE
  Lsm6d3s_WriteByte(LSM6DS3TR_C_ACC_GYRO_CTRL10_C, 0x38);

  //WAKE_UP INTERRUPT Configuration
  Lsm6d3s_WriteByte(LSM6DS3TR_C_ACC_GYRO_CTRL1_XL, 0x6c);
  Lsm6d3s_WriteByte(LSM6DS3TR_C_ACC_GYRO_TAP_CFG1, 0x90);
  Lsm6d3s_WriteByte(LSM6DS3TR_C_ACC_GYRO_WAKE_UP_DUR, 0x00);
  Lsm6d3s_WriteByte(LSM6DS3TR_C_ACC_GYRO_WAKE_UP_THS, 0x02);
  Lsm6d3s_WriteByte(LSM6DS3TR_C_ACC_GYRO_MD1_CFG, 0x20);
  //6D Orientation Configuration
  Lsm6d3s_WriteByte(LSM6DS3TR_C_ACC_GYRO_TAP_THS_6D, 0x40);
  Lsm6d3s_WriteByte(LSM6DS3TR_C_ACC_GYRO_CTRL8_XL, 0x01);
}
