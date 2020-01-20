/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
* File Name          : gyro_acc.c
* Author             : danceww
* Version            : V0.0.1
* Date               : 2011/01/15
* Description        : functions about sensor control
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "lsm6ds3.h"
#include "iic/myiic.h"
#include "system/delay/delay.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


u8 Lsm6d3s_ReadByte(u8 addr)
{
  u8 temp=0;

  IIC_Start();

  IIC_Send_Byte(0XD4);	   //发送写命令
  IIC_Wait_Ack();
  IIC_Send_Byte(addr);//发送高地址
  IIC_Wait_Ack();
  IIC_Start();
  IIC_Send_Byte(0XD5);           //进入接收模式
  IIC_Wait_Ack();
  temp=IIC_Read_Byte(0);
  IIC_Stop();//产生一个停止条件
  return temp;
}

void Lsm6d3s_ReadLenByte(u8 ReadAddr,u8* data,u8 Len)
{
  u8 t;

  for(t=0; t<Len; t++)
  {
    data[t]=Lsm6d3s_ReadByte(ReadAddr);
    ReadAddr++;
  }

}

void Lsm6d3s_WriteByte(u8 regaddr,u8 data)
{
  IIC_Start();

  IIC_Send_Byte(0XD4);	    //发送写命令
  IIC_Wait_Ack();
  IIC_Send_Byte(regaddr);//发送地址
  IIC_Wait_Ack();
  IIC_Send_Byte(data);     //发送字节
  IIC_Wait_Ack();
  IIC_Stop();//产生一个停止条件
  delay_ms(10);

}

void Lsm6d3s_Configuration(void)
{
  //while(Lsm6d3s_ReadByte(0x0F)!=0x69);//测试使用

  Lsm6d3s_WriteByte(LSM6DS3TR_C_ACC_GYRO_CTRL2_G,0x1C);
  //ENABLE
  Lsm6d3s_WriteByte(LSM6DS3TR_C_ACC_GYRO_CTRL10_C,0x38);

  //WAKE_UP INTERRUPT Configuration
  Lsm6d3s_WriteByte(LSM6DS3TR_C_ACC_GYRO_CTRL1_XL,0x60);
  Lsm6d3s_WriteByte(LSM6DS3TR_C_ACC_GYRO_TAP_CFG1,0x90);
  Lsm6d3s_WriteByte(LSM6DS3TR_C_ACC_GYRO_WAKE_UP_DUR,0x00);
  Lsm6d3s_WriteByte(LSM6DS3TR_C_ACC_GYRO_WAKE_UP_THS,0x02);
  Lsm6d3s_WriteByte(LSM6DS3TR_C_ACC_GYRO_MD1_CFG,0x20);
  //6D Orientation Configuration
  Lsm6d3s_WriteByte(LSM6DS3TR_C_ACC_GYRO_TAP_THS_6D,0x40);
  Lsm6d3s_WriteByte(LSM6DS3TR_C_ACC_GYRO_CTRL8_XL,0x01);
}





