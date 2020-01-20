#ifndef __VL53L0_H
#define __VL53L0_H

#include "stm32f4xx.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_cali.h"

#define Xshut_RCC    RCC_AHB1Periph_GPIOB
#define Xshut_GPIO   GPIOB
#define Xshut_PIN    GPIO_Pin_14

//控制Xshut电平,从而使能VL53L0X工作 1:使能 0:关闭
#define Xshut_H()    GPIO_SetBits(Xshut_GPIO, Xshut_PIN)
#define Xshut_L()    GPIO_ResetBits(Xshut_GPIO, Xshut_PIN)

//使能2.8V IO电平模式
#define USE_I2C_2V8  1

#define STATUS_OK       0x00
#define STATUS_FAIL     0x01

//测量模式
#define Default_Mode   0// 默认
#define HIGH_ACCURACY  1//高精度
#define LONG_RANGE     2//长距离
#define HIGH_SPEED     3//高速

//vl53l0x模式配置参数集
typedef __packed struct
{
	FixPoint1616_t signalLimit;    //Signal极限数值 
	FixPoint1616_t sigmaLimit;     //Sigmal极限数值
	uint32_t timingBudget;         //采样时间周期
	uint8_t preRangeVcselPeriod ;  //VCSEL脉冲周期
	uint8_t finalRangeVcselPeriod ;//VCSEL脉冲周期范围
	
}mode_data;



//VL53L0X传感器上电默认IIC地址为0X52(不包含最低位)
#define VL53L0_Addr 0x54

#define INFO(fmt,...)
enum
{
	TimeOut=0,//超时
	TurnLeft,
	TurnRight,
	RunForward,
	RunRetreat,
};



extern mode_data Mode_data[];
extern uint8_t AjustOK;
extern VL53L0X_Dev_t Sensor;


VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *dev,u8 select);//初始化vl53l0x

void vl53l0x_reset(VL53L0X_Dev_t *dev);//vl53l0x复位
void SysRefInit(VL53L0X_DEV dev,u8 mode);
void SetupSingleShot(VL53L0X_DEV Dev);
int Vl53L0GestureCheck(void);


void VL53L0_ALL_Init(void);
u16 VL53L0_Get(void);

/*IIC相关*/
u8 VL53L0X_write_byte(u8 address,u8 index,u8 data);              //IIC写一个8位数据
u8 VL53L0X_write_word(u8 address,u8 index,u16 data);             //IIC写一个16位数据
u8 VL53L0X_write_dword(u8 address,u8 index,u32 data);            //IIC写一个32位数据
u8 VL53L0X_write_multi(u8 address, u8 index,u8 *pdata,u16 count);//IIC连续写

u8 VL53L0X_read_byte(u8 address,u8 index,u8 *pdata);             //IIC读一个8位数据
u8 VL53L0X_read_word(u8 address,u8 index,u16 *pdata);            //IIC读一个16位数据
u8 VL53L0X_read_dword(u8 address,u8 index,u32 *pdata);           //IIC读一个32位数据
u8 VL53L0X_read_multi(u8 address,u8 index,u8 *pdata,u16 count);  //IIC连续读

#endif


