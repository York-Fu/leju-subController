#ifndef __S_IIC_H
#define __S_IIC_H
#include "system/sys/sys.h"

//IO方向设置
#define SDA_IN() {GPIOC->MODER &= ~(3 << (9 * 2));GPIOC->MODER |= 0 << 9 * 2;} //输入模式
#define SDA_OUT() {GPIOC->MODER &= ~(3 << (9 * 2));GPIOC->MODER |= 1 << 9 * 2;} //输出模式
//IO操作函数
#define IIC_SCL PAout(8) //SCL
#define IIC_SDA PCout(9) //SDA
#define READ_SDA PCin(9) //输入SDA

//IIC所有操作函数
void IIC_Init(void);								 //初始化IIC的IO口
void IIC_Start(void);								 //发送IIC开始信号
void IIC_Stop(void);								 //发送IIC停止信号
void IIC_Send_Byte(u8 txd);					 //IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack); //IIC读取一个字节
u8 IIC_Wait_Ack(void);							 //IIC等待ACK信号
void IIC_Ack(void);									 //IIC发送ACK信号
void IIC_NAck(void);								 //IIC不发送ACK信号

uint8_t IIC_WriteByte(uint8_t i2cAddr, uint8_t reg, uint8_t* data);
uint8_t IIC_ReadByte(uint8_t i2cAddr, uint8_t reg, uint8_t *data);
uint8_t IIC_WriteNByte(uint8_t i2cAddr, uint8_t reg, uint8_t *data, uint16_t len);
uint8_t IIC_ReadNByte(uint8_t i2cAddr, uint8_t reg, uint8_t *data, uint16_t len);;

#endif
