#ifndef __S_IIC_H
#define __S_IIC_H
#include "system/sys/sys.h"

//IO��������
#define SDA_IN() {GPIOC->MODER &= ~(3 << (9 * 2));GPIOC->MODER |= 0 << 9 * 2;} //����ģʽ
#define SDA_OUT() {GPIOC->MODER &= ~(3 << (9 * 2));GPIOC->MODER |= 1 << 9 * 2;} //���ģʽ
//IO��������
#define IIC_SCL PAout(8) //SCL
#define IIC_SDA PCout(9) //SDA
#define READ_SDA PCin(9) //����SDA

//IIC���в�������
void IIC_Init(void);								 //��ʼ��IIC��IO��
void IIC_Start(void);								 //����IIC��ʼ�ź�
void IIC_Stop(void);								 //����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);					 //IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack); //IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void);							 //IIC�ȴ�ACK�ź�
void IIC_Ack(void);									 //IIC����ACK�ź�
void IIC_NAck(void);								 //IIC������ACK�ź�

uint8_t IIC_WriteByte(uint8_t i2cAddr, uint8_t reg, uint8_t* data);
uint8_t IIC_ReadByte(uint8_t i2cAddr, uint8_t reg, uint8_t *data);
uint8_t IIC_WriteNByte(uint8_t i2cAddr, uint8_t reg, uint8_t *data, uint16_t len);
uint8_t IIC_ReadNByte(uint8_t i2cAddr, uint8_t reg, uint8_t *data, uint16_t len);;

#endif
