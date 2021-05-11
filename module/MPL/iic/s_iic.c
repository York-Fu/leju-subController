#include "s_iic.h"
#include "system/delay/delay.h"

//��ʼ��IIC
void IIC_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      //��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       //����

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //ʹ��GPIOBʱ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;             //GPIO��ʼ������
  GPIO_Init(GPIOA, &GPIO_InitStructure);                //��ʼ��

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //ʹ��GPIOBʱ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;             //GPIO��ʼ������
  GPIO_Init(GPIOC, &GPIO_InitStructure);                //��ʼ��

  IIC_SCL = 1;
  IIC_SDA = 1;
}
//����IIC��ʼ�ź�
void IIC_Start(void)
{
  SDA_OUT(); //sda�����
  IIC_SDA = 1;
  IIC_SCL = 1;
  delay_us(1);
  IIC_SDA = 0; //START:when CLK is high,DATA change form high to low
  delay_us(1);
  IIC_SCL = 0; //ǯסI2C���ߣ�׼�����ͻ��������
}
//����IICֹͣ�ź�
void IIC_Stop(void)
{
  SDA_OUT(); //sda�����
  IIC_SCL = 0;
  IIC_SDA = 0; //STOP:when CLK is high DATA change form low to high
  delay_us(1);
  IIC_SCL = 1;
  IIC_SDA = 1; //����I2C���߽����ź�
  delay_us(1);
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
  u8 ucErrTime = 0;
  SDA_IN(); //SDA����Ϊ����
  IIC_SDA = 1;
  delay_us(1);
  IIC_SCL = 1;
  delay_us(1);
  while (READ_SDA)
  {
    ucErrTime++;
    if (ucErrTime > 250)
    {
      IIC_Stop();
      return 1;
    }
  }
  IIC_SCL = 0; //ʱ�����0
  return 0;
}
//����ACKӦ��
void IIC_Ack(void)
{
  IIC_SCL = 0;
  SDA_OUT();
  IIC_SDA = 0;
  delay_us(1);
  IIC_SCL = 1;
  delay_us(1);
  IIC_SCL = 0;
}
//������ACKӦ��
void IIC_NAck(void)
{
  IIC_SCL = 0;
  SDA_OUT();
  IIC_SDA = 1;
  delay_us(1);
  IIC_SCL = 1;
  delay_us(1);
  IIC_SCL = 0;
}
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��
void IIC_Send_Byte(u8 txd)
{
  u8 t;
  SDA_OUT();
  IIC_SCL = 0; //����ʱ�ӿ�ʼ���ݴ���
  for (t = 0; t < 8; t++)
  {
    IIC_SDA = (txd & 0x80) >> 7;
    txd <<= 1;
    delay_us(1); //��TEA5767��������ʱ���Ǳ����
    IIC_SCL = 1;
    delay_us(1);
    IIC_SCL = 0;
    delay_us(1);
  }
}
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK
u8 IIC_Read_Byte(unsigned char ack)
{
  unsigned char i, receive = 0;
  SDA_IN(); //SDA����Ϊ����
  for (i = 0; i < 8; i++)
  {
    IIC_SCL = 0;
    delay_us(1);
    IIC_SCL = 1;
    receive <<= 1;
    if (READ_SDA)
      receive++;
    delay_us(1);
  }
  if (!ack)
    IIC_NAck(); //����nACK
  else
    IIC_Ack(); //����ACK
  return receive;
}

/**
*	@brief: ���i2cдһ���ֽ�
*/
uint8_t IIC_WriteByte(uint8_t i2cAddr, uint8_t reg, uint8_t *data)
{
  IIC_Start();
  IIC_Send_Byte(i2cAddr & 0xfe);
  if (IIC_Wait_Ack() != 0)
  {
    IIC_Stop();
    return 1;
  }
  IIC_Send_Byte(reg);
  if (IIC_Wait_Ack() != 0)
  {
    IIC_Stop();
    return 2;
  }
  IIC_Send_Byte(*data);
  if (IIC_Wait_Ack() != 0)
  {
    IIC_Stop();
    return 3;
  }
  IIC_Stop();
  return 0;
}

/**
*	@brief: ���i2c��һ���ֽ�
*/
uint8_t IIC_ReadByte(uint8_t i2cAddr, uint8_t reg, uint8_t *data)
{
  IIC_Start();
  IIC_Send_Byte(i2cAddr & 0xfe);
  if (IIC_Wait_Ack() != 0)
  {
    IIC_Stop();
    return 1;
  }
  IIC_Send_Byte(reg);
  if (IIC_Wait_Ack() != 0)
  {
    IIC_Stop();
    return 2;
  }
  IIC_Start();
  IIC_Send_Byte(i2cAddr | 0x01);
  if (IIC_Wait_Ack() != 0)
  {
    IIC_Stop();
    return 3;
  }
  *data = IIC_Read_Byte(0);
  IIC_Stop();
  return 0;
}

/**
*	@brief: ���i2cдN�ֽ�
*/
uint8_t IIC_WriteNByte(uint8_t i2cAddr, uint8_t reg, uint8_t *data, uint16_t len)
{
  IIC_Start();
  IIC_Send_Byte(i2cAddr & 0xfe);
  if (IIC_Wait_Ack() != 0)
  {
    IIC_Stop();
    return 1;
  }
  IIC_Send_Byte(reg);
  if (IIC_Wait_Ack() != 0)
  {
    IIC_Stop();
    return 2;
  }
  for (; len > 0; len--)
  {
    IIC_Send_Byte(*data++);
    if (IIC_Wait_Ack() != 0)
    {
      IIC_Stop();
      return 3;
    }
  }
  IIC_Stop();
  return 0;
}

/**
*	@brief: ���i2c��N�ֽ�
*/
uint8_t IIC_ReadNByte(uint8_t i2cAddr, uint8_t reg, uint8_t *data, uint16_t len)
{
  uint8_t ack;
  IIC_Start();
  IIC_Send_Byte(i2cAddr & 0xfe);
  if (IIC_Wait_Ack() != 0)
  {
    IIC_Stop();
    return 1;
  }
  IIC_Send_Byte(reg);
  if (IIC_Wait_Ack() != 0)
  {
    IIC_Stop();
    return 2;
  }
  IIC_Start();
  IIC_Send_Byte(i2cAddr | 0x01);
  if (IIC_Wait_Ack() != 0)
  {
    IIC_Stop();
    return 3;
  }
  for (; len > 0; len--)
  {
    if (len == 0)
      ack = 0;
    else
      ack = 1;
    *data++ = IIC_Read_Byte(ack);
  }
  IIC_Stop();
  return 0;
}
