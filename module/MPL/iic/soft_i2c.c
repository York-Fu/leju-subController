#include "soft_i2c.h"

/*
*********************************************************************************************************
*	函 数 名: i2c_GPIO_Config
*	功能说明: I2C GPIO 初始化
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_GPIO_Config()
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  // SCL
  RCC_AHB1PeriphClockCmd(I2C_SCL_GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN;
  GPIO_Init(I2C_SCL_GPIO_PORT, &GPIO_InitStructure);
  // SDA
  RCC_AHB1PeriphClockCmd(I2C_SDA_GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = I2C_SDA_PIN;
  GPIO_Init(I2C_SDA_GPIO_PORT, &GPIO_InitStructure);
}

/*
*********************************************************************************************************
*	函 数 名: i2c_Delay
*	功能说明: I2C总线位延迟，最快400KHz
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void i2c_Delay(void)
{
  uint8_t i;

  /*　
	 	下面的时间是通过逻辑分析仪测试得到的。
    工作条件：CPU主频180MHz ，MDK编译环境，1级优化
      
		循环次数为50时，SCL频率 = 333KHz 
		循环次数为30时，SCL频率 = 533KHz，  
	 	循环次数为20时，SCL频率 = 727KHz， 
  */
  for (i = 0; i < 30 * 2; i++)
    ;
}

/*
*********************************************************************************************************
*	函 数 名: i2c_Start
*	功能说明: CPU发起I2C总线启动信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_Start(void)
{
  /* 当SCL高电平时，SDA出现一个下跳沿表示I2C总线启动信号 */
  I2C_SDA_1();
  I2C_SCL_1();
  i2c_Delay();
  I2C_SDA_0();
  i2c_Delay();
  I2C_SCL_0();
  i2c_Delay();
}

/*
*********************************************************************************************************
*	函 数 名: i2c_Start
*	功能说明: CPU发起I2C总线停止信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_Stop(void)
{
  /* 当SCL高电平时，SDA出现一个上跳沿表示I2C总线停止信号 */
  I2C_SDA_0();
  I2C_SCL_1();
  i2c_Delay();
  I2C_SDA_1();
}

/*
*********************************************************************************************************
*	函 数 名: i2c_SendByte
*	功能说明: CPU向I2C总线设备发送8bit数据
*	形    参：_ucByte ： 等待发送的字节
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_SendByte(uint8_t _ucByte)
{
  uint8_t i;

  /* 先发送字节的高位bit7 */
  for (i = 0; i < 8; i++)
  {
    if (_ucByte & 0x80)
    {
      I2C_SDA_1();
    }
    else
    {
      I2C_SDA_0();
    }
    i2c_Delay();
    I2C_SCL_1();
    i2c_Delay();
    I2C_SCL_0();
    if (i == 7)
    {
      I2C_SDA_1(); // 释放总线
    }
    _ucByte <<= 1; /* 左移一个bit */
    i2c_Delay();
  }
}

/*
*********************************************************************************************************
*	函 数 名: i2c_ReceiveByte
*	功能说明: CPU从I2C总线设备读取8bit数据
*	形    参：无
*	返 回 值: 读到的数据
*********************************************************************************************************
*/
uint8_t i2c_ReceiveByte(void)
{
  uint8_t i;
  uint8_t value;

  /* 读到第1个bit为数据的bit7 */
  value = 0;
  for (i = 0; i < 8; i++)
  {
    value <<= 1;
    I2C_SCL_1();
    i2c_Delay();
    if (I2C_SDA_READ())
    {
      value++;
    }
    I2C_SCL_0();
    i2c_Delay();
  }
  return value;
}

/*
*********************************************************************************************************
*	函 数 名: i2c_WaitAck
*	功能说明: CPU产生一个时钟，并读取器件的ACK应答信号
*	形    参：无
*	返 回 值: 返回0表示正确应答，1表示无器件响应
*********************************************************************************************************
*/
uint8_t i2c_WaitAck(void)
{
  uint8_t re = 0, timeout = 0;

  I2C_SDA_1(); /* CPU释放SDA总线 */
  i2c_Delay();
  I2C_SCL_1(); /* CPU驱动SCL = 1, 此时器件会返回ACK应答 */
  i2c_Delay();
  while (I2C_SDA_READ())
  {
    timeout++;
    if (timeout > 240)
      re = 1;
  }
  I2C_SCL_0();
  i2c_Delay();
  return re;
}

/*
*********************************************************************************************************
*	函 数 名: i2c_Ack
*	功能说明: CPU产生一个ACK信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_Ack(void)
{
  I2C_SDA_0(); /* CPU驱动SDA = 0 */
  i2c_Delay();
  I2C_SCL_1(); /* CPU产生1个时钟 */
  i2c_Delay();
  I2C_SCL_0();
  i2c_Delay();
  I2C_SDA_1(); /* CPU释放SDA总线 */
}

/*
*********************************************************************************************************
*	函 数 名: i2c_NAck
*	功能说明: CPU产生1个NACK信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_NAck(void)
{
  I2C_SDA_1(); /* CPU驱动SDA = 1 */
  i2c_Delay();
  I2C_SCL_1(); /* CPU产生1个时钟 */
  i2c_Delay();
  I2C_SCL_0();
  i2c_Delay();
}

/*
*********************************************************************************************************
*	函 数 名: i2c_WriteByte
*	功能说明: 
*	形    参：
*	返 回 值: 
*********************************************************************************************************
*/
uint8_t i2c_WriteByte(uint8_t i2cAddr, uint8_t reg, uint8_t *data)
{
  i2c_Start();
  i2c_SendByte(i2cAddr & 0xfe);
  if (i2c_WaitAck() != 0)
  {
    i2c_Stop();
    return 1;
  }
  i2c_SendByte(reg);
  if (i2c_WaitAck() != 0)
  {
    i2c_Stop();
    return 2;
  }
  i2c_SendByte(*data);
  if (i2c_WaitAck() != 0)
  {
    i2c_Stop();
    return 3;
  }
  i2c_Stop();
  return 0;
}

/*
*********************************************************************************************************
*	函 数 名: i2c_ReadByte
*	功能说明: 
*	形    参：
*	返 回 值: 
*********************************************************************************************************
*/
uint8_t i2c_ReadByte(uint8_t i2cAddr, uint8_t reg, uint8_t *data)
{
  i2c_Start();
  i2c_SendByte(i2cAddr & 0xfe);
  if (i2c_WaitAck() != 0)
  {
    i2c_Stop();
    return 1;
  }
  i2c_SendByte(reg);
  if (i2c_WaitAck() != 0)
  {
    i2c_Stop();
    return 2;
  }
  i2c_Start();
  i2c_SendByte(i2cAddr | 0x01);
  if (i2c_WaitAck() != 0)
  {
    i2c_Stop();
    return 3;
  }
  *data = i2c_ReceiveByte();
  i2c_Stop();
  return 0;
}

/**
*	@brief: 软件i2c读N字节
*/
uint8_t i2c_ReadNByte(uint8_t i2cAddr, uint8_t reg, uint8_t *data, uint8_t len)
{
  i2c_Start();
  i2c_SendByte(i2cAddr & 0xfe);
  if (i2c_WaitAck() != 0)
  {
    i2c_Stop();
    return 1;
  }
  i2c_SendByte(reg);
  if (i2c_WaitAck() != 0)
  {
    i2c_Stop();
    return 2;
  }
  i2c_Start();
  i2c_SendByte(i2cAddr | 0x01);
  if (i2c_WaitAck() != 0)
  {
    i2c_Stop();
    return 3;
  }

  for (; len > 0; len--)
  {
    *data = i2c_ReceiveByte();
    if (len == 0)
      i2c_NAck();
    else
      i2c_Ack();
  }
  i2c_Stop();
  return 0;
}
