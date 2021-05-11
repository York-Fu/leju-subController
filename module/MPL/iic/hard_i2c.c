#include "hard_i2c.h"

/**
  * @brief  I2Cx I/O配置
  * @param  无
  * @retval 无
  */
static void I2C3_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /*!< I2C3_SCL_GPIO_CLK and I2C3_SDA_GPIO_CLK Periph clock enable */
  RCC_AHB1PeriphClockCmd(I2C3_SCL_GPIO_CLK | I2C3_SDA_GPIO_CLK, ENABLE);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  /*!< GPIO configuration */
  /* Connect PXx to I2C_SCL*/
  GPIO_PinAFConfig(I2C3_SCL_GPIO_PORT, I2C3_SCL_SOURCE, GPIO_AF_I2C3);
  /* Connect PXx to I2C_SDA*/
  GPIO_PinAFConfig(I2C3_SDA_GPIO_PORT, I2C3_SDA_SOURCE, GPIO_AF_I2C3);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

  /*!< Configure I2Cx pins: SCL */
  GPIO_InitStructure.GPIO_Pin = I2C3_SCL_PIN;
  GPIO_Init(I2C3_SCL_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure I2Cx pins: SDA */
  GPIO_InitStructure.GPIO_Pin = I2C3_SDA_PIN;
  GPIO_Init(I2C3_SDA_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  I2C 工作模式配置
  * @param  无
  * @retval 无
  */
void I2C3_Config(void)
{
  I2C3_GPIO_Config();

  I2C_InitTypeDef I2C_InitStructure;
  /* I2C Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C3, ENABLE);
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C3, DISABLE);

  I2C_DeInit(I2C3);
  /* I2C 配置 */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2; /* 高电平数据稳定，低电平数据变化 SCL 时钟线的占空比 */
  I2C_InitStructure.I2C_OwnAddress1 = I2C_OWN_ADDRESS7;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; /* I2C的寻址模式 */
  I2C_InitStructure.I2C_ClockSpeed = I2C3_SPEED;                            /* 通信速率 */
  I2C_Init(I2C3, &I2C_InitStructure);                                       /* I2Cx 初始化 */
  I2C_Cmd(I2C3, ENABLE);                                                    /* 使能 I2Cx */

  //  I2C_AcknowledgeConfig(I2C3, ENABLE);
}

uint8_t I2C_ReadOneByte(I2C_TypeDef *I2Cx, uint8_t slaveAddr, uint8_t regAddr, uint8_t *data)
{
  uint32_t I2CTimeout = I2C_LONG_TIMEOUT;
  while ((--I2CTimeout) && I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
    ;
  if (I2CTimeout == 0)
    return 1;

  I2C_GenerateSTART(I2Cx, ENABLE);
  I2CTimeout = I2C_TIMEOUT;
  while ((--I2CTimeout) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)))
    ;
  if (I2CTimeout == 0)
    return 2;

  I2C_Send7bitAddress(I2Cx, slaveAddr, I2C_Direction_Transmitter);
  I2CTimeout = I2C_TIMEOUT;
  while ((--I2CTimeout) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)))
    ;
  if (I2CTimeout == 0)
  {
    I2C_GenerateSTOP(I2Cx, ENABLE);
    return 3;
  }

  I2C_SendData(I2Cx, regAddr);
  I2CTimeout = I2C_TIMEOUT;
  while ((--I2CTimeout) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)))
    ;
  if (I2CTimeout == 0)
  {
    I2C_GenerateSTOP(I2Cx, ENABLE);
    return 4;
  }

  I2C_GenerateSTART(I2Cx, ENABLE);
  I2CTimeout = I2C_TIMEOUT;
  while ((--I2CTimeout) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)))
    ;
  if (I2CTimeout == 0)
    return 5;

  I2C_Send7bitAddress(I2Cx, slaveAddr, I2C_Direction_Receiver);
  I2CTimeout = I2C_TIMEOUT;
  while ((--I2CTimeout) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)))
    ;
  if (I2CTimeout == 0)
  {
    I2C_GenerateSTOP(I2Cx, ENABLE);
    return 6;
  }

  I2C_AcknowledgeConfig(I2Cx, DISABLE);
  I2C_GenerateSTOP(I2Cx, ENABLE);
  I2CTimeout = I2C_TIMEOUT;
  while ((--I2CTimeout) && (!(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))))
    ; /* EV7 */
  if (I2CTimeout == 0)
    return 7;

  *data = I2C_ReceiveData(I2Cx);
  I2C_AcknowledgeConfig(I2Cx, ENABLE);
  return 0;
}

uint8_t I2C_WriteOneByte(I2C_TypeDef *I2Cx, uint8_t slaveAddr, uint8_t regAddr, uint8_t *data)
{
  uint32_t I2CTimeout = I2C_LONG_TIMEOUT;
  while ((--I2CTimeout) && I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
    ;
  if (I2CTimeout == 0)
    return 1;
  while ((--I2CTimeout) && I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
    ;
  if (I2CTimeout == 0)
    return 2;

  I2C_GenerateSTART(I2Cx, ENABLE);
  I2CTimeout = I2C_TIMEOUT;
  while ((--I2CTimeout) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)))
    ;
  if (I2CTimeout == 0)
    return 3;

  I2C_Send7bitAddress(I2Cx, slaveAddr, I2C_Direction_Transmitter);
  I2CTimeout = I2C_TIMEOUT;
  while ((--I2CTimeout) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)))
    ;
  if (I2CTimeout == 0)
  {
    I2C_GenerateSTOP(I2Cx, ENABLE);
    return 4;
  }

  I2C_SendData(I2Cx, regAddr);
  I2CTimeout = I2C_TIMEOUT;
  while ((--I2CTimeout) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)))
    ;
  if (I2CTimeout == 0)
  {
    I2C_GenerateSTOP(I2Cx, ENABLE);
    return 5;
  }

  I2C_SendData(I2Cx, *data);
  I2CTimeout = I2C_TIMEOUT;
  while ((--I2CTimeout) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)))
    ;
  if (I2CTimeout == 0)
  {
    I2C_GenerateSTOP(I2Cx, ENABLE);
    return 6;
  }

  I2C_GenerateSTOP(I2Cx, ENABLE);
  //I2C_AcknowledgeConfig(I2Cx, DISABLE);
  return 0;
}

uint8_t I2C_ReadNByte(I2C_TypeDef *I2Cx, uint8_t slaveAddr, uint8_t regAddr, uint8_t *data, uint16_t len)
{
  uint16_t sta = 0;
  (void)sta;
  uint32_t I2CTimeout = I2C_LONG_TIMEOUT;
  while ((--I2CTimeout) && I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
    ;
  if (I2CTimeout == 0)
    return 1;

  I2C_GenerateSTART(I2Cx, ENABLE);
  I2CTimeout = I2C_TIMEOUT;
  while ((--I2CTimeout) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)))
    ;
  if (I2CTimeout == 0)
    return 2;

  I2C_Send7bitAddress(I2Cx, slaveAddr, I2C_Direction_Transmitter);
  I2CTimeout = I2C_TIMEOUT;
  while ((--I2CTimeout) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)))
    ;
  if (I2CTimeout == 0)
  {
    I2C_GenerateSTOP(I2Cx, ENABLE);
    return 3;
  }
  sta = I2Cx->SR2; //软件读取SR1寄存器后,对SR2寄存器的读操作将清除ADDR位，不可少！！！！！！！！！

  I2C_SendData(I2Cx, regAddr);
  I2CTimeout = I2C_TIMEOUT;
  while ((--I2CTimeout) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)))
    ;
  if (I2CTimeout == 0)
  {
    I2C_GenerateSTOP(I2Cx, ENABLE);
    return 4;
  }

  I2C_GenerateSTART(I2Cx, ENABLE);
  I2CTimeout = I2C_TIMEOUT;
  while ((--I2CTimeout) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)))
    ;
  if (I2CTimeout == 0)
    return 5;

  I2C_Send7bitAddress(I2Cx, slaveAddr, I2C_Direction_Receiver);
  I2CTimeout = I2C_TIMEOUT;
  while ((--I2CTimeout) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)))
    ;
  if (I2CTimeout == 0)
  {
    I2C_GenerateSTOP(I2Cx, ENABLE);
    return 6;
  }
  sta = I2Cx->SR2; //软件读取SR1寄存器后,对SR2寄存器的读操作将清除ADDR位，不可少！！！！！！！！！
  while (len)
  {
    if (len == 1)
    {
      I2C_AcknowledgeConfig(I2Cx, DISABLE);
    }
    I2CTimeout = I2C_TIMEOUT;
    while ((--I2CTimeout) && (!(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))))
      ; /* EV7 */
    if (I2CTimeout == 0)
      return 7;
    *data++ = I2C_ReceiveData(I2Cx);
    len--;
  }
  I2C_GenerateSTOP(I2Cx, ENABLE);
  I2C_AcknowledgeConfig(I2Cx, ENABLE);
  return 0;
}

uint8_t I2C_WriteNByte(I2C_TypeDef *I2Cx, uint8_t slaveAddr, uint8_t regAddr, uint8_t *pBuffer, uint16_t len)
{
  uint16_t sta = 0;
  (void)sta;
  uint32_t I2CTimeout = I2C_LONG_TIMEOUT;
  while ((--I2CTimeout) && I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
    ;
  if (I2CTimeout == 0)
    return 1;
  while ((--I2CTimeout) && I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
    ;
  if (I2CTimeout == 0)
    return 2;

  I2C_GenerateSTART(I2Cx, ENABLE);
  I2CTimeout = I2C_TIMEOUT;
  while ((--I2CTimeout) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)))
    ;
  if (I2CTimeout == 0)
    return 3;

  I2C_Send7bitAddress(I2Cx, slaveAddr, I2C_Direction_Transmitter);
  I2CTimeout = I2C_TIMEOUT;
  while ((--I2CTimeout) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)))
    ;
  if (I2CTimeout == 0)
  {
    I2C_GenerateSTOP(I2Cx, ENABLE);
    return 4;
  }
  sta = I2Cx->SR2; //软件读取SR1寄存器后,对SR2寄存器的读操作将清除ADDR位，不可少！！！！！！！！！

  I2C_SendData(I2Cx, regAddr);
  I2CTimeout = I2C_TIMEOUT;
  while ((--I2CTimeout) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)))
    ;
  if (I2CTimeout == 0)
  {
    I2C_GenerateSTOP(I2Cx, ENABLE);
    return 5;
  }

  while (len)
  {
    I2C_SendData(I2Cx, *pBuffer++);
    I2CTimeout = I2C_TIMEOUT;
    while ((--I2CTimeout) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)))
      ;
    if (I2CTimeout == 0)
    {
      I2C_GenerateSTOP(I2Cx, ENABLE);
      return 6;
    }
    len--;
  }

  I2C_GenerateSTOP(I2Cx, ENABLE);
  return 0;
}
