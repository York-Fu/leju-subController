#ifndef _hard_i2c_h_
#define _hard_i2c_h_

#include "stm32f4xx.h"

/* STM32 I2C 快速模式 */
#define I2C3_SPEED               400000

/* 这个地址只要与STM32外挂的I2C器件地址不一样即可 */
#define I2C_OWN_ADDRESS7        0X0A

/*I2C GPIO接口*/
#define I2C3_SCL_PIN            GPIO_Pin_8
#define I2C3_SCL_GPIO_PORT      GPIOA
#define I2C3_SCL_GPIO_CLK       RCC_AHB1Periph_GPIOA
#define I2C3_SCL_SOURCE         GPIO_PinSource8

#define I2C3_SDA_PIN            GPIO_Pin_9
#define I2C3_SDA_GPIO_PORT      GPIOC
#define I2C3_SDA_GPIO_CLK       RCC_AHB1Periph_GPIOC
#define I2C3_SDA_SOURCE         GPIO_PinSource9

/*等待超时时间*/
#define I2C_TIMEOUT             ((uint32_t)0x1000)
#define I2C_LONG_TIMEOUT        ((uint32_t)(10 * I2C_TIMEOUT))

void I2C3_Config(void);
uint8_t I2C_ReadOneByte(I2C_TypeDef *I2Cx, uint8_t slaveAddr, uint8_t regAddr, uint8_t *data);
uint8_t I2C_WriteOneByte(I2C_TypeDef *I2Cx, uint8_t slaveAddr, uint8_t regAddr, uint8_t *data);
uint8_t I2C_ReadNByte(I2C_TypeDef *I2Cx, uint8_t slaveAddr, uint8_t regAddr, uint8_t *pBuffer, uint16_t len);
uint8_t I2C_WriteNByte(I2C_TypeDef *I2Cx, uint8_t slaveAddr, uint8_t regAddr, uint8_t *pBuffer, uint16_t len);

#endif
