#ifndef _soft_i2c_h_
#define _soft_i2c_h_

#include "stm32f4xx.h"

#define I2C_SCL_GPIO_CLK       RCC_AHB1Periph_GPIOA
#define I2C_SCL_GPIO_PORT      GPIOA
#define I2C_SCL_PIN            GPIO_Pin_8

#define I2C_SDA_GPIO_CLK       RCC_AHB1Periph_GPIOC
#define I2C_SDA_GPIO_PORT      GPIOC
#define I2C_SDA_PIN            GPIO_Pin_9

#define I2C_SCL_1()  GPIO_SetBits(I2C_SCL_GPIO_PORT, I2C_SCL_PIN)		/* SCL = 1 */
#define I2C_SCL_0()  GPIO_ResetBits(I2C_SCL_GPIO_PORT, I2C_SCL_PIN)		/* SCL = 0 */

#define I2C_SDA_1()  GPIO_SetBits(I2C_SDA_GPIO_PORT, I2C_SDA_PIN)		/* SDA = 1 */
#define I2C_SDA_0()  GPIO_ResetBits(I2C_SDA_GPIO_PORT, I2C_SDA_PIN)		/* SDA = 0 */

#define I2C_SDA_PIN_NUM     9
#define I2C_SDA_IN()        do{I2C_SDA_GPIO_PORT->MODER &= ~(3<<(I2C_SDA_PIN_NUM*2)); I2C_SDA_GPIO_PORT->MODER |= (0<<(I2C_SDA_PIN_NUM*2));}while(0)
#define I2C_SDA_OUT()       do{I2C_SDA_GPIO_PORT->MODER &= ~(3<<(I2C_SDA_PIN_NUM*2)); I2C_SDA_GPIO_PORT->MODER |= (1<<(I2C_SDA_PIN_NUM*2));}while(0)

#define I2C_SDA_READ()  GPIO_ReadInputDataBit(I2C_SDA_GPIO_PORT, I2C_SDA_PIN)	/* 读SDA口线状态 */



void i2c_GPIO_Config(void);
void i2c_Start(void);
void i2c_Stop(void);
void i2c_SendByte(uint8_t _ucByte);
uint8_t i2c_ReceiveByte(void);
uint8_t i2c_WaitAck(void);
void i2c_Ack(void);
void i2c_NAck(void);

uint8_t i2c_ReadByte(uint8_t i2cAddr, uint8_t reg, uint8_t *data);
uint8_t i2c_WriteByte(uint8_t i2cAddr, uint8_t reg, uint8_t *data);

#endif
