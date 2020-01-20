#ifndef _key_h_
#define _key_h_

#include "stm32f4xx.h"

#define KEY_IRQ_ENABLE  1


#define KEY1_RCC        RCC_AHB1Periph_GPIOC
#define KEY1_GPIO       GPIOC
#define KEY1_PIN        GPIO_Pin_11

#define KEY2_RCC        RCC_AHB1Periph_GPIOC
#define KEY2_GPIO       GPIOC
#define KEY2_PIN        GPIO_Pin_12

#define KEY3_RCC        RCC_AHB1Periph_GPIOD
#define KEY3_GPIO       GPIOD
#define KEY3_PIN        GPIO_Pin_0

#define KEY4_RCC        RCC_AHB1Periph_GPIOD
#define KEY4_GPIO       GPIOD
#define KEY4_PIN        GPIO_Pin_1

#define KEY5_RCC        RCC_AHB1Periph_GPIOD
#define KEY5_GPIO       GPIOD
#define KEY5_PIN        GPIO_Pin_2


#define KEY1_READ()     GPIO_ReadInputDataBit(KEY1_GPIO, KEY1_PIN)
#define KEY2_READ()     GPIO_ReadInputDataBit(KEY2_GPIO, KEY2_PIN)
#define KEY3_READ()     GPIO_ReadInputDataBit(KEY3_GPIO, KEY3_PIN)
#define KEY4_READ()     GPIO_ReadInputDataBit(KEY4_GPIO, KEY4_PIN)
#define KEY5_READ()     GPIO_ReadInputDataBit(KEY5_GPIO, KEY5_PIN)

#if (!KEY_IRQ_ENABLE)

void KeyInit(void);
int8_t KeyStatusGet(uint8_t index);
uint8_t KeyStatusAllGet(void);

#else

#include "exti/exti.h"
#include "system/delay/delay.h"
#include "sensor.h"
#include "DXL.h"
void KeyInit(void);
void PowerControl(void);
int8_t KeyStatusGet(uint8_t index);
uint8_t KeyStatusAllGet(void);

#endif // #if (!KEY_IRQ_ENABLE)

#endif

