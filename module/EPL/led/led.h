#ifndef _led_h_
#define _led_h_

#include "stm32f4xx.h"


#define LED1_RCC    RCC_AHB1Periph_GPIOD
#define LED1_GPIO   GPIOD
#define LED1_PIN    GPIO_Pin_3

#define LED2_RCC    RCC_AHB1Periph_GPIOD
#define LED2_GPIO   GPIOD
#define LED2_PIN    GPIO_Pin_4

#define LED3_RCC    RCC_AHB1Periph_GPIOD
#define LED3_GPIO   GPIOD
#define LED3_PIN    GPIO_Pin_5

#define LED4_RCC    RCC_AHB1Periph_GPIOD
#define LED4_GPIO   GPIOD
#define LED4_PIN    GPIO_Pin_6

#define LED5_RCC    RCC_AHB1Periph_GPIOD
#define LED5_GPIO   GPIOD
#define LED5_PIN    GPIO_Pin_7


#define LDE1_H()    GPIO_SetBits(LED1_GPIO, LED1_PIN)
#define LDE1_L()    GPIO_ResetBits(LED1_GPIO, LED1_PIN)

#define LDE2_H()    GPIO_SetBits(LED2_GPIO, LED2_PIN)
#define LDE2_L()    GPIO_ResetBits(LED2_GPIO, LED2_PIN)

#define LDE3_H()    GPIO_SetBits(LED3_GPIO, LED3_PIN)
#define LDE3_L()    GPIO_ResetBits(LED3_GPIO, LED3_PIN)

#define LDE4_H()    GPIO_SetBits(LED4_GPIO, LED4_PIN)
#define LDE4_L()    GPIO_ResetBits(LED4_GPIO, LED4_PIN)

#define LDE5_H()    GPIO_SetBits(LED5_GPIO, LED5_PIN)
#define LDE5_L()    GPIO_ResetBits(LED5_GPIO, LED5_PIN)


void LedInit(void);
void LedOn(uint8_t index);
void LedOff(uint8_t index);

#endif

