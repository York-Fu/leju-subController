/*
 * @file    adc.h
 * @author  fuyou
 * @brief   ADC config and get
 */

#ifndef __ADC_H
#define __ADC_H

#include "stm32f4xx.h"

#define ADCx_CHANNEL_GPIO_CLK   RCC_AHB1Periph_GPIOC
#define ADCx_GPIO_PORT          GPIOC
#define ADCx_GPIO_PIN           GPIO_Pin_1

#define ADCx_CLK                RCC_APB2Periph_ADC1
#define ADCx                    ADC1
#define ADC_CHANNEL             ADC_Channel_11
#define ADCx_DR_ADDRESS         ((uint32_t)&ADC1->DR)

#define DMA_CHANNELx            DMA_Channel_0
#define DMA_STREAMx             DMA2_Stream0

void ADC_Config(void);
uint16_t ADC_GetValue(void);

#endif
