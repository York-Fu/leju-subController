#ifndef _misrophone_h_
#define _misrophone_h_

#include "stm32f4xx.h"

#define Mike_RCC     RCC_AHB1Periph_GPIOB
#define Mike_GPIO    GPIOB
#define Mike_PIN     GPIO_Pin_15

#define Mike_Enable()     GPIO_SetBits(Mike_GPIO, Mike_PIN)    //Ê¹ÄÜÑïÉùÆ÷
#define Mike_Disable()    GPIO_ResetBits(Mike_GPIO, Mike_PIN)  //Ê§ÄÜÑïÉùÆ÷


void Misrophone_Init(void);
	
#endif
