#include "misrophone.h"

void misrophone_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(Mike_RCC,ENABLE); 
	
	GPIO_InitStructure.GPIO_Pin =Mike_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;       //ÍÆÍìÊä³ö
	GPIO_Init(Mike_GPIO, &GPIO_InitStructure);
	
	Mike_Enable();
}


