#include "Buzzer.h"


void BuzzerInit()
{
	Tim3PwmConfiguration();
}



void BuzzerRing()
{
  TIM3->CCER = 0x0001;
	TIM_SetCompare1(TIM3,1000);
}

void BuzzerRingStop()
{
  TIM3->CCER = 0x0000;
	TIM_SetCompare1(TIM3,0);
}
