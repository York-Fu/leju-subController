#ifndef _tim_h_
#define _tim_h_

#include "stdio.h"
#include "stm32f4xx.h"

void Tim2IntInit(uint16_t arr,uint16_t psc);
void Tim2Irq_Set(void (*func)(void));
void Tim3PwmConfiguration(void);

#endif

