#ifndef __DELAY_H
#define __DELAY_H 			   
#include "system/sys/sys.h" 	  

////////////////////////////////////////////////////////////////////////////////// 	 

typedef uint64_t SystemMs_t;

void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);

uint32_t SystemUsGet(void);
SystemMs_t SystemMsGet(void);

#endif





























