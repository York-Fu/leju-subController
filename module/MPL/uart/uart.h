#ifndef __USART_H
#define __USART_H

#include "stdio.h"	
#include "stm32f4xx.h"

void Uart1_Init(uint32_t bpr);
void Uart1_SendByte(unsigned char x);
void Uart1_SetIrqCall_RXNE(void (*fun)(unsigned char ));
void Uart1_SetIrqCall_IT_TC(void (*fun)(void));

void uart2_init(unsigned int bpr);
void uart2_putc( char x);
void uart2_set_input(int (*fun)( char ) );

void Uart3_Init(uint32_t bpr);
void Uart3_SendByte(unsigned char x);
void Uart3_SetIrqCall_RXNE(void (*fun)(unsigned char ));
void Uart3_SetIrqCall_IT_TC(void (*fun)(void));

#endif


