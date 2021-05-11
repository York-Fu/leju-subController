#ifndef _buzzer_h_
#define _buzzer_h_

#include "tim/tim.h"

//#define Low_Power 11.0f
//#define irq_time 40

typedef struct
{
	float irq;   //�ж�ʱ��
	u16 timelong;    //����ʱ�䣨40������
	u16 timeshort;   //���ʱ��
	uint64_t count;  //ʵ��ʱ��
	uint64_t warring;  //��ѹ���������趨ֵ��ʱ��
}BuzzerTime_t;

typedef struct
{
	u8 now;      //����,0�͵磬1����
	u8 pre;      //�ϴ�
	u8 stab;     //�ȶ�ֵ
	float lowpower;  //�͵�ѹֵ
}BatStatus_t;


void buzzer_init(void);
void key_buzzer(uint8_t flag);
void Buzzer(float power);
void buzzer_ring(void);
void buzzer_ringStop(void);

#endif
