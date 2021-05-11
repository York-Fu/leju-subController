#ifndef _buzzer_h_
#define _buzzer_h_

#include "tim/tim.h"

//#define Low_Power 11.0f
//#define irq_time 40

typedef struct
{
	float irq;   //中断时间
	u16 timelong;    //短鸣时间（40倍数）
	u16 timeshort;   //间隔时间
	uint64_t count;  //实际时间
	uint64_t warring;  //电压持续低于设定值的时间
}BuzzerTime_t;

typedef struct
{
	u8 now;      //本次,0低电，1正常
	u8 pre;      //上次
	u8 stab;     //稳定值
	float lowpower;  //低电压值
}BatStatus_t;


void buzzer_init(void);
void key_buzzer(uint8_t flag);
void Buzzer(float power);
void buzzer_ring(void);
void buzzer_ringStop(void);

#endif
