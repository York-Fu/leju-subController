#include "stm32f4xx.h"
#include "system/delay/delay.h"
#include "DXL.h"
#include "DxlRegister.h"
#include "sensor.h"



int main()
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
  delay_init(168);		//延时初始化
  
  DXL_Init();
  DxlRegisterInit();
  SensorInit();
  
  while(1)
  {
    DXL_Poll();
  }
}

