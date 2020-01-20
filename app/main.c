#include "stm32f4xx.h"
#include "system/delay/delay.h"
#include "DXL.h"
#include "DxlRegister.h"
#include "sensor.h"



int main()
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
  delay_init(168);		//��ʱ��ʼ��
  
  DXL_Init();
  DxlRegisterInit();
  SensorInit();
  
  while(1)
  {
    DXL_Poll();
  }
}

