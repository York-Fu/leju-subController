#include "stm32f4xx.h"
#include "system/delay/delay.h"
#include "led/led.h"
#include "Buzzer/Buzzer.h"
#include "DXL.h"
#include "DxlRegister.h"
#include "sensor.h"
#include "misrophone/misrophone.h"

void prompt_powerOn()
{
  buzzer_ring();
  delay_ms(60);
  buzzer_ringStop();
}

int main()
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����ϵͳ�ж����ȼ�����2
  delay_init(168);                                //��ʱ��ʼ��
  led_init();
  buzzer_init();
  prompt_powerOn();
  misrophone_init();
  dxl_cm730Init();
  dxl_init();
  sensor_init();

  while (1)
  {
    dxl_poll();
  }
}
