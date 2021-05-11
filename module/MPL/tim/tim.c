#include "tim/tim.h"

//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz 84Mhz
void Tim2IntInit(uint16_t arr, uint16_t psc)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); ///ʹ��TIMʱ��

  TIM_TimeBaseInitStructure.TIM_Period = arr;                     //�Զ���װ��ֵ
  TIM_TimeBaseInitStructure.TIM_Prescaler = psc;                  //��ʱ����Ƶ
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure); //��ʼ��TIM

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;              //��ʱ���ж�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02; //��ռ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;        //�����ȼ�
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //����ʱ�������ж�
  TIM_Cmd(TIM2, ENABLE);                     //ʹ�ܶ�ʱ��
}

//TIM3_CH1 pwm
void Tim3PwmConfiguration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  TIM_TimeBaseStructure.TIM_Period = 1000 - 1;
  TIM_TimeBaseStructure.TIM_Prescaler = 42 - 1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_Cmd(TIM3, ENABLE);

  TIM3->CCER = 0x0000;
  TIM_SetCompare1(TIM3, 0);
}

static void (*Tim2Irq)(void) = NULL;

void Tim2Irq_Set(void (*func)(void))
{
  Tim2Irq = func;
}

//��ʱ���жϷ�����
void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) //����ж�
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update); //����жϱ�־λ

    if (Tim2Irq != NULL)
      Tim2Irq();
  }
}
