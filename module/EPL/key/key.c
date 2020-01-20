#include "key/key.h"


#if (!KEY_IRQ_ENABLE)

void KeyInit()
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  // KEY1
  RCC_AHB1PeriphClockCmd(KEY1_RCC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = KEY1_PIN;
  GPIO_Init(KEY1_GPIO, &GPIO_InitStructure);
  // KEY2
  RCC_AHB1PeriphClockCmd(KEY2_RCC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = KEY2_PIN;
  GPIO_Init(KEY2_GPIO, &GPIO_InitStructure);
  // KEY3
  RCC_AHB1PeriphClockCmd(KEY3_RCC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = KEY3_PIN;
  GPIO_Init(KEY3_GPIO, &GPIO_InitStructure);
  // KEY4
  RCC_AHB1PeriphClockCmd(KEY4_RCC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = KEY4_PIN;
  GPIO_Init(KEY4_GPIO, &GPIO_InitStructure);
  // KEY5
  RCC_AHB1PeriphClockCmd(KEY5_RCC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = KEY5_PIN;
  GPIO_Init(KEY5_GPIO, &GPIO_InitStructure);
}


int8_t KeyStatusGet(uint8_t index)
{
  switch(index)
  {
    case 1:
      return KEY1_READ();
    case 2:
      return KEY2_READ();
    case 3:
      return KEY3_READ();
    case 4:
      return KEY4_READ();
    case 5:
      return KEY5_READ();
  }
  return -1;
}


uint8_t KeyStatusAllGet()
{
  uint8_t KeyStatus = 0;
  
  if(KEY1_READ())
    KeyStatus &= ~(1<<0);
  else
    KeyStatus |= (1<<0);
  
  if(KEY2_READ())
    KeyStatus &= ~(1<<1);
  else
    KeyStatus |= (1<<1);
  
  if(KEY3_READ())
    KeyStatus &= ~(1<<2);
  else
    KeyStatus |= (1<<2);
  
  if(KEY4_READ())
    KeyStatus &= ~(1<<3);
  else
    KeyStatus |= (1<<3);
  
  if(KEY5_READ())
    KeyStatus &= ~(1<<4);
  else
    KeyStatus |= (1<<4);
  
  return KeyStatus;
}

#else // #if (!KEY_IRQ_ENABLE)

static uint8_t KeyStatus = 0;
int8_t KeyStatusGet(uint8_t index)
{
  switch(index)
  {
    case 1:
      return KeyStatus&(1<<0);
    case 2:
      return KeyStatus&(1<<1);
    case 3:
      return KeyStatus&(1<<2);
    case 4:
      return KeyStatus&(1<<3);
    case 5:
      return KeyStatus&(1<<4);
  }
  return -1;
}


uint8_t KeyStatusAllGet()
{
  return KeyStatus;
}

void Key1Irq(void)
{
  if(KEY1_READ())
  {
    KeyStatus &= ~(1<<0);
  }
  else
  {
    KeyStatus |= (1<<0);
  }
}

void Key2Irq(void)
{
  if(KEY2_READ())
  {
    KeyStatus &= ~(1<<1);
  }
  else
  {
    KeyStatus |= (1<<1);
  }
}

void Key3Irq(void)
{
  if(KEY3_READ())
  {
    KeyStatus &= ~(1<<2);
  }
  else
  {
    KeyStatus |= (1<<2);
  }
}

void Key4Irq(void)
{
  if(KEY4_READ())
  {
    KeyStatus &= ~(1<<3);
  }
  else
  {
    KeyStatus |= (1<<3);
  }
}

void Key5Irq(void)
{
  if(KEY5_READ())
  {
    KeyStatus &= ~(1<<4);
  }
  else
  {
    KeyStatus |= (1<<4);
  }
}


void KeyInit()
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  // KEY1
  RCC_AHB1PeriphClockCmd(KEY1_RCC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = KEY1_PIN;
  GPIO_Init(KEY1_GPIO, &GPIO_InitStructure);
  
  Exti15_10_Init(EXTI_PortSourceGPIOC, EXTI_PinSource11); // 注意优先级和触发沿设置
  Exti15_10_IrqCall_Set(11, Key1Irq);
  // KEY2
  RCC_AHB1PeriphClockCmd(KEY2_RCC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = KEY2_PIN;
  GPIO_Init(KEY2_GPIO, &GPIO_InitStructure);
  
  Exti15_10_Init(EXTI_PortSourceGPIOC, EXTI_PinSource12);
  Exti15_10_IrqCall_Set(12, Key2Irq);
  // KEY3
  RCC_AHB1PeriphClockCmd(KEY3_RCC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = KEY3_PIN;
  GPIO_Init(KEY3_GPIO, &GPIO_InitStructure);
  
  Exti0_Init(EXTI_PortSourceGPIOD);
  Exti0IrqCall_Set(Key3Irq);
  // KEY4
  RCC_AHB1PeriphClockCmd(KEY4_RCC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = KEY4_PIN;
  GPIO_Init(KEY4_GPIO, &GPIO_InitStructure);
  
  Exti1_Init(EXTI_PortSourceGPIOD);
  Exti1IrqCall_Set(Key4Irq);
  // KEY5
  RCC_AHB1PeriphClockCmd(KEY5_RCC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = KEY5_PIN;
  GPIO_Init(KEY5_GPIO, &GPIO_InitStructure);
  
  Exti2_Init(EXTI_PortSourceGPIOD);
  Exti2IrqCall_Set(Key5Irq);
}

#endif // #if (!KEY_IRQ_ENABLE)

