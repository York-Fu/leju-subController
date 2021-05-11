#include "Buzzer/Buzzer.h"
#include "system/delay/delay.h"

/*
   蜂鸣器优先级：低电响三声>按键>有电静音
*/

static uint8_t KeyNum = 0; //按键状态，0空闲，1掉电，2上电
static BuzzerTime_t BTime = {
    .irq = 40,
    .timeshort = 40,
    .timelong = 1000,
    .count = 0,
    .warring = 200,
};

static BatStatus_t BStatus = {
    .pre = 1,
    .now = 1,
    .stab = 1,
    .lowpower = 10.4,
};

void buzzer_init()
{
  Tim3PwmConfiguration();
}

void buzzer_ring()
{
  TIM3->CCER = 0x0001;
  TIM_SetCompare1(TIM3, 250);
}

void buzzer_ringStop()
{
  TIM3->CCER = 0x0000;
  TIM_SetCompare1(TIM3, 0);
}

//用于按键触发时
void key_buzzer(uint8_t flag)
{
  if (BStatus.stab == 0)
    return; //低电不响应按键声音
  else
  {
    buzzer_ringStop();
    KeyNum = flag;
    BTime.count = 0;
  }
}

void Buzzer(float power)
{
  static u32 WarringCount = 0;
  BStatus.now = power < BStatus.lowpower ? 0 : 1;

  if (BStatus.pre != BStatus.now) //电池状态发生改变
  {
    if (BStatus.now == 0) //有电变没电
    {
      BTime.count = 0;
      WarringCount = 0;
    }

    else if (BStatus.now == 1) //没电变有电
    {
      buzzer_ringStop();
      WarringCount = 0;
      BStatus.stab = 1;
    }
  }
  BStatus.pre = BStatus.now;
  if (BStatus.now == 0)
    WarringCount += BTime.irq;
  if (WarringCount == BTime.warring) //stab更改为低电
  {
    BTime.count = 0;
    BStatus.stab = 0;
  }
  BTime.count += BTime.irq;

  if (BStatus.stab == 0)
  {
    if (BTime.count == BTime.irq)
      buzzer_ring();
    else if (BTime.count == BTime.timeshort + BTime.irq)
      buzzer_ringStop();
    else if (BTime.count == BTime.timeshort * 2 + BTime.irq)
      buzzer_ring();
    else if (BTime.count == BTime.timeshort * 3 + BTime.irq)
      buzzer_ringStop();
    else if (BTime.count == BTime.timeshort * 4 + BTime.irq)
      buzzer_ring();
    else if (BTime.count == BTime.timeshort * 5 + BTime.irq)
      buzzer_ringStop();
    else if (BTime.count == BTime.timeshort * 5 + BTime.irq + BTime.timelong)
    {
      buzzer_ringStop();
      BTime.count = 0;
    }
  }

  else if (KeyNum == 1) //掉电
  {
    if (BTime.count == BTime.irq)
      buzzer_ring();
    else if (BTime.count == BTime.timeshort + BTime.irq + 160)
    {
      buzzer_ringStop();
      KeyNum = 0;
    }
  }

  else if (KeyNum == 2) //上电
  {
    if (BTime.count == BTime.irq)
      buzzer_ring();
    else if (BTime.count == BTime.timeshort + BTime.irq)
      buzzer_ringStop();
    else if (BTime.count == BTime.timeshort * 2 + BTime.irq)
      buzzer_ring();
    else if (BTime.count == BTime.timeshort * 3 + BTime.irq)
      buzzer_ringStop();
    else if (BTime.count == BTime.timeshort * 3 + BTime.irq + BTime.timelong)
    {
      buzzer_ringStop();
      KeyNum = 0;
    }
  }
}
