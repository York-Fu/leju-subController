#include "DxlRegister.h"
#include "DXL.h"
#include "sensor.h"

//note:此处配置了固件中的初始数值
uint8_t ROM_INITIAL_DATA[] = {
  0, 0x73, PROGRAM_VERSION, DEFAULT_ID, DEFAULT_BAUD_RATE,
  0, CW_ANGLE_FIXED_LIMIT & 0xff, CW_ANGLE_FIXED_LIMIT >> 8, CCW_ANGLE_FIXED_LIMIT & 0xff, CCW_ANGLE_FIXED_LIMIT >> 8,
  0, 85 - 5, 60, 190, 255,
  3, 2 /*0Ver8*/, 0x24, 0x24, 0,
  0 & 0xff, 0 >> 8, 0 & 0xff, 0 >> 8
};

uint8_t gbpParameterRange[][2] = {
  //ROM area
  {1, 0},    //MODEL_NUMBER_L      0
  {1, 0},    //MODEL_NUMBER_H      1
  {1, 0},    //VERSION             2
  {0, 253},  //ID                3
  {1, 254},  //BAUD_RATE         4
  {0, 254},  //Return Delay time 5
  {0, 255},  //CW_ANGLE_LIMIT_L   6
  {0, 3},    //CW_ANGLE_LIMIT_H   7
  {0, 255},  //CCW_ANGLE_LIMIT_L   8
  {0, 3},    //CCW_ANGLE_LIMIT_H   9
  {1, 0},    //RESERVED SYSTEM_DATA2  10
  {0, 150},  //LIMIT_TEMPERATURE  11
  {50, 250}, //DOWN_LIMIT_VOLTAGE  12
  {50, 250}, //UP_LIMIT_VOLTAGE   13
  {0, 255},  //MAX_TORQUE_L        14
  {0, 3},    //MAX_TORQUE_H          15
  {0, 2},    //RETURN_LEVEL          16
  {0, 0x7f}, //ALARM_LED          17
  {0, 0x7f}, //ALARM_SHUTDOWN     18
  {0, 255},  //RESERVED OPERATING_MODE      19
  {1, 0},    //DOWN_CALIBRATION_L    20
  {1, 0},    //DOWN_CALIBRATION_H    21
  {1, 0},    //UP_CALIBRATION_L      22
  {1, 0},    //UP_CALIBRATOIN_H      23

  //RAM area
  {0, 1},   //	P_DYNAMIXEL_POWER	24
  {0, 7},   //	P_LED_PANNEL	25
  {0, 255}, //	P_LED_HEAD	26
  {0, 127}, //	-	27
  {0, 255}, //	P_LED_EYE	28
  {0, 127}, //	-	29
  {1, 0},   //	P_BUTTON	30
  {1, 0},   //	-	31
  {1, 0},   //	-	32
  {1, 0},   //	-	33
  {1, 0},   //	-	34
  {1, 0},   //	-	35
  {1, 0},   //	-	36
  {1, 0},   //	-	37
  {1, 0},   //	P_GYRO_Z	38
  {1, 0},   //	-	39
  {1, 0},   //	P_GYRO_Y	40
  {1, 0},   //	-	41
  {1, 0},   //	P_GYRO_X	42
  {1, 0},   //	-	43
  {1, 0},   //	P_ACC_X	44
  {1, 0},   //	-	45
  {1, 0},   //	P_ACC_Y	46
  {1, 0},   //	-	47
  {1, 0},   //	P_ACC_Z	48
  {1, 0},   //	-	49
  {1, 0},   //	P_PRESENT_VOLTAGE	50
  {1, 0},   //	P_LEFT_MIC		51
  {1, 0},   //	-				52
  {1, 0},   //	P_ADC_CH2		  53
  {1, 0},   //	-				54
  {1, 0},   //	P_ADC_CH3			55
  {1, 0},   //	-				56
  {1, 0},   //	P_ADC_CH4			57
  {1, 0},   //	-				58
  {1, 0},   //	P_ADC_CH5			59
  {1, 0},   //	-				60
  {1, 0},   //	P_ADC_CH6			61
  {1, 0},   //	-				62
  {1, 0},   //	P_ADC_CH7			63
  {1, 0},   //	-				64
  {1, 0},   //	P_ADC_CH8			65
  {1, 0},   //	-				66
  {1, 0},   //	P_RIGHT_MIC		67
  {1, 0},   //	-				68
  {1, 0},   //	P_ADC_CH10	  69
  {1, 0},   //	-				70
  {1, 0},   //	P_ADC_CH11		71
  {1, 0},   //	-				72
  {1, 0},   //	P_ADC_CH12		73
  {1, 0},   //	-				74
  {1, 0},   //	P_ADC_CH13		75
  {1, 0},   //	-				76
  {1, 0},   //	P_ADC_CH14		77
  {1, 0},   //	-				78
  {1, 0},   //	P_ADC_CH15		79
  {1, 0},   //	-				80
};

uint8_t gbpDataSize[] = {
  //RAM area
  2, //MODEL_NUMBER_L      0
  0, //MODEL_NUMBER_H      1
  1, //VERSION             2
  1, //ID                  3
  1, //BAUD_RATE           4
  1, //SYSTEM_DATA         5
  2, //CW_ANGLE_LIMIT_L    6
  0, //CW_ANGLE_LIMIT_H    7
  2, //CCW_ANGLE_LIMIT_L   8
  0, //CCW_ANGLE_LIMIT_H   9
  1, //ACCEL               10
  1, //LIMIT_TEMPERATURE   11
  1, //UP_LIMIT_VOLTAGE    12
  1, //DOWN_LIMIT_VOLTAGE  13
  2, //MAX_TORQUE_L        14
  0, //MAX_TORQUE_H        15
  1, //RETURN_LEVEL        16
  1, //ALARM_LED           17
  1, //ALARM_SHUTDOWN      18
  1, //OPERATING_MODE      19
  2, //DOWN_CALIBRATION_L  20
  0, //DOWN_CALIBRATION_H  21
  2, //UP_CALIBRATION_L    22
  0, //UP_CALIBRATOIN_H    23

  //RAM area
  1, //	P_DYNAMIXEL_POWER	24
  1, //	P_LED_PANNEL	25
  2, //	P_LED_HEAD	26
  0, //	-	27
  2, //	P_LED_EYE	28
  0, //	-	29
  1, //	P_BUTTON	30
  1, //	-	31
  1, //	-	32
  1, //	-	33
  1, //	-	34
  1, //	-	35
  1, //	-	36
  1, //	-	37
  2, //	P_GYRO_Z	38
  0, //	-	39
  2, //	P_GYRO_Y	40
  0, //	-	41
  2, //	P_GYRO_X	42
  0, //	-	43
  2, //	P_ACC_X	44
  0, //	-	45
  2, //	P_ACC_Y	46
  0, //	-	47
  2, //	P_ACC_Z	48
  0, //	-	49
  1, //	P_PRESENT_VOLTAGE	50
  2, //	P_LEFT_MIC		51
  0, //	-				52
  2, //	P_ADC_CH2		  53
  0, //	-				54
  2, //	P_ADC_CH3			55
  0, //	-				56
  2, //	P_ADC_CH4			57
  0, //	-				58
  2, //	P_ADC_CH5			59
  0, //	-				60
  2, //	P_ADC_CH6			61
  0, //	-				62
  2, //	P_ADC_CH7			63
  0, //	-				64
  2, //	P_ADC_CH8			65
  0, //	-				66
  2, // P_RIGHT_MIC		67
  0, //	-				68
  2, //	P_ADC_CH10		69
  0, //	-				70
  2, //	P_ADC_CH11	  71
  0, //	-				72
  2, //	P_ADC_CH12		73
  0, //	-				74
  2, //	P_ADC_CH13		75
  0, //	-				76
  2, //	P_ADC_CH14		77
  0, //	-				78
  2, //	P_ADC_CH15		79
  0, //	-				80
};

volatile uint8_t gbpControlTable[CONTROL_TABLE_LEN + 1];

void dxl_cm730Init()
{
  for (uint8_t i = 0; i < ROM_CONTROL_TABLE_LEN; i++)
  {
    gbpControlTable[i] = ROM_INITIAL_DATA[i];
  }
}

void DR_setBaudRate()
{
  dxl_portInit(DATA_BAUD_RATE);
}

void DR_tableProcess(uint8_t addr, uint8_t len)
{
  uint8_t offset;
  for (offset = addr; offset < (addr + len);)
  {
    switch (offset)
    {
    case P_ID:
      break;
    case P_BAUD_RATE:
      DR_setBaudRate();
      break;
    case P_RETURN_DELAY_TIME:
      break;
    case P_RETURN_LEVEL:
      break;
    case P_DYNAMIXEL_POWER:
      dxl_setPower(DATA_DYNAMIXEL_POWER);
      break;
    default:
      sensor_set();
    }
    offset += gbpDataSize[offset];
  }
}

uint8_t DR_read(uint8_t *data, uint8_t addr, uint8_t length)
{
  for (uint8_t i = 0; i < length; i++)
  {
    data[i] = gbpControlTable[i + addr];
  }
  return 0;
}

uint8_t DR_writeParamCheck(uint8_t addr, uint8_t *param, uint8_t paramLen)
{
  if (paramLen > gbpDataSize[addr])
    return 1;
  for (uint8_t i = 0; i < paramLen; i++)
  {
    if ((param[i] < gbpParameterRange[i + addr][0]) ||
        (param[i] > gbpParameterRange[i + addr][1]))
    {
      return 2;
    }
  }
  return 0;
}

uint8_t DR_write(uint8_t addr, uint8_t *param, uint8_t paramLen)
{
  if (DR_writeParamCheck(addr, param, paramLen) != 0)
  {
    return 1;
  }

  for (uint8_t i = 0; i < paramLen; i++)
  {
    gbpControlTable[i + addr] = param[i];
  }
  DR_tableProcess(addr, paramLen);

  if (DATA_RETURN_LEVEL < RETURN_ALL_PACKET)
  {
    return 2;
  }
  return 0;
}
