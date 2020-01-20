#ifndef _DxlRegister_h_
#define _DxlRegister_h_

#include "stm32f4xx.h"


#define DEFAULT_BAUD_RATE 1    //1Mbps at 16MHz
#define PROGRAM_VERSION 0x13     // for ax12, freq. selectable
#define CW_ANGLE_FIXED_LIMIT 0 // 0+30 dudung031002
#define CCW_ANGLE_FIXED_LIMIT (1023) // 300-30 dudung031002

#define RETURN_NO_PACKET      0
#define RETURN_READ_PACKET    1
#define RETURN_ALL_PACKET     2

#define ROM_CONTROL_TABLE_LEN   24
#define RAM_CONTROL_TABLE_LEN   56
#define CONTROL_TABLE_LEN       (ROM_CONTROL_TABLE_LEN+RAM_CONTROL_TABLE_LEN)

#define HALFWORD_CAST(val)		(*(uint16_t *)(&(val)))

#define MX_28         0
#define CM_730        1

#define TABLE_TYPE    CM_730

/* Exported constants --------------------------------------------------------*/
//EEPROM AREA
#define P_MODEL_NUMBER_L      0
#define P_MODOEL_NUMBER_H     1
#define P_VERSION             2
#define P_ID                  3
#define P_BAUD_RATE           4
#define P_RETURN_DELAY_TIME   5
#define P_CW_ANGLE_LIMIT_L    6
#define P_CW_ANGLE_LIMIT_H    7
#define P_CCW_ANGLE_LIMIT_L   8
#define P_CCW_ANGLE_LIMIT_H   9
#define P_SYSTEM_DATA2        10
#define P_LIMIT_TEMPERATURE   11
#define P_DOWN_LIMIT_VOLTAGE  12
#define P_UP_LIMIT_VOLTAGE    13
#define P_MAX_TORQUE_L        14
#define P_MAX_TORQUE_H        15
#define P_RETURN_LEVEL        16
#define P_ALARM_LED           17
#define P_ALARM_SHUTDOWN      18
#define P_OPERATING_MODE      19
#define P_DOWN_CALIBRATION_L  20
#define P_DOWN_CALIBRATION_H  21
#define P_UP_CALIBRATION_L    22
#define P_UP_CALIBRATION_H    23

//RAM AREA
#if (TABLE_TYPE == MX_28)

#define P_TORQUE_ENABLE         (24)
#define P_LED                   (25)
#define P_CW_COMPLIANCE_MARGIN  (26)
#define P_CCW_COMPLIANCE_MARGIN (27)
#define P_CW_COMPLIANCE_SLOPE   (28)
#define P_CCW_COMPLIANCE_SLOPE  (29)
#define P_GOAL_POSITION_L       (30)
#define P_GOAL_POSITION_H       (31)
#define P_GOAL_SPEED_L          (32)
#define P_GOAL_SPEED_H          (33)
#define P_TORQUE_LIMIT_L        (34)
#define P_TORQUE_LIMIT_H        (35)
#define P_PRESENT_POSITION_L    (36)
#define P_PRESENT_POSITION_H    (37)
#define P_PRESENT_SPEED_L       (38)
#define P_PRESENT_SPEED_H       (39)
#define P_PRESENT_LOAD_L        (40)
#define P_PRESENT_LOAD_H        (41)
#define P_PRESENT_VOLTAGE       (42)
#define P_PRESENT_TEMPERATURE   (43)
#define P_REGISTERED_INSTRUCTION (44)
#define P_PAUSE_TIME            (45)
#define P_MOVING (46)
#define P_LOCK                  (47)
#define P_PUNCH_L               (48)
#define P_PUNCH_H               (49)
#define P_RESERVED4             (50)
#define P_RESERVED5             (51)
#define P_POT_L                 (52)
#define P_POT_H                 (53)
#define P_PWM_OUT_L             (54)
#define P_PWM_OUT_H             (55)

#elif (TABLE_TYPE == CM_730)

#define	P_DYNAMIXEL_POWER       24
#define	P_LED_PANNEL            25
#define	P_LED_HEAD              26
//#define	-	27
#define	P_LED_EYE               28
//#define	-	29
#define	P_BUTTON                30
//#define     -		31
//#define     -		32
//#define     -		33
//#define     -		34
//#define     -		35
//#define     -		36
//#define     -		37
#define	P_GYRO_X                38
//#define	-	39
#define	P_GYRO_Y                40
//#define	-	41
#define	P_GYRO_Z                42
//#define	-	43
#define	P_ACC_X                 44
//#define	-	45
#define	P_ACC_Y                 46
//#define	-	47
#define	P_ACC_Z                 48
//#define	-	49
#define	P_PRESENT_VOLTAGE       50
#define	P_LEFT_MIC              51
//#define	-			52
#define	P_ADC_CH2               53
//#define	-			54
#define	P_ADC_CH3               55
//#define	-			56
#define	P_ADC_CH4               57
//#define	-			58
#define	P_ADC_CH5               59
//#define	-			60
#define	P_ADC_CH6               61
//#define	-			62
#define	P_ADC_CH7               63
//#define	-			64
#define	P_ADC_CH8               65
//#define	-			66
#define	P_RIGHT_MIC             67
//#define	-			68
#define	P_ADC_CH10              69
//#define	-			70
#define	P_ADC_CH11              71
//#define	-			72
#define	P_ADC_CH12              73
//#define	-			74
#define	P_ADC_CH13              75
//#define	-			76
#define	P_ADC_CH14              77
//#define	-			78
#define	P_ADC_CH15              79
//#define	-			80

#endif


/* 映射数据定义 --------------------------------------------------------*/
#define DATA_BAUD_RATE                gbpControlTable[P_BAUD_RATE]
#define DATA_RETURN_LEVEL             gbpControlTable[P_RETURN_LEVEL]
#define DATA_DYNAMIXEL_POWER          gbpControlTable[P_DYNAMIXEL_POWER]



typedef enum
{
  RETURN_PACKET_PING = 0,
  RETURN_PACKET_READ,
  RETURN_PACKET_OTHER,
}ReturnPackType_t;


extern volatile uint8_t gbpControlTable[CONTROL_TABLE_LEN+1];


void DxlRegisterInit(void);

uint8_t DxlRegisterRead(uint8_t *data, uint8_t addr, uint8_t length);
uint8_t DxlRegisterWrite(uint8_t addr, uint8_t *param, uint8_t paramLen);
void DR_TableProcess(uint8_t addr, uint8_t len);

#endif
