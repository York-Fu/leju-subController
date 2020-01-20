#ifndef __VL53L0_H
#define __VL53L0_H

#include "stm32f4xx.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_cali.h"

#define Xshut_RCC    RCC_AHB1Periph_GPIOB
#define Xshut_GPIO   GPIOB
#define Xshut_PIN    GPIO_Pin_14

//����Xshut��ƽ,�Ӷ�ʹ��VL53L0X���� 1:ʹ�� 0:�ر�
#define Xshut_H()    GPIO_SetBits(Xshut_GPIO, Xshut_PIN)
#define Xshut_L()    GPIO_ResetBits(Xshut_GPIO, Xshut_PIN)

//ʹ��2.8V IO��ƽģʽ
#define USE_I2C_2V8  1

#define STATUS_OK       0x00
#define STATUS_FAIL     0x01

//����ģʽ
#define Default_Mode   0// Ĭ��
#define HIGH_ACCURACY  1//�߾���
#define LONG_RANGE     2//������
#define HIGH_SPEED     3//����

//vl53l0xģʽ���ò�����
typedef __packed struct
{
	FixPoint1616_t signalLimit;    //Signal������ֵ 
	FixPoint1616_t sigmaLimit;     //Sigmal������ֵ
	uint32_t timingBudget;         //����ʱ������
	uint8_t preRangeVcselPeriod ;  //VCSEL��������
	uint8_t finalRangeVcselPeriod ;//VCSEL�������ڷ�Χ
	
}mode_data;



//VL53L0X�������ϵ�Ĭ��IIC��ַΪ0X52(���������λ)
#define VL53L0_Addr 0x54

#define INFO(fmt,...)
enum
{
	TimeOut=0,//��ʱ
	TurnLeft,
	TurnRight,
	RunForward,
	RunRetreat,
};



extern mode_data Mode_data[];
extern uint8_t AjustOK;
extern VL53L0X_Dev_t Sensor;


VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *dev,u8 select);//��ʼ��vl53l0x

void vl53l0x_reset(VL53L0X_Dev_t *dev);//vl53l0x��λ
void SysRefInit(VL53L0X_DEV dev,u8 mode);
void SetupSingleShot(VL53L0X_DEV Dev);
int Vl53L0GestureCheck(void);


void VL53L0_ALL_Init(void);
u16 VL53L0_Get(void);

/*IIC���*/
u8 VL53L0X_write_byte(u8 address,u8 index,u8 data);              //IICдһ��8λ����
u8 VL53L0X_write_word(u8 address,u8 index,u16 data);             //IICдһ��16λ����
u8 VL53L0X_write_dword(u8 address,u8 index,u32 data);            //IICдһ��32λ����
u8 VL53L0X_write_multi(u8 address, u8 index,u8 *pdata,u16 count);//IIC����д

u8 VL53L0X_read_byte(u8 address,u8 index,u8 *pdata);             //IIC��һ��8λ����
u8 VL53L0X_read_word(u8 address,u8 index,u16 *pdata);            //IIC��һ��16λ����
u8 VL53L0X_read_dword(u8 address,u8 index,u32 *pdata);           //IIC��һ��32λ����
u8 VL53L0X_read_multi(u8 address,u8 index,u8 *pdata,u16 count);  //IIC������

#endif


