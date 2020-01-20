# Talos下位机主板程序
## 概述
下位机主板程序，主要功能为
- 控制机器人电源
- 实现上位机与机器人之间的[Protocol 1.0][Protocol 1.0]协议
- 实现了一个Dynamixel设备，默认ID为200，型号参考CM_730
- 实现板载传感器驱动  

下述[Protocol 1.0][Protocol 1.0]协议使用`DXL协议`代替。
#### 电源控制
下位机主板使用12V直流电源供电，板载五个DXL协议接口，可外挂至少253个设备，接口功能包括通信和供电。接口供电功能的开关，可通过DXL协议控制，相关代码如下
```
#define	P_DYNAMIXEL_POWER               24
#define DATA_DYNAMIXEL_POWER            gbpControlTable[P_DYNAMIXEL_POWER]
DXL_SetPower(DATA_DYNAMIXEL_POWER);
```
#### 实现Protocol 1.0协议
下位机主板有一个重要功能，就是实现DXL协议上的单总线半双工和上位机串口双线全双工之间的数据通信。主板接收到上位机数据后会直接转发到DXL协议总线上，如果数据包ID和自己有关，则会执行相关处理。收到DXL协议总线上的数据后也会直接转发给上位机，如果数据包ID和自己有关，则会执行相关处理。
#### 实现Dynamixel设备
下位机主板与舵机、外挂传感器在DXL协议上是平等的，都属于总线上的一个设备，默认ID为200，型号参考CM_730。  
#### 实现板载传感器驱动
下位机主板上板载了一些传感器，程序实现了这些传感器的驱动。
## 板载传感器定义
下位机会周期性的获取板载传感器的数据，存放到Dynamixel设备使用的寄存器数组中；传感器的控制和数据获取，都是通过读写寄存器数组实现的，寄存器数组相定义如下：
```
#define ROM_CONTROL_TABLE_LEN   24
#define RAM_CONTROL_TABLE_LEN   56
#define CONTROL_TABLE_LEN       (ROM_CONTROL_TABLE_LEN+RAM_CONTROL_TABLE_LEN)

volatile uint8_t gbpControlTable[CONTROL_TABLE_LEN+1];
```
获取传感器数据由sensor.c文件中的`SensorGet()`函数实现，控制板载执行器由sensor.c文件中的`SensorSet()`函数实现。
#### 姿态传感器
姿态传感器，包含三轴角速度和三轴加速度，总共六轴uint16_t类型的原始数据。相关代码如下：
```
#define	P_GYRO_X                38
//#define   -   39
#define	P_GYRO_Y                40
//#define   -   41
#define	P_GYRO_Z                42
//#define   -   43
#define	P_ACC_X                 44
//#define   -   45
#define	P_ACC_Y                 46
//#define   -   47
#define	P_ACC_Z                 48
//#define   -   49
```
```
#define DATA_GYRO_X                   HALFWORD_CAST(gbpControlTable[P_GYRO_X])
#define DATA_GYRO_Y                   HALFWORD_CAST(gbpControlTable[P_GYRO_Y])
#define DATA_GYRO_Z                   HALFWORD_CAST(gbpControlTable[P_GYRO_Z])
#define DATA_ACCEL_X                  HALFWORD_CAST(gbpControlTable[P_ACC_X])
#define DATA_ACCEL_Y                  HALFWORD_CAST(gbpControlTable[P_ACC_Y])
#define DATA_ACCEL_Z                  HALFWORD_CAST(gbpControlTable[P_ACC_Z])
```
```
void SensorGetImu()
{
  ImuData_t imuData = {0};
  
  IMU_GetData(&imuData);
  
  DATA_GYRO_X = imuData.gray.x;
  DATA_GYRO_Y = imuData.gray.y;
  DATA_GYRO_Z = imuData.gray.z;
  DATA_ACCEL_X = imuData.accel.x;
  DATA_ACCEL_Y = imuData.accel.y;
  DATA_ACCEL_Z = imuData.accel.z;
  DATA_MAGNETIC_X = imuData.magnetic.x;
  DATA_MAGNETIC_Y = imuData.magnetic.y;
  DATA_MAGNETIC_Z = imuData.magnetic.z;
}
```
#### 地磁传感器
地磁传感器，包含三轴uint16_t类型的原始数据，表示磁场强度在三个方向上的分量。相关代码如下：
```
#define	P_LEFT_MIC              51
//#define   -   52
#define	P_ADC_CH2               53
//#define   -   54
#define	P_ADC_CH3               55
//#define   -   56
```
```
#define DATA_MAGNETIC_X               HALFWORD_CAST(gbpControlTable[P_LEFT_MIC])
#define DATA_MAGNETIC_Y               HALFWORD_CAST(gbpControlTable[P_ADC_CH2])
#define DATA_MAGNETIC_Z               HALFWORD_CAST(gbpControlTable[P_ADC_CH3])
```
#### RGB灯
rgb灯由数据寄存器的两个8位组成的16位数控制，由低到高每3位控制一个灯，总共使用15位控制5个灯。对于单个灯，3位中的第1位为0时红灯灭，3位中的第1位为1时红灯亮；3位中的第2位为0时绿灯灭，3位中的第2位为1时绿灯亮；3位中的第3位为0时蓝灯灭，3位中的第3位为1时蓝灯亮（001：红灯亮,111:全亮）。相关代码如下：
```
#define	P_LED_HEAD                      26
//#define   -   27
```
```
#define DATA_RGB_STATUS                 HALFWORD_CAST(gbpControlTable[P_LED_HEAD])
```
#### 按键
下位机板载5个用户按键，程序会周期性更新按键状态到数组寄存器，相关代码如下：
```
#define	P_BUTTON                        30
//#define   -   31
//#define   -   32
//#define   -   33
//#define   -   34
```
```
#define DATA_KEY1_STATUS                gbpControlTable[P_BUTTON]
#define DATA_KEY2_STATUS                gbpControlTable[P_BUTTON+1]
#define DATA_KEY3_STATUS                gbpControlTable[P_BUTTON+2]
#define DATA_KEY4_STATUS                gbpControlTable[P_BUTTON+3]
#define DATA_KEY5_STATUS                gbpControlTable[P_BUTTON+4]
```
```
void SensorGetKey()
{
  uint8_t keyStatus = 0;
  
  keyStatus = KeyStatusAllGet();
  
  DATA_KEY1_STATUS = (keyStatus>>0)&(0x01);
  DATA_KEY2_STATUS = (keyStatus>>1)&(0x01);
  DATA_KEY3_STATUS = (keyStatus>>2)&(0x01);
  DATA_KEY4_STATUS = (keyStatus>>3)&(0x01);
  DATA_KEY5_STATUS = (keyStatus>>4)&(0x01);
}
```
## 附录
[Protocol 1.0]: http://emanual.robotis.com/docs/en/dxl/protocol1/