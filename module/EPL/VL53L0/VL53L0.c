#include "VL53L0.h"
#include "iic/s_iic.h"

VL53L0X_Dev_t Sensor; //设备I2C数据参数

VL53L0X_DeviceInfo_t vl53l0x_dev_info; //设备ID版本信息
uint8_t AjustOK = 0;                   //校准标志位

VL53L0X_RangingMeasurementData_t RangData;

//VL53L0X各精度模式参数
//0：默认;1:高精度;2:长距离;3:高速
mode_data Mode_data[] = {
    {(FixPoint1616_t)(0.25 * 65536),
     (FixPoint1616_t)(18 * 65536),
     33000,
     14,
     10}, //默认

    {(FixPoint1616_t)(0.25 * 65536),
     (FixPoint1616_t)(18 * 65536),
     200000,
     14,
     10}, //高精度

    {(FixPoint1616_t)(0.1 * 65536),
     (FixPoint1616_t)(60 * 65536),
     33000,
     18,
     14}, //长距离

    {(FixPoint1616_t)(0.25 * 65536),
     (FixPoint1616_t)(32 * 65536),
     20000,
     14,
     10}, //高速
};

//IIC写n字节数据
uint8_t VL_IIC_Write_nByte(uint8_t SlaveAddress, uint8_t REG_Address, uint16_t len, uint8_t *buf)
{
  uint8_t ret = 0;
  for (uint16_t i = 0; i < len; i++)
  {
    ret = IIC_WriteByte(SlaveAddress, REG_Address++, buf++);
    if (ret != 0)
      return ret;
  }
  return ret;
}

//IIC读n字节数据
uint8_t VL_IIC_Read_nByte(uint8_t SlaveAddress, uint8_t REG_Address, uint16_t len, uint8_t *buf)
{
  uint8_t ret = 0;
  for (uint16_t i = 0; i < len; i++)
  {
    ret = IIC_ReadByte(SlaveAddress, REG_Address++, buf++);
    if (ret != 0)
      return ret;
  }
  return ret;
}

void VL53L0_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(VL53L0_EN_CLK, ENABLE); //使能GPIOB时钟

  GPIO_InitStructure.GPIO_Pin = VL53L0_EN_GPIO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽输出
  GPIO_Init(VL53L0_EN_GPIO_PORT, &GPIO_InitStructure);

  VL53L0_EN_ON();
}

void VL53L0_ALL_Init()
{
  VL53L0_GPIO_Init();
  vl53l0x_init(&Sensor, VL53L0_Addr);
  SetupSingleShot(&Sensor);
  SysRefInit(&Sensor, HIGH_ACCURACY);
  VL53L0X_StartMeasurement(&Sensor); //启动测量
}

/**************************************************
 函数  ：检测距离
 参数  ：无
 返回值: 距离(u16)
 *************************************************/

u16 VL53L0_Get(void)
{
  // VL53L0X_StartMeasurement(&Sensor);                     //启动测量
  VL53L0X_GetRangingMeasurementData(&Sensor, &RangData); //获取测量距离,并且显示距离
  return RangData.RangeMilliMeter;
}
//API错误信息打印
//Status：详情看VL53L0X_Error参数的定义
void print_pal_error(VL53L0X_Error Status)
{

  char buf[VL53L0X_MAX_STRING_LENGTH];

  VL53L0X_GetPalErrorString(Status, buf); //根据Status状态获取错误信息字符串

  INFO("API Status: %i : %s\r\n", Status, buf); //打印状态和错误信息
}

//VL53L0X 写多个数据
//address:地址
//index:偏移地址
//pdata:数据指针
//count:长度 最大65535
u8 VL53L0X_write_multi(u8 address, u8 index, u8 *pdata, u16 count)
{
  u8 status = STATUS_OK;

  if (VL_IIC_Write_nByte(address, index, count, pdata))
  {
    status = STATUS_FAIL;
  }

  return status;
}

//VL53L0X 读多个数据
//address:地址
//index:偏移地址
//pdata:数据指针
//count:长度 最大65535
u8 VL53L0X_read_multi(u8 address, u8 index, u8 *pdata, u16 count)
{
  u8 status = STATUS_OK;

  if (VL_IIC_Read_nByte(address, index, count, pdata))
  {
    status = STATUS_FAIL;
  }

  return status;
}

//VL53L0X 写1个数据(单字节)
//address:地址
//index:偏移地址
//data:数据(8位)
u8 VL53L0X_write_byte(u8 address, u8 index, u8 data)
{
  u8 status = STATUS_OK;

  status = VL53L0X_write_multi(address, index, &data, 1);

  return status;
}

//VL53L0X 写1个数据(双字节)
//address:地址
//index:偏移地址
//data:数据(16位)
u8 VL53L0X_write_word(u8 address, u8 index, u16 data)
{
  u8 status = STATUS_OK;

  u8 buffer[2];

  //将16位数据拆分成8位
  buffer[0] = (u8)(data >> 8);   //高八位
  buffer[1] = (u8)(data & 0xff); //低八位

  if (index % 2 == 1)
  {
    //串行通信不能处理对非2字节对齐寄存器的字节
    status = VL53L0X_write_multi(address, index, &buffer[0], 1);
    status = VL53L0X_write_multi(address, index, &buffer[0], 1);
  }
  else
  {
    status = VL53L0X_write_multi(address, index, buffer, 2);
  }

  return status;
}

//VL53L0X 写1个数据(四字节)
//address:地址
//index:偏移地址
//data:数据(32位)
u8 VL53L0X_write_dword(u8 address, u8 index, u32 data)
{

  u8 status = STATUS_OK;

  u8 buffer[4];

  //将32位数据拆分成8位
  buffer[0] = (u8)(data >> 24);
  buffer[1] = (u8)((data & 0xff0000) >> 16);
  buffer[2] = (u8)((data & 0xff00) >> 8);
  buffer[3] = (u8)(data & 0xff);

  status = VL53L0X_write_multi(address, index, buffer, 4);

  return status;
}

//VL53L0X 读1个数据(单字节)
//address:地址
//index:偏移地址
//data:数据(8位)
u8 VL53L0X_read_byte(u8 address, u8 index, u8 *pdata)
{
  u8 status = STATUS_OK;

  status = VL53L0X_read_multi(address, index, pdata, 1);

  return status;
}

//VL53L0X 读个数据(双字节)
//address:地址
//index:偏移地址
//data:数据(16位)
u8 VL53L0X_read_word(u8 address, u8 index, u16 *pdata)
{
  u8 status = STATUS_OK;

  u8 buffer[2];

  status = VL53L0X_read_multi(address, index, buffer, 2);

  *pdata = ((u16)buffer[0] << 8) + (u16)buffer[1];

  return status;
}

//VL53L0X 读1个数据(四字节)
//address:地址
//index:偏移地址
//data:数据(32位)
u8 VL53L0X_read_dword(u8 address, u8 index, u32 *pdata)
{
  u8 status = STATUS_OK;

  u8 buffer[4];

  status = VL53L0X_read_multi(address, index, buffer, 4);

  *pdata = ((u32)buffer[0] << 24) + ((u32)buffer[1] << 16) + ((u32)buffer[2] << 8) + ((u32)buffer[3]);

  return status;
}

//配置VL53L0X设备I2C地址
//dev:设备I2C参数结构体
//newaddr:设备新I2C地址
VL53L0X_Error vl53l0x_Addr_set(VL53L0X_Dev_t *dev, uint8_t newaddr)
{
  uint16_t Id;
  uint8_t FinalAddress;
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  u8 sta = 0x00;

  FinalAddress = newaddr;

  if (FinalAddress == dev->I2cDevAddr) //新设备I2C地址与旧地址一致,直接退出
    return VL53L0X_ERROR_NONE;
  //在进行第一个寄存器访问之前设置I2C标准模式(400Khz)
  Status = VL53L0X_WrByte(dev, 0x88, 0x00);
  if (Status != VL53L0X_ERROR_NONE)
  {
    sta = 0x01; //设置I2C标准模式出错
    goto set_error;
  }
  //尝试使用默认的0x52地址读取一个寄存器
  Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
  if (Status != VL53L0X_ERROR_NONE)
  {
    sta = 0x02; //读取寄存器出错
    goto set_error;
  }
  if (Id == 0xEEAA)
  {
    //设置设备新的I2C地址
    Status = VL53L0X_SetDeviceAddress(dev, FinalAddress);
    if (Status != VL53L0X_ERROR_NONE)
    {
      sta = 0x03; //设置I2C地址出错
      goto set_error;
    }
    //修改参数结构体的I2C地址
    dev->I2cDevAddr = FinalAddress;
    //检查新的I2C地址读写是否正常
    Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
    if (Status != VL53L0X_ERROR_NONE)
    {
      sta = 0x04; //新I2C地址读写出错
      goto set_error;
    }
  }
set_error:
  if (Status != VL53L0X_ERROR_NONE)
  {
    print_pal_error(Status); //打印错误信息
  }
  if (sta != 0)
    INFO("sta:0x%x\r\n", sta);
  return Status;
}

VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *dev, u8 select)
{
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  VL53L0X_Dev_t *pMyDevice = dev;

  pMyDevice->I2cDevAddr = 0x52; //I2C地址(上电默认0x52)

  pMyDevice->comms_type = 1;        //I2C通信模式
  pMyDevice->comms_speed_khz = 400; //I2C通信速率

  VL53L0_EN_OFF(); //失能VL53L0X
  delay_ms(30);
  VL53L0_EN_ON(); //使能VL53L0X,让传感器处于工作(I2C地址会恢复默认0X52)
  delay_ms(30);

  Status = vl53l0x_Addr_set(pMyDevice, VL53L0_Addr); //设置VL53L0X传感器I2C地址

  if (Status != VL53L0X_ERROR_NONE)
    goto error;

  Status = VL53L0X_DataInit(pMyDevice); //设备初始化
  if (Status != VL53L0X_ERROR_NONE)
    goto error;

  delay_ms(2);

  Status = VL53L0X_GetDeviceInfo(pMyDevice, &vl53l0x_dev_info); //获取设备ID信息
  if (Status != VL53L0X_ERROR_NONE)
    goto error;

  //AT24CXX_Read(0,(u8*)&Vl53l0x_data,sizeof(_vl53l0x_adjust));//读取24c02保存的校准数据,若已校准 Vl53l0x_data.adjustok==0xAA
  //if(Vl53l0x_data.adjustok==0xAA)//已校准
  //  AjustOK=1;
  //else //没校准
  AjustOK = 0;

  /*设置API跟踪级别*/

error:
  if (Status != VL53L0X_ERROR_NONE)
  {
    print_pal_error(Status); //打印错误信息
    return Status;
  }

  return Status;
}

void SysRefInit(VL53L0X_DEV dev, u8 mode)
{
  u8 status;

  status = VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); //使能连续测量模式
  if (status != VL53L0X_ERROR_NONE)
    goto error;
  delay_ms(2);
  status = VL53L0X_SetInterMeasurementPeriodMilliSeconds(dev, Mode_data[mode].timingBudget); //设置内部周期测量时间
  if (status != VL53L0X_ERROR_NONE)
    goto error;
  delay_ms(2);
  status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1); //使能SIGMA范围检查
  if (status != VL53L0X_ERROR_NONE)
    goto error;
  delay_ms(2);
  status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1); //使能信号速率范围检查
  if (status != VL53L0X_ERROR_NONE)
    goto error;
  delay_ms(2);
  status = VL53L0X_SetLimitCheckValue(dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, Mode_data[mode].sigmaLimit); //设定SIGMA范围
  if (status != VL53L0X_ERROR_NONE)
    goto error;
  delay_ms(2);
  status = VL53L0X_SetLimitCheckValue(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, Mode_data[mode].signalLimit); //设定信号速率范围范围
  if (status != VL53L0X_ERROR_NONE)
    goto error;
  delay_ms(2);
  status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev, Mode_data[mode].timingBudget); //设定完整测距最长时间
  if (status != VL53L0X_ERROR_NONE)
    goto error;
  delay_ms(2);
  status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, Mode_data[mode].preRangeVcselPeriod); //设定VCSEL脉冲周期
  if (status != VL53L0X_ERROR_NONE)
    goto error;
  delay_ms(2);
  status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, Mode_data[mode].finalRangeVcselPeriod); //设定VCSEL脉冲周期范围
  if (status != VL53L0X_ERROR_NONE)
    goto error;
  delay_ms(2);
  status = VL53L0X_StopMeasurement(dev); //停止测量
  if (status != VL53L0X_ERROR_NONE)
    goto error;
  delay_ms(2);

error: //错误信息
  if (status != VL53L0X_ERROR_NONE)
  {
    print_pal_error(status);
    return;
  }
}

void SetupSingleShot(VL53L0X_DEV Dev)
{
  int status;
  uint8_t VhvSettings;
  uint8_t PhaseCal;
  uint32_t refSpadCount;
  uint8_t isApertureSpads;

  status = VL53L0X_StaticInit(Dev);
  if (status)
  {
    INFO("VL53L0X_StaticInit fail");
  }

  status = VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
  if (status)
  {
    INFO("VL53L0X_PerformRefCalibration");
  }
  status = VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);

  if (status)
  {
    INFO("VL53L0X_PerformRefSpadManagement");
  }
  status = VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
  if (status)
  {
    INFO("VL53L0X_SetDeviceMode");
  }

  status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 20 * 1000);
  if (status)
  {
    INFO("VL53L0X_SetMeasurementTimingBudgetMicroSeconds");
  }
}
