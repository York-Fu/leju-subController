#include "VL53L0.h"
#include "iic/s_iic.h"

VL53L0X_Dev_t Sensor; //�豸I2C���ݲ���

VL53L0X_DeviceInfo_t vl53l0x_dev_info; //�豸ID�汾��Ϣ
uint8_t AjustOK = 0;                   //У׼��־λ

VL53L0X_RangingMeasurementData_t RangData;

//VL53L0X������ģʽ����
//0��Ĭ��;1:�߾���;2:������;3:����
mode_data Mode_data[] = {
    {(FixPoint1616_t)(0.25 * 65536),
     (FixPoint1616_t)(18 * 65536),
     33000,
     14,
     10}, //Ĭ��

    {(FixPoint1616_t)(0.25 * 65536),
     (FixPoint1616_t)(18 * 65536),
     200000,
     14,
     10}, //�߾���

    {(FixPoint1616_t)(0.1 * 65536),
     (FixPoint1616_t)(60 * 65536),
     33000,
     18,
     14}, //������

    {(FixPoint1616_t)(0.25 * 65536),
     (FixPoint1616_t)(32 * 65536),
     20000,
     14,
     10}, //����
};

//IICдn�ֽ�����
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

//IIC��n�ֽ�����
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
  RCC_AHB1PeriphClockCmd(VL53L0_EN_CLK, ENABLE); //ʹ��GPIOBʱ��

  GPIO_InitStructure.GPIO_Pin = VL53L0_EN_GPIO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //�������
  GPIO_Init(VL53L0_EN_GPIO_PORT, &GPIO_InitStructure);

  VL53L0_EN_ON();
}

void VL53L0_ALL_Init()
{
  VL53L0_GPIO_Init();
  vl53l0x_init(&Sensor, VL53L0_Addr);
  SetupSingleShot(&Sensor);
  SysRefInit(&Sensor, HIGH_ACCURACY);
  VL53L0X_StartMeasurement(&Sensor); //��������
}

/**************************************************
 ����  ��������
 ����  ����
 ����ֵ: ����(u16)
 *************************************************/

u16 VL53L0_Get(void)
{
  // VL53L0X_StartMeasurement(&Sensor);                     //��������
  VL53L0X_GetRangingMeasurementData(&Sensor, &RangData); //��ȡ��������,������ʾ����
  return RangData.RangeMilliMeter;
}
//API������Ϣ��ӡ
//Status�����鿴VL53L0X_Error�����Ķ���
void print_pal_error(VL53L0X_Error Status)
{

  char buf[VL53L0X_MAX_STRING_LENGTH];

  VL53L0X_GetPalErrorString(Status, buf); //����Status״̬��ȡ������Ϣ�ַ���

  INFO("API Status: %i : %s\r\n", Status, buf); //��ӡ״̬�ʹ�����Ϣ
}

//VL53L0X д�������
//address:��ַ
//index:ƫ�Ƶ�ַ
//pdata:����ָ��
//count:���� ���65535
u8 VL53L0X_write_multi(u8 address, u8 index, u8 *pdata, u16 count)
{
  u8 status = STATUS_OK;

  if (VL_IIC_Write_nByte(address, index, count, pdata))
  {
    status = STATUS_FAIL;
  }

  return status;
}

//VL53L0X ���������
//address:��ַ
//index:ƫ�Ƶ�ַ
//pdata:����ָ��
//count:���� ���65535
u8 VL53L0X_read_multi(u8 address, u8 index, u8 *pdata, u16 count)
{
  u8 status = STATUS_OK;

  if (VL_IIC_Read_nByte(address, index, count, pdata))
  {
    status = STATUS_FAIL;
  }

  return status;
}

//VL53L0X д1������(���ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(8λ)
u8 VL53L0X_write_byte(u8 address, u8 index, u8 data)
{
  u8 status = STATUS_OK;

  status = VL53L0X_write_multi(address, index, &data, 1);

  return status;
}

//VL53L0X д1������(˫�ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(16λ)
u8 VL53L0X_write_word(u8 address, u8 index, u16 data)
{
  u8 status = STATUS_OK;

  u8 buffer[2];

  //��16λ���ݲ�ֳ�8λ
  buffer[0] = (u8)(data >> 8);   //�߰�λ
  buffer[1] = (u8)(data & 0xff); //�Ͱ�λ

  if (index % 2 == 1)
  {
    //����ͨ�Ų��ܴ���Է�2�ֽڶ���Ĵ������ֽ�
    status = VL53L0X_write_multi(address, index, &buffer[0], 1);
    status = VL53L0X_write_multi(address, index, &buffer[0], 1);
  }
  else
  {
    status = VL53L0X_write_multi(address, index, buffer, 2);
  }

  return status;
}

//VL53L0X д1������(���ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(32λ)
u8 VL53L0X_write_dword(u8 address, u8 index, u32 data)
{

  u8 status = STATUS_OK;

  u8 buffer[4];

  //��32λ���ݲ�ֳ�8λ
  buffer[0] = (u8)(data >> 24);
  buffer[1] = (u8)((data & 0xff0000) >> 16);
  buffer[2] = (u8)((data & 0xff00) >> 8);
  buffer[3] = (u8)(data & 0xff);

  status = VL53L0X_write_multi(address, index, buffer, 4);

  return status;
}

//VL53L0X ��1������(���ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(8λ)
u8 VL53L0X_read_byte(u8 address, u8 index, u8 *pdata)
{
  u8 status = STATUS_OK;

  status = VL53L0X_read_multi(address, index, pdata, 1);

  return status;
}

//VL53L0X ��������(˫�ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(16λ)
u8 VL53L0X_read_word(u8 address, u8 index, u16 *pdata)
{
  u8 status = STATUS_OK;

  u8 buffer[2];

  status = VL53L0X_read_multi(address, index, buffer, 2);

  *pdata = ((u16)buffer[0] << 8) + (u16)buffer[1];

  return status;
}

//VL53L0X ��1������(���ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(32λ)
u8 VL53L0X_read_dword(u8 address, u8 index, u32 *pdata)
{
  u8 status = STATUS_OK;

  u8 buffer[4];

  status = VL53L0X_read_multi(address, index, buffer, 4);

  *pdata = ((u32)buffer[0] << 24) + ((u32)buffer[1] << 16) + ((u32)buffer[2] << 8) + ((u32)buffer[3]);

  return status;
}

//����VL53L0X�豸I2C��ַ
//dev:�豸I2C�����ṹ��
//newaddr:�豸��I2C��ַ
VL53L0X_Error vl53l0x_Addr_set(VL53L0X_Dev_t *dev, uint8_t newaddr)
{
  uint16_t Id;
  uint8_t FinalAddress;
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  u8 sta = 0x00;

  FinalAddress = newaddr;

  if (FinalAddress == dev->I2cDevAddr) //���豸I2C��ַ��ɵ�ַһ��,ֱ���˳�
    return VL53L0X_ERROR_NONE;
  //�ڽ��е�һ���Ĵ�������֮ǰ����I2C��׼ģʽ(400Khz)
  Status = VL53L0X_WrByte(dev, 0x88, 0x00);
  if (Status != VL53L0X_ERROR_NONE)
  {
    sta = 0x01; //����I2C��׼ģʽ����
    goto set_error;
  }
  //����ʹ��Ĭ�ϵ�0x52��ַ��ȡһ���Ĵ���
  Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
  if (Status != VL53L0X_ERROR_NONE)
  {
    sta = 0x02; //��ȡ�Ĵ�������
    goto set_error;
  }
  if (Id == 0xEEAA)
  {
    //�����豸�µ�I2C��ַ
    Status = VL53L0X_SetDeviceAddress(dev, FinalAddress);
    if (Status != VL53L0X_ERROR_NONE)
    {
      sta = 0x03; //����I2C��ַ����
      goto set_error;
    }
    //�޸Ĳ����ṹ���I2C��ַ
    dev->I2cDevAddr = FinalAddress;
    //����µ�I2C��ַ��д�Ƿ�����
    Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
    if (Status != VL53L0X_ERROR_NONE)
    {
      sta = 0x04; //��I2C��ַ��д����
      goto set_error;
    }
  }
set_error:
  if (Status != VL53L0X_ERROR_NONE)
  {
    print_pal_error(Status); //��ӡ������Ϣ
  }
  if (sta != 0)
    INFO("sta:0x%x\r\n", sta);
  return Status;
}

VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *dev, u8 select)
{
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  VL53L0X_Dev_t *pMyDevice = dev;

  pMyDevice->I2cDevAddr = 0x52; //I2C��ַ(�ϵ�Ĭ��0x52)

  pMyDevice->comms_type = 1;        //I2Cͨ��ģʽ
  pMyDevice->comms_speed_khz = 400; //I2Cͨ������

  VL53L0_EN_OFF(); //ʧ��VL53L0X
  delay_ms(30);
  VL53L0_EN_ON(); //ʹ��VL53L0X,�ô��������ڹ���(I2C��ַ��ָ�Ĭ��0X52)
  delay_ms(30);

  Status = vl53l0x_Addr_set(pMyDevice, VL53L0_Addr); //����VL53L0X������I2C��ַ

  if (Status != VL53L0X_ERROR_NONE)
    goto error;

  Status = VL53L0X_DataInit(pMyDevice); //�豸��ʼ��
  if (Status != VL53L0X_ERROR_NONE)
    goto error;

  delay_ms(2);

  Status = VL53L0X_GetDeviceInfo(pMyDevice, &vl53l0x_dev_info); //��ȡ�豸ID��Ϣ
  if (Status != VL53L0X_ERROR_NONE)
    goto error;

  //AT24CXX_Read(0,(u8*)&Vl53l0x_data,sizeof(_vl53l0x_adjust));//��ȡ24c02�����У׼����,����У׼ Vl53l0x_data.adjustok==0xAA
  //if(Vl53l0x_data.adjustok==0xAA)//��У׼
  //  AjustOK=1;
  //else //ûУ׼
  AjustOK = 0;

  /*����API���ټ���*/

error:
  if (Status != VL53L0X_ERROR_NONE)
  {
    print_pal_error(Status); //��ӡ������Ϣ
    return Status;
  }

  return Status;
}

void SysRefInit(VL53L0X_DEV dev, u8 mode)
{
  u8 status;

  status = VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); //ʹ����������ģʽ
  if (status != VL53L0X_ERROR_NONE)
    goto error;
  delay_ms(2);
  status = VL53L0X_SetInterMeasurementPeriodMilliSeconds(dev, Mode_data[mode].timingBudget); //�����ڲ����ڲ���ʱ��
  if (status != VL53L0X_ERROR_NONE)
    goto error;
  delay_ms(2);
  status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1); //ʹ��SIGMA��Χ���
  if (status != VL53L0X_ERROR_NONE)
    goto error;
  delay_ms(2);
  status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1); //ʹ���ź����ʷ�Χ���
  if (status != VL53L0X_ERROR_NONE)
    goto error;
  delay_ms(2);
  status = VL53L0X_SetLimitCheckValue(dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, Mode_data[mode].sigmaLimit); //�趨SIGMA��Χ
  if (status != VL53L0X_ERROR_NONE)
    goto error;
  delay_ms(2);
  status = VL53L0X_SetLimitCheckValue(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, Mode_data[mode].signalLimit); //�趨�ź����ʷ�Χ��Χ
  if (status != VL53L0X_ERROR_NONE)
    goto error;
  delay_ms(2);
  status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev, Mode_data[mode].timingBudget); //�趨��������ʱ��
  if (status != VL53L0X_ERROR_NONE)
    goto error;
  delay_ms(2);
  status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, Mode_data[mode].preRangeVcselPeriod); //�趨VCSEL��������
  if (status != VL53L0X_ERROR_NONE)
    goto error;
  delay_ms(2);
  status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, Mode_data[mode].finalRangeVcselPeriod); //�趨VCSEL�������ڷ�Χ
  if (status != VL53L0X_ERROR_NONE)
    goto error;
  delay_ms(2);
  status = VL53L0X_StopMeasurement(dev); //ֹͣ����
  if (status != VL53L0X_ERROR_NONE)
    goto error;
  delay_ms(2);

error: //������Ϣ
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
