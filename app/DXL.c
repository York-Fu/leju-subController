#include "DXL.h"
#include "uart/uart.h"
#include "ringbuf/ringbuf.h"
#include "system/delay/delay.h"
#include "DxlRegister.h"


static struct ringbuf DXL_RingBuffer;
static struct ringbuf PC_RingBuffer;
static struct ringbuf Local_RingBuffer;

static uint8_t DXL_Buffer[BUFFER_SIZE];
static uint8_t PC_Buffer[BUFFER_SIZE];
static uint8_t Local_Buffer[BUFFER_SIZE];

static volatile uint8_t DXL2PC_Transmitting = 0;
static uint8_t CurrentId = 0;


static DXL_Status_t DXL = {
  .power = 0,
  .mode = DXL_IN,
};



void DXL_ISR()
{
  u16 ReceivedData;

  if(USART_GetITStatus(DXL_USART, USART_IT_RXNE) != RESET)
  {
    // Read one byte from the receive data register
    ReceivedData = USART_ReceiveData(DXL_USART);
    ringbuf_put(&DXL_RingBuffer, ReceivedData);
    ringbuf_put(&Local_RingBuffer, ReceivedData);
    
    if(!DXL2PC_Transmitting)
    {
      DXL2PC_Transmitting = 1;
      USART_ITConfig(PC_USART, USART_IT_TC, ENABLE);
      USART_SendData(PC_USART, ringbuf_get(&DXL_RingBuffer));
    }
  }
  
  if(USART_GetITStatus(DXL_USART, USART_IT_TC) != RESET)
  {
    if (ringbuf_elements(&PC_RingBuffer) > 0)
    {
      USART_SendData(DXL_USART, ringbuf_get(&PC_RingBuffer));
    }
    else
    {
      DXL_SetMode(DXL_IN);
      USART_ITConfig(DXL_USART, USART_IT_TC, DISABLE);
    }
  }
}


void PC_ISR()
{
  u16 ReceivedData;

  if(USART_GetITStatus(PC_USART, USART_IT_RXNE) != RESET)
  {
    // Read one byte from the receive data register
    ReceivedData = USART_ReceiveData(PC_USART);
    ringbuf_put(&PC_RingBuffer, ReceivedData);
    ringbuf_put(&Local_RingBuffer, ReceivedData);
    
    if(DXL_GetMode() == DXL_IN)
    {
      DXL_SetMode(DXL_OUT);
      USART_ITConfig(DXL_USART, USART_IT_TC, ENABLE);
      USART_SendData(DXL_USART, ringbuf_get(&PC_RingBuffer));
    }
  }
  
  if(USART_GetITStatus(PC_USART, USART_IT_TC) != RESET)
  {
    if (ringbuf_elements(&DXL_RingBuffer) > 0)
    {
      USART_SendData(PC_USART, ringbuf_get(&DXL_RingBuffer));
    }
    else
    {
      DXL2PC_Transmitting = 0;
      USART_ITConfig(PC_USART, USART_IT_TC, DISABLE);
    }
  }
}


#if 0

void DXL_SendByte(uint8_t byte)
{
  Uart1_SendByte(byte);
}

void PC_SendByte(uint8_t byte)
{
  Uart3_SendByte(byte);
}

#else

#define DXL_SendByte(byte)  Uart1_SendByte(byte)
#define PC_SendByte(byte)   Uart3_SendByte(byte)

#endif


void DXL_SetPower(uint8_t status)
{
  if(status == 1)
  {
    DXL_POWER_H();
    DXL.power = 1;
  }
  else
  {
    DXL_POWER_L();
    DXL.power = 0;
  }
}


uint8_t DXL_GetPower()
{
  return DXL.power;
}


void DXL_SetMode(DXL_Mode_t mode)
{
  if(mode == DXL_OUT)
  {
    DXL_RX_EN_L(); // 先关再开
    DXL_TX_EN_H();
  }
  else
  {
    DXL_TX_EN_L();
    DXL_RX_EN_H();
  }
  DXL.mode = mode;
}


DXL_Mode_t DXL_GetMode()
{
  return DXL.mode;
}


void DXL2PC_StatusSet(uint8_t status)
{
  DXL2PC_Transmitting = status;
}


uint8_t DXL2PC_StatusGet()
{
  return DXL2PC_Transmitting;
}


void DXL_RingBufferInit()
{
  ringbuf_init(&DXL_RingBuffer, DXL_Buffer, (uint8_t)BUFFER_SIZE);
  ringbuf_init(&PC_RingBuffer, PC_Buffer, (uint8_t)BUFFER_SIZE);
  ringbuf_init(&Local_RingBuffer, Local_Buffer, (uint8_t)BUFFER_SIZE);
}


void DXL_Init()
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  // TX EN
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  // RX EN
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  // Power control
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // DXL uart
  Uart1_Init(1000000);
  // PC uart
  Uart3_Init(1000000);

  DXL_RingBufferInit();

  DXL_SetPower(1);
  DATA_DYNAMIXEL_POWER = 1;
  DXL_SetMode(DXL_IN);
}


void DXL_BaudRateSet(uint32_t baud)
{
  Uart1_Init(baud);
  Uart3_Init(baud);
}


/*-------------------------------------------------------------------*/

void DXL_SendData(uint8_t *data, uint16_t len)
{
  while(DXL_GetMode() == DXL_OUT);
  DXL_SetMode(DXL_OUT);
  
  for(uint16_t i=0; i<len; i++)
  {
    DXL_SendByte(data[i]);
  }
  
  DXL_SetMode(DXL_IN);
}


void PC_SendData(uint8_t *data, uint16_t len)
{
  while(DXL2PC_Transmitting);
  DXL2PC_Transmitting = 1;
  
  for(uint16_t i=0; i<len; i++)
  {
    PC_SendByte(data[i]);
  }
  
  DXL2PC_Transmitting = 0;
}


uint8_t PackIdCheck(uint8_t id)
{
  if((id == DEFAULT_ID) || (id == BROADCASTING_ID))
  {
    CurrentId = id;
    return 1;
  }
  return 0;
}


uint8_t PackCheckSumProcess(uint8_t *pack, uint16_t len)
{
  uint8_t checkSum = 0;
  for(uint16_t i=2;i<len;i++)
  {
    checkSum += pack[i];
  }
  if(checkSum == 0xFF)
    return 0;
  return 1;
}


void ReturnStatusPacket(uint8_t status, uint8_t *param, uint8_t paramLen)
{
  uint8_t checkSun = 0;
  uint8_t buffer[255] = {0xFF, 0XFF, DEFAULT_ID, paramLen+2, status, 0X00};
  if(param == NULL)
  {
    buffer[5] = ~(DEFAULT_ID + (paramLen+2) + status);
  }
  else
  {
    for(uint8_t i=0;i<paramLen;i++)
    {
      buffer[i+5] = param[i];
      checkSun += param[i];
    }
    buffer[paramLen+5] = ~(DEFAULT_ID + (paramLen+2) + status + checkSun);
  }
  
  PC_SendData(buffer, paramLen+6);
  DXL_SendData(buffer, paramLen+6);
}


void DXL_Read(uint8_t addr, uint8_t length)
{
  if(CurrentId == DEFAULT_ID)
  {
    uint8_t param[255] = {0};
    
    if(DxlRegisterRead(param, addr, length) == 0)
    {
      ReturnStatusPacket(0, param, length);
    }
  }
}


void DXL_Write(uint8_t addr, uint8_t *param, uint8_t paramLen)
{
  if(DxlRegisterWrite(addr, param, paramLen) == 0)
  {
    if(CurrentId == DEFAULT_ID)
      ReturnStatusPacket(0, NULL, 0);
  }
}


uint8_t DXL_BulkReadListening(uint8_t id)
{
  if(id == 0)
    return 1;
  
  uint8_t buffer[3] = {0};
  SystemMs_t timestamp = SystemMsGet();
  
  while(1)
  {
    if(ringbuf_elements(&Local_RingBuffer) > 0)
    {
      buffer[0] = buffer[1];
      buffer[1] = buffer[2];
      buffer[2] = ringbuf_get(&Local_RingBuffer);
      if((buffer[0] == 0xFF) && (buffer[1] == 0xFF) && (buffer[2] == id))
      {
        return 1;
      }
      timestamp = SystemMsGet();
    }
    else if((SystemMsGet()-timestamp) > 100) // timeout
    {
      break;
    }
  }
  return 0;
}


void DXL_BulkRead(uint8_t *param, uint8_t length)
{
  uint8_t param0[255] = {0};
  uint8_t param1[3] = {0};
  uint8_t param2[3] = {0};
  uint8_t BulkReadStatus = 0;
  
  for(uint8_t i=0;i<length;i+=3)
  {
    if(param[i+1] == DEFAULT_ID)
    {
      param2[0] = param[i];
      param2[1] = param[i+1];
      param2[2] = param[i+2];
      BulkReadStatus = 1;
      break;
    }
    param1[0] = param[i];
    param1[1] = param[i+1];
    param1[2] = param[i+2];
  }
  
  if(BulkReadStatus == 1)
  {
    if(DXL_BulkReadListening(param1[1]) == 1)
    {
      if(DxlRegisterRead(param0, param2[2], param2[0]) == 0)
      {
        ReturnStatusPacket(0, param0, param2[0]);
      }
    }
  }
}


void PackProcess(uint8_t *pack, uint16_t len)
{
  if(PackCheckSumProcess(pack, len) == 0)
  {
    uint8_t paramLength = pack[3] - 2;
    uint8_t instruction = pack[4];
    uint8_t param[255] = {0};
    for(uint8_t i=0;i<paramLength;i++)
    {
      param[i] = pack[i+5];
    }
    
    switch(instruction)
    {
      case INST_PING:
        ReturnStatusPacket(0, NULL, 0);
        break;
      case INST_READ:
        DXL_Read(param[0], param[1]);
        break;
      case INST_WRITE:
        DXL_Write(param[0], &param[1], paramLength-1);
        break;
      case INST_REG_WRITE:
        break;
      case INST_ACTION:
        break;
      case INST_RESET:
      case INST_DIGITAL_RESET:
        ReturnStatusPacket(0, NULL, 0);
        MCU_Reset();
        break;
      case INST_SYNC_WRITE:
        break;
      case INST_BULK_READ:
        DXL_BulkRead(&param[1], paramLength-1);
        break;
      default:
        ReturnStatusPacket(INSTRUCTION_ERROR_BIT, NULL, 0);
    }
  }
  else
  {
    ReturnStatusPacket(CHECKSUM_ERROR_BIT, NULL, 0);
  }
  CurrentId = 0;
}




void DXL_Poll()
{
  static Buffer_t buffer;
  static uint8_t packStatus = 0;
  static uint16_t packLength = 0;
  static SystemMs_t timestamp = 0;
  
  while(ringbuf_elements(&Local_RingBuffer) > 0)
  {
    if(packStatus & 0x02) // 读取包数据
    {
      buffer.data[buffer.offset++] = ringbuf_get(&Local_RingBuffer);
      if(buffer.offset >= packLength)
      {
        PackProcess(buffer.data, packLength);
        memset(buffer.data, 0, packLength);
        buffer.offset = 0;
        packLength = 0;
        packStatus = 0;
      }
    }
    else if(packStatus & 0x01) // 得到包头，检查ID，计算包长
    {
      buffer.data[buffer.offset++] = ringbuf_get(&Local_RingBuffer);
      if(buffer.offset > 3)
      {
        if(PackIdCheck(buffer.data[2]))
        {
          packLength = buffer.data[3] + 4;
          if(packLength < BUFFER_SIZE)
          {
            packStatus = 0x02;
          }
          else
          {
            memset(buffer.data, 0, buffer.offset);
            buffer.offset = 0;
            packLength = 0;
            packStatus = 0;
          }
        }
        else
        {
          memset(buffer.data, 0, buffer.offset);
          buffer.offset = 0;
          packStatus = 0;
        }
      }
    }
    else // 检测包头
    {
      buffer.data[0] = buffer.data[1];
      buffer.data[1] = ringbuf_get(&Local_RingBuffer);
      if((buffer.data[0] == 0xFF) && (buffer.data[1] == 0xFF))
      {
        buffer.offset = 2;
        packStatus = 0x01;
      }
    }
    timestamp = SystemMsGet();
  }
  if((packStatus != 0) && ((SystemMsGet()-timestamp) > 100)) // timeout
  {
    memset(buffer.data, 0, buffer.offset);
    buffer.offset = 0;
    packLength = 0;
    packStatus = 0;
  }
}

