#include "DXL.h"
#include "uart/uart.h"
#include "ringbuf/ringbuf.h"
#include "system/delay/delay.h"
#include "DxlRegister.h"

static struct ringbuf Ringbuf_TxPc;
static struct ringbuf Ringbuf_TxDxl;
static struct ringbuf Ringbuf_Rx;

static uint8_t Buffer_TxPc[BUFFER_SIZE];
static uint8_t Buffer_TxDxl[BUFFER_SIZE];
static uint8_t Buffer_Rx[BUFFER_SIZE];

static volatile uint8_t PcTransmitting = 0;
static volatile uint8_t DxlTransmitting = 0;
static volatile uint8_t CurrentId = 0;

static volatile DXL_Status_t DXL = {
    .power = 0,
    .dir = DXL_DIR_IN,
};

void dxl_irqBus()
{
  uint16_t ReceivedData;

  if (USART_GetITStatus(USART_DXL, USART_IT_RXNE) != RESET)
  {
    // Read one byte from the receive data register
    ReceivedData = USART_ReceiveData(USART_DXL);
    ringbuf_put(&Ringbuf_TxPc, ReceivedData);
    ringbuf_put(&Ringbuf_Rx, ReceivedData);

    if (PcTransmitting == 0)
    {
      PcTransmitting = 1;
      USART_ITConfig(USART_PC, USART_IT_TC, ENABLE);
      USART_SendData(USART_PC, ringbuf_get(&Ringbuf_TxPc));
    }
  }

  if (USART_GetITStatus(USART_DXL, USART_IT_TC) != RESET)
  {
    if (ringbuf_elements(&Ringbuf_TxDxl) > 0)
    {
      USART_SendData(USART_DXL, ringbuf_get(&Ringbuf_TxDxl));
    }
    else
    {
      dxl_setDirection(DXL_DIR_IN);
      DxlTransmitting = 0;
      USART_ITConfig(USART_DXL, USART_IT_TC, DISABLE);
    }
  }
}

void dxl_irqPc()
{
  uint16_t ReceivedData;

  if (USART_GetITStatus(USART_PC, USART_IT_RXNE) != RESET)
  {
    // Read one byte from the receive data register
    ReceivedData = USART_ReceiveData(USART_PC);
    ringbuf_put(&Ringbuf_TxDxl, ReceivedData);
    ringbuf_put(&Ringbuf_Rx, ReceivedData);

    if (DxlTransmitting == 0)
    {
      dxl_setDirection(DXL_DIR_OUT);
      DxlTransmitting = 1;
      USART_ITConfig(USART_DXL, USART_IT_TC, ENABLE);
      USART_SendData(USART_DXL, ringbuf_get(&Ringbuf_TxDxl));
    }
  }

  if (USART_GetITStatus(USART_PC, USART_IT_TC) != RESET)
  {
    if (ringbuf_elements(&Ringbuf_TxPc) > 0)
    {
      USART_SendData(USART_PC, ringbuf_get(&Ringbuf_TxPc));
    }
    else
    {
      PcTransmitting = 0;
      USART_ITConfig(USART_PC, USART_IT_TC, DISABLE);
    }
  }
}

void dxl_portInit(uint8_t value)
{
  uint32_t baudRate;
  if (value < 250)
    baudRate = 2000000 / (value + 1);
  else
  {
    switch (value)
    {
    case 250:
      baudRate = 2250000;
      break;
    case 251:
      baudRate = 2500000;
      break;
    case 252:
      baudRate = 3000000;
      break;
    default:
      baudRate = 115200;
    }
  }
  Uart1_Init(baudRate); // BUS uart
  Uart3_Init(baudRate); // PC uart
}

void dxl_ringBufferInit()
{
  ringbuf_init(&Ringbuf_TxPc, Buffer_TxPc, (uint8_t)BUFFER_SIZE);
  ringbuf_init(&Ringbuf_TxDxl, Buffer_TxDxl, (uint8_t)BUFFER_SIZE);
  ringbuf_init(&Ringbuf_Rx, Buffer_Rx, (uint8_t)BUFFER_SIZE);
}

void dxl_init()
{
  GPIO_InitTypeDef GPIO_InitStructure;

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

  dxl_portInit(DATA_BAUD_RATE);
  dxl_ringBufferInit();

  dxl_setPower(0);
  delay_ms(500);
  dxl_setPower(1);
  DATA_DYNAMIXEL_POWER = 1;
  dxl_setDirection(DXL_DIR_IN);
}

/*-------------------------------------------------------------------*/

void dxl_sendBus(uint8_t *data, uint16_t len)
{
  for (uint16_t i = 0; i < len; i++)
  {
    ringbuf_put(&Ringbuf_TxDxl, data[i]);
  }
  dxl_setDirection(DXL_DIR_OUT);
  DxlTransmitting = 1;
  USART_ITConfig(USART_DXL, USART_IT_TC, ENABLE);
  USART_SendData(USART_DXL, ringbuf_get(&Ringbuf_TxDxl));
  while (DxlTransmitting)
    ;
}

void dxl_sendPc(uint8_t *data, uint16_t len)
{
  for (uint16_t i = 0; i < len; i++)
  {
    ringbuf_put(&Ringbuf_TxPc, data[i]);
  }
  PcTransmitting = 1;
  USART_ITConfig(USART_PC, USART_IT_TC, ENABLE);
  USART_SendData(USART_PC, ringbuf_get(&Ringbuf_TxPc));
  while (PcTransmitting)
    ;
}

void dxl_setPower(uint8_t status)
{
  if (status)
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

uint8_t dxl_getPower()
{
  return DXL.power;
}

void dxl_setDirection(DXL_Direction_t dir)
{
  if (dir == DXL_DIR_OUT)
  {
    DXL_RX_EN_L(); // 先关再开
    DXL_TX_EN_H();
  }
  else
  {
    DXL_TX_EN_L();
    DXL_RX_EN_H();
  }
  DXL.dir = dir;
}

DXL_Direction_t dxl_getDirection()
{
  return DXL.dir;
}

uint8_t PackIdCheck(uint8_t id)
{
  if ((id == DEFAULT_ID) || (id == BROADCASTING_ID))
  {
    CurrentId = id;
    return 1;
  }
  return 0;
}

uint8_t SlaveCheck(uint8_t byte, uint8_t *buffer)
{
  buffer[0] = buffer[1];
  buffer[1] = buffer[2];
  buffer[2] = byte;
  if ((buffer[0] == 0xFF) && (buffer[1] == 0xFF) &&
      ((buffer[2] == BROADCASTING_ID) || (buffer[2] == DEFAULT_ID)))
  {
    CurrentId = buffer[2];
    return 1;
  }
  return 0;
}

uint8_t PackCheckSumProcess(uint8_t *pack, uint16_t len)
{
  uint8_t checkSum = 0;
  for (uint16_t i = 2; i < len; i++)
  {
    checkSum += pack[i];
  }
  if (checkSum == 0xFF)
    return 0;
  return 1;
}

void ReturnStatusPacket(uint8_t status, uint8_t *param, uint8_t paramLen)
{
  uint8_t checkSun = 0;
  uint8_t buffer[255] = {0xFF, 0XFF, DEFAULT_ID, paramLen + 2, status, 0X00};
  if (param == NULL)
  {
    buffer[5] = ~(DEFAULT_ID + (paramLen + 2) + status);
  }
  else
  {
    for (uint8_t i = 0; i < paramLen; i++)
    {
      buffer[i + 5] = param[i];
      checkSun += param[i];
    }
    buffer[paramLen + 5] = ~(DEFAULT_ID + (paramLen + 2) + status + checkSun);
  }
  dxl_sendPc(buffer, paramLen + 6);
  dxl_sendBus(buffer, paramLen + 6);
}

void dxl_read(uint8_t addr, uint8_t length)
{
  uint8_t param[255] = {0};

  if (DR_read(param, addr, length) == 0)
  {
    ReturnStatusPacket(0, param, length);
  }
}

void dxl_write(uint8_t addr, uint8_t *param, uint8_t paramLen)
{
  if (DR_write(addr, param, paramLen) == 0)
  {
    ReturnStatusPacket(0, NULL, 0);
  }
}

uint8_t dxl_bulkReadListening(uint16_t byteNum)
{
  if (byteNum == 0)
    return 1;

  uint16_t byteCount = 0;
  SystemMs_t timestamp = SystemMsGet();

  while (1)
  {
    if (ringbuf_elements(&Ringbuf_Rx) > 0)
    {
      ringbuf_get(&Ringbuf_Rx);
      byteCount++;
      if (byteCount >= byteNum)
      {
        return 1;
      }
      timestamp = SystemMsGet();
    }
    else if ((SystemMsGet() - timestamp) > BYTE2BYTE_TIME) // timeout
    {
      break;
    }
  }
  return 0;
}

void dxl_bulkRead(uint8_t *param, uint8_t length)
{
  uint8_t retParam[255] = {0};
  uint8_t param1[3] = {0};
  uint8_t BulkReadStatus = 0;
  uint16_t listheingByte = 0;

  for (uint8_t i = 0; i < length; i += 3)
  {
    if (param[i + 1] == DEFAULT_ID)
    {
      param1[0] = param[i];
      param1[1] = param[i + 1];
      param1[2] = param[i + 2];
      BulkReadStatus = 1;
      break;
    }
    listheingByte += (param[i] + 6);
  }

  if (BulkReadStatus == 1)
  {
    if (dxl_bulkReadListening(listheingByte) == 1)
    {
      if (DR_read(retParam, param1[2], param1[0]) == 0)
      {
        ReturnStatusPacket(0, retParam, param1[0]);
      }
    }
  }
}

void PackProcess(uint8_t *pack, uint16_t len)
{
  if (PackCheckSumProcess(pack, len) == 0)
  {
    uint8_t paramLength = pack[3] - 2;
    uint8_t instruction = pack[4];
    uint8_t *param = pack + 5;

    switch (instruction)
    {
    case INST_PING:
      ReturnStatusPacket(0, NULL, 0);
      break;
    case INST_READ:
      dxl_read(param[0], param[1]);
      break;
    case INST_WRITE:
      dxl_write(param[0], &param[1], paramLength - 1);
      break;
    case INST_REG_WRITE:
      break;
    case INST_ACTION:
      break;
    case INST_RESET:
    case INST_DIGITAL_RESET:
      ReturnStatusPacket(0, NULL, 0);
      sys_reset();
      break;
    case INST_SYNC_WRITE:
      break;
    case INST_BULK_READ:
      dxl_bulkRead(&param[1], paramLength - 1);
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

void dxl_poll()
{
  static uint8_t buffer[255] = {0};
  static uint16_t offset = 0, packLength = 0;
  static uint8_t status = 0;
  static SystemMs_t timestamp = 0;

  while (ringbuf_elements(&Ringbuf_Rx) > 0)
  {
    if ((status & 0x02) == 0x02)
    {
      buffer[offset++] = ringbuf_get(&Ringbuf_Rx);
      if (offset >= packLength)
      {
        PackProcess(buffer, packLength);
        offset = 0;
        packLength = 0;
        status = 0;
      }
    }
    else if ((status & 0x01) == 0x01)
    {
      buffer[offset++] = ringbuf_get(&Ringbuf_Rx);
      packLength = buffer[3] + 4;
      status = 0x02;
      if (packLength > BUFFER_SIZE)
      {
        offset = 0;
        packLength = 0;
        status = 0;
      }
    }
    else if (SlaveCheck(ringbuf_get(&Ringbuf_Rx), buffer))
    {
      offset = 3;
      status = 0x01;
    }
    timestamp = SystemMsGet();
  }
  if ((status != 0) && ((SystemMsGet() - timestamp) > BYTE2BYTE_TIME)) // timeout
  {
    offset = 0;
    packLength = 0;
    status = 0;
  }
}
