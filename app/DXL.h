#ifndef _DXL_h_
#define _DXL_h_

#include "stdio.h"
#include "string.h"
#include "stm32f4xx.h"

#define INST_PING           0x01
#define INST_READ           0x02
#define INST_WRITE          0x03
#define INST_REG_WRITE      0x04
#define INST_ACTION         0x05
#define INST_RESET          0x06
#define INST_DIGITAL_RESET  0x08
#define INST_SYSTEM_READ    0x0C
#define INST_SYSTEM_WRITE   0x0D
#define INST_SYNC_WRITE     0x83
#define INST_SYNC_REG_WRITE 0x84
#define INST_BULK_READ      0x92

#define VOLTAGE_ERROR_BIT     0x01
#define ANGLE_LIMIT_ERROR_BIT 0x02
#define OVERHEATING_ERROR_BIT 0x04
#define RANGE_ERROR_BIT       0x08
#define CHECKSUM_ERROR_BIT    0x10
#define OVERLOAD_ERROR_BIT    0x20
#define INSTRUCTION_ERROR_BIT 0x40

#define BROADCASTING_ID 0xfe
#define DEFAULT_ID 200
#define BYTE2BYTE_TIME 100 // ms
#define BUFFER_SIZE 256

#define USART_DXL         USART1
#define USART_PC          USART3

#define DXL_POWER_L()     GPIO_ResetBits(GPIOB, GPIO_Pin_8)
#define DXL_POWER_H()     GPIO_SetBits(GPIOB, GPIO_Pin_8)

#define DXL_TX_EN_L()     GPIO_ResetBits(GPIOB, GPIO_Pin_4)
#define DXL_TX_EN_H()     GPIO_SetBits(GPIOB, GPIO_Pin_4)

#define DXL_RX_EN_L()     GPIO_ResetBits(GPIOB, GPIO_Pin_5)
#define DXL_RX_EN_H()     GPIO_SetBits(GPIOB, GPIO_Pin_5)

typedef enum
{
  DXL_DIR_IN = 0,
  DXL_DIR_OUT
}DXL_Direction_t;

typedef struct
{
  uint8_t power;
  DXL_Direction_t dir;
}DXL_Status_t;

void dxl_irqBus(void);
void dxl_irqPc(void);

void dxl_portInit(uint8_t value);
void dxl_setPower(uint8_t status);
uint8_t dxl_getPower(void);
void dxl_setDirection(DXL_Direction_t mode);
DXL_Direction_t dxl_getDirection(void);

void dxl_init(void);
void dxl_setBaudRate(uint32_t baud);
void dxl_poll(void);


#endif
