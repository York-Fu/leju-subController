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
#define DEFAULT_ID	200

#define BUFFER_SIZE     256


#define DXL_USART         USART1
#define PC_USART          USART3

#define DXL_POWER_L()     GPIO_ResetBits(GPIOB, GPIO_Pin_8)
#define DXL_POWER_H()     GPIO_SetBits(GPIOB, GPIO_Pin_8)

#define DXL_TX_EN_L()     GPIO_ResetBits(GPIOB, GPIO_Pin_4)
#define DXL_TX_EN_H()     GPIO_SetBits(GPIOB, GPIO_Pin_4)

#define DXL_RX_EN_L()     GPIO_ResetBits(GPIOB, GPIO_Pin_5)
#define DXL_RX_EN_H()     GPIO_SetBits(GPIOB, GPIO_Pin_5)


typedef enum
{
  DXL_IN = 0,
  DXL_OUT,
}DXL_Mode_t;


typedef struct
{
  uint8_t power;
  DXL_Mode_t mode;
}DXL_Status_t;


typedef struct
{
  uint8_t data[255];
  uint16_t offset;
  uint16_t flags;
}Buffer_t;




void DXL_ISR(void);
void PC_ISR(void);

void DXL_SetPower(uint8_t status);
uint8_t DXL_GetPower(void);

void DXL_SetMode(DXL_Mode_t mode);
DXL_Mode_t DXL_GetMode(void);

void DXL2PC_StatusSet(uint8_t status);
uint8_t DXL2PC_StatusGet(void);

void DXL_Init(void);
void DXL_BaudRateSet(uint32_t baud);

void DXL_SendData(uint8_t *data, uint16_t len);
void PC_SendData(uint8_t *data, uint16_t len);
void ReturnStatusPacket(uint8_t status, uint8_t *param, uint8_t paramLen);
void DXL_Poll(void);


#endif
