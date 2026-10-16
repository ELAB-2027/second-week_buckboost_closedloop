#ifndef __JDY31_H__
#define __JDY31_H__

#include "main.h"

#define JDY31_RX_LINE_MAX 64

void JDY31_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef JDY31_StartReceiveIT(void);
void JDY31_RxCpltCallback(UART_HandleTypeDef *huart);
void JDY31_ErrorCallback(UART_HandleTypeDef *huart);
uint8_t JDY31_ReadLine(char *buffer, uint16_t max_len);
HAL_StatusTypeDef JDY31_Send(uint8_t *data, uint16_t len, uint32_t timeout);
HAL_StatusTypeDef JDY31_SendString(const char *str);
HAL_StatusTypeDef JDY31_SendAT(const char *cmd);

#endif
