#include "jdy31.h"

#include <string.h>

static UART_HandleTypeDef *s_jdy31_huart = NULL;
static uint8_t s_rx_byte = 0;
static char s_rx_line[JDY31_RX_LINE_MAX];
static volatile uint16_t s_rx_len = 0;
static volatile uint8_t s_line_ready = 0;

void JDY31_Init(UART_HandleTypeDef *huart)
{
  s_jdy31_huart = huart;
  s_rx_byte = 0;
  s_rx_len = 0;
  s_line_ready = 0;
  memset(s_rx_line, 0, sizeof(s_rx_line));
}

HAL_StatusTypeDef JDY31_StartReceiveIT(void)
{
  if (s_jdy31_huart == NULL)
  {
    return HAL_ERROR;
  }

  return HAL_UART_Receive_IT(s_jdy31_huart, &s_rx_byte, 1);
}

void JDY31_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((s_jdy31_huart == NULL) || (huart != s_jdy31_huart))
  {
    return;
  }

  if (s_rx_byte == '*')
  {
    if (s_rx_len > 0U)
    {
      s_rx_line[s_rx_len] = '\0';
      s_line_ready = 1;
      s_rx_len = 0;
    }
  }
  else if (s_line_ready == 0U)
  {
    if (s_rx_len < (JDY31_RX_LINE_MAX - 1U))
    {
      s_rx_line[s_rx_len++] = (char)s_rx_byte;
    }
    else
    {
      s_rx_len = 0;
    }
  }

  (void)JDY31_StartReceiveIT();
}

void JDY31_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((s_jdy31_huart == NULL) || (huart != s_jdy31_huart))
  {
    return;
  }

  __HAL_UART_CLEAR_OREFLAG(s_jdy31_huart);
  __HAL_UART_CLEAR_NEFLAG(s_jdy31_huart);
  __HAL_UART_CLEAR_FEFLAG(s_jdy31_huart);
  __HAL_UART_CLEAR_PEFLAG(s_jdy31_huart);
  (void)JDY31_StartReceiveIT();
}

uint8_t JDY31_ReadLine(char *buffer, uint16_t max_len)
{
  uint16_t copy_len;

  if ((buffer == NULL) || (max_len == 0U) || (s_line_ready == 0U))
  {
    return 0;
  }

  copy_len = (uint16_t)strlen(s_rx_line);
  if (copy_len >= max_len)
  {
    copy_len = max_len - 1U;
  }

  memcpy(buffer, s_rx_line, copy_len);
  buffer[copy_len] = '\0';
  memset(s_rx_line, 0, sizeof(s_rx_line));
  s_line_ready = 0;
  return 1;
}

HAL_StatusTypeDef JDY31_Send(uint8_t *data, uint16_t len, uint32_t timeout)
{
  if ((s_jdy31_huart == NULL) || (data == NULL) || (len == 0U))
  {
    return HAL_ERROR;
  }

  return HAL_UART_Transmit(s_jdy31_huart, data, len, timeout);
}

HAL_StatusTypeDef JDY31_SendString(const char *str)
{
  if (str == NULL)
  {
    return HAL_ERROR;
  }

  return JDY31_Send((uint8_t *)str, (uint16_t)strlen(str), HAL_MAX_DELAY);
}

HAL_StatusTypeDef JDY31_SendAT(const char *cmd)
{
  HAL_StatusTypeDef status;

  if (cmd == NULL)
  {
    return HAL_ERROR;
  }

  status = JDY31_SendString(cmd);
  if (status != HAL_OK)
  {
    return status;
  }

  return JDY31_SendString("\r\n");
}
