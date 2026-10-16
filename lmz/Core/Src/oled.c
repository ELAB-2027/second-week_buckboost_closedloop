#include "oled.h"
#include "i2c.h"

#include <string.h>

#define OLED_ADDRESS (0x3CU << 1)
#define OLED_TIMEOUT 100U
#define OLED_PAGE_COUNT 8U
#define OLED_WIDTH 128U
#define OLED_HEIGHT 64U

static uint8_t s_oled_buffer[OLED_PAGE_COUNT][OLED_WIDTH];

static void OLED_WriteCommand(uint8_t command)
{
  uint8_t tx_data[2];

  tx_data[0] = 0x00;
  tx_data[1] = command;
  (void)HAL_I2C_Master_Transmit(&hi2c3, OLED_ADDRESS, tx_data, 2, OLED_TIMEOUT);
}

static void OLED_WriteData(const uint8_t *data, uint8_t count)
{
  uint8_t tx_data[OLED_WIDTH + 1];

  tx_data[0] = 0x40;
  memcpy(&tx_data[1], data, count);
  (void)HAL_I2C_Master_Transmit(&hi2c3, OLED_ADDRESS, tx_data, count + 1U, OLED_TIMEOUT);
}

static void OLED_SetCursor(uint8_t page, uint8_t x)
{
  OLED_WriteCommand(0xB0U | page);
  OLED_WriteCommand(0x10U | ((x & 0xF0U) >> 4));
  OLED_WriteCommand(0x00U | (x & 0x0FU));
}

static void OLED_ShowImage(uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *image)
{
  uint8_t i;
  uint8_t j;
  uint8_t page_count;

  if ((image == NULL) || (x >= OLED_WIDTH) || (y >= OLED_HEIGHT))
  {
    return;
  }

  page_count = (uint8_t)(((height - 1U) / 8U) + 1U);
  for (j = 0; j < page_count; j++)
  {
    for (i = 0; i < width; i++)
    {
      if ((uint16_t)x + i >= OLED_WIDTH)
      {
        break;
      }

      if ((uint16_t)(y / 8U) + j >= OLED_PAGE_COUNT)
      {
        return;
      }

      s_oled_buffer[(y / 8U) + j][x + i] |= (uint8_t)(image[j * width + i] << (y % 8U));
      if (((y % 8U) != 0U) && (((uint16_t)(y / 8U) + j + 1U) < OLED_PAGE_COUNT))
      {
        s_oled_buffer[(y / 8U) + j + 1U][x + i] |= (uint8_t)(image[j * width + i] >> (8U - (y % 8U)));
      }
    }
  }
}

void OLED_Init(void)
{
  HAL_Delay(20);

  OLED_WriteCommand(0xAE);
  OLED_WriteCommand(0xD5);
  OLED_WriteCommand(0x80);
  OLED_WriteCommand(0xA8);
  OLED_WriteCommand(0x3F);
  OLED_WriteCommand(0xD3);
  OLED_WriteCommand(0x00);
  OLED_WriteCommand(0x40);
  OLED_WriteCommand(0xA1);
  OLED_WriteCommand(0xC8);
  OLED_WriteCommand(0xDA);
  OLED_WriteCommand(0x12);
  OLED_WriteCommand(0x81);
  OLED_WriteCommand(0xCF);
  OLED_WriteCommand(0xD9);
  OLED_WriteCommand(0xF1);
  OLED_WriteCommand(0xDB);
  OLED_WriteCommand(0x30);
  OLED_WriteCommand(0xA4);
  OLED_WriteCommand(0xA6);
  OLED_WriteCommand(0x8D);
  OLED_WriteCommand(0x14);
  OLED_WriteCommand(0xAF);

  OLED_Clear();
  OLED_Update();
}

void OLED_Update(void)
{
  uint8_t page;

  for (page = 0; page < OLED_PAGE_COUNT; page++)
  {
    OLED_SetCursor(page, 0);
    OLED_WriteData(s_oled_buffer[page], OLED_WIDTH);
  }
}

void OLED_Clear(void)
{
  memset(s_oled_buffer, 0, sizeof(s_oled_buffer));
}

void OLED_Reverse(void)
{
  uint8_t page;
  uint8_t column;

  for (page = 0; page < OLED_PAGE_COUNT; page++)
  {
    for (column = 0; column < OLED_WIDTH; column++)
    {
      s_oled_buffer[page][column] ^= 0xFFU;
    }
  }
}

void OLED_ShowChar(uint8_t x, uint8_t y, char ch, uint8_t font_size)
{
  if ((ch < ' ') || (ch > '~'))
  {
    ch = '?';
  }

  if (font_size == OLED_8X16)
  {
    OLED_ShowImage(x, y, 8, 16, OLED_F8x16[(uint8_t)(ch - ' ')]);
  }
  else if (font_size == OLED_6X8)
  {
    OLED_ShowImage(x, y, 6, 8, OLED_F6x8[(uint8_t)(ch - ' ')]);
  }
}

void OLED_ShowString(uint8_t x, uint8_t y, char *string, uint8_t font_size)
{
  uint8_t cursor_x;
  uint8_t step;

  if (string == NULL)
  {
    return;
  }

  cursor_x = x;
  step = font_size;
  while (*string != '\0')
  {
    OLED_ShowChar(cursor_x, y, *string, font_size);
    cursor_x = (uint8_t)(cursor_x + step);
    string++;
  }
}
