#ifndef __OLED_H__
#define __OLED_H__

#include "main.h"
#include "oled_data.h"

#define OLED_8X16 8
#define OLED_6X8 6

void OLED_Init(void);
void OLED_Update(void);
void OLED_Clear(void);
void OLED_Reverse(void);
void OLED_ShowChar(uint8_t x, uint8_t y, char ch, uint8_t font_size);
void OLED_ShowString(uint8_t x, uint8_t y, char *string, uint8_t font_size);

#endif
