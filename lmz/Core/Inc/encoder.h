#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h"

void Encoder_Init(void);
void Encoder_HandleExti(uint16_t gpio_pin);
void Encoder_Poll(void);
int8_t Encoder_GetDelta(void);
uint8_t Encoder_GetKeyPressed(void);

#endif
