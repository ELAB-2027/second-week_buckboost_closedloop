#ifndef __PWM_BRAKE_H__
#define __PWM_BRAKE_H__

#include "main.h"

void PwmBrake_Init(void);
void PwmBrake_Start(void);
void PwmBrake_Stop(void);
void PwmBrake_StartChannel(uint8_t channel_index);
void PwmBrake_StopChannel(uint8_t channel_index);
void PwmBrake_SetDutyPermille(uint16_t duty_permille);
void PwmBrake_SetAllDutyPermille(uint16_t duty_ch1_permille, uint16_t duty_ch2_permille, uint16_t duty_ch3_permille);
uint16_t PwmBrake_GetDutyPermilleByChannel(uint8_t channel_index);
uint8_t PwmBrake_IsRunning(void);
uint8_t PwmBrake_IsChannelRunning(uint8_t channel_index);
uint16_t PwmBrake_GetDutyPermille(void);
uint16_t PwmBrake_GetDefaultDutyPermille(void);

#endif
