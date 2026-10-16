#ifndef __POWER_PWM_H__
#define __POWER_PWM_H__

#include "main.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* HRTIM1 half bridges used as the power carrier:
 * CH_C -> PB12 (OC) / PB13 (OR)
 * CH_D -> PB14 (OC) / PB15 (OR)
 * CH_F -> PC6  (OC) / PC7  (OR)
 */
typedef enum
{
  POWER_PWM_CH_C = 0,
  POWER_PWM_CH_D,
  POWER_PWM_CH_F,
  POWER_PWM_CHANNEL_COUNT
} PowerPwmChannel;

#define POWER_PWM_FREQ_HZ 100000U
#define POWER_PWM_DEADTIME_NS 250U
#define POWER_PWM_MAX_DUTY 0.85f
#define POWER_PWM_DEFAULT_DUTY 0.12f

/* ADC trigger 1 is armed on the master period event, so the sampling instant is
 * locked to the start of every carrier cycle.
 */
#define POWER_PWM_ADC_TRIGGER HRTIM_ADCTRIGGER_1

HAL_StatusTypeDef PowerPwm_Init(void);
HAL_StatusTypeDef PowerPwm_SetDuty(PowerPwmChannel channel, float duty);
void PowerPwm_SetDutyAll(float duty);
float PowerPwm_GetDuty(PowerPwmChannel channel);
HAL_StatusTypeDef PowerPwm_StartChannel(PowerPwmChannel channel);
void PowerPwm_StartAll(void);
HAL_StatusTypeDef PowerPwm_StopChannel(PowerPwmChannel channel);
void PowerPwm_StopAll(void);
uint8_t PowerPwm_IsRunning(void);
uint8_t PowerPwm_IsChannelRunning(PowerPwmChannel channel);

#ifdef __cplusplus
}
#endif

#endif
