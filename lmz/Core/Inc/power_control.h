#ifndef __POWER_CONTROL_H__
#define __POWER_CONTROL_H__

#include "power_adc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  POWER_CTRL_MODE_OFF = 0,
  POWER_CTRL_MODE_CV,
  POWER_CTRL_MODE_CC,
  POWER_CTRL_MODE_FAULT
} PowerCtrlMode;

typedef struct
{
  PowerAdcSample adc_samples[POWER_ADC_CHANNEL_COUNT];
  float voltage_feedback;
  float voltage_ref;
  float current_feedback;
  float current_ref;
  float duty_voltage_loop;
  float duty_current_loop;
  float duty_command;
  PowerCtrlMode mode;
  uint32_t adc_sequence_counter;
  uint32_t sample_counter;
  uint8_t control_enabled;
  uint8_t protection_active;
  uint8_t last_status_ok;
} PowerCtrlSnapshot;

void PowerCtrl_Init(void);
void PowerCtrl_Run(void);
void PowerCtrl_EnableControl(uint8_t enable);
/* A manual voltage reference wins over the auxiliary-winding tracking reference
   until control is disabled again. */
void PowerCtrl_SetVoltageRef(float voltage_ref);
void PowerCtrl_SetCurrentRef(float current_ref);
const PowerCtrlSnapshot *PowerCtrl_GetSnapshot(void);

#ifdef __cplusplus
}
#endif

#endif
