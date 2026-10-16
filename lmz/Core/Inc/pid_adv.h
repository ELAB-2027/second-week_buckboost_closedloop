#ifndef __PID_ADV_H__
#define __PID_ADV_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float kp;
  float ki;
  float kd;
  float integral;
  float prev_measurement;
  float last_proportional;
  float last_derivative;
  float output;
  float output_min;
  float output_max;
  float ramp_rate;
  uint8_t initialized;
} PidAdvController;

void PidAdv_Init(PidAdvController *controller, float kp, float ki, float kd, float output_min, float output_max);
void PidAdv_Reset(PidAdvController *controller);
void PidAdv_SetRampRate(PidAdvController *controller, float ramp_rate);
void PidAdv_SetHeldOutput(PidAdvController *controller, float held_output);
float PidAdv_Update(PidAdvController *controller, float setpoint, float measurement, float dt_s);

#ifdef __cplusplus
}
#endif

#endif
