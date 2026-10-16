#include "pid_adv.h"

#include <stddef.h>

static float PidAdv_Clamp(float value, float min, float max)
{
  if (value < min)
  {
    return min;
  }

  if (value > max)
  {
    return max;
  }

  return value;
}

void PidAdv_Init(PidAdvController *controller, float kp, float ki, float kd, float output_min, float output_max)
{
  if (controller == NULL)
  {
    return;
  }

  controller->kp = kp;
  controller->ki = ki;
  controller->kd = kd;
  controller->output_min = output_min;
  controller->output_max = output_max;
  controller->ramp_rate = 0.0f;
  controller->initialized = 1U;
  PidAdv_Reset(controller);
}

void PidAdv_Reset(PidAdvController *controller)
{
  if (controller == NULL)
  {
    return;
  }

  controller->integral = 0.0f;
  controller->prev_measurement = 0.0f;
  controller->last_proportional = 0.0f;
  controller->last_derivative = 0.0f;
  controller->output = 0.0f;
}

void PidAdv_SetHeldOutput(PidAdvController *controller, float held_output)
{
  float target;
  float integral;

  if ((controller == NULL) || (controller->initialized == 0U))
  {
    return;
  }

  target = PidAdv_Clamp(held_output, controller->output_min, controller->output_max);
  integral = target - controller->last_proportional - controller->last_derivative;
  controller->integral = PidAdv_Clamp(integral, -controller->output_max, controller->output_max);
  controller->output = target;
}

void PidAdv_SetRampRate(PidAdvController *controller, float ramp_rate)
{
  if (controller == NULL)
  {
    return;
  }

  controller->ramp_rate = ramp_rate;
}

float PidAdv_Update(PidAdvController *controller, float setpoint, float measurement, float dt_s)
{
  float error;
  float proportional;
  float derivative;
  float candidate_integral;
  float unsaturated_output;
  float output;
  float max_delta;

  if ((controller == NULL) || (controller->initialized == 0U))
  {
    return 0.0f;
  }

  if (dt_s <= 0.0f)
  {
    return controller->output;
  }

  error = setpoint - measurement;
  proportional = controller->kp * error;
  derivative = -controller->kd * (measurement - controller->prev_measurement) / dt_s;

  candidate_integral = controller->integral + controller->ki * error * dt_s;
  candidate_integral = PidAdv_Clamp(candidate_integral, -controller->output_max, controller->output_max);
  unsaturated_output = proportional + candidate_integral + derivative;

  if (((unsaturated_output > controller->output_max) && (error > 0.0f))
      || ((unsaturated_output < controller->output_min) && (error < 0.0f)))
  {
    candidate_integral = controller->integral;
    unsaturated_output = proportional + candidate_integral + derivative;
  }

  controller->integral = candidate_integral;
  output = PidAdv_Clamp(unsaturated_output, controller->output_min, controller->output_max);

  if (controller->ramp_rate > 0.0f)
  {
    max_delta = controller->ramp_rate * dt_s;
    if ((output - controller->output) > max_delta)
    {
      output = controller->output + max_delta;
    }
    else if ((controller->output - output) > max_delta)
    {
      output = controller->output - max_delta;
    }
  }

  controller->prev_measurement = measurement;
  controller->last_proportional = proportional;
  controller->last_derivative = derivative;
  controller->output = output;
  return output;
}
