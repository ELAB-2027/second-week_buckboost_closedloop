#include "mppt.h"

#include <stddef.h>

static float Mppt_Clamp(float value, float min, float max)
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

void Mppt_Init(MpptController *controller, float delta, float max, float min)
{
  if (controller == NULL)
  {
    return;
  }

  controller->delta = delta;
  controller->max = max;
  controller->min = min;
  controller->prev_power = 0.0f;
  controller->direction = 1;
  controller->has_prev_power = 0U;
}

float Mppt_Update(MpptController *controller, float voltage, float current, float command_now)
{
  float power;
  float command_next;

  if (controller == NULL)
  {
    return command_now;
  }

  power = voltage * current;

  if (controller->has_prev_power != 0U)
  {
    if (power < controller->prev_power)
    {
      controller->direction = (int8_t)(-controller->direction);
    }
  }
  else
  {
    controller->has_prev_power = 1U;
  }

  command_next = command_now + ((float)controller->direction * controller->delta);
  command_next = Mppt_Clamp(command_next, controller->min, controller->max);
  controller->prev_power = power;

  return command_next;
}
