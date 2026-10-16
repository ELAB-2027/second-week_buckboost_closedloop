#ifndef __MPPT_H__
#define __MPPT_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float delta;
  float max;
  float min;
  float prev_power;
  int8_t direction;
  uint8_t has_prev_power;
} MpptController;

void Mppt_Init(MpptController *controller, float delta, float max, float min);
float Mppt_Update(MpptController *controller, float voltage, float current, float command_now);

#ifdef __cplusplus
}
#endif

#endif
