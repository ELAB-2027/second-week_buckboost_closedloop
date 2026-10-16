#include "encoder.h"

static volatile int8_t s_delta = 0;
static uint8_t s_key_event = 0;
static GPIO_PinState s_key_last_raw = GPIO_PIN_SET;
static GPIO_PinState s_key_stable = GPIO_PIN_SET;
static uint32_t s_key_tick = 0;

static int8_t Encoder_ClampDelta(int16_t value)
{
  if (value > 127)
  {
    return 127;
  }

  if (value < -127)
  {
    return -127;
  }

  return (int8_t)value;
}

void Encoder_Init(void)
{
  s_delta = 0;
  s_key_event = 0;
  s_key_last_raw = HAL_GPIO_ReadPin(Encoder_Key_GPIO_Port, Encoder_Key_Pin);
  s_key_stable = s_key_last_raw;
  s_key_tick = HAL_GetTick();
}

void Encoder_HandleExti(uint16_t gpio_pin)
{
  int16_t next_delta;

  if (gpio_pin != Encoder_A_Pin)
  {
    return;
  }

  next_delta = s_delta;
  if (HAL_GPIO_ReadPin(Encoder_B_GPIO_Port, Encoder_B_Pin) == GPIO_PIN_SET)
  {
    next_delta--;
  }
  else
  {
    next_delta++;
  }

  s_delta = Encoder_ClampDelta(next_delta);
}

void Encoder_Poll(void)
{
  GPIO_PinState raw_state;

  raw_state = HAL_GPIO_ReadPin(Encoder_Key_GPIO_Port, Encoder_Key_Pin);
  if (raw_state != s_key_last_raw)
  {
    s_key_last_raw = raw_state;
    s_key_tick = HAL_GetTick();
  }

  if ((HAL_GetTick() - s_key_tick) < 20U)
  {
    return;
  }

  if (raw_state != s_key_stable)
  {
    s_key_stable = raw_state;
    if (s_key_stable == GPIO_PIN_RESET)
    {
      s_key_event = 1;
    }
  }
}

int8_t Encoder_GetDelta(void)
{
  int8_t delta;

  delta = s_delta;
  s_delta = 0;
  return delta;
}

uint8_t Encoder_GetKeyPressed(void)
{
  uint8_t pressed;

  pressed = s_key_event;
  s_key_event = 0;
  return pressed;
}
