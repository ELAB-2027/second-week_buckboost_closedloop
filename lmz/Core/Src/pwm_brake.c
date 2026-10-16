#include "pwm_brake.h"

#include "tim.h"

#define PWM_BRAKE_CHANNEL_COUNT 3U
#define PWM_BRAKE_DEFAULT_DUTY_PERMILLE 120U
#define PWM_BRAKE_DUTY_PERMILLE_BASE 1000U
#define PWM_BRAKE_MIN_DUTY_PERMILLE 100U
#define PWM_BRAKE_MAX_DUTY_PERMILLE 850U
#define PWM_BRAKE_DEFAULT_DEADTIME_NS 250U
#define PWM_BRAKE_TIM8_CLOCK_HZ 170000000ULL
#define PWM_BRAKE_TIM8_TDTS_DIVIDER 4ULL
#define PWM_BRAKE_DEADTIME_SCALE (PWM_BRAKE_TIM8_TDTS_DIVIDER * 1000000000ULL)

static uint16_t s_duty_permille[PWM_BRAKE_CHANNEL_COUNT] =
{
  PWM_BRAKE_DEFAULT_DUTY_PERMILLE,
  PWM_BRAKE_DEFAULT_DUTY_PERMILLE,
  PWM_BRAKE_DEFAULT_DUTY_PERMILLE
};
static uint8_t s_running_mask = 0U;

static uint16_t PwmBrake_ClampDuty(uint16_t duty_permille)
{
  if (duty_permille == 0U)
  {
    return 0U;
  }

  if (duty_permille < PWM_BRAKE_MIN_DUTY_PERMILLE)
  {
    return PWM_BRAKE_MIN_DUTY_PERMILLE;
  }

  if (duty_permille > PWM_BRAKE_MAX_DUTY_PERMILLE)
  {
    return PWM_BRAKE_MAX_DUTY_PERMILLE;
  }

  return duty_permille;
}

static uint32_t PwmBrake_DutyToCompare(uint16_t duty_permille)
{
  uint32_t period;
  uint32_t compare;

  period = __HAL_TIM_GET_AUTORELOAD(&htim8);
  compare = ((uint32_t)(period + 1UL) * duty_permille) / PWM_BRAKE_DUTY_PERMILLE_BASE;
  if (compare > period)
  {
    compare = period;
  }

  return compare;
}

static uint8_t PwmBrake_DeadTimeCodeFromNs(uint32_t deadtime_ns)
{
  uint64_t ticks;

  ticks = ((uint64_t)deadtime_ns * PWM_BRAKE_TIM8_CLOCK_HZ + (PWM_BRAKE_DEADTIME_SCALE / 2ULL)) / PWM_BRAKE_DEADTIME_SCALE;
  if ((ticks == 0ULL) && (deadtime_ns != 0U))
  {
    ticks = 1ULL;
  }

  if (ticks > 127ULL)
  {
    ticks = 127ULL;
  }

  return (uint8_t)ticks;
}

static uint32_t PwmBrake_ChannelToTimChannel(uint8_t channel_index)
{
  static const uint32_t s_tim_channels[PWM_BRAKE_CHANNEL_COUNT] =
  {
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3
  };

  return s_tim_channels[channel_index];
}

static void PwmBrake_ApplyDuty(void)
{
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PwmBrake_DutyToCompare(s_duty_permille[0]));
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PwmBrake_DutyToCompare(s_duty_permille[1]));
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, PwmBrake_DutyToCompare(s_duty_permille[2]));
}

static uint8_t PwmBrake_ApplyDefaultDeadTime(void)
{
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = PwmBrake_DeadTimeCodeFromNs(PWM_BRAKE_DEFAULT_DEADTIME_NS);
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;

  return (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) == HAL_OK) ? 1U : 0U;
}

void PwmBrake_Init(void)
{
  (void)PwmBrake_ApplyDefaultDeadTime();
  PwmBrake_ApplyDuty();
  s_running_mask = 0U;
}

void PwmBrake_Start(void)
{
  PwmBrake_StartChannel(0U);
  PwmBrake_StartChannel(1U);
  PwmBrake_StartChannel(2U);
}

void PwmBrake_Stop(void)
{
  PwmBrake_StopChannel(0U);
  PwmBrake_StopChannel(1U);
  PwmBrake_StopChannel(2U);
}

void PwmBrake_StartChannel(uint8_t channel_index)
{
  uint32_t tim_channel;
  uint8_t channel_mask;

  if (channel_index >= PWM_BRAKE_CHANNEL_COUNT)
  {
    return;
  }

  channel_mask = (uint8_t)(1U << channel_index);
  if ((s_running_mask & channel_mask) != 0U)
  {
    return;
  }

  tim_channel = PwmBrake_ChannelToTimChannel(channel_index);
  PwmBrake_ApplyDuty();

  if (PwmBrake_ApplyDefaultDeadTime() == 0U)
  {
    return;
  }

  if (HAL_TIM_PWM_Start(&htim8, tim_channel) != HAL_OK)
  {
    return;
  }

  if (HAL_TIMEx_PWMN_Start(&htim8, tim_channel) != HAL_OK)
  {
    (void)HAL_TIM_PWM_Stop(&htim8, tim_channel);
    return;
  }

  __HAL_TIM_MOE_ENABLE(&htim8);
  s_running_mask = (uint8_t)(s_running_mask | channel_mask);
}

void PwmBrake_StopChannel(uint8_t channel_index)
{
  uint32_t tim_channel;
  uint8_t channel_mask;

  if (channel_index >= PWM_BRAKE_CHANNEL_COUNT)
  {
    return;
  }

  channel_mask = (uint8_t)(1U << channel_index);
  if ((s_running_mask & channel_mask) == 0U)
  {
    return;
  }

  tim_channel = PwmBrake_ChannelToTimChannel(channel_index);

  (void)HAL_TIMEx_PWMN_Stop(&htim8, tim_channel);
  (void)HAL_TIM_PWM_Stop(&htim8, tim_channel);
  __HAL_TIM_SET_COMPARE(&htim8, tim_channel, 0U);
  s_running_mask = (uint8_t)(s_running_mask & (uint8_t)(~channel_mask));

  if (s_running_mask == 0U)
  {
    __HAL_TIM_MOE_DISABLE(&htim8);
  }
}

void PwmBrake_SetDutyPermille(uint16_t duty_permille)
{
  PwmBrake_SetAllDutyPermille(duty_permille, duty_permille, duty_permille);
}

void PwmBrake_SetAllDutyPermille(uint16_t duty_ch1_permille, uint16_t duty_ch2_permille, uint16_t duty_ch3_permille)
{
  s_duty_permille[0] = PwmBrake_ClampDuty(duty_ch1_permille);
  s_duty_permille[1] = PwmBrake_ClampDuty(duty_ch2_permille);
  s_duty_permille[2] = PwmBrake_ClampDuty(duty_ch3_permille);

  PwmBrake_ApplyDuty();
}

uint8_t PwmBrake_IsRunning(void)
{
  return (s_running_mask != 0U) ? 1U : 0U;
}

uint8_t PwmBrake_IsChannelRunning(uint8_t channel_index)
{
  if (channel_index >= PWM_BRAKE_CHANNEL_COUNT)
  {
    return 0U;
  }

  return ((s_running_mask & (uint8_t)(1U << channel_index)) != 0U) ? 1U : 0U;
}

uint16_t PwmBrake_GetDutyPermilleByChannel(uint8_t channel_index)
{
  if (channel_index >= PWM_BRAKE_CHANNEL_COUNT)
  {
    return 0U;
  }

  return s_duty_permille[channel_index];
}

uint16_t PwmBrake_GetDutyPermille(void)
{
  return s_duty_permille[0];
}

uint16_t PwmBrake_GetDefaultDutyPermille(void)
{
  return PWM_BRAKE_DEFAULT_DUTY_PERMILLE;
}
