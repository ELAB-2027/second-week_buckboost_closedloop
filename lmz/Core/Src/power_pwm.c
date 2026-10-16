#include "power_pwm.h"

#include "hrtim.h"

#define POWER_PWM_FHRTIM_HZ 170000000UL
#define POWER_PWM_DLL_MULTIPLIER 32UL
#define POWER_PWM_DTG_MULTIPLIER 2UL
#define POWER_PWM_NS_SCALE 1000000000UL

#define POWER_PWM_CARRIER_COUNTS \
  ((uint32_t)(((uint64_t)POWER_PWM_FHRTIM_HZ * POWER_PWM_DLL_MULTIPLIER) / POWER_PWM_FREQ_HZ))
#define POWER_PWM_CARRIER_PERIOD (POWER_PWM_CARRIER_COUNTS - 1UL)

#define POWER_PWM_DEADTIME_COUNTS \
  ((uint32_t)(((uint64_t)POWER_PWM_FHRTIM_HZ * POWER_PWM_DTG_MULTIPLIER * POWER_PWM_DEADTIME_NS) \
              / POWER_PWM_NS_SCALE))

#define POWER_PWM_MIN_COMPARE 1UL
#define POWER_PWM_COUNTER_BIT_WIDTH 16UL

typedef struct
{
  uint32_t timer_index;
  uint32_t timer_id;
  uint32_t high_output;
  uint32_t low_output;
  uint32_t output_mask;
  uint32_t delayed_protection;
} PowerPwmTimerMap;

static const PowerPwmTimerMap s_timer_map[POWER_PWM_CHANNEL_COUNT] =
{
  {HRTIM_TIMERINDEX_TIMER_C, HRTIM_TIMERID_TIMER_C,
   HRTIM_OUTPUT_TC1, HRTIM_OUTPUT_TC2, HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2,
   HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED},
  {HRTIM_TIMERINDEX_TIMER_D, HRTIM_TIMERID_TIMER_D,
   HRTIM_OUTPUT_TD1, HRTIM_OUTPUT_TD2, HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2,
   HRTIM_TIMER_D_E_DELAYEDPROTECTION_DISABLED},
  {HRTIM_TIMERINDEX_TIMER_F, HRTIM_TIMERID_TIMER_F,
   HRTIM_OUTPUT_TF1, HRTIM_OUTPUT_TF2, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2,
   HRTIM_TIMER_F_DELAYEDPROTECTION_DISABLED}
};

static float s_duty[POWER_PWM_CHANNEL_COUNT];
static uint8_t s_running_mask = 0U;
static uint8_t s_initialized = 0U;

static float PowerPwm_ClampDuty(float duty)
{
  if (duty < 0.0f)
  {
    return 0.0f;
  }

  if (duty > POWER_PWM_MAX_DUTY)
  {
    return POWER_PWM_MAX_DUTY;
  }

  return duty;
}

static uint32_t PowerPwm_DutyToCompare(uint32_t timer_index, float duty)
{
  uint32_t period;
  uint32_t compare;

  period = __HAL_HRTIM_GETPERIOD(&hhrtim1, timer_index);
  duty = PowerPwm_ClampDuty(duty);
  compare = (uint32_t)((float)period * duty);

  if (compare < POWER_PWM_MIN_COMPARE)
  {
    compare = POWER_PWM_MIN_COMPARE;
  }

  if (compare > period)
  {
    compare = period;
  }

  return compare;
}

static HAL_StatusTypeDef PowerPwm_ConfigureTimebase(void)
{
  HRTIM_TimeBaseCfgTypeDef time_base = {0};
  uint32_t i;

  if ((POWER_PWM_CARRIER_COUNTS == 0UL) || (POWER_PWM_CARRIER_COUNTS > (1UL << POWER_PWM_COUNTER_BIT_WIDTH)))
  {
    return HAL_ERROR;
  }

  time_base.Period = POWER_PWM_CARRIER_PERIOD;
  time_base.RepetitionCounter = 0U;
  time_base.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL32;
  time_base.Mode = HRTIM_MODE_CONTINUOUS;

  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &time_base) != HAL_OK)
  {
    return HAL_ERROR;
  }

  for (i = 0U; i < POWER_PWM_CHANNEL_COUNT; i++)
  {
    if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, s_timer_map[i].timer_index, &time_base) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}

static HAL_StatusTypeDef PowerPwm_ConfigureTimer(uint32_t timer_index, uint32_t delayed_protection)
{
  HRTIM_TimerCfgTypeDef timer_cfg = {0};
  HRTIM_TimerCtlTypeDef timer_ctl = {0};
  HRTIM_DeadTimeCfgTypeDef dead_time = {0};

  timer_ctl.UpDownMode = HRTIM_TIMERUPDOWNMODE_UP;
  timer_ctl.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, timer_index, &timer_ctl) != HAL_OK)
  {
    return HAL_ERROR;
  }

  timer_cfg.InterruptRequests = HRTIM_TIM_IT_NONE;
  timer_cfg.DMARequests = HRTIM_TIM_DMA_NONE;
  timer_cfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  timer_cfg.InterleavedMode = HRTIM_INTERLEAVED_MODE_DISABLED;
  timer_cfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  timer_cfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  timer_cfg.DACSynchro = HRTIM_DACSYNC_NONE;
  timer_cfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  timer_cfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  timer_cfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  timer_cfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
  timer_cfg.ReSyncUpdate = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL;
  timer_cfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  timer_cfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  timer_cfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  timer_cfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
  timer_cfg.DelayedProtectionMode = delayed_protection;
  timer_cfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
  timer_cfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
  timer_cfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, timer_index, &timer_cfg) != HAL_OK)
  {
    return HAL_ERROR;
  }

  dead_time.Prescaler = HRTIM_TIMDEADTIME_PRESCALERRATIO_MUL2;
  dead_time.RisingValue = POWER_PWM_DEADTIME_COUNTS;
  dead_time.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
  dead_time.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
  dead_time.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
  /* Dead band from the rising delay line only: with the falling value at 0 each
     output still drops out at its reference edge and switches on one dead time
     later, so the pair cannot overlap. Loading both delay lines positively can
     degenerate into a pure waveform shift instead.
   */
  dead_time.FallingValue = 0U;
  dead_time.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
  dead_time.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
  dead_time.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, timer_index, &dead_time) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef PowerPwm_ConfigureOutputs(const PowerPwmTimerMap *map, float duty)
{
  HRTIM_CompareCfgTypeDef compare_cfg = {0};
  HRTIM_OutputCfgTypeDef output_cfg = {0};

  compare_cfg.CompareValue = PowerPwm_DutyToCompare(map->timer_index, duty);
  compare_cfg.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  compare_cfg.AutoDelayedTimeout = 0U;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, map->timer_index, HRTIM_COMPAREUNIT_1, &compare_cfg) != HAL_OK)
  {
    return HAL_ERROR;
  }

  output_cfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  output_cfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  output_cfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  output_cfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  output_cfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  output_cfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;

  /* High side: active from the start of the cycle until compare 1. */
  output_cfg.SetSource = HRTIM_OUTPUTSET_TIMPER;
  output_cfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, map->timer_index, map->high_output, &output_cfg) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Low side: the plain complement of the high side, the dead-time generator
   * then re-times both rising edges.
   */
  output_cfg.SetSource = HRTIM_OUTPUTSET_TIMCMP1;
  output_cfg.ResetSource = HRTIM_OUTPUTRESET_TIMPER;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, map->timer_index, map->low_output, &output_cfg) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef PowerPwm_ConfigureAdcTrigger(void)
{
  HRTIM_ADCTriggerCfgTypeDef adc_trigger = {0};

  adc_trigger.UpdateSource = HRTIM_ADCTRIGGERUPDATE_MASTER;
  adc_trigger.Trigger = HRTIM_ADCTRIGGEREVENT13_MASTER_PERIOD;

  return HAL_HRTIM_ADCTriggerConfig(&hhrtim1, POWER_PWM_ADC_TRIGGER, &adc_trigger);
}

HAL_StatusTypeDef PowerPwm_Init(void)
{
  uint32_t i;
  uint32_t timer_ids = HRTIM_TIMERID_MASTER;

  if (PowerPwm_ConfigureTimebase() != HAL_OK)
  {
    return HAL_ERROR;
  }

  for (i = 0U; i < POWER_PWM_CHANNEL_COUNT; i++)
  {
    if (PowerPwm_ConfigureTimer(s_timer_map[i].timer_index, s_timer_map[i].delayed_protection) != HAL_OK)
    {
      return HAL_ERROR;
    }

    if (PowerPwm_ConfigureOutputs(&s_timer_map[i], POWER_PWM_DEFAULT_DUTY) != HAL_OK)
    {
      return HAL_ERROR;
    }

    s_duty[i] = POWER_PWM_DEFAULT_DUTY;
    timer_ids |= s_timer_map[i].timer_id;
  }

  if (PowerPwm_ConfigureAdcTrigger() != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Counters run from init on: the ADC trigger has to keep sampling while the
   * outputs are stopped, otherwise the loops and the protection lose feedback.
   */
  if (HAL_HRTIM_WaveformCountStart(&hhrtim1, timer_ids) != HAL_OK)
  {
    return HAL_ERROR;
  }

  s_running_mask = 0U;
  s_initialized = 1U;
  return HAL_OK;
}

HAL_StatusTypeDef PowerPwm_SetDuty(PowerPwmChannel channel, float duty)
{
  if ((s_initialized == 0U) || ((uint32_t)channel >= POWER_PWM_CHANNEL_COUNT))
  {
    return HAL_ERROR;
  }

  duty = PowerPwm_ClampDuty(duty);
  s_duty[channel] = duty;

  __HAL_HRTIM_SETCOMPARE(&hhrtim1, s_timer_map[channel].timer_index, HRTIM_COMPAREUNIT_1,
                         PowerPwm_DutyToCompare(s_timer_map[channel].timer_index, duty));
  return HAL_OK;
}

void PowerPwm_SetDutyAll(float duty)
{
  uint32_t i;

  for (i = 0U; i < POWER_PWM_CHANNEL_COUNT; i++)
  {
    (void)PowerPwm_SetDuty((PowerPwmChannel)i, duty);
  }
}

float PowerPwm_GetDuty(PowerPwmChannel channel)
{
  if ((uint32_t)channel >= POWER_PWM_CHANNEL_COUNT)
  {
    return 0.0f;
  }

  return s_duty[channel];
}

HAL_StatusTypeDef PowerPwm_StartChannel(PowerPwmChannel channel)
{
  if ((s_initialized == 0U) || ((uint32_t)channel >= POWER_PWM_CHANNEL_COUNT))
  {
    return HAL_ERROR;
  }

  if ((s_running_mask & (uint8_t)(1U << channel)) != 0U)
  {
    return HAL_OK;
  }

  (void)PowerPwm_SetDuty(channel, s_duty[channel]);

  if (HAL_HRTIM_WaveformOutputStart(&hhrtim1, s_timer_map[channel].output_mask) != HAL_OK)
  {
    return HAL_ERROR;
  }

  s_running_mask = (uint8_t)(s_running_mask | (uint8_t)(1U << channel));
  return HAL_OK;
}

void PowerPwm_StartAll(void)
{
  uint32_t i;

  for (i = 0U; i < POWER_PWM_CHANNEL_COUNT; i++)
  {
    (void)PowerPwm_StartChannel((PowerPwmChannel)i);
  }
}

HAL_StatusTypeDef PowerPwm_StopChannel(PowerPwmChannel channel)
{
  if ((s_initialized == 0U) || ((uint32_t)channel >= POWER_PWM_CHANNEL_COUNT))
  {
    return HAL_ERROR;
  }

  if (HAL_HRTIM_WaveformOutputStop(&hhrtim1, s_timer_map[channel].output_mask) != HAL_OK)
  {
    return HAL_ERROR;
  }

  s_running_mask = (uint8_t)(s_running_mask & (uint8_t)(~(1U << channel)));
  return HAL_OK;
}

void PowerPwm_StopAll(void)
{
  uint32_t i;

  for (i = 0U; i < POWER_PWM_CHANNEL_COUNT; i++)
  {
    (void)PowerPwm_StopChannel((PowerPwmChannel)i);
  }
}

uint8_t PowerPwm_IsRunning(void)
{
  return (s_running_mask != 0U) ? 1U : 0U;
}

uint8_t PowerPwm_IsChannelRunning(PowerPwmChannel channel)
{
  if ((uint32_t)channel >= POWER_PWM_CHANNEL_COUNT)
  {
    return 0U;
  }

  return ((s_running_mask & (uint8_t)(1U << channel)) != 0U) ? 1U : 0U;
}
