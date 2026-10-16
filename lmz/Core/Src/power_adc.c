#include "power_adc.h"

#include "adc.h"

#include <stddef.h>
#include <string.h>

#define POWER_ADC_MEDIAN_SAMPLE_COUNT 3U
#define POWER_ADC_CONTROL_FILTER_DECIMATION 10U
#define POWER_ADC_MONITOR_COUNT (POWER_ADC_CHANNEL_COUNT - POWER_ADC_CONTROL_RANK_COUNT)
#define POWER_ADC_MONITOR_TIMEOUT_MS 10U
#define POWER_ADC_CONTROL_SAMPLING_TIME ADC_SAMPLETIME_47CYCLES_5
#define POWER_ADC_MONITOR_SAMPLING_TIME ADC_SAMPLETIME_92CYCLES_5
#define POWER_ADC_CONTROL_LPF_ALPHA 0.3f
#define POWER_ADC_DISPLAY_LPF_ALPHA 0.12f
#define POWER_ADC_MONITOR_MIN_INTERVAL_MS 50U

typedef struct
{
  ADC_HandleTypeDef *hadc;
  uint32_t channel;
  uint32_t single_diff;
  uint32_t rank;
} PowerAdcControlRank;

typedef struct
{
  ADC_HandleTypeDef *hadc;
  uint32_t channel;
  uint32_t single_diff;
} PowerAdcConfig;

typedef struct
{
  float stage1;
  float stage2;
  uint8_t initialized;
} PowerAdcFilterState;

typedef struct
{
  float value;
  uint8_t initialized;
} PowerAdcDisplayFilterState;

/* Rank order matches PowerAdcChannel 0..2 so the DMA buffer index is the channel index. */
static const PowerAdcControlRank s_control_ranks[POWER_ADC_CONTROL_RANK_COUNT] =
{
  {&hadc1, ADC_CHANNEL_1, ADC_DIFFERENTIAL_ENDED, ADC_REGULAR_RANK_1},
  {&hadc1, ADC_CHANNEL_3, ADC_DIFFERENTIAL_ENDED, ADC_REGULAR_RANK_2},
  {&hadc1, ADC_CHANNEL_9, ADC_SINGLE_ENDED, ADC_REGULAR_RANK_3}
};

static const PowerAdcConfig s_monitor_config[POWER_ADC_MONITOR_COUNT] =
{
  {&hadc3, ADC_CHANNEL_1, ADC_SINGLE_ENDED},
  {&hadc3, ADC_CHANNEL_12, ADC_SINGLE_ENDED},
  {&hadc2, ADC_CHANNEL_12, ADC_SINGLE_ENDED}
};

static uint16_t s_dma_raw[POWER_ADC_CONTROL_RANK_COUNT];
static PowerAdcSample s_control_samples[POWER_ADC_CONTROL_RANK_COUNT];
static PowerAdcSample s_samples[POWER_ADC_CHANNEL_COUNT];
static PowerAdcFilterState s_control_filter_state[POWER_ADC_CONTROL_RANK_COUNT];
static PowerAdcDisplayFilterState s_display_filter_state[POWER_ADC_SIGNAL_COUNT];
static uint16_t s_median_history[POWER_ADC_CONTROL_RANK_COUNT][POWER_ADC_MEDIAN_SAMPLE_COUNT];
static uint8_t s_median_slot = 0U;
static uint8_t s_median_filled = 0U;
static uint8_t s_filter_prescaler = 0U;
static volatile uint32_t s_sequence_counter = 0U;
static uint32_t s_last_sequence_taken = 0U;
static uint32_t s_last_monitor_tick = 0U;
static uint8_t s_initialized = 0U;

static uint16_t PowerAdc_Median3(uint16_t a, uint16_t b, uint16_t c)
{
  if (a > b)
  {
    uint16_t tmp = a;
    a = b;
    b = tmp;
  }

  if (b > c)
  {
    uint16_t tmp = b;
    b = c;
    c = tmp;
  }

  if (a > b)
  {
    b = a;
  }

  return b;
}

static uint16_t PowerAdc_MedianOfHistory(uint32_t channel_index, uint16_t latest)
{
  s_median_history[channel_index][s_median_slot] = latest;

  if (s_median_filled < POWER_ADC_MEDIAN_SAMPLE_COUNT)
  {
    s_median_filled++;
    return latest;
  }

  return PowerAdc_Median3(s_median_history[channel_index][0],
                          s_median_history[channel_index][1],
                          s_median_history[channel_index][2]);
}

static uint16_t PowerAdc_ApplyControlLowPass(uint32_t channel_index, uint16_t raw)
{
  float input;
  float stage1;
  float stage2;

  if (channel_index >= POWER_ADC_CONTROL_RANK_COUNT)
  {
    return raw;
  }

  input = (float)raw;
  if (s_control_filter_state[channel_index].initialized == 0U)
  {
    s_control_filter_state[channel_index].stage1 = input;
    s_control_filter_state[channel_index].stage2 = input;
    s_control_filter_state[channel_index].initialized = 1U;
    return raw;
  }

  stage1 = s_control_filter_state[channel_index].stage1
           + POWER_ADC_CONTROL_LPF_ALPHA * (input - s_control_filter_state[channel_index].stage1);
  stage2 = s_control_filter_state[channel_index].stage2
           + POWER_ADC_CONTROL_LPF_ALPHA * (stage1 - s_control_filter_state[channel_index].stage2);
  s_control_filter_state[channel_index].stage1 = stage1;
  s_control_filter_state[channel_index].stage2 = stage2;

  if (stage2 <= 0.0f)
  {
    return 0U;
  }

  if (stage2 >= 4095.0f)
  {
    return 4095U;
  }

  return (uint16_t)(stage2 + 0.5f);
}

static float PowerAdc_GetLoadDiffOutputVoltageFromRaw(uint16_t raw)
{
  return (float)raw * POWER_ADC_LOAD_OUTPUT_VOLTAGE_SCALE;
}

static float PowerAdc_GetLoadDiffVoltageFromRaw(uint16_t raw)
{
  return (PowerAdc_GetLoadDiffOutputVoltageFromRaw(raw) - POWER_ADC_LOAD_OFFSET_VOLTAGE)
         * POWER_ADC_LOAD_REAL_GAIN;
}

static float PowerAdc_GetAuxOutputVoltageFromRaw(uint16_t raw)
{
  return (float)raw * POWER_ADC_AUX_OUTPUT_VOLTAGE_SCALE;
}

static float PowerAdc_GetAuxVoltageFromRaw(uint16_t raw)
{
  return (PowerAdc_GetAuxOutputVoltageFromRaw(raw) - POWER_ADC_AUX_OFFSET_VOLTAGE)
         * POWER_ADC_AUX_REAL_GAIN;
}

static float PowerAdc_GetCurrentSenseOutputVoltageFromRaw(uint16_t raw)
{
  return (float)raw * POWER_ADC_PIN_VOLTAGE_SCALE;
}

static float PowerAdc_GetCurrentSenseCurrentFromRaw(uint16_t raw)
{
  return POWER_ADC_CURRENT_LINEAR_A * PowerAdc_GetCurrentSenseOutputVoltageFromRaw(raw)
         + POWER_ADC_CURRENT_LINEAR_B;
}

static float PowerAdc_ApplyDisplayLowPass(PowerAdcSignal signal, float value)
{
  if ((uint32_t)signal >= POWER_ADC_SIGNAL_COUNT)
  {
    return value;
  }

  if (s_display_filter_state[signal].initialized == 0U)
  {
    s_display_filter_state[signal].value = value;
    s_display_filter_state[signal].initialized = 1U;
  }
  else
  {
    s_display_filter_state[signal].value += POWER_ADC_DISPLAY_LPF_ALPHA * (value - s_display_filter_state[signal].value);
  }

  return s_display_filter_state[signal].value;
}

static HAL_StatusTypeDef PowerAdc_ConfigureControlSequence(void)
{
  ADC_ChannelConfTypeDef rank_cfg = {0};
  uint32_t i;

  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = POWER_ADC_CONTROL_RANK_COUNT;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_HRTIM_TRG1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;

  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  for (i = 0U; i < POWER_ADC_CONTROL_RANK_COUNT; i++)
  {
    rank_cfg.Channel = s_control_ranks[i].channel;
    rank_cfg.Rank = s_control_ranks[i].rank;
    rank_cfg.SamplingTime = POWER_ADC_CONTROL_SAMPLING_TIME;
    rank_cfg.SingleDiff = s_control_ranks[i].single_diff;
    rank_cfg.OffsetNumber = ADC_OFFSET_NONE;
    rank_cfg.Offset = 0;

    if (HAL_ADC_ConfigChannel(&hadc1, &rank_cfg) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}

static HAL_StatusTypeDef PowerAdc_ReadOnce(const PowerAdcConfig *config, uint16_t *value)
{
  ADC_ChannelConfTypeDef rank_cfg = {0};
  HAL_StatusTypeDef status;

  if ((config == NULL) || (value == NULL))
  {
    return HAL_ERROR;
  }

  rank_cfg.Channel = config->channel;
  rank_cfg.Rank = ADC_REGULAR_RANK_1;
  rank_cfg.SamplingTime = POWER_ADC_MONITOR_SAMPLING_TIME;
  rank_cfg.SingleDiff = config->single_diff;
  rank_cfg.OffsetNumber = ADC_OFFSET_NONE;
  rank_cfg.Offset = 0;

  status = HAL_ADC_ConfigChannel(config->hadc, &rank_cfg);
  if (status != HAL_OK)
  {
    return status;
  }

  status = HAL_ADC_Start(config->hadc);
  if (status != HAL_OK)
  {
    return status;
  }

  status = HAL_ADC_PollForConversion(config->hadc, POWER_ADC_MONITOR_TIMEOUT_MS);
  if (status == HAL_OK)
  {
    *value = (uint16_t)HAL_ADC_GetValue(config->hadc);
  }

  (void)HAL_ADC_Stop(config->hadc);
  return status;
}

static HAL_StatusTypeDef PowerAdc_SampleMonitors(void)
{
  uint32_t i;
  uint32_t j;
  uint16_t history[POWER_ADC_MEDIAN_SAMPLE_COUNT];
  HAL_StatusTypeDef status;

  for (i = 0U; i < POWER_ADC_MONITOR_COUNT; i++)
  {
    for (j = 0U; j < POWER_ADC_MEDIAN_SAMPLE_COUNT; j++)
    {
      status = PowerAdc_ReadOnce(&s_monitor_config[i], &history[j]);
      if (status != HAL_OK)
      {
        return status;
      }
    }

    s_samples[POWER_ADC_CONTROL_RANK_COUNT + i].raw = PowerAdc_Median3(history[0], history[1], history[2]);
    s_samples[POWER_ADC_CONTROL_RANK_COUNT + i].pin_voltage =
      (float)s_samples[POWER_ADC_CONTROL_RANK_COUNT + i].raw * POWER_ADC_PIN_VOLTAGE_SCALE;
  }

  return HAL_OK;
}

static void PowerAdc_UpdateControlSnapshot(void)
{
  uint32_t i;

  __disable_irq();
  for (i = 0U; i < POWER_ADC_CONTROL_RANK_COUNT; i++)
  {
    s_samples[i] = s_control_samples[i];
  }
  __enable_irq();
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  uint32_t i;
  uint16_t median;

  if (hadc->Instance != ADC1)
  {
    return;
  }

  s_sequence_counter++;

  /* Filtering every carrier cycle would keep the interrupt busy for a sizeable
     part of the 10 us period; 10 kHz is still far above the control rate. */
  s_filter_prescaler++;
  if (s_filter_prescaler < POWER_ADC_CONTROL_FILTER_DECIMATION)
  {
    return;
  }

  s_filter_prescaler = 0U;

  for (i = 0U; i < POWER_ADC_CONTROL_RANK_COUNT; i++)
  {
    median = PowerAdc_MedianOfHistory(i, s_dma_raw[i]);
    s_control_samples[i].raw = PowerAdc_ApplyControlLowPass(i, median);
    s_control_samples[i].pin_voltage = (float)s_control_samples[i].raw * POWER_ADC_PIN_VOLTAGE_SCALE;
  }

  s_median_slot = (uint8_t)((s_median_slot + 1U) % POWER_ADC_MEDIAN_SAMPLE_COUNT);
}

HAL_StatusTypeDef PowerAdc_Init(void)
{
  HAL_StatusTypeDef status;

  (void)memset(s_control_filter_state, 0, sizeof(s_control_filter_state));
  (void)memset(s_display_filter_state, 0, sizeof(s_display_filter_state));
  (void)memset(s_median_history, 0, sizeof(s_median_history));
  s_median_slot = 0U;
  s_median_filled = 0U;
  s_filter_prescaler = 0U;
  s_sequence_counter = 0U;
  s_last_sequence_taken = 0U;

  /* Requires PowerPwm_Init() to have armed HRTIM ADC trigger 1 already. */
  if (PowerAdc_ConfigureControlSequence() != HAL_OK)
  {
    return HAL_ERROR;
  }

  status = HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  if (status != HAL_OK)
  {
    return status;
  }

  status = HAL_ADCEx_Calibration_Start(&hadc1, ADC_DIFFERENTIAL_ENDED);
  if (status != HAL_OK)
  {
    return status;
  }

  status = HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  if (status != HAL_OK)
  {
    return status;
  }

  status = HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
  if (status != HAL_OK)
  {
    return status;
  }

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&s_dma_raw[0], POWER_ADC_CONTROL_RANK_COUNT) != HAL_OK)
  {
    return HAL_ERROR;
  }

  s_initialized = 1U;
  s_last_monitor_tick = HAL_GetTick();
  return PowerAdc_SampleMonitors();
}

HAL_StatusTypeDef PowerAdc_SampleAll(void)
{
  uint32_t sequence;

  if (s_initialized == 0U)
  {
    return HAL_ERROR;
  }

  sequence = s_sequence_counter;
  if (sequence == s_last_sequence_taken)
  {
    /* HRTIM trigger or DMA stopped firing: keep the previous snapshot but report it. */
    return HAL_ERROR;
  }
  s_last_sequence_taken = sequence;

  PowerAdc_UpdateControlSnapshot();

  if ((HAL_GetTick() - s_last_monitor_tick) >= POWER_ADC_MONITOR_MIN_INTERVAL_MS)
  {
    s_last_monitor_tick = HAL_GetTick();
    return PowerAdc_SampleMonitors();
  }

  return HAL_OK;
}

uint32_t PowerAdc_GetSequenceCounter(void)
{
  return s_sequence_counter;
}

uint16_t PowerAdc_GetRaw(PowerAdcChannel channel)
{
  if ((uint32_t)channel >= POWER_ADC_CHANNEL_COUNT)
  {
    return 0U;
  }

  return s_samples[channel].raw;
}

float PowerAdc_GetPinVoltage(PowerAdcChannel channel)
{
  if ((uint32_t)channel >= POWER_ADC_CHANNEL_COUNT)
  {
    return 0.0f;
  }

  return s_samples[channel].pin_voltage;
}

float PowerAdc_GetDisplaySignal(PowerAdcSignal signal)
{
  switch (signal)
  {
    case POWER_ADC_SIGNAL_LOAD_OUTPUT_VOLTAGE:
      return PowerAdc_ApplyDisplayLowPass(signal,
                                          PowerAdc_GetLoadDiffOutputVoltageFromRaw(PowerAdc_GetRaw(POWER_ADC_CHANNEL_LOAD_DIFF)));

    case POWER_ADC_SIGNAL_LOAD_VOLTAGE:
      return PowerAdc_ApplyDisplayLowPass(signal,
                                          PowerAdc_GetLoadDiffVoltageFromRaw(PowerAdc_GetRaw(POWER_ADC_CHANNEL_LOAD_DIFF)));

    case POWER_ADC_SIGNAL_AUX_OUTPUT_VOLTAGE:
      return PowerAdc_ApplyDisplayLowPass(signal,
                                          PowerAdc_GetAuxOutputVoltageFromRaw(PowerAdc_GetRaw(POWER_ADC_CHANNEL_AUX_VOLTAGE)));

    case POWER_ADC_SIGNAL_AUX_VOLTAGE:
      return PowerAdc_ApplyDisplayLowPass(signal,
                                          PowerAdc_GetAuxVoltageFromRaw(PowerAdc_GetRaw(POWER_ADC_CHANNEL_AUX_VOLTAGE)));

    case POWER_ADC_SIGNAL_CURRENT_SENSE_OUTPUT_VOLTAGE:
      return PowerAdc_ApplyDisplayLowPass(signal,
                                          PowerAdc_GetCurrentSenseOutputVoltageFromRaw(PowerAdc_GetRaw(POWER_ADC_CHANNEL_CURRENT_SENSE)));

    case POWER_ADC_SIGNAL_CURRENT_SENSE_CURRENT:
      return PowerAdc_ApplyDisplayLowPass(signal,
                                          PowerAdc_GetCurrentSenseCurrentFromRaw(PowerAdc_GetRaw(POWER_ADC_CHANNEL_CURRENT_SENSE)));

    default:
      return 0.0f;
  }
}

float PowerAdc_GetLoadDiffOutputVoltage(void)
{
  return PowerAdc_GetLoadDiffOutputVoltageFromRaw(PowerAdc_GetRaw(POWER_ADC_CHANNEL_LOAD_DIFF));
}

float PowerAdc_GetLoadDiffVoltage(void)
{
  return PowerAdc_GetLoadDiffVoltageFromRaw(PowerAdc_GetRaw(POWER_ADC_CHANNEL_LOAD_DIFF));
}

float PowerAdc_GetAuxOutputVoltage(void)
{
  return PowerAdc_GetAuxOutputVoltageFromRaw(PowerAdc_GetRaw(POWER_ADC_CHANNEL_AUX_VOLTAGE));
}

float PowerAdc_GetAuxVoltage(void)
{
  return PowerAdc_GetAuxVoltageFromRaw(PowerAdc_GetRaw(POWER_ADC_CHANNEL_AUX_VOLTAGE));
}

float PowerAdc_GetCurrentSenseOutputVoltage(void)
{
  return PowerAdc_GetCurrentSenseOutputVoltageFromRaw(PowerAdc_GetRaw(POWER_ADC_CHANNEL_CURRENT_SENSE));
}

float PowerAdc_GetCurrentSenseCurrent(void)
{
  return PowerAdc_GetCurrentSenseCurrentFromRaw(PowerAdc_GetRaw(POWER_ADC_CHANNEL_CURRENT_SENSE));
}

const PowerAdcSample *PowerAdc_GetSamples(void)
{
  return s_samples;
}
