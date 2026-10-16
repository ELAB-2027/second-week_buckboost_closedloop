#ifndef __POWER_ADC_H__
#define __POWER_ADC_H__

#include "main.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define POWER_ADC_CHANNEL_COUNT 6U

/* Ranks 0..2 are converted by ADC1 on the HRTIM ADC trigger 1 event and landed
   in memory by DMA; indices 3..5 are software-polled monitors. The differential
   selection is per ADC channel, so PA2 is only ever scanned as IN3(+)/IN4(-). */
#define POWER_ADC_CONTROL_RANK_COUNT 3U

#define POWER_ADC_PIN_VOLTAGE_SCALE (3.3f / 4095.0f)


#define POWER_ADC_LOAD_OUTPUT_VOLTAGE_SCALE (3.3f / 4095.0f)
#define POWER_ADC_LOAD_OFFSET_VOLTAGE 1.649116522f
#define POWER_ADC_LOAD_REAL_GAIN 39.95525476f

#define POWER_ADC_AUX_OUTPUT_VOLTAGE_SCALE (3.3f / 4095.0f)
#define POWER_ADC_AUX_OFFSET_VOLTAGE 1.6498307f
#define POWER_ADC_AUX_REAL_GAIN 41.123049f


#define POWER_ADC_CURRENT_LINEAR_A 0.718837f
#define POWER_ADC_CURRENT_LINEAR_B (-0.5638816f)

typedef enum
{
  POWER_ADC_CHANNEL_ADC1_IN1 = 0,
  POWER_ADC_CHANNEL_ADC1_IN3_4_DIFF,
  POWER_ADC_CHANNEL_ADC1_IN9,
  POWER_ADC_CHANNEL_ADC3_IN1,
  POWER_ADC_CHANNEL_ADC3_IN12,
  POWER_ADC_CHANNEL_ADC2_IN12
} PowerAdcChannel;

typedef enum
{
  POWER_ADC_SIGNAL_LOAD_OUTPUT_VOLTAGE = 0,
  POWER_ADC_SIGNAL_LOAD_VOLTAGE,
  POWER_ADC_SIGNAL_AUX_OUTPUT_VOLTAGE,
  POWER_ADC_SIGNAL_AUX_VOLTAGE,
  POWER_ADC_SIGNAL_CURRENT_SENSE_OUTPUT_VOLTAGE,
  POWER_ADC_SIGNAL_CURRENT_SENSE_CURRENT,
  POWER_ADC_SIGNAL_COUNT
} PowerAdcSignal;

#define POWER_ADC_CHANNEL_LOAD_DIFF POWER_ADC_CHANNEL_ADC1_IN1
#define POWER_ADC_CHANNEL_AUX_VOLTAGE POWER_ADC_CHANNEL_ADC1_IN3_4_DIFF
#define POWER_ADC_CHANNEL_CURRENT_SENSE POWER_ADC_CHANNEL_ADC1_IN9

typedef struct
{
  uint16_t raw;
  float pin_voltage;
} PowerAdcSample;

HAL_StatusTypeDef PowerAdc_Init(void);
HAL_StatusTypeDef PowerAdc_SampleAll(void);
uint32_t PowerAdc_GetSequenceCounter(void);
uint16_t PowerAdc_GetRaw(PowerAdcChannel channel);
float PowerAdc_GetPinVoltage(PowerAdcChannel channel);
float PowerAdc_GetDisplaySignal(PowerAdcSignal signal);
float PowerAdc_GetLoadDiffOutputVoltage(void);
float PowerAdc_GetLoadDiffVoltage(void);
float PowerAdc_GetAuxOutputVoltage(void);
float PowerAdc_GetAuxVoltage(void);
float PowerAdc_GetCurrentSenseOutputVoltage(void);
float PowerAdc_GetCurrentSenseCurrent(void);
const PowerAdcSample *PowerAdc_GetSamples(void);

#ifdef __cplusplus
}
#endif

#endif
