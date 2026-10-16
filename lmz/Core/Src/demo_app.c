#include "demo_app.h"

#include "encoder.h"
#include "jdy31.h"
#include "oled.h"
#include "power_adc.h"
#include "power_control.h"
#include "power_pwm.h"
#include "usart.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define APP_BT_LINE_MAX 64
#define APP_TEXT_MAX 40
#define APP_OLED_LINE_MAX 21
#define APP_STEP_COUNT 3
#define APP_OLED_PAGE_COUNT 4U
#define APP_OLED_REFRESH_MS 200U

typedef enum
{
  DEMO_APP_CONTROL_MODE_OPEN_LOOP_SEPARATE_START = 0,
  DEMO_APP_CONTROL_MODE_CLOSED_LOOP_LINKED_START
} DemoAppControlMode;

static const int16_t s_step_table[APP_STEP_COUNT] = {1, 5, 10};
static const DemoAppControlMode s_control_mode = DEMO_APP_CONTROL_MODE_CLOSED_LOOP_LINKED_START;

static uint8_t s_step_index = 0;
static uint8_t s_inverse = 0;
static uint8_t s_dirty = 1;
static uint8_t s_oled_page = 0U;
static uint32_t s_last_oled_tick = 0U;
static char s_bt_line[APP_BT_LINE_MAX];
static char s_pwm_status[12] = "STOP";
static char s_text[APP_TEXT_MAX + 1];
static char s_last_event[APP_OLED_LINE_MAX + 1];

static float DemoApp_ClampVoltageRef(float value)
{
  if (value < 0.0f)
  {
    return 0.0f;
  }

  if (value > 60.0f)
  {
    return 60.0f;
  }

  return value;
}

static void DemoApp_CopyAscii(char *dst, const char *src, uint16_t max_len)
{
  uint16_t i;

  if ((dst == NULL) || (max_len == 0U))
  {
    return;
  }

  if (src == NULL)
  {
    dst[0] = '\0';
    return;
  }

  for (i = 0; (src[i] != '\0') && (i < (uint16_t)(max_len - 1U)); i++)
  {
    if ((src[i] >= 32) && (src[i] <= 126))
    {
      dst[i] = src[i];
    }
    else
    {
      dst[i] = ' ';
    }
  }

  dst[i] = '\0';
}

static void DemoApp_SetLastEvent(const char *text)
{
  DemoApp_CopyAscii(s_last_event, text, sizeof(s_last_event));
}

static void DemoApp_SendAck(const char *text)
{
  if (text != NULL)
  {
    (void)JDY31_SendString(text);
  }
}

static void DemoApp_UpdatePwmStatusText(void)
{
  char line[12];

  snprintf(line, sizeof(line), "%c%c%c",
           PowerPwm_IsChannelRunning(POWER_PWM_CH_C) ? '1' : '-',
           PowerPwm_IsChannelRunning(POWER_PWM_CH_D) ? '2' : '-',
           PowerPwm_IsChannelRunning(POWER_PWM_CH_F) ? '3' : '-');
  DemoApp_CopyAscii(s_pwm_status, line, sizeof(s_pwm_status));
}

static uint8_t DemoApp_IsLinkedStartMode(void)
{
  return (s_control_mode == DEMO_APP_CONTROL_MODE_CLOSED_LOOP_LINKED_START) ? 1U : 0U;
}

static void DemoApp_StopOutput(void)
{
  PowerCtrl_EnableControl(0U);
  PowerPwm_StopAll();
}

static void DemoApp_StartOutput(void)
{
  PowerPwm_StartAll();

  if (DemoApp_IsLinkedStartMode() != 0U)
  {
    PowerCtrl_EnableControl(1U);
  }
  else
  {
    PowerCtrl_EnableControl(0U);
    PowerPwm_SetDutyAll(POWER_PWM_DEFAULT_DUTY);
  }
}

static const char *DemoApp_GetModeTag(void)
{
  return (DemoApp_IsLinkedStartMode() != 0U) ? "CL" : "OL";
}

static void DemoApp_FormatSignedCenti(char *dst, uint16_t size, int32_t value)
{
  int32_t abs_value;

  if ((dst == NULL) || (size == 0U))
  {
    return;
  }

  abs_value = (value < 0) ? (-value) : value;
  snprintf(dst, size, "%s%ld.%02ld",
           (value < 0) ? "-" : "",
           (long)(abs_value / 100),
           (long)(abs_value % 100));
}

static void DemoApp_FormatSignedFixed4(char *dst, uint16_t size, float value)
{
  int32_t scaled_value;
  int32_t abs_value;

  if ((dst == NULL) || (size == 0U))
  {
    return;
  }

  scaled_value = (int32_t)((value >= 0.0f) ? (value * 10000.0f + 0.5f) : (value * 10000.0f - 0.5f));
  abs_value = (scaled_value < 0) ? (-scaled_value) : scaled_value;

  snprintf(dst, size, "%s%ld.%04ld",
           (scaled_value < 0) ? "-" : "",
           (long)(abs_value / 10000),
           (long)(abs_value % 10000));
}

static const char *DemoApp_GetLoopModeTag(PowerCtrlMode mode)
{
  if (mode == POWER_CTRL_MODE_CC)
  {
    return "CC";
  }

  if (mode == POWER_CTRL_MODE_CV)
  {
    return "CV";
  }

  if (mode == POWER_CTRL_MODE_FAULT)
  {
    return "FLT";
  }

  return "OFF";
}

static void DemoApp_RenderStatusPage(void)
{
  char line[APP_OLED_LINE_MAX + 1];
  char value1[10];
  char value2[10];
  const PowerCtrlSnapshot *snapshot;
  int32_t vout_centi_v;
  int32_t vref_centi_v;
  int32_t iout_centi_a;
  int32_t iref_centi_a;
  int32_t vaux_centi_v;
  int32_t duty_permille;

  snapshot = PowerCtrl_GetSnapshot();
  vout_centi_v = (int32_t)(snapshot->voltage_feedback * 100.0f);
  vref_centi_v = (int32_t)(snapshot->voltage_ref * 100.0f);
  iout_centi_a = (int32_t)(snapshot->current_feedback * 100.0f);
  iref_centi_a = (int32_t)(snapshot->current_ref * 100.0f);
  vaux_centi_v = (int32_t)(PowerAdc_GetDisplaySignal(POWER_ADC_SIGNAL_AUX_VOLTAGE) * 100.0f);
  duty_permille = (int32_t)(snapshot->duty_command * 1000.0f);

  OLED_ShowString(0, 0, "PAGE1 CC-CV LOOP", OLED_6X8);

  DemoApp_FormatSignedCenti(value1, sizeof(value1), vout_centi_v);
  DemoApp_FormatSignedCenti(value2, sizeof(value2), vref_centi_v);
  snprintf(line, sizeof(line), "V:%.7s/%.7s", value1, value2);
  OLED_ShowString(0, 8, line, OLED_6X8);

  DemoApp_FormatSignedCenti(value1, sizeof(value1), iout_centi_a);
  DemoApp_FormatSignedCenti(value2, sizeof(value2), iref_centi_a);
  snprintf(line, sizeof(line), "I:%.6s/%.6sA", value1, value2);
  OLED_ShowString(0, 16, line, OLED_6X8);

  DemoApp_FormatSignedCenti(value1, sizeof(value1), vaux_centi_v);
  snprintf(line, sizeof(line), "V2:%sV", value1);
  OLED_ShowString(0, 24, line, OLED_6X8);

  snprintf(line, sizeof(line), "D:%ld.%01ld%% %s",
           (long)(duty_permille / 10),
           (long)labs(duty_permille % 10),
           DemoApp_GetLoopModeTag(snapshot->mode));
  OLED_ShowString(0, 32, line, OLED_6X8);

  snprintf(line, sizeof(line), "CH:%s %s:%s",
           s_pwm_status,
           DemoApp_GetModeTag(),
           snapshot->control_enabled ? "ON" : "OFF");
  OLED_ShowString(0, 40, line, OLED_6X8);

  snprintf(line, sizeof(line), "ADC:%s PROT:%s",
           snapshot->last_status_ok ? "OK" : "ERR",
           snapshot->protection_active ? "ON" : "OFF");
  OLED_ShowString(0, 48, line, OLED_6X8);

  OLED_ShowString(0, 56, "ROT:PAGE KEY:PWM", OLED_6X8);
}

static void DemoApp_RenderTextPage(void)
{
  char text_line1[APP_OLED_LINE_MAX + 1];
  char text_line2[APP_OLED_LINE_MAX + 1];

  memset(text_line1, 0, sizeof(text_line1));
  memset(text_line2, 0, sizeof(text_line2));
  strncpy(text_line1, s_text, APP_OLED_LINE_MAX);
  if (strlen(s_text) > APP_OLED_LINE_MAX)
  {
    strncpy(text_line2, &s_text[APP_OLED_LINE_MAX], APP_OLED_LINE_MAX);
  }

  OLED_ShowString(0, 0, "PAGE2 EVENT/TEXT", OLED_6X8);
  OLED_ShowString(0, 16, "EVENT:", OLED_6X8);
  OLED_ShowString(0, 24, s_last_event, OLED_6X8);
  OLED_ShowString(0, 40, "TEXT:", OLED_6X8);
  OLED_ShowString(0, 48, text_line1, OLED_6X8);
  OLED_ShowString(0, 56, text_line2, OLED_6X8);
}

static void DemoApp_RenderAdcUsedPage(void)
{
  char line[APP_OLED_LINE_MAX + 1];
  char value[10];
  float vdo_output_v;
  int32_t vdr_centi_v;
  float a4o_output_v;
  int32_t a4r_centi_v;

  vdo_output_v = PowerAdc_GetDisplaySignal(POWER_ADC_SIGNAL_LOAD_OUTPUT_VOLTAGE);
  vdr_centi_v = (int32_t)(PowerAdc_GetDisplaySignal(POWER_ADC_SIGNAL_LOAD_VOLTAGE) * 100.0f);
  a4o_output_v = PowerAdc_GetDisplaySignal(POWER_ADC_SIGNAL_AUX_OUTPUT_VOLTAGE);
  a4r_centi_v = (int32_t)(PowerAdc_GetDisplaySignal(POWER_ADC_SIGNAL_AUX_VOLTAGE) * 100.0f);

  OLED_ShowString(0, 0, "PAGE3 ADC USED", OLED_6X8);

  snprintf(line, sizeof(line), "1.1:%4u 1.3:%4u", PowerAdc_GetRaw(POWER_ADC_CHANNEL_ADC1_IN1), PowerAdc_GetRaw(POWER_ADC_CHANNEL_AUX_VOLTAGE));
  OLED_ShowString(0, 8, line, OLED_6X8);

  DemoApp_FormatSignedFixed4(value, sizeof(value), vdo_output_v);
  snprintf(line, sizeof(line), "1.1DO:%s", value);
  OLED_ShowString(0, 16, line, OLED_6X8);

  DemoApp_FormatSignedCenti(value, sizeof(value), vdr_centi_v);
  snprintf(line, sizeof(line), "1.1DR:%s", value);
  OLED_ShowString(0, 24, line, OLED_6X8);

  DemoApp_FormatSignedFixed4(value, sizeof(value), a4o_output_v);
  snprintf(line, sizeof(line), "1.3O:%s", value);
  OLED_ShowString(0, 32, line, OLED_6X8);

  DemoApp_FormatSignedCenti(value, sizeof(value), a4r_centi_v);
  snprintf(line, sizeof(line), "1.3V:%s", value);
  OLED_ShowString(0, 40, line, OLED_6X8);

  OLED_ShowString(0, 56, "ROT:PAGE ADC USED", OLED_6X8);
}

static void DemoApp_RenderAdcUnusedPage(void)
{
  char line[APP_OLED_LINE_MAX + 1];

  int32_t i9o_centi_v;
  int32_t i9a_centi_a;
  char value[10];

  i9o_centi_v = (int32_t)(PowerAdc_GetDisplaySignal(POWER_ADC_SIGNAL_CURRENT_SENSE_OUTPUT_VOLTAGE) * 100.0f);
  i9a_centi_a = (int32_t)(PowerAdc_GetDisplaySignal(POWER_ADC_SIGNAL_CURRENT_SENSE_CURRENT) * 100.0f);

  OLED_ShowString(0, 0, "PAGE4 ADC MONITOR", OLED_6X8);

  snprintf(line, sizeof(line), "1.9:%4u", PowerAdc_GetRaw(POWER_ADC_CHANNEL_CURRENT_SENSE));
  OLED_ShowString(0, 8, line, OLED_6X8);

  DemoApp_FormatSignedCenti(value, sizeof(value), i9o_centi_v);
  snprintf(line, sizeof(line), "1.9VO:%s", value);
  OLED_ShowString(0, 16, line, OLED_6X8);

  DemoApp_FormatSignedCenti(value, sizeof(value), i9a_centi_a);
  snprintf(line, sizeof(line), "1.9I:%sA", value);
  OLED_ShowString(0, 24, line, OLED_6X8);

  snprintf(line, sizeof(line), "3.1:%4u 3.12:%4u", PowerAdc_GetRaw(POWER_ADC_CHANNEL_ADC3_IN1), PowerAdc_GetRaw(POWER_ADC_CHANNEL_ADC3_IN12));
  OLED_ShowString(0, 32, line, OLED_6X8);

  snprintf(line, sizeof(line), "2.12:%4u", PowerAdc_GetRaw(POWER_ADC_CHANNEL_ADC2_IN12));
  OLED_ShowString(0, 40, line, OLED_6X8);

  OLED_ShowString(0, 56, "ROT:PAGE ADC MONITOR", OLED_6X8);
}

static void DemoApp_Render(void)
{
  DemoApp_UpdatePwmStatusText();
  OLED_Clear();

  if (s_oled_page == 0U)
  {
    DemoApp_RenderStatusPage();
  }
  else if (s_oled_page == 1U)
  {
    DemoApp_RenderTextPage();
  }
  else if (s_oled_page == 2U)
  {
    DemoApp_RenderAdcUsedPage();
  }
  else
  {
    DemoApp_RenderAdcUnusedPage();
  }

  if (s_inverse != 0U)
  {
    OLED_Reverse();
  }

  OLED_Update();
  s_last_oled_tick = HAL_GetTick();
  s_dirty = 0;
}

void DemoApp_HandleBluetoothCommand(const char *cmd)
{
  char ack[64];
  int32_t value;

  if (cmd == NULL)
  {
    return;
  }

  if (strcmp(cmd, "PWMON") == 0)
  {
    DemoApp_StartOutput();
    DemoApp_SetLastEvent((DemoApp_IsLinkedStartMode() != 0U) ? "PWM ON CC-CV ON" : "PWM ON OPEN LOOP");
    s_dirty = 1;
    DemoApp_SendAck((DemoApp_IsLinkedStartMode() != 0U) ? "OK PWMON CCCVON\r\n" : "OK PWMON OPENLOOP\r\n");
    return;
  }

  if (strcmp(cmd, "PWMOFF") == 0)
  {
    DemoApp_StopOutput();
    DemoApp_SetLastEvent((DemoApp_IsLinkedStartMode() != 0U) ? "PWM OFF CC-CV OFF" : "PWM OFF OPEN LOOP");
    s_dirty = 1;
    DemoApp_SendAck((DemoApp_IsLinkedStartMode() != 0U) ? "OK PWMOFF CCCVOFF\r\n" : "OK PWMOFF OPENLOOP\r\n");
    return;
  }

  if (strcmp(cmd, "PIDON") == 0)
  {
    PowerCtrl_EnableControl(1U);
    DemoApp_SetLastEvent("CC-CV LOOP ON");
    s_dirty = 1;
    DemoApp_SendAck("OK PIDON CCCVON\r\n");
    return;
  }

  if (strcmp(cmd, "PIDOFF") == 0)
  {
    PowerCtrl_EnableControl(0U);
    PowerPwm_SetDutyAll(POWER_PWM_DEFAULT_DUTY);
    DemoApp_SetLastEvent("CC-CV LOOP OFF");
    s_dirty = 1;
    DemoApp_SendAck("OK PIDOFF\r\n");
    return;
  }

  if (strncmp(cmd, "TEXT=", 5) == 0)
  {
    DemoApp_CopyAscii(s_text, cmd + 5, sizeof(s_text));
    DemoApp_SetLastEvent("BT TEXT");
    s_dirty = 1;
    snprintf(ack, sizeof(ack), "OK TEXT\r\n");
    DemoApp_SendAck(ack);
    return;
  }

  if (strncmp(cmd, "SET=", 4) == 0)
  {
    float voltage_ref;

    value = strtol(cmd + 4, NULL, 10);
    voltage_ref = DemoApp_ClampVoltageRef((float)value / 100.0f);
    PowerCtrl_SetVoltageRef(voltage_ref);
    DemoApp_SetLastEvent("SET VREF");
    s_dirty = 1;
    {
      int32_t voltage_ref_centi_v;

      voltage_ref_centi_v = (int32_t)(voltage_ref * 100.0f);
      snprintf(ack, sizeof(ack), "OK VREF=%s%ld.%02ldV\r\n",
               (voltage_ref_centi_v < 0) ? "-" : "",
               (long)(labs(voltage_ref_centi_v) / 100),
               (long)(labs(voltage_ref_centi_v) % 100));
    }
    DemoApp_SendAck(ack);
    return;
  }

  if (strncmp(cmd, "SETI=", 5) == 0)
  {
    int32_t current_ref_centi_a;

    value = strtol(cmd + 5, NULL, 10);
    PowerCtrl_SetCurrentRef((float)value / 100.0f);
    current_ref_centi_a = (int32_t)(PowerCtrl_GetSnapshot()->current_ref * 100.0f);
    DemoApp_SetLastEvent("SET IREF");
    s_dirty = 1;
    snprintf(ack, sizeof(ack), "OK IREF=%s%ld.%02ldA\r\n",
             (current_ref_centi_a < 0) ? "-" : "",
             (long)(labs(current_ref_centi_a) / 100),
             (long)(labs(current_ref_centi_a) % 100));
    DemoApp_SendAck(ack);
    return;
  }

  if (strncmp(cmd, "STEP=", 5) == 0)
  {
    value = strtol(cmd + 5, NULL, 10);
    if (value == 1)
    {
      s_step_index = 0;
    }
    else if (value == 5)
    {
      s_step_index = 1;
    }
    else if (value == 10)
    {
      s_step_index = 2;
    }
    else
    {
      DemoApp_SendAck("ERR STEP\r\n");
      return;
    }

    DemoApp_SetLastEvent("BT STEP");
    s_dirty = 1;
    snprintf(ack, sizeof(ack), "OK STEP=%d\r\n", s_step_table[s_step_index]);
    DemoApp_SendAck(ack);
    return;
  }

  if (strcmp(cmd, "CLS") == 0)
  {
    s_text[0] = '\0';
    DemoApp_SetLastEvent("BT CLS");
    s_dirty = 1;
    DemoApp_SendAck("OK CLS\r\n");
    return;
  }

  if (strcmp(cmd, "INV") == 0)
  {
    s_inverse = 1;
    DemoApp_SetLastEvent("BT INV");
    s_dirty = 1;
    DemoApp_SendAck("OK INV\r\n");
    return;
  }

  if (strcmp(cmd, "NOR") == 0)
  {
    s_inverse = 0;
    DemoApp_SetLastEvent("BT NOR");
    s_dirty = 1;
    DemoApp_SendAck("OK NOR\r\n");
    return;
  }

  if (strcmp(cmd, "HELP") == 0)
  {
    DemoApp_SetLastEvent("BT HELP");
    s_dirty = 1;
    DemoApp_SendAck((DemoApp_IsLinkedStartMode() != 0U)
                        ? "CL MODE; PWM and CC-CV linked on start; CMD PWMON/PWMOFF PIDON/PIDOFF SET=* SETI=*\r\n"
                        : "OL MODE; power-on open loop, CC-CV starts by command; CMD PWMON/PWMOFF PIDON/PIDOFF SET=* SETI=*\r\n");
    return;
  }

  DemoApp_SetLastEvent("BT ERR");
  s_dirty = 1;
  DemoApp_SendAck("ERR CMD\r\n");
}

void DemoApp_Init(void)
{
  Encoder_Init();
  OLED_Init();
  JDY31_Init(&huart3);
  (void)JDY31_StartReceiveIT();

  /* PowerPwm_Init() arms the HRTIM ADC trigger the conversions depend on, so it
     has to run before PowerCtrl_Init() starts the DMA sequence. */
  if (PowerPwm_Init() != HAL_OK)
  {
    Error_Handler();
  }

  PowerCtrl_Init();

  DemoApp_CopyAscii(s_text, "READY", sizeof(s_text));
  DemoApp_UpdatePwmStatusText();
  DemoApp_StopOutput();
  DemoApp_SetLastEvent((DemoApp_IsLinkedStartMode() != 0U) ? "PWM OFF CC-CV OFF" : "PWM OFF OPEN LOOP");
  DemoApp_SendAck((DemoApp_IsLinkedStartMode() != 0U)
                      ? "CL MODE HRTIM F=100k DT=250ns ADC=HRTIMTRG+DMA CC-CV IREF=1.50A SETI=centiamp VREF=TRACK SET=centivolt ROT=PAGE KEY=PWM CMD PWMON/OFF PIDON/OFF\r\n"
                      : "OL MODE HRTIM F=100k DT=250ns ADC=HRTIMTRG+DMA OPENLOOP DUTY=120 IREF=1.50A SETI=centiamp VREF=TRACK SET=centivolt ROT=PAGE KEY=PWM CMD PWMON/OFF PIDON/OFF\r\n");
  s_last_oled_tick = HAL_GetTick();
  DemoApp_Render();
}

const char *DemoApp_GetPwmStatusText(void)
{
  return s_pwm_status;
}

void DemoApp_Run(void)
{
  int8_t delta;
  char event[APP_OLED_LINE_MAX + 1];

  PowerCtrl_Run();
  Encoder_Poll();
  delta = Encoder_GetDelta();
  (void)delta;

  if (delta > 0)
  {
    s_oled_page = (uint8_t)((s_oled_page + 1U) % APP_OLED_PAGE_COUNT);
    snprintf(event, sizeof(event), "PAGE %u", (unsigned int)(s_oled_page + 1U));
    DemoApp_SetLastEvent(event);
    DemoApp_SendAck("OK ROT+\r\n");
    s_dirty = 1;
  }
  else if (delta < 0)
  {
    s_oled_page = (uint8_t)((s_oled_page + APP_OLED_PAGE_COUNT - 1U) % APP_OLED_PAGE_COUNT);
    snprintf(event, sizeof(event), "PAGE %u", (unsigned int)(s_oled_page + 1U));
    DemoApp_SetLastEvent(event);
    DemoApp_SendAck("OK ROT-\r\n");
    s_dirty = 1;
  }

  if (Encoder_GetKeyPressed() != 0U)
  {
    if (PowerPwm_IsRunning() != 0U)
    {
      DemoApp_StopOutput();
      DemoApp_SetLastEvent((DemoApp_IsLinkedStartMode() != 0U) ? "KEY PWM OFF CC-CV OFF" : "KEY PWM OFF OPEN LOOP");
      DemoApp_SendAck((DemoApp_IsLinkedStartMode() != 0U) ? "OK KEY PWMOFF CCCVOFF\r\n" : "OK KEY PWMOFF OPENLOOP\r\n");
    }
    else
    {
      DemoApp_StartOutput();
      DemoApp_SetLastEvent((DemoApp_IsLinkedStartMode() != 0U) ? "KEY PWM ON CC-CV ON" : "KEY PWM ON OPEN LOOP");
      DemoApp_SendAck((DemoApp_IsLinkedStartMode() != 0U) ? "OK KEY PWMON CCCVON\r\n" : "OK KEY PWMON OPENLOOP\r\n");
    }
    s_dirty = 1;
  }

  if (JDY31_ReadLine(s_bt_line, sizeof(s_bt_line)) != 0U)
  {
    DemoApp_HandleBluetoothCommand(s_bt_line);
  }

  if ((HAL_GetTick() - s_last_oled_tick) >= APP_OLED_REFRESH_MS)
  {
    s_dirty = 1;
  }

  if (s_dirty != 0U)
  {
    DemoApp_Render();
  }
}
