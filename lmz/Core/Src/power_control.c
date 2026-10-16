#include "power_control.h"

#include "pid_adv.h"
#include "power_pwm.h"

#include <math.h>
#include <string.h>

#define POWER_CTRL_FAST_PERIOD_MS 1U
#define POWER_CTRL_VOLTAGE_REF_MIN 0.0f
#define POWER_CTRL_VOLTAGE_REF_MAX 120.0f
/* The current sense calibration tops out near 1.81 A at 3.3 V, so the constant
   current ceiling cannot be set above that without re-calibrating the shunt. */
#define POWER_CTRL_CURRENT_REF_MIN 0.0f
#define POWER_CTRL_CURRENT_REF_MAX 1.8f
#define POWER_CTRL_CURRENT_REF_DEFAULT 1.5f
#define POWER_CTRL_DUTY_MIN 0.10f
#define POWER_CTRL_DUTY_MAX 0.85f
#define POWER_CTRL_PROTECT_VOLTAGE_MAX 120.0f
#define POWER_CTRL_PROTECT_CURRENT_MAX 100.0f
#define POWER_CTRL_TRACKING_GAIN 0.5f

#define POWER_CTRL_VOLTAGE_PID_KP 0.01f
#define POWER_CTRL_VOLTAGE_PID_KI 0.02f
#define POWER_CTRL_CURRENT_PID_KP 0.20f
#define POWER_CTRL_CURRENT_PID_KI 1.00f
#define POWER_CTRL_PID_RAMP 0.5f

static PowerCtrlSnapshot s_snapshot;
static PidAdvController s_voltage_pid;
static PidAdvController s_current_pid;
static uint8_t s_voltage_ref_manual = 0U;
static uint32_t s_last_run_tick = 0U;
static uint8_t s_initialized = 0U;

static float PowerCtrl_Clamp(float value, float min, float max)
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

static float PowerCtrl_GetTrackingVoltageRef(void)
{
  return PowerCtrl_Clamp(PowerAdc_GetAuxVoltage() * POWER_CTRL_TRACKING_GAIN,
                         POWER_CTRL_VOLTAGE_REF_MIN,
                         POWER_CTRL_VOLTAGE_REF_MAX);
}

static float PowerCtrl_GetAppliedDuty(void)
{
  return PowerPwm_GetDuty(POWER_PWM_CH_C);
}

static void PowerCtrl_PrimePidState(void)
{
  float applied = PowerCtrl_Clamp(PowerCtrl_GetAppliedDuty(), POWER_CTRL_DUTY_MIN, POWER_CTRL_DUTY_MAX);

  s_voltage_pid.prev_measurement = s_snapshot.voltage_feedback;
  s_voltage_pid.output = applied;
  s_current_pid.prev_measurement = s_snapshot.current_feedback;
  s_current_pid.output = applied;
}

static void PowerCtrl_UpdateSnapshot(void)
{
  const PowerAdcSample *samples = PowerAdc_GetSamples();

  memcpy(s_snapshot.adc_samples, samples, sizeof(s_snapshot.adc_samples));
  s_snapshot.adc_sequence_counter = PowerAdc_GetSequenceCounter();
  s_snapshot.sample_counter++;
}

static void PowerCtrl_TripProtection(void)
{
  s_snapshot.protection_active = 1U;
  s_snapshot.control_enabled = 0U;
  s_snapshot.mode = POWER_CTRL_MODE_FAULT;
  s_snapshot.duty_command = 0.0f;
  PidAdv_Reset(&s_voltage_pid);
  PidAdv_Reset(&s_current_pid);
  PowerPwm_SetDutyAll(0.0f);
  PowerPwm_StopAll();
}

static void PowerCtrl_RunClosedLoop(float dt_s)
{
  float duty_voltage;
  float duty_current;
  uint8_t current_limits;

  s_snapshot.voltage_feedback = PowerAdc_GetLoadDiffVoltage();
  s_snapshot.current_feedback = PowerAdc_GetCurrentSenseCurrent();

  if (s_voltage_ref_manual == 0U)
  {
    s_snapshot.voltage_ref = PowerCtrl_GetTrackingVoltageRef();
  }

  if (s_snapshot.control_enabled == 0U)
  {
    return;
  }

  if ((s_snapshot.voltage_feedback >= POWER_CTRL_PROTECT_VOLTAGE_MAX)
      || (s_snapshot.current_feedback >= POWER_CTRL_PROTECT_CURRENT_MAX)
      || (s_snapshot.current_feedback <= (-POWER_CTRL_PROTECT_CURRENT_MAX)))
  {
    PowerCtrl_TripProtection();
    return;
  }

  duty_voltage = PidAdv_Update(&s_voltage_pid,
                               s_snapshot.voltage_ref,
                               s_snapshot.voltage_feedback,
                               dt_s);
  duty_current = PidAdv_Update(&s_current_pid,
                               s_snapshot.current_ref,
                               fabsf(s_snapshot.current_feedback),
                               dt_s);

  /* Min-select: whichever loop asks for less duty owns the switch, and the idle
     loop is back-calculated onto that duty so the handover is bumpless. A
     current ceiling above what the load can draw leaves CV in charge. */
  current_limits = (duty_current <= duty_voltage) ? 1U : 0U;
  if (current_limits != 0U)
  {
    PidAdv_SetHeldOutput(&s_voltage_pid, duty_current);
  }
  else
  {
    PidAdv_SetHeldOutput(&s_current_pid, duty_voltage);
  }

  s_snapshot.duty_voltage_loop = duty_voltage;
  s_snapshot.duty_current_loop = duty_current;
  s_snapshot.duty_command = PowerCtrl_Clamp(current_limits ? duty_current : duty_voltage,
                                            POWER_CTRL_DUTY_MIN,
                                            POWER_CTRL_DUTY_MAX);
  s_snapshot.mode = (current_limits != 0U) ? POWER_CTRL_MODE_CC : POWER_CTRL_MODE_CV;
  PowerPwm_SetDutyAll(s_snapshot.duty_command);
}

void PowerCtrl_Init(void)
{
  memset(&s_snapshot, 0, sizeof(s_snapshot));

  if (PowerAdc_Init() != HAL_OK)
  {
    Error_Handler();
  }

  PidAdv_Init(&s_voltage_pid, POWER_CTRL_VOLTAGE_PID_KP, POWER_CTRL_VOLTAGE_PID_KI, 0.0f,
              POWER_CTRL_DUTY_MIN, POWER_CTRL_DUTY_MAX);
  PidAdv_SetRampRate(&s_voltage_pid, POWER_CTRL_PID_RAMP);

  PidAdv_Init(&s_current_pid, POWER_CTRL_CURRENT_PID_KP, POWER_CTRL_CURRENT_PID_KI, 0.0f,
              POWER_CTRL_DUTY_MIN, POWER_CTRL_DUTY_MAX);
  PidAdv_SetRampRate(&s_current_pid, POWER_CTRL_PID_RAMP);

  s_snapshot.voltage_feedback = PowerAdc_GetLoadDiffVoltage();
  s_snapshot.current_feedback = PowerAdc_GetCurrentSenseCurrent();
  s_snapshot.voltage_ref = PowerCtrl_GetTrackingVoltageRef();
  s_snapshot.current_ref = POWER_CTRL_CURRENT_REF_DEFAULT;
  s_snapshot.duty_command = PowerCtrl_GetAppliedDuty();
  s_snapshot.mode = POWER_CTRL_MODE_OFF;
  PowerCtrl_PrimePidState();
  s_snapshot.control_enabled = 0U;
  s_snapshot.protection_active = 0U;
  s_snapshot.last_status_ok = 1U;
  s_voltage_ref_manual = 0U;
  s_last_run_tick = HAL_GetTick();
  s_initialized = 1U;
}

void PowerCtrl_Run(void)
{
  uint32_t now_tick;
  float dt_s;

  if (s_initialized == 0U)
  {
    return;
  }

  now_tick = HAL_GetTick();
  if ((now_tick - s_last_run_tick) < POWER_CTRL_FAST_PERIOD_MS)
  {
    return;
  }

  dt_s = (float)(now_tick - s_last_run_tick) / 1000.0f;
  s_last_run_tick = now_tick;

  if (PowerAdc_SampleAll() != HAL_OK)
  {
    s_snapshot.last_status_ok = 0U;
    return;
  }

  s_snapshot.last_status_ok = 1U;
  PowerCtrl_UpdateSnapshot();
  PowerCtrl_RunClosedLoop(dt_s);
}

void PowerCtrl_EnableControl(uint8_t enable)
{
  s_snapshot.control_enabled = (enable != 0U) ? 1U : 0U;

  if (s_snapshot.control_enabled == 0U)
  {
    s_snapshot.mode = POWER_CTRL_MODE_OFF;
    s_snapshot.duty_command = PowerCtrl_GetAppliedDuty();
    s_snapshot.protection_active = 0U;
    s_voltage_ref_manual = 0U;
    PidAdv_Reset(&s_voltage_pid);
    PidAdv_Reset(&s_current_pid);
    return;
  }

  s_voltage_ref_manual = 0U;
  s_snapshot.voltage_ref = PowerCtrl_GetTrackingVoltageRef();
  s_snapshot.duty_command = PowerCtrl_GetAppliedDuty();
  s_snapshot.protection_active = 0U;
  PidAdv_Reset(&s_voltage_pid);
  PidAdv_Reset(&s_current_pid);
  PowerCtrl_PrimePidState();
}

void PowerCtrl_SetVoltageRef(float voltage_ref)
{
  s_snapshot.voltage_ref = PowerCtrl_Clamp(voltage_ref,
                                           POWER_CTRL_VOLTAGE_REF_MIN,
                                           POWER_CTRL_VOLTAGE_REF_MAX);
  s_voltage_ref_manual = 1U;
}

void PowerCtrl_SetCurrentRef(float current_ref)
{
  s_snapshot.current_ref = PowerCtrl_Clamp(current_ref,
                                           POWER_CTRL_CURRENT_REF_MIN,
                                           POWER_CTRL_CURRENT_REF_MAX);
}

const PowerCtrlSnapshot *PowerCtrl_GetSnapshot(void)
{
  return &s_snapshot;
}
