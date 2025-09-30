#include "app_control.h"
#include "bsp_as5600.h"

/* ================== Motor index and direction settings ================== */
#define IDX_M2006   0   /* Default 0 → corresponds to CAN ID 0x201 */
#define IDX_M3508   1   /* Default 1 → corresponds to CAN ID 0x202 */

/* ================== Torque / Position / Speed parameters ================== */
#define Kt_M2006_Nm_per_A     0.10f
#define GEAR_M2006            36.0f
#define ETA_M2006             0.90f
#define C610_COUNTS_PER_AMP   1000.0f
#define TORQUE_STEP_NM        0.20f     /* Torque step per gear (Nm/step), gear index 0..7 */

/* Position loop parameters */
#define KP_M2006_POS              6.0f
#define KI_M2006_POS              0.00f
#define KD_M2006_POS              0.20f
#define PID_POS_MAX_OUT_M2006     400.0f
#define PID_POS_INT_LIM_M2006     200.0f
#define USE_POS_ERR_SHAPING_M2006   1
#define M2006_POS_DEADBAND_UNITS    0.0f
#define M2006_POS_MAX_ERR_UNITS   360.0f

/* Speed loop parameters */
#define KP_M2006_SPD          1.5f
#define KI_M2006_SPD          0.1f
#define KD_M2006_SPD          0.0f
#define PID_MAX_OUT_M2006     16384
#define PID_INT_LIM_M2006      5000
#define IQ_LIMIT_M2006        15000

#define KP_M3508_SPD          1.5f
#define KI_M3508_SPD          0.10f
#define KD_M3508_SPD          0.0f
#define PID_MAX_OUT_M3508     20000
#define PID_INT_LIM_M3508     20000
#define IQ_LIMIT_M3508        18000

/* Gear and key definitions */
#define GEAR_MAX_INDEX        7
#define GEAR_STEP_RPM         50
#define SHORT_PRESS_MAX_MS    500
#define LONG_PRESS_MIN_MS     2000

/* Encoder CPR and angle conversion for output shaft */
#define ECD_CPR 8192.0f
#define M2006_POS_STEP_UNITS  ( (45.0f) * (ECD_CPR/360.0f) * (GEAR_M2006) )

/* ========== Internal variables ========== */
static int8_t dir[4] = { +1, +1, +1, +1 };
static volatile m2006_mode_t m2006_mode = M2006_MODE_SPEED;
static int8_t  gear_m2006 = 0, gear_m3508 = 0;
static volatile float pos_ref_m2006 = 0.0f;

static pid_t pid_tau_counts_m2006;
static pid_t pid_pos_m2006;

/* AS5600 state object */
static AS5600_State_t as5600_state;

/* ========== Utility functions ========== */
static inline int16_t clip16(int32_t x, int32_t lim)
{ if (x > lim) x = lim; else if (x < -lim) x = -lim; return (int16_t)x; }

static inline int16_t A_to_counts(float I_A) { return (int16_t)(I_A * C610_COUNTS_PER_AMP); }
static inline float  tau_to_currentA(float tau_Nm) { return tau_Nm / (Kt_M2006_Nm_per_A * GEAR_M2006 * ETA_M2006); }

static inline float err_shape_f(float ref, float meas, float deadband, float maxerr)
{
  float e = ref - meas;
  if (e > -deadband && e < deadband) e = 0.0f;
  if (e >  maxerr) e =  maxerr;
  else if (e < -maxerr) e = -maxerr;
  return ref - e;
}

/* External interface */
void M2006_SetMode(m2006_mode_t mode)      { m2006_mode = mode; }
void M2006_SetPosRef(float pos_units)      { pos_ref_m2006 = pos_units; }
float M2006_GetPosRef(void)                { return pos_ref_m2006; }

/* Key handling */
static void KEY_Process(void)
{
  static uint8_t  key_prev = 0;
  static uint32_t t0 = 0;
  static uint8_t  long_fired = 0;

  uint32_t now = HAL_GetTick();
  uint8_t key_now = (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET) ? 1 : 0;

  if (key_prev == 0 && key_now == 1) {
    t0 = now;
    long_fired = 0;
  }
  else if (key_prev == 1 && key_now == 1) {
    uint32_t dt = now - t0;
    if (!long_fired && dt >= LONG_PRESS_MIN_MS) {
      /* Long press: reset angle accumulators, optionally zero AS5600 */
      moto_chassis[IDX_M2006].total_angle = 0;
      moto_chassis[IDX_M3508].total_angle = 0;

      /* Optionally: also set AS5600 zero to current angle */
      BSP_AS5600_SetZeroHere(&as5600_state);

      long_fired = 1;
    }
  }
  else if (key_prev == 1 && key_now == 0) {
    uint32_t dt = now - t0;
    if (!long_fired && dt < SHORT_PRESS_MAX_MS) {
      /* Short press: change gear / step position / toggle direction */
      if (m2006_mode == M2006_MODE_POSITION) {
        pos_ref_m2006 += (float)dir[IDX_M2006] * M2006_POS_STEP_UNITS;
      } else {
        gear_m2006 = (int8_t)((gear_m2006 + 1) % (GEAR_MAX_INDEX + 1));
      }
      gear_m3508 = (int8_t)((gear_m3508 + 1) % (GEAR_MAX_INDEX + 1));
      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
      HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    }
  }
  key_prev = key_now;
}

/* Task entry */
void app_control_start(void)
{
  PID_struct_init(&pid_spd[IDX_M2006], POSITION_PID, PID_MAX_OUT_M2006, PID_INT_LIM_M2006,
                  KP_M2006_SPD, KI_M2006_SPD, KD_M2006_SPD);
  PID_struct_init(&pid_spd[IDX_M3508], POSITION_PID, PID_MAX_OUT_M3508, PID_INT_LIM_M3508,
                  KP_M3508_SPD, KI_M3508_SPD, KD_M3508_SPD);

  PID_struct_init(&pid_tau_counts_m2006, POSITION_PID,
                  6000, 3000, 0.5f,  20.0f, 0.0f);

  PID_struct_init(&pid_pos_m2006, POSITION_PID,
                  PID_POS_MAX_OUT_M2006, PID_POS_INT_LIM_M2006,
                  KP_M2006_POS, KI_M2006_POS, KD_M2006_POS);

  /* ===== AS5600 initialization ===== */
  BSP_AS5600_Init(&as5600_state, 0.2f);        /* Speed LPF coefficient 0.0~1.0 */
  osDelay(10);                                  /* Allow power-up settling (optional) */
  BSP_AS5600_SetZeroHere(&as5600_state);        /* Take current angle as zero (optional) */

#if (INCLUDE_vTaskDelayUntil == 1)
  TickType_t last = xTaskGetTickCount();
  for(;;)
  {
    KEY_Process();

    int16_t target_speed_m2006 = gear_m2006 * GEAR_STEP_RPM;
    int16_t target_speed_m3508 = gear_m3508 * GEAR_STEP_RPM;
    int16_t spd_ref_m2006 = dir[IDX_M2006] * target_speed_m2006;
    int16_t spd_ref_m3508 = dir[IDX_M3508] * target_speed_m3508;

    int16_t iq_m2006 = 0, iq_m3508 = 0;

    if (m2006_mode == M2006_MODE_SPEED) {
      pid_calc(&pid_spd[IDX_M2006], moto_chassis[IDX_M2006].speed_rpm, spd_ref_m2006);
      iq_m2006 = clip16(pid_spd[IDX_M2006].pos_out, IQ_LIMIT_M2006);
    }
    else if (m2006_mode == M2006_MODE_TORQUE) {
      float   tau_set       = gear_m2006 * TORQUE_STEP_NM;
      float   I_ref_A       = tau_to_currentA(tau_set);
      int16_t iq_ref_counts = dir[IDX_M2006] * A_to_counts(I_ref_A);
      int16_t iq_meas_counts= moto_chassis[IDX_M2006].given_current;
      pid_calc(&pid_tau_counts_m2006, iq_meas_counts, iq_ref_counts);
      iq_m2006 = clip16(pid_tau_counts_m2006.pos_out, IQ_LIMIT_M2006);
    }
    else { /* POSITION mode */
      float pos_ref_dir = dir[IDX_M2006] * pos_ref_m2006;
      float pos_meas    = moto_chassis[IDX_M2006].total_angle;
#if USE_POS_ERR_SHAPING_M2006
      pos_meas = err_shape_f(pos_ref_dir, pos_meas,
                             M2006_POS_DEADBAND_UNITS, M2006_POS_MAX_ERR_UNITS);
#endif
      pid_calc(&pid_pos_m2006, pos_meas, pos_ref_dir);
      int16_t spd_ref_from_pos = pid_pos_m2006.pos_out;
      pid_calc(&pid_spd[IDX_M2006], moto_chassis[IDX_M2006].speed_rpm, spd_ref_from_pos);
      iq_m2006 = clip16(pid_spd[IDX_M2006].pos_out, IQ_LIMIT_M2006);
    }

    pid_calc(&pid_spd[IDX_M3508], moto_chassis[IDX_M3508].speed_rpm, spd_ref_m3508);
    iq_m3508 = clip16(pid_spd[IDX_M3508].pos_out, IQ_LIMIT_M3508);

    set_moto_current(&hcan1, iq_m2006, iq_m3508, 0, 0);

    /* ===== Read AS5600 every 10ms ===== */
    BSP_AS5600_Update(&as5600_state, 0.01f);   /* 10ms = 0.01s */

    /* For debugging via UART, for example:
       printf("AS5600: ang=%.2f deg vel=%.2f dps md=%d\n",
              as5600_state.angle_deg, as5600_state.vel_dps, as5600_state.mag_ok);
    */

    vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
  }
#else
  for(;;) {
    KEY_Process();
    osDelay(10);
  }
#endif
}


