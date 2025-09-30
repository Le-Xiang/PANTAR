#include "bsp_as5600.h"
#include "as5600_port_hal.h"

static inline float wrap_deg(float x) {
    while (x > 180.f) x -= 360.f;
    while (x < -180.f) x += 360.f;
    return x;
}

void BSP_AS5600_Init(AS5600_State_t *s, float vel_lpf_alpha)
{
    if (!s) return;
    s->zero_raw = 0;
    s->last_raw = 0;
    s->turns    = 0;
    s->angle_deg= 0.f;
    s->vel_dps  = 0.f;
    s->lp_vel   = 0.f;
    if (vel_lpf_alpha < 0.f)      vel_lpf_alpha = 0.f;
    else if (vel_lpf_alpha > 1.f) vel_lpf_alpha = 1.f;
    s->lp_alpha = vel_lpf_alpha;
    s->_prev_unwrap_deg = 0.f;
    s->mag_ok  = 0;

    // Read once to initialize angle
    uint16_t raw = 0;
    if (AS5600_ReadAngle12(&raw) == HAL_OK) {
        s->last_raw = raw;
        s->_prev_unwrap_deg = (raw * 360.0f) / 4096.0f;
    }
    // Read status
    uint8_t st = 0;
    if (AS5600_ReadStatus(&st) == HAL_OK) {
        s->mag_ok = ((st & (1U<<5)) != 0); // MD = bit5
    }
}

void BSP_AS5600_SetZeroHere(AS5600_State_t *s)
{
    if (!s) return;
    uint16_t raw = 0;
    if (AS5600_ReadAngle12(&raw) == HAL_OK) {
        s->zero_raw = raw;
        s->last_raw = raw;
        s->turns    = 0;
        s->angle_deg= 0.f;
        s->vel_dps  = 0.f;
        s->lp_vel   = 0.f;
        s->_prev_unwrap_deg = (raw * 360.0f) / 4096.0f;
    }
}

void BSP_AS5600_Update(AS5600_State_t *s, float dt_s)
{
    if (!s || dt_s <= 0.f) return;

    uint16_t raw = 0;
    if (AS5600_ReadAngle12(&raw) != HAL_OK) {
        return; // Skip this frame if read fails
    }

    // Unwrapping: detect 0/4095 roll-over
    int16_t diff = (int16_t)raw - (int16_t)s->last_raw;
    if (diff >  2048) s->turns--; // High → Low across zero
    if (diff < -2048) s->turns++; // Low → High across zero
    s->last_raw = raw;

    // Unwrapped angle before zero offset (used for velocity)
    float unwrap_now = (raw * 360.0f) / 4096.0f + s->turns * 360.0f;

    // Velocity (deg/s): differentiate unwrapped angle + wrap suppression
    float d_deg = wrap_deg(unwrap_now - s->_prev_unwrap_deg);
    s->_prev_unwrap_deg = unwrap_now;
    float v_inst = d_deg / dt_s;

    // First-order low-pass filter
    s->lp_vel  = s->lp_vel + s->lp_alpha * (v_inst - s->lp_vel);
    s->vel_dps = s->lp_vel;

    // Absolute angle with zero offset applied
    int32_t raw_off = (int32_t)raw - (int32_t)s->zero_raw;
    if (raw_off < 0) raw_off += 4096;
    s->angle_deg = (raw_off * 360.0f) / 4096.0f + s->turns * 360.0f;

    // Status (can be polled less often; here read every update)
    uint8_t st = 0;
    if (AS5600_ReadStatus(&st) == HAL_OK) {
        s->mag_ok = ((st & (1U<<5)) != 0);
    }
}

