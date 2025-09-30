#ifndef __BSP_AS5600_H__
#define __BSP_AS5600_H__

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t zero_raw;          // Power-on / calibrated zero point (raw 0~4095)
    uint16_t last_raw;          // Last raw value (0~4095)
    int32_t  turns;             // Multi-turn counter
    float    angle_deg;         // Current angle (degrees; zero-adjusted + unwrapped)
    float    vel_dps;           // Velocity estimate (degrees per second)
    float    _prev_unwrap_deg;  // Internal: last unwrapped angle (before zero adjustment)
    float    lp_vel;            // Internal: low-pass filtered velocity state
    float    lp_alpha;          // Low-pass filter coefficient (0~1)
    uint8_t  mag_ok;            // Magnet detected (STATUS.MD)
} AS5600_State_t;

void BSP_AS5600_Init(AS5600_State_t *s, float vel_lpf_alpha);
void BSP_AS5600_SetZeroHere(AS5600_State_t *s);
void BSP_AS5600_Update(AS5600_State_t *s, float dt_s);

#ifdef __cplusplus
}
#endif
#endif

