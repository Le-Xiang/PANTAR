#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "pid.h"
#include "can.h"

/* M2006 operation modes */
typedef enum { 
    M2006_MODE_SPEED = 0, 
    M2006_MODE_TORQUE = 1, 
    M2006_MODE_POSITION = 2 
} m2006_mode_t;

/* Public interfaces */
void app_control_start(void);

void M2006_SetMode(m2006_mode_t mode);
void M2006_SetPosRef(float pos_units);
float M2006_GetPosRef(void);

#endif /* __APP_CONTROL_H */


