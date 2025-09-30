/******************************************************************************
/// @brief
/// @copyright Copyright (c) 2017 <dji-innovations, Corp. RM Dept.>
/// @license MIT License
*******************************************************************************/
#ifndef __pid_H
#define __pid_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

enum{
    LLAST = 0,
    LAST  = 1,
    NOW   = 2,

    POSITION_PID,
    DELTA_PID,
};

typedef struct __pid_t
{
    float p;
    float i;
    float d;

    float set[3];
    float get[3];
    float err[3];

    float pout;
    float iout;
    float dout;

    float pos_out;
    float last_pos_out;
    float delta_u;
    float delta_out;
    float last_delta_out;

    float max_err;
    float deadband;
    uint32_t pid_mode;
    uint32_t MaxOutput;
    uint32_t IntegralLimit;

    void (*f_param_init)(struct __pid_t *pid,
                         uint32_t pid_mode,
                         uint32_t maxOutput,
                         uint32_t integralLimit,
                         float p,
                         float i,
                         float d);
    void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d);

} pid_t;

void PID_struct_init(pid_t* pid,
                     uint32_t mode,
                     uint32_t maxout,
                     uint32_t intergral_limit,
                     float kp,
                     float ki,
                     float kd);

float pid_calc(pid_t* pid, float fdb, float ref);

/* These externs are referenced in your original demo, so keep them */
extern pid_t pid_rol;
extern pid_t pid_pit;
extern pid_t pid_yaw;
extern pid_t pid_pit_omg;
extern pid_t pid_yaw_omg;
extern pid_t pid_spd[4];
extern pid_t pid_yaw_alfa;
extern pid_t pid_chassis_angle;
extern pid_t pid_poke;
extern pid_t pid_poke_omg;
extern pid_t pid_imu_tmp;
extern pid_t pid_cali_bby;
extern pid_t pid_cali_bbp;
extern pid_t pid_omg;
extern pid_t pid_pos;

#endif

