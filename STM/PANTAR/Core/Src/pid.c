/******************************************************************************
/// @brief
/// @copyright Copyright (c) 2017 <dji-innovations, Corp. RM Dept.>
/// @license MIT License
*******************************************************************************/
/**
  ******************************************************************************
  * @file      pid.c
  * @version   V1.0.0
  * @date      2016-11-11 17:21:36
  *******************************************************************************/

#include "pid.h"
#include "mytype.h"
#include <math.h>

#define ABSf(x)   (( (x) > 0.0f ) ? (x) : -(x))

static void pid_param_init(pid_t *pid,
                           uint32_t mode,
                           uint32_t maxout,
                           uint32_t intergral_limit,
                           float kp,
                           float ki,
                           float kd)
{
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput     = maxout;
    pid->pid_mode      = mode;

    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}

static void pid_reset(pid_t *pid, float kp, float ki, float kd)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}

static void abs_limit(float *a, float ABS_MAX){
    if (*a >  ABS_MAX) *a =  ABS_MAX;
    if (*a < -ABS_MAX) *a = -ABS_MAX;
}

/* Position / Incremental PID calculation */
float pid_calc(pid_t* pid, float get, float set)
{
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;

    if (pid->max_err != 0 && ABSf(pid->err[NOW]) > pid->max_err) return 0.0f;
    if (pid->deadband != 0 && ABSf(pid->err[NOW]) < pid->deadband) return 0.0f;

    if (pid->pid_mode == POSITION_PID) {
        pid->pout = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);

        abs_limit(&pid->iout,    pid->IntegralLimit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&pid->pos_out, pid->MaxOutput);
        pid->last_pos_out = pid->pos_out;
    } else { // DELTA_PID
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2.0f*pid->err[LAST] + pid->err[LLAST]);

        abs_limit(&pid->iout,     pid->IntegralLimit);
        pid->delta_u   = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        abs_limit(&pid->delta_out, pid->MaxOutput);
        pid->last_delta_out = pid->delta_out;
    }

    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST]  = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST]  = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST]  = pid->set[NOW];

    return (pid->pid_mode == POSITION_PID) ? pid->pos_out : pid->delta_out;
}

/* Special: PID with gyro compensation (kept from your previous code) */
float pid_sp_calc(pid_t* pid, float get, float set, float gyro)
{
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;

    if (pid->pid_mode == POSITION_PID) {
        pid->pout = pid->p * pid->err[NOW];
        if (fabsf(pid->i) >= 0.001f) pid->iout += pid->i * pid->err[NOW];
        else                         pid->iout  = 0.0f;
        pid->dout = -pid->d * (gyro / 100.0f);

        abs_limit(&pid->iout,    pid->IntegralLimit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&pid->pos_out, pid->MaxOutput);
        pid->last_pos_out = pid->pos_out;
    } else {
        /* Incremental branch left empty in your old code; keep unchanged */
    }

    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST]  = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST]  = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST]  = pid->set[NOW];

    return (pid->pid_mode == POSITION_PID) ? pid->pos_out : pid->delta_out;
}

void PID_struct_init(pid_t* pid,
                     uint32_t mode,
                     uint32_t maxout,
                     uint32_t intergral_limit,
                     float kp, float ki, float kd)
{
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset  = pid_reset;
    pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
}


/* ===== Global PID objects (linked across modules) ===== */
pid_t pid_omg = {0};
pid_t pid_pos = {0};
pid_t pid_spd[4] = {0};
/* These globals may be defined elsewhere, don’t duplicate to avoid symbol conflicts */
// pid_t pid_rol, pid_pit, pid_yaw, pid_pit_omg, pid_yaw_omg;
// pid_t pid_spd[4];
// pid_t pid_yaw_alfa, pid_chassis_angle, pid_poke, pid_poke_omg;
// pid_t pid_imu_tmp, pid_cali_bby, pid_cali_bbp, pid_omg, pid_pos;

void pid_test_init(void)
{
    /* Keep original empty implementation */
}

