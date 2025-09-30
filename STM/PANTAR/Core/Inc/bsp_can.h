/******************************************************************************
/// @brief
/// @copyright Copyright (c) 2017 <dji-innovations, Corp. RM Dept.>
/// @license MIT License
*******************************************************************************/
#ifndef __BSP_CAN
#define __BSP_CAN

#include "stm32f4xx_hal.h"
#include "mytype.h"
#include "can.h"   // CubeMX generated CAN handles, etc.

#ifdef __cplusplus
extern "C" {
#endif

/* CAN transmit/receive IDs */
typedef enum
{
    CAN_TxPY12V_ID        = 0x200,
    CAN_TxPY24V_ID        = 0x1FF,

    CAN_YAW_FEEDBACK_ID   = 0x205,
    CAN_PIT_FEEDBACK_ID   = 0x206,
    CAN_POKE_FEEDBACK_ID  = 0x207,
    CAN_ZGYRO_RST_ID      = 0x404,
    CAN_ZGYRO_FEEDBACK_MSG_ID = 0x401,

    CAN_MotorLF_ID        = 0x041,
    CAN_MotorRF_ID        = 0x042,
    CAN_MotorLB_ID        = 0x043,
    CAN_MotorRB_ID        = 0x044,

    CAN_EC60_four_ID      = 0x200,
    CAN_backLeft_EC60_ID  = 0x203,
    CAN_frontLeft_EC60_ID = 0x201,
    CAN_backRight_EC60_ID = 0x202,
    CAN_frontRight_EC60_ID= 0x204,

    // add by langgo
    CAN_3510Moto_ALL_ID   = 0x200,
    CAN_3510Moto1_ID      = 0x201,
    CAN_3510Moto2_ID      = 0x202,
    CAN_3510Moto3_ID      = 0x203,
    CAN_3510Moto4_ID      = 0x204,
    CAN_DriverPower_ID    = 0x80,

    CAN_HeartBeat_ID      = 0x156,
} CAN_Message_ID;

#define FILTER_BUF_LEN  (5)

/* Parameters structure for received motor feedback */
typedef struct{
    int16_t   speed_rpm;
    int16_t   real_current;
    int16_t   given_current;
    uint8_t   hall;
    uint16_t  angle;         // absolute angle [0,8191]
    uint16_t  last_angle;
    uint16_t  offset_angle;
    int32_t   round_cnt;
    int32_t   total_angle;
    u8        buf_idx;
    u16       angle_buf[FILTER_BUF_LEN];
    u16       fited_angle;
    u32       msg_cnt;
} moto_measure_t;

/* Globals */
extern moto_measure_t moto_chassis[4];
extern moto_measure_t moto_yaw, moto_pit, moto_poke, moto_info;
extern float real_current_from_judgesys;
extern float dynamic_limit_current;
extern float ZGyroMod


