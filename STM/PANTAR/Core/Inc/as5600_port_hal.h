#ifndef __AS5600_PORT_HAL_H__
#define __AS5600_PORT_HAL_H__

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef u8
#define u8  uint8_t
#endif
#ifndef u16
#define u16 uint16_t
#endif

// 7-bit / 8-bit I2C address (following your demo style: pass 8-bit address 0x36<<1)
#define AS5600_ADDR_7B   (0x36U)
#define AS5600_ADDR_8B   (AS5600_ADDR_7B << 1)

// By default use I2C2; you can define AS5600_I2C_HANDLE before including this header
// to switch to another handle (e.g. hi2c1 / hi2c3).
#ifndef AS5600_I2C_EXTERN
extern I2C_HandleTypeDef hi2c2;      
#define AS5600_I2C_HANDLE hi2c2
#else
// If you declared `extern I2C_HandleTypeDef AS5600_I2C_HANDLE;` elsewhere
#endif

// Single-byte read (compatible with demo)
u8  AS5600_IIC_Read_OneByte(u8 deviceaddr, u8 reg);

// Convenience API: read 12-bit angle (0~4095)
HAL_StatusTypeDef AS5600_ReadAngle12(u16 *angle12);

// Read status register 0x0B (MD/ML/MH bits)
HAL_StatusTypeDef AS5600_ReadStatus(u8 *status);

#ifdef __cplusplus
}
#endif
#endif

