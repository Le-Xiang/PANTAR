#ifndef __AS5048A_PORT_HAL_H__
#define __AS5048A_PORT_HAL_H__

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
#ifndef u32
#define u32 uint32_t
#endif

/* ======================= Configuration section (modify as needed) ======================= */
/* 1) SPI handle: declare `extern SPI_HandleTypeDef hspiX;` elsewhere, then select here */
#ifndef AS5048A_SPI_EXTERN
extern SPI_HandleTypeDef hspi1;           /* Default use hspi1 */
#define AS5048A_SPI_HANDLE  hspi1
#else
/* If you declared `extern SPI_HandleTypeDef AS5048A_SPI_HANDLE;` elsewhere */
#endif

/* 2) Chip select (CS) pin: if you use hardware NSS, leave these empty and implement with HW NSS */
#ifndef AS5048A_CS_GPIO_Port
#define AS5048A_CS_GPIO_Port   GPIOA       /* Adjust to your wiring */
#endif
#ifndef AS5048A_CS_Pin
#define AS5048A_CS_Pin         GPIO_PIN_4  /* Adjust to your wiring */
#endif
/* ======================================================================================== */

/* Angle resolution and range (AS5048A = 14-bit) */
#define AS5048A_ANGLE_BITS     (14U)
#define AS5048A_ANGLE_MAX      ((1U << AS5048A_ANGLE_BITS) - 1U)  /* 0..16383 */

/* SPI frame basics:
 * - 16-bit frame: bit15 = R/W (1=read, 0=write), bit14..bit0 = address/data
 * - Even parity is calculated over 15 bits (bit14..bit0), result goes into bit15 (different
 *   datasheets describe bit definitions slightly differently; handle consistently in .c file)
 * - This header only provides the interface; frame building/parity is implemented in .c
 */

/* ======================= Common high-level interfaces ======================= */

/* Low-level: one 16-bit transfer (handles CS low/high and calls HAL_SPI_TransmitReceive) */
HAL_StatusTypeDef AS5048A_SPI_Transfer16(u16 tx, u16 *rx);

/* Compute even parity for 15-bit value (details implemented/tested in .c) */
u8  AS5048A_EvenParity15(u16 x);

/* Read register (reg = 15-bit address; function handles frame/parity; note AS5048A reads
 * typically require a “two-frame” mechanism) */
HAL_StatusTypeDef AS5048A_ReadRegister(u16 reg, u16 *value);

/* Write register (used for zero-position configuration, etc.; many applications only need reads) */
HAL_StatusTypeDef AS5048A_WriteRegister(u16 reg, u16 value);

/* Read 14-bit angle (0..16383) */
HAL_StatusTypeDef AS5048A_ReadAngle14(u16 *angle14);

/* Read diagnostic/status register (names vary: STATUS / DIAAGC / ERRFL; fix the exact address in .c) */
HAL_StatusTypeDef AS5048A_ReadStatus(u16 *status);

/* Clear error flag (some implementations require a specific command frame; implemented in .c) */
HAL_StatusTypeDef AS5048A_ClearErrorFlag(void);

/* Optional: convert 14-bit angle to degrees (0.0~360.0) for convenience */
static inline float AS5048A_AngleToDeg(u16 angle14)
{
    return (360.0f * (float)angle14) / (float)(AS5048A_ANGLE_MAX + 1U);
}

#ifdef __cplusplus
}
#endif
#endif /* __AS5048A_PORT_HAL_H__ */


