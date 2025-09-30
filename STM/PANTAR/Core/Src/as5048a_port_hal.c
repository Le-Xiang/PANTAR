#include "as5048a_port_hal.h"

/* ---------------- Internal definitions ---------------- */
#define AS5048A_REG_NOP            0x0000u
#define AS5048A_REG_CLEAR_ERRFL    0x0001u
#define AS5048A_REG_ZPOS_HI        0x0016u
#define AS5048A_REG_ZPOS_LO        0x0017u
#define AS5048A_REG_DIAAGC         0x3FFDu
#define AS5048A_REG_MAG            0x3FFEu
#define AS5048A_REG_ANGLE          0x3FFFu

/* SPI command frame: [15]=PAR (even parity) [14]=RWn (1=read, 0=write) [13:0]=address */
#define AS5048A_CMD_RW_READ        1u
#define AS5048A_CMD_RW_WRITE       0u

/* Simple timeout (ms) */
#ifndef AS5048A_HAL_TIMEOUT
#define AS5048A_HAL_TIMEOUT        5u
#endif

/* Drive CS low/high (if using hardware NSS, these can be no-ops) */
static inline void CS_L(void) {
    HAL_GPIO_WritePin(AS5048A_CS_GPIO_Port, AS5048A_CS_Pin, GPIO_PIN_RESET);
}
static inline void CS_H(void) {
    HAL_GPIO_WritePin(AS5048A_CS_GPIO_Port, AS5048A_CS_Pin, GPIO_PIN_SET);
}

/* ---------------- Helper functions ---------------- */

/* Even parity over 15 bits (bit14..bit0); returns the PAR bit to place into bit15 */
u8 AS5048A_EvenParity15(u16 x)
{
    /* Count ones over bit14..bit0 only */
    x &= 0x7FFFu;
    /* Common bit-parity fold */
    u16 v = x;
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    /* (v & 1) is 1 if odd number of ones. Even parity requires total ones to be even:
       odd → PAR=1, even → PAR=0 */
    return (u8)(v & 1u);
}

/* Build a 16-bit command frame (with PAR) */
static u16 AS5048A_MakeCmd(u8 rw, u16 addr14)
{
    u16 frame15 = ((rw & 1u) << 14) | (addr14 & 0x3FFFu);   /* bits 15..1 */
    u16 par = (u16)AS5048A_EvenParity15(frame15);
    return (u16)((par << 15) | frame15);
}

/* Single 16-bit SPI exchange (configure SPI as 16-bit; if using 8-bit mode, send two bytes) */
HAL_StatusTypeDef AS5048A_SPI_Transfer16(u16 tx, u16 *rx)
{
    HAL_StatusTypeDef st;
    CS_L();
    st = HAL_SPI_TransmitReceive(&AS5048A_SPI_HANDLE,
                                 (uint8_t*)&tx, (uint8_t*)rx,
                                 1, AS5048A_HAL_TIMEOUT);
    CS_H();

    /* Timing requirement: CS high ≥ 350 ns. HAL overhead is typically sufficient;
       add a tiny delay if needed. */
    return st;
}

/* Two-frame read:
   1) send READ(cmd), 2) send NOP to fetch the data.
   Returned MISO frame format: [15]=PAR [14]=EF [13:0]=DATA */
HAL_StatusTypeDef AS5048A_ReadRegister(u16 reg, u16 *value)
{
    if (!value) return HAL_ERROR;

    HAL_StatusTypeDef st;
    u16 rx;

    /* First frame: send READ command (address = reg) */
    u16 cmd = AS5048A_MakeCmd(AS5048A_CMD_RW_READ, reg);
    st = AS5048A_SPI_Transfer16(cmd, &rx);
    if (st != HAL_OK) return st;

    /* Second frame: send NOP to retrieve data for previous READ */
    u16 nop = AS5048A_MakeCmd(AS5048A_CMD_RW_READ, AS5048A_REG_NOP);
    st = AS5048A_SPI_Transfer16(nop, &rx);
    if (st != HAL_OK) return st;

    /* Check EF (error flag) */
    if ((rx & 0x4000u) != 0u) {
        /* Communication error; consider clearing error and/or returning raw value */
        *value = (rx & 0x3FFFu);
        return HAL_ERROR;
    }

    *value = (rx & 0x3FFFu);  /* 14-bit data */
    return HAL_OK;
}

/* Register write (two frames: first the write command, then the data frame) */
HAL_StatusTypeDef AS5048A_WriteRegister(u16 reg, u16 value)
{
    HAL_StatusTypeDef st;
    u16 rx;

    /* Write command */
    u16 cmd = AS5048A_MakeCmd(AS5048A_CMD_RW_WRITE, reg);
    st = AS5048A_SPI_Transfer16(cmd, &rx);
    if (st != HAL_OK) return st;

    /* Data frame: [15]=PAR [14]=R (must be 0) [13:0]=Data */
    u16 data15 = (value & 0x3FFFu);                /* 14-bit data */
    u16 frame15 = (0u << 14) | data15;             /* R bit = 0 */
    u16 par = (u16)AS5048A_EvenParity15(frame15);
    u16 data_frame = (u16)((par << 15) | frame15);

    st = AS5048A_SPI_Transfer16(data_frame, &rx);
    if (st != HAL_OK) return st;

    return HAL_OK;
}

/* Read 14-bit angle (0..16383) */
HAL_StatusTypeDef AS5048A_ReadAngle14(u16 *angle14)
{
    if (!angle14) return HAL_ERROR;
    u16 val = 0;
    HAL_StatusTypeDef st = AS5048A_ReadRegister(AS5048A_REG_ANGLE, &val);
    if (st != HAL_OK) return st;
    *angle14 = (val & 0x3FFFu);
    return HAL_OK;
}

/* Read diagnostics/status (DIAAGC: 0x3FFD; contains OCF/COF flags and AGC value) */
HAL_StatusTypeDef AS5048A_ReadStatus(u16 *status)
{
    if (!status) return HAL_ERROR;
    return AS5048A_ReadRegister(AS5048A_REG_DIAAGC, status);
}

/* Clear error flag (Clear Error Flag command: 0x0001, treated as a READ):
   1) send READ 0x0001, 2) send NOP to receive and clear EF */
HAL_StatusTypeDef AS5048A_ClearErrorFlag(void)
{
    HAL_StatusTypeDef st;
    u16 rx;

    /* Send CLEAR_ERRFL command */
    u16 cmd = AS5048A_MakeCmd(AS5048A_CMD_RW_READ, AS5048A_REG_CLEAR_ERRFL);
    st = AS5048A_SPI_Transfer16(cmd, &rx);
    if (st != HAL_OK) return st;

    /* Send NOP to read back error register and finish the clear */
    u16 nop = AS5048A_MakeCmd(AS5048A_CMD_RW_READ, AS5048A_REG_NOP);
    st = AS5048A_SPI_Transfer16(nop, &rx);
    return st;   /* If needed, extract error content via (rx & 0x3FFF) */
}

