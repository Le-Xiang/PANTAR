#include "as5600_port_hal.h"

#ifndef AS5600_I2C_EXTERN
extern I2C_HandleTypeDef AS5600_I2C_HANDLE;
#endif

u8 AS5600_IIC_Read_OneByte(u8 deviceaddr, u8 reg)
{
    uint8_t data = 0;
    if (HAL_I2C_Mem_Read(&AS5600_I2C_HANDLE, deviceaddr, reg,
                         I2C_MEMADD_SIZE_8BIT, &data, 1, 5) != HAL_OK) {
        return 0;
    }
    return data;
}

HAL_StatusTypeDef AS5600_ReadAngle12(u16 *angle12)
{
    if (!angle12) return HAL_ERROR;
    uint8_t h = 0, l = 0;
    if (HAL_I2C_Mem_Read(&AS5600_I2C_HANDLE, AS5600_ADDR_8B, 0x0E,
                         I2C_MEMADD_SIZE_8BIT, &h, 1, 5) != HAL_OK) return HAL_ERROR;
    if (HAL_I2C_Mem_Read(&AS5600_I2C_HANDLE, AS5600_ADDR_8B, 0x0F,
                         I2C_MEMADD_SIZE_8BIT, &l, 1, 5) != HAL_OK) return HAL_ERROR;
    *angle12 = ((((uint16_t)h << 8) | l) & 0x0FFF);
    return HAL_OK;
}

HAL_StatusTypeDef AS5600_ReadStatus(u8 *status)
{
    if (!status) return HAL_ERROR;
    return HAL_I2C_Mem_Read(&AS5600_I2C_HANDLE, AS5600_ADDR_8B, 0x0B,
                            I2C_MEMADD_SIZE_8BIT, status, 1, 5);
}
