/**
 * @file AT24Cxx.cpp
 * @brief I2C EEPROM driver (AT24Cxx series) implementation.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "AT24Cxx.hpp"

#if USE_AT24CXX
#include "i2c.h"

// 非常怪的问题, 只能在 fast mode plus 下工作??
uint8_t AT24Cxx_Read(uint16_t addr, uint8_t *data, uint16_t len)
{
    if(HAL_I2C_Mem_Read(&AT24CXX_I2C, AT24Cxx_ADDRESS, addr, I2C_MEMADD_SIZE_8BIT, data, len, 100) != HAL_OK)
    {
        return -1;
    }
    return 0;
}

uint8_t AT24Cxx_Write(uint16_t addr, uint8_t *data, uint16_t len)
{
    if(HAL_I2C_Mem_Write(&AT24CXX_I2C, AT24Cxx_ADDRESS, addr, I2C_MEMADD_SIZE_8BIT, data, len, 100) != HAL_OK)
    {
        return -1;
    }
    return 0;
}

#endif // USE_AT24CXX