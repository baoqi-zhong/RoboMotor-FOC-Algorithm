/**
 * @file AT24Cxx.cpp
 * @brief STM32 AT24Cxx EEPROM driver implementation.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "AT24Cxx.hpp"

#if (PLATFORM_ST) && (USE_AT24CXX)

namespace Drivers
{
namespace AT24Cxx
{
uint8_t read(void* i2cHandle, uint16_t addr, uint8_t* data, uint16_t len)
{
    if (!i2cHandle)
        return 1;
    
    if (HAL_I2C_Mem_Read(static_cast<I2C_HandleTypeDef*>(i2cHandle), DEVICE_ADDRESS, addr, I2C_MEMADD_SIZE_8BIT, data, len, 100) != HAL_OK)
    {
        return 1;
    }
    return 0;
}

uint8_t write(void* i2cHandle, uint16_t addr, uint8_t* data, uint16_t len)
{
    if (!i2cHandle)
        return 1;

    if (HAL_I2C_Mem_Write(static_cast<I2C_HandleTypeDef*>(i2cHandle), DEVICE_ADDRESS, addr, I2C_MEMADD_SIZE_8BIT, data, len, 100) != HAL_OK)
    {
        return 1;
    }
    return 0;
}

} // namespace AT24Cxx
} // namespace Drivers

#endif // PLATFORM_ST && USE_AT24CXX
