/**
 * @file ST_I2CBus.hpp
 * @brief Hardware abstraction layer for I2C bus communication.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#pragma once

#include "Config.hpp"
#if (HARDWARE_PLATFORM == HARDWARE_PLATFORM_STM32)

#include <stdint.h>
#include "main.h"

#if defined(HAL_I2C_MODULE_ENABLED)

namespace HAL
{
class ST_I2CBus
{
public:
    using Handle = I2C_HandleTypeDef*;

    static bool readRegister(I2C_HandleTypeDef* hi2c, uint16_t deviceAddress, uint16_t registerAddress, uint8_t registerAddressSize, uint8_t* data, uint16_t length, uint32_t timeout)
    {
        if (hi2c == nullptr)
        {
            return false;
        }
        const uint16_t halDeviceAddress = static_cast<uint16_t>(deviceAddress << 1);
        return (HAL_I2C_Mem_Read(hi2c, halDeviceAddress, registerAddress, (registerAddressSize == 1) ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT, data, length, timeout) == HAL_OK);
    }

    static bool writeRegister(I2C_HandleTypeDef* hi2c, uint16_t deviceAddress, uint16_t registerAddress, uint8_t registerAddressSize, const uint8_t* data, uint16_t length, uint32_t timeout)
    {
        if (hi2c == nullptr)
        {
            return false;
        }
        const uint16_t halDeviceAddress = static_cast<uint16_t>(deviceAddress << 1);
        return (HAL_I2C_Mem_Write(hi2c, halDeviceAddress, registerAddress, (registerAddressSize == 1) ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT, const_cast<uint8_t*>(data), length, timeout) == HAL_OK);
    }
};
    
} // namespace HAL

#endif // HAL_I2C_MODULE_ENABLED

#endif
