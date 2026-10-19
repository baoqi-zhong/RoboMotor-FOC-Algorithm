/**
 * @file I2CBus.hpp
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

#include <stdint.h>

namespace HAL
{
template <typename Platform, auto Handle>
class I2CBus
{
public:
    /**
     * @brief Read a register from an I2C device.
     * @param deviceAddress The 7-bit I2C address of the device.
     * @param registerAddress The address of the register to read.
     * @param registerAddressSize The size of the register address. Could be 1 or 2 bytes.
     * @param data The buffer to store the read data.
     * @param length The number of bytes to read.
     * @param timeout The timeout for the operation.
     */
    static bool readRegister(uint16_t deviceAddress, uint16_t registerAddress, uint8_t registerAddressSize, uint8_t* data, uint16_t length, uint32_t timeout = 100)
    {
        if (deviceAddress > 0x7F)
        {
            return false;
        }
        if (registerAddressSize != 1 && registerAddressSize != 2)
        {
            return false;
        }
        if (registerAddressSize == 1 && registerAddress > UINT8_MAX)
        {
            return false;
        }
        if (data == nullptr || length == 0)
        {
            return false;
        }
        return Platform::readRegister(Handle, deviceAddress, registerAddress, registerAddressSize, data, length, timeout);
    }

    /**
     * @brief Write a register to an I2C device.
     * @param deviceAddress The 7-bit I2C address of the device.
     * @param registerAddress The address of the register to write.
     * @param registerAddressSize The size of the register address. Could be 1 or 2 bytes.
     * @param data The buffer containing the data to write.
     * @param length The number of bytes to write.
     * @param timeout The timeout for the operation.
     */
    static bool writeRegister(uint16_t deviceAddress, uint16_t registerAddress, uint8_t registerAddressSize, const uint8_t* data, uint16_t length, uint32_t timeout = 100)
    {
        if (deviceAddress > 0x7F)
        {
            return false;
        }
        if (registerAddressSize != 1 && registerAddressSize != 2)
        {
            return false;
        }
        if (registerAddressSize == 1 && registerAddress > UINT8_MAX)
        {
            return false;
        }
        if (data == nullptr || length == 0)
        {
            return false;
        }
        return Platform::writeRegister(Handle, deviceAddress, registerAddress, registerAddressSize, data, length, timeout);
    }
};
    
} // namespace HAL
