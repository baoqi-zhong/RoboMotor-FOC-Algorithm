/**
 * @file AT24Cxx.hpp
 * @brief Generic AT24Cxx EEPROM driver interface.
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

namespace Drivers
{
template <typename I2CBus>
class AT24Cxx
{
public:
    bool read(uint16_t addr, uint8_t* data, uint16_t len) const
    {
        return I2CBus::readRegister(DEVICE_ADDRESS, addr, MEMORY_ADDRESS_SIZE, data, len, OPERATION_TIMEOUT_MS);
    }

    bool write(uint16_t addr, const uint8_t* data, uint16_t len) const
    {
        return I2CBus::writeRegister(DEVICE_ADDRESS, addr, MEMORY_ADDRESS_SIZE, data, len, OPERATION_TIMEOUT_MS);
    }

private:
    static constexpr uint16_t DEVICE_ADDRESS = 0x50;
    static constexpr uint8_t MEMORY_ADDRESS_SIZE = 1;
    static constexpr uint32_t OPERATION_TIMEOUT_MS = 100;
}; 
} // namespace Drivers
