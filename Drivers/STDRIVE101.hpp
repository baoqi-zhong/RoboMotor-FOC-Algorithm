/**
 * @file STDRIVE101.hpp
 * @brief STM32 STSPIN32G4 MOSFET driver interface.
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

namespace Drivers
{

template <typename I2CBus, typename GPIO>
class STDRIVE101
{
public:
    struct Pins
    {
        typename GPIO::Handle enable;
        typename GPIO::Handle fault;
    };

    static constexpr uint16_t DEVICE_ADDRESS = 0x47;

    static constexpr uint8_t REGISTER_POWMNG = 0x01;
    static constexpr uint8_t REGISTER_LOGIC = 0x02;
    static constexpr uint8_t REGISTER_READY = 0x03;
    static constexpr uint8_t REGISTER_NFAULT = 0x08;
    static constexpr uint8_t REGISTER_CLEAR = 0x09;
    static constexpr uint8_t REGISTER_STBY = 0x0A;
    static constexpr uint8_t REGISTER_LOCK = 0x0B;
    static constexpr uint8_t REGISTER_RESET = 0x0C;
    static constexpr uint8_t REGISTER_STATUS = 0x80;

    explicit STDRIVE101(const Pins& pins_) : pins(pins_) {}

    bool setEnabled(bool enabled) const
    {
        return GPIO::write(pins.enable, enabled);
    }

    bool isFaultActive(bool& active) const
    {
        bool level = false;
        if (!GPIO::read(pins.fault, level))
        {
            return false;
        }

        active = !level;
        return true;
    }

    bool init() const
    {
        // The power stage must be supplied before its I2C interface is available.
        if (!setEnabled(true))
        {
            return false;
        }
        HAL_Delay(ENABLE_DELAY_MS);

        if (!writeRegister(REGISTER_RESET, 0xFF))
        {
            return false;
        }
        if (!writeRegisterVerify(REGISTER_LOCK, 0x0F))
        {
            return false;
        }
        if (!writeRegisterVerify(REGISTER_POWMNG, 0x01))
        {
            return false;
        }
        if (!writeRegisterVerify(REGISTER_LOGIC, 0x71))
        {
            return false;
        }
        if (!writeRegister(REGISTER_CLEAR, 0xFF))
        {
            return false;
        }
        if (!writeRegisterVerify(REGISTER_LOCK, 0x00))
        {
            return false;
        }

        return true;
    }

    bool readStatus(uint8_t& status) const
    {
        return readRegister(REGISTER_STATUS, status);
    }

    bool clearFault() const
    {
        return writeRegister(REGISTER_CLEAR, 0xFF);
    }

private:
    static constexpr uint8_t REGISTER_ADDRESS_SIZE = 1;
    static constexpr uint32_t I2C_TIMEOUT_MS = 100;
    static constexpr uint32_t ENABLE_DELAY_MS = 1;

    bool readRegister(uint8_t address, uint8_t& value) const
    {
        return I2CBus::readRegister(DEVICE_ADDRESS,
                                    address,
                                    REGISTER_ADDRESS_SIZE,
                                    &value,
                                    1,
                                    I2C_TIMEOUT_MS);
    }

    bool writeRegister(uint8_t address, uint8_t value) const
    {
        return I2CBus::writeRegister(DEVICE_ADDRESS,
                                     address,
                                     REGISTER_ADDRESS_SIZE,
                                     &value,
                                     1,
                                     I2C_TIMEOUT_MS);
    }

    bool writeRegisterVerify(uint8_t address, uint8_t value) const
    {
        if (!writeRegister(address, value))
        {
            return false;
        }

        uint8_t readback = 0;
        return readRegister(address, readback) && readback == value;
    }

    Pins pins;
};

} // namespace Drivers

#endif // HARDWARE_PLATFORM == HARDWARE_PLATFORM_STM32
