/**
 * @file MA732.hpp
 * @brief STM32 MA732 magnetic encoder register interface.
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

namespace Sensor::Encoder
{

template <typename SPIBus>
class MA732
{
public:
    using SPIHandle = typename SPIBus::Handle;

    static constexpr uint8_t READ_CMD = 0x40;
    static constexpr uint8_t WRITE_CMD = 0x80;

    explicit MA732(const SPIHandle& handle_) : handle(handle_) {}

    bool writeRegister(uint8_t address, uint8_t value) const
    {
        if (address > MAX_REGISTER_ADDRESS)
        {
            return false;
        }

        uint16_t rxFrame = 0;
        const uint16_t txFrame = makeCommand(WRITE_CMD, address, value);
        if (!transferFrame(txFrame, rxFrame))
        {
            return false;
        }

        HAL_Delay(REGISTER_WRITE_DELAY_MS);
        return true;
    }

    bool readRegister(uint8_t address, uint8_t& value) const
    {
        if (address > MAX_REGISTER_ADDRESS)
        {
            return false;
        }

        typename SPIBus::Transaction transaction(handle);
        if (!transaction)
        {
            return false;
        }

        uint16_t rxFrame = 0;
        if (!transferFrameSelected(makeCommand(READ_CMD, address), rxFrame))
        {
            return false;
        }

        HAL_Delay(REGISTER_READ_GAP_MS);
        if (!transferFrameSelected(0, rxFrame))
        {
            return false;
        }

        value = static_cast<uint8_t>((rxFrame >> 8) & 0xFF);
        return true;
    }

private:
    static constexpr uint8_t MAX_REGISTER_ADDRESS = 0x1F;
    static constexpr uint32_t TRANSFER_TIMEOUT_MS = 100;
    static constexpr uint32_t REGISTER_READ_GAP_MS = 1;
    static constexpr uint32_t REGISTER_WRITE_DELAY_MS = 20;

    static constexpr uint16_t makeCommand(uint8_t command, uint8_t address, uint8_t data = 0)
    {
        return static_cast<uint16_t>(command) << 8 |
               static_cast<uint16_t>(address) << 8 |
               static_cast<uint16_t>(data);
    }

    bool transferFrame(uint16_t txFrame, uint16_t& rxFrame) const
    {
        return SPIBus::transfer(handle,
                                reinterpret_cast<const uint8_t*>(&txFrame),
                                sizeof(txFrame),
                                reinterpret_cast<uint8_t*>(&rxFrame),
                                sizeof(rxFrame),
                                TRANSFER_TIMEOUT_MS);
    }

    bool transferFrameSelected(uint16_t txFrame, uint16_t& rxFrame) const
    {
        return SPIBus::transferSelected(handle,
                                        reinterpret_cast<const uint8_t*>(&txFrame),
                                        sizeof(txFrame),
                                        reinterpret_cast<uint8_t*>(&rxFrame),
                                        sizeof(rxFrame),
                                        TRANSFER_TIMEOUT_MS);
    }

    SPIHandle handle;
};

} // namespace Sensor::Encoder

#endif // HARDWARE_PLATFORM == HARDWARE_PLATFORM_STM32
