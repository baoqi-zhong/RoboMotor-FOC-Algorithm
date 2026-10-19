/**
 * @file ST_SPIBus.hpp
 * @brief Hardware abstraction layer for SPI bus communication.
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
#include "spi.h"
#include "GPIO.hpp"
#include "ST/ST_GPIO.hpp"

namespace HAL
{
/**
 * @brief STM32 SPI peripheral and its active-low chip-select GPIO.
 */
struct ST_SPIHandle
{
    SPI_HandleTypeDef* hspi = nullptr;
    ST_GPIO::Handle chipSelect;
};

class ST_SPIBus
{
public:
    using Handle = ST_SPIHandle;
    using GPIO = HAL::GPIO<HAL::ST_GPIO>;

    static bool select(const Handle& handle)
    {
        return handle.hspi != nullptr && GPIO::write(handle.chipSelect, false);
    }

    static void deselect(const Handle& handle)
    {
        GPIO::write(handle.chipSelect, true);
    }

    static bool transferSelected(const Handle& handle, const uint8_t* txBuffer, uint32_t txLength, uint8_t* rxBuffer, uint32_t rxLength, uint32_t timeout)
    {
        if (handle.hspi == nullptr)
        {
            return false;
        }

        uint32_t bytesPerFrame = 0;
        if (handle.hspi->Init.DataSize == SPI_DATASIZE_8BIT)
        {
            bytesPerFrame = 1;
        }
        else if (handle.hspi->Init.DataSize == SPI_DATASIZE_16BIT)
        {
            bytesPerFrame = 2;
        }
        else
        {
            return false;
        }

        if (txLength % bytesPerFrame != 0 || rxLength % bytesPerFrame != 0)
        {
            return false;
        }
        if (bytesPerFrame == 2)
        {
            const bool txUnaligned = txLength != 0 && (reinterpret_cast<uintptr_t>(txBuffer) % bytesPerFrame != 0);
            const bool rxUnaligned = rxLength != 0 && (reinterpret_cast<uintptr_t>(rxBuffer) % bytesPerFrame != 0);
            if (txUnaligned || rxUnaligned)
            {
                return false;
            }
        }

        const uint32_t txFrames = txLength / bytesPerFrame;
        const uint32_t rxFrames = rxLength / bytesPerFrame;
        if (txFrames > UINT16_MAX || rxFrames > UINT16_MAX)
        {
            return false;
        }

        const uint32_t fullDuplexFrames = txFrames < rxFrames ? txFrames : rxFrames;

        if (fullDuplexFrames != 0 &&
            HAL_SPI_TransmitReceive(handle.hspi, txBuffer, rxBuffer, static_cast<uint16_t>(fullDuplexFrames), timeout) != HAL_OK)
        {
            return false;
        }

        const uint32_t consumedBytes = fullDuplexFrames * bytesPerFrame;
        if (txFrames > fullDuplexFrames &&
            HAL_SPI_Transmit(handle.hspi,
                             txBuffer + consumedBytes,
                             static_cast<uint16_t>(txFrames - fullDuplexFrames),
                             timeout) != HAL_OK)
        {
            return false;
        }
        if (rxFrames > fullDuplexFrames &&
            HAL_SPI_Receive(handle.hspi,
                            rxBuffer + consumedBytes,
                            static_cast<uint16_t>(rxFrames - fullDuplexFrames),
                            timeout) != HAL_OK)
        {
            return false;
        }

        return true;
    }
};
    
} // namespace HAL

#endif
