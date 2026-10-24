/**
 * @file ST_SPIBusGPIO.hpp
 * @brief STM32 GPIO bit-banged SPI bus implementation.
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
#include <string.h>

#include "GPIO.hpp"
#include "ST/ST_GPIO.hpp"

namespace HAL
{

enum class ST_SPIBusGPIODataSize : uint8_t
{
    Bits8 = 8,
    Bits16 = 16
};

/**
 * @brief GPIO bit-banged SPI configuration.
 *
 * Data is always transferred MSB first. cpol and cpha select the SPI mode:
 * mode 0 = (false, false), mode 1 = (false, true),
 * mode 2 = (true, false), and mode 3 = (true, true).
 *
 */
struct ST_SPIBusGPIOConfig
{
    ST_SPIBusGPIODataSize dataSize = ST_SPIBusGPIODataSize::Bits16;
    bool cpol = false;
    bool cpha = false;
};

/**
 * @brief GPIO pins and configuration for one bit-banged SPI bus.
 *
 * chipSelect, sck, mosi, and miso are active according to their GPIO
 * interface. chipSelect is active-low, matching the hardware SPI backend.
 * mosi may be left empty for receive-only transfers, and miso may be left
 * empty for transmit-only transfers.
 */
struct ST_SPIBusGPIOHandle
{
    ST_GPIO::Handle chipSelect{};
    ST_GPIO::Handle sck{};
    ST_GPIO::Handle mosi{};
    ST_GPIO::Handle miso{};
    ST_SPIBusGPIOConfig config;
};

class ST_SPIBusGPIO
{
public:
    using Handle = ST_SPIBusGPIOHandle;

    static bool select(const Handle& handle)
    {
        if (!isPinConfigured(handle.sck))
        {
            return false;
        }

        // Start every transaction from the configured idle clock level.
        if (!GPIO::write(handle.sck, handle.config.cpol) ||
            !GPIO::write(handle.chipSelect, false))
        {
            return false;
        }

        return true;
    }

    static void deselect(const Handle& handle)
    {
        GPIO::write(handle.chipSelect, true);
    }

    /**
     * @brief Transfer byte buffers without controlling chip select.
     *
     * A null txBuffer with txLength == 0 performs a receive-only transfer.
     * A null rxBuffer with rxLength == 0 performs a transmit-only transfer.
     * The corresponding MOSI or MISO GPIO may also be left unconfigured.
     */
    static bool transferNoCs(const Handle& handle,
                             const uint8_t* txBuffer,
                             uint32_t txLength,
                             uint8_t* rxBuffer,
                             uint32_t rxLength,
                             uint32_t timeout)
    {
        (void)timeout;

        if (txLength == 0 && rxLength == 0)
        {
            return false;
        }
        if ((txLength != 0 && txBuffer == nullptr) ||
            (rxLength != 0 && rxBuffer == nullptr))
        {
            return false;
        }
        if (!isPinConfigured(handle.sck) ||
            (txLength != 0 && !isPinConfigured(handle.mosi)) ||
            (rxLength != 0 && !isPinConfigured(handle.miso)))
        {
            return false;
        }

        const uint32_t bytesPerFrame = frameBytes(handle.config.dataSize);
        if (bytesPerFrame == 0 ||
            txLength % bytesPerFrame != 0 ||
            rxLength % bytesPerFrame != 0)
        {
            return false;
        }

        const uint32_t txFrames = txLength / bytesPerFrame;
        const uint32_t rxFrames = rxLength / bytesPerFrame;
        const uint32_t frameCount = txFrames > rxFrames ? txFrames : rxFrames;
        const uint16_t dummyFrame = maskFor(handle.config.dataSize);

        for (uint32_t frame = 0; frame < frameCount; ++frame)
        {
            const bool transmit = frame < txFrames;
            const bool receive = frame < rxFrames;
            const uint16_t txData = transmit
                ? loadFrame(txBuffer + frame * bytesPerFrame, handle.config.dataSize)
                : dummyFrame;
            uint16_t rxData = 0;

            if (!transferFrame(handle,
                               txData,
                               rxData,
                               static_cast<uint32_t>(handle.config.dataSize),
                               transmit,
                               receive))
            {
                return false;
            }

            if (receive)
            {
                storeFrame(rxBuffer + frame * bytesPerFrame,
                           rxData,
                           handle.config.dataSize);
            }
        }
        return true;
    }

    static bool transferWordNoCs(const Handle& handle,
                                 uint16_t txData,
                                 uint16_t& rxData,
                                 uint32_t timeout)
    {
        (void)timeout;

        if (!isPinConfigured(handle.sck) || !isPinConfigured(handle.miso))
        {
            return false;
        }

        uint16_t received = 0;
        // transferWord always represents one complete 16-bit frame, even
        // when the configured buffer frame size is 8 bits.
        if (!transferFrame(handle,
                           txData,
                           received,
                           16U,
                           isPinConfigured(handle.mosi),
                           true))
        {
            return false;
        }

        rxData = received;
        return true;
    }

private:
    using GPIO = HAL::GPIO<HAL::ST_GPIO>;

    static bool isPinConfigured(const ST_GPIO::Handle& pin)
    {
        return pin.port != nullptr && pin.pin != 0;
    }

    static constexpr uint32_t frameBytes(ST_SPIBusGPIODataSize dataSize)
    {
        if (dataSize == ST_SPIBusGPIODataSize::Bits8)
        {
            return 1;
        }
        if (dataSize == ST_SPIBusGPIODataSize::Bits16)
        {
            return 2;
        }
        return 0;
    }

    static constexpr uint16_t maskFor(ST_SPIBusGPIODataSize dataSize)
    {
        return dataSize == ST_SPIBusGPIODataSize::Bits8 ? 0x00FFU : 0xFFFFU;
    }

    static uint16_t loadFrame(const uint8_t* buffer, ST_SPIBusGPIODataSize dataSize)
    {
        if (dataSize == ST_SPIBusGPIODataSize::Bits8)
        {
            return buffer[0];
        }

        uint16_t value = 0;
        memcpy(&value, buffer, sizeof(value));
        return value;
    }

    static void storeFrame(uint8_t* buffer, uint16_t value, ST_SPIBusGPIODataSize dataSize)
    {
        if (dataSize == ST_SPIBusGPIODataSize::Bits8)
        {
            buffer[0] = static_cast<uint8_t>(value);
            return;
        }

        memcpy(buffer, &value, sizeof(value));
    }

    static bool transferFrame(const Handle& handle,
                              uint16_t txData,
                              uint16_t& rxData,
                              uint32_t bitCount,
                              bool transmit,
                              bool receive)
    {
        uint16_t received = 0;

        if (!GPIO::write(handle.sck, handle.config.cpol))
        {
            return false;
        }

        for (uint32_t bit = 0; bit < bitCount; ++bit)
        {
            const uint32_t shift = bitCount - 1U - bit;
            const bool txBit = ((txData >> shift) & 0x0001U) != 0;
            bool rxBit = false;

            if (!transferBit(handle, txBit, transmit, receive, rxBit))
            {
                return false;
            }

            if (receive && rxBit)
            {
                received = static_cast<uint16_t>(received | (0x0001U << shift));
            }
        }

        const uint16_t dataMask = bitCount == 16U ? 0xFFFFU : 0x00FFU;
        rxData = received & dataMask;
        return true;
    }

    static bool transferBit(const Handle& handle,
                            bool txBit,
                            bool transmit,
                            bool receive,
                            bool& rxBit)
    {
        const bool idleLevel = handle.config.cpol;
        const bool activeLevel = !idleLevel;

        if (!handle.config.cpha)
        {
            // CPHA=0: change MOSI while idle, sample MISO on the leading edge.
            if (!writeMosi(handle, txBit, transmit))
            {
                return false;
            }
            if (!GPIO::write(handle.sck, activeLevel))
            {
                return false;
            }
            if (receive && !GPIO::read(handle.miso, rxBit))
            {
                return false;
            }
            if (!GPIO::write(handle.sck, idleLevel))
            {
                return false;
            }
            return true;
        }

        // CPHA=1: change MOSI on the leading edge, sample MISO on the trailing edge.
        if (!GPIO::write(handle.sck, activeLevel))
        {
            return false;
        }
        if (!writeMosi(handle, txBit, transmit))
        {
            return false;
        }
        if (!GPIO::write(handle.sck, idleLevel))
        {
            return false;
        }
        if (receive && !GPIO::read(handle.miso, rxBit))
        {
            return false;
        }
        return true;
    }

    static bool writeMosi(const Handle& handle, bool level, bool transmit)
    {
        if (isPinConfigured(handle.mosi))
        {
            return GPIO::write(handle.mosi, level);
        }

        // A receive-only transfer does not require a MOSI pin.
        return !transmit;
    }

};

} // namespace HAL

#endif // HARDWARE_PLATFORM == HARDWARE_PLATFORM_STM32
