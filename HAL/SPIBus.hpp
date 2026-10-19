/**
 * @file SPIBus.hpp
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

#include <stdint.h>

namespace HAL
{
template <typename Platform>
class SPIBus
{
public:
    using Handle = typename Platform::Handle;

    static bool select(const Handle& handle)
    {
        return Platform::select(handle);
    }

    static void deselect(const Handle& handle)
    {
        Platform::deselect(handle);
    }

    /**
     * @brief Perform one SPI transfer.
     *
     * Buffer lengths are expressed in bytes. A zero TX length selects a
     * receive-only transfer, and a zero RX length selects a transmit-only
     * transfer. When cs is true, CS is selected before the transfer and
     * deselected afterwards. When cs is false, the caller owns CS.
     */
    static bool transfer(const Handle& handle,
                         const uint8_t* txBuffer,
                         uint32_t txLength,
                         uint8_t* rxBuffer,
                         uint32_t rxLength,
                         uint32_t timeout = 100,
                         bool cs = true)
    {
        if (txLength == 0 && rxLength == 0)
        {
            return false;
        }
        if ((txLength != 0 && txBuffer == nullptr) || (rxLength != 0 && rxBuffer == nullptr))
        {
            return false;
        }

        if (cs && !select(handle))
        {
            return false;
        }

        const bool success = Platform::transferUnselected(handle, txBuffer, txLength, rxBuffer, rxLength, timeout);
        
        if (cs)
        {
            deselect(handle);
        }
        return success;
    }

    /**
     * @brief Transfer one 16-bit SPI frame.
     */
    static bool transferWord(const Handle& handle,
                             uint16_t txData,
                             uint16_t& rxData,
                             uint32_t timeout = 100,
                             bool cs = true)
    {
        if (cs && !select(handle))
        {
            return false;
        }

        const bool success = Platform::transferWordNoCs(handle, txData, rxData, timeout);
        if (cs)
        {
            deselect(handle);
        }
        return success;
    }

private:
    static bool transferUnselected(const Handle& handle,
                                   const uint8_t* txBuffer,
                                   uint32_t txLength,
                                   uint8_t* rxBuffer,
                                   uint32_t rxLength,
                                   uint32_t timeout)
    {
        if (txLength == 0 && rxLength == 0)
        {
            return false;
        }
        if ((txLength != 0 && txBuffer == nullptr) || (rxLength != 0 && rxBuffer == nullptr))
        {
            return false;
        }
        return Platform::transferNoCs(handle, txBuffer, txLength, rxBuffer, rxLength, timeout);
    }
};
    
} // namespace HAL
