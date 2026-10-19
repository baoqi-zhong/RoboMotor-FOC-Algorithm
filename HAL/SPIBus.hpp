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

    class Transaction
    {
    public:
        explicit Transaction(const Handle& handle_) : handle(handle_), selected(Platform::select(handle_)) {}

        ~Transaction()
        {
            if (selected)
            {
                Platform::deselect(handle);
            }
        }

        Transaction(const Transaction&) = delete;
        Transaction& operator=(const Transaction&) = delete;

        explicit operator bool() const
        {
            return selected;
        }

    private:
        Handle handle;
        bool selected;
    };

    /**
     * @brief Perform one SPI transaction while the platform adapter controls CS.
     *
     * Buffer lengths are expressed in bytes. A zero TX length selects a
     * receive-only transfer, and a zero RX length selects a transmit-only
     * transfer.
     */
    static bool transfer(const Handle& handle, const uint8_t* txBuffer, uint32_t txLength, uint8_t* rxBuffer, uint32_t rxLength, uint32_t timeout = 100)
    {
        Transaction transaction(handle);
        if (!transaction)
        {
            return false;
        }
        return transferSelected(handle, txBuffer, txLength, rxBuffer, rxLength, timeout);
    }

    /**
     * @brief Transfer data while the caller owns the chip-select transaction.
     */
    static bool transferSelected(const Handle& handle, const uint8_t* txBuffer, uint32_t txLength, uint8_t* rxBuffer, uint32_t rxLength, uint32_t timeout = 100)
    {
        if (txLength == 0 && rxLength == 0)
        {
            return false;
        }
        if ((txLength != 0 && txBuffer == nullptr) || (rxLength != 0 && rxBuffer == nullptr))
        {
            return false;
        }
        return Platform::transferSelected(handle, txBuffer, txLength, rxBuffer, rxLength, timeout);
    }
};
    
} // namespace HAL
