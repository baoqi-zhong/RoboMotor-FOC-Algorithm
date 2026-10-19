/**
 * @file FDCANBus.hpp
 * @brief Hardware abstraction layer for FDCAN communication.
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
class FDCANBus
{
public:
    using RxCallback = typename Platform::RxCallback;
    using Handle = typename Platform::Handle;

    explicit FDCANBus(const Handle& handle_) : handle(handle_) {}

    bool init(uint32_t filterMask, uint32_t filterId) const
    {
        return Platform::init(handle, filterMask, filterId);
    }

    bool transmit(uint32_t canId, const uint8_t* data) const
    {
        if (canId > 0x7FF || data == nullptr)
        {
            return false;
        }
        return Platform::transmit(handle, canId, data);
    }

    void registerCallback(RxCallback callback) const
    {
        Platform::registerCallback(callback);
    }

private:
    Handle handle;
};

} // namespace HAL
