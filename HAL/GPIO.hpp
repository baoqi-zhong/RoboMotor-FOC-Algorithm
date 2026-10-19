/**
 * @file GPIO.hpp
 * @brief Hardware abstraction layer for general-purpose I/O.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#pragma once

namespace HAL
{

template <typename Platform>
class GPIO
{
public:
    using Handle = typename Platform::Handle;

    static bool write(const Handle& handle, bool level)
    {
        return Platform::write(handle, level);
    }

    static bool read(const Handle& handle, bool& level)
    {
        return Platform::read(handle, level);
    }
};

} // namespace HAL
