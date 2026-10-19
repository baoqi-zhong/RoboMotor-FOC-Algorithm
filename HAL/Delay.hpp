/**
 * @file Delay.hpp
 * @brief Hardware abstraction layer for blocking delays.
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
class Delay
{
public:
    static void milliseconds(uint32_t duration)
    {
        Platform::milliseconds(duration);
    }

    static void microseconds(uint32_t duration)
    {
        Platform::microseconds(duration);
    }
};

} // namespace HAL
