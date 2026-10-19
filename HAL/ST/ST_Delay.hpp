/**
 * @file ST_Delay.hpp
 * @brief STM32 blocking delay implementation.
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

namespace HAL
{

class ST_Delay
{
public:
    static void milliseconds(uint32_t duration)
    {
        HAL_Delay(duration);
    }

    static void microseconds(uint32_t duration)
    {
        if (duration == 0)
        {
            return;
        }

        const uint32_t iterations = duration * (SystemCoreClock / 4000000U);
        for (uint32_t i = 0; i < iterations; ++i)
        {
            __NOP();
            __asm__ volatile("" ::: "memory");
        }
    }
};

} // namespace HAL

#endif // HARDWARE_PLATFORM == HARDWARE_PLATFORM_STM32
