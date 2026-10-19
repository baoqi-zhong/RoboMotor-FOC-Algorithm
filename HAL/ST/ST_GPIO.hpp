/**
 * @file ST_GPIO.hpp
 * @brief STM32 GPIO hardware abstraction implementation.
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

struct ST_GPIOHandle
{
    GPIO_TypeDef* port = nullptr;
    uint16_t pin = 0;
};

class ST_GPIO
{
public:
    using Handle = ST_GPIOHandle;

    static bool write(const Handle& handle, bool level)
    {
        if (handle.port == nullptr || handle.pin == 0)
        {
            return false;
        }
        HAL_GPIO_WritePin(handle.port, handle.pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
        return true;
    }

    static bool read(const Handle& handle, bool& level)
    {
        if (handle.port == nullptr || handle.pin == 0)
        {
            return false;
        }
        level = HAL_GPIO_ReadPin(handle.port, handle.pin) == GPIO_PIN_SET;
        return true;
    }
};

} // namespace HAL

#endif // HARDWARE_PLATFORM == HARDWARE_PLATFORM_STM32
