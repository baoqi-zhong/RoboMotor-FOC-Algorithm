/**
 * @file GPIOLED.hpp
 * @brief GPIO LED driver class.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#pragma once


#include "LED.hpp"

namespace Drivers
{
namespace LED
{

class GPIOLED : public GenericLED
{
public:
    GPIOLED(LEDFunctionType functionType_, GPIO_TypeDef* GPIOPort_, uint16_t GPIOPin_);
    void update();

private:
    GPIO_TypeDef* GPIOPort;
    uint16_t GPIOPin;
};

} // namespace LED
} // namespace Drivers