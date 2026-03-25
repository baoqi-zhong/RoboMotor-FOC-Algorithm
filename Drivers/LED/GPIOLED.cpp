/**
 * @file GPIOLED.cpp
 * @brief GPIO implementation for GenericLED.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "GPIOLED.hpp"

namespace Drivers
{
namespace LED
{
GPIOLED::GPIOLED(LEDFunctionType functionType_, GPIO_TypeDef* GPIOPort_, uint16_t GPIOPin_) : GenericLED(functionType_, LEDDriverType::GPIO)
{
    this->GPIOPort = GPIOPort_;
    this->GPIOPin = GPIOPin_;
}

void GPIOLED::update()
{
    if(this->blinkCB.blinking)
    {
        this->blinkCB.time++;
        if(this->blinkCB.time <= this->blinkCB.onDuration)
        {
            HAL_GPIO_WritePin(this->GPIOPort, this->GPIOPin, GPIO_PIN_SET);
        }
        else if(this->blinkCB.time <= this->blinkCB.onDuration + this->blinkCB.offDuration + this->blinkCB.waitDuration)
        {
            HAL_GPIO_WritePin(this->GPIOPort, this->GPIOPin, GPIO_PIN_RESET);
        }
        else
        {
            this->blinkCB.time = 0;
        }
    }
    else
    {
        HAL_GPIO_WritePin(this->GPIOPort, this->GPIOPin, this->blinkCB.value ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

} // namespace LED
} // namespace Drivers