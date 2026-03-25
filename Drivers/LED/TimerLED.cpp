/**
 * @file TimerLED.cpp
 * @brief Timer (PWM) implementation for GenericLED.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "TimerLED.hpp"

namespace Drivers
{
namespace LED
{

TimerLED::TimerLED(LEDFunctionType functionType_, TIM_HandleTypeDef* htim_, uint16_t channel_) : GenericLED(functionType_, LEDDriverType::TIM)
{
    this->htim = htim_;
    this->channel = channel_;
}

void TimerLED::update()
{
    if(this->blinkCB.blinking)
    {
        this->blinkCB.time++;
        if(this->blinkCB.time <= this->blinkCB.onDuration)
        {
            __HAL_TIM_SET_COMPARE(this->htim, this->channel, this->htim->Instance->ARR * this->brightness / 255.0f);
        }
        else if(this->blinkCB.time <= this->blinkCB.onDuration + this->blinkCB.offDuration + this->blinkCB.waitDuration)
        {
            __HAL_TIM_SET_COMPARE(this->htim, this->channel, 0);
        }
        else
        {
            this->blinkCB.time = 0;
        }
    }
    else
    {
        __HAL_TIM_SET_COMPARE(this->htim, this->channel, this->htim->Instance->ARR * this->brightness / 255.0f);
    }
}

} // namespace LED
} // namespace Drivers