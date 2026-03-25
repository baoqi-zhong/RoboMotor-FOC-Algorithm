/**
 * @file TimerLED.hpp
 * @brief Timer LED driver class.
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
#include "tim.h"

namespace Drivers
{
namespace LED
{
    
class TimerLED : public GenericLED
{
public:
    TimerLED(LEDFunctionType functionType_, TIM_HandleTypeDef* htim_, uint16_t channel_);
    void update();

private:
    TIM_HandleTypeDef* htim;
    uint32_t channel;
};

} // namespace LED
} // namespace Drivers