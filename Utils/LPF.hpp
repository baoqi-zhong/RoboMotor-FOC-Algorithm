/**
 * @file LPF.hpp
 * @brief Low-pass filter utility for signal processing.
 * @authorbaoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */

#pragma once

#include "main.h"
#include "stdint.h"

namespace Utils
{
class LPF
{
public:
    LPF(float alpha_ = 0.1f) : alpha(alpha_), lastValue(0.0f) {}
    void setAlpha(float alpha_) { this->alpha = alpha_; }
    float operator()(float value)
    {
        lastValue = alpha * value + (1.0f - alpha) * lastValue;
        return lastValue;
    }
    
private:
    float alpha;
    float lastValue;
};
} // namespace Utils