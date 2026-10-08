/**
 * @file SinCosEncoder.cpp
 * @brief Sin/cos analog angle estimator.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "SinCosEncoder.hpp"

#include "Trigonometric.hpp"
#include "Math.hpp"

namespace Sensor::SinCosEncoder
{

uint16_t update(float a, float b)
{
    return Utils::Trigonometric::phaseQ16(b, a);
}

} // namespace Sensor::SinCosEncoder
