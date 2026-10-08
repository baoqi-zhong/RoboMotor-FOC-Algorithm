/**
 * @file Trigonometric.hpp
 * @brief Trigonometric backend abstraction.
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
#include "stdint.h"

namespace Utils::Trigonometric
{

void init();
void sinCos(float angle, float* sinValue, float* cosValue);
void sinCosMultiply(float value, float angle, float* sinMulValue, float* cosMulValue);
uint16_t phaseQ16(float y, float x);

} // namespace Utils::Trigonometric
