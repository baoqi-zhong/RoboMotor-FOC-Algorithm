/**
 * @file SinCosEncoder.hpp
 * @brief Sin/cos analog angle estimator.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#pragma once

#include "stdint.h"

namespace Sensor::SinCosEncoder
{

uint16_t update(float a, float b);

} // namespace Sensor::SinCosEncoder
