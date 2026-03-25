/**
 * @file Boards.hpp
 * @brief Hardware abstraction layer initialization interface.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
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

#include "ThreePhaseFOC.hpp"
#include "ADC.hpp"

namespace Boards
{
/* init() 函数为所有硬件参数对外的接口 */
void init();
}