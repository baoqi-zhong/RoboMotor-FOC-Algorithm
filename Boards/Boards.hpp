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

#define BOARD_RM_DOCK_FOC       0
#define BOARD_RM2027_TRI_FOC    0


namespace Boards
{
    void init();
} // namespace Boards
