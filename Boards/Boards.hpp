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

#define BOARD_RM_DOCK_FOC   1

namespace Sensor
{
namespace ADC
{
struct ADCConfig;
struct ADCCalibrationData;
} // namespace ADC
} // namespace Sensor

namespace Boards
{
    extern const Sensor::ADC::ADCConfig staticADCConfig;
    extern const Sensor::ADC::ADCCalibrationData staticADCCalibrationData;

    void startTimerBase();
    void startTimerPWMLowSide();
    void startTimerPWMHighSide();
    void stopTimerPWM();
    inline void setTimerPWMDutyCycle(float dutyCycleA, float dutyCycleB, float dutyCycleC);

    void startAnalog();
    void init();
} // namespace Boards
