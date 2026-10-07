/**
 * @file MotorHAL.hpp
 * @brief Hardware Abstraction Layer for Motor control.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#pragma once


namespace Hardware
{

class MotorHAL
{
public:
    /* Timer */
    virtual inline void startTimerBase() = 0;
    virtual inline void enableTimerPWMLowSideOutput() = 0;
    virtual inline void enableTimerPWMHighSideOutput() = 0;
    virtual inline void disableTimerPWMOutput() = 0;
    virtual inline void setPWMDutyCycle(float dutyCycleA, float dutyCycleB, float dutyCycleC) = 0;
};
    
} // namespace Hardware
