/**
 * @file MotorHardware.hpp
 * @brief Hardware abstraction layer for the motor power stage and fault signals.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#pragma once


namespace HAL
{

class MotorHardware
{
public:
    virtual inline void enableGateDriver() {};
    virtual bool isGateDriverEnabled() {return true;};
    virtual inline void enableTimerPWMLowSideOutput() = 0;
    virtual inline void enableTimerPWMHighSideOutput() = 0;
    virtual inline void disableTimerPWMOutput() = 0;
    virtual inline void setPWMDutyCycle(float dutyCycleA, float dutyCycleB, float dutyCycleC) = 0;

    /**
     * @brief Read the platform-specific 1 kHz error source.
     * @return true when an error is active or its state cannot be read.
     */
    virtual bool readError() {return false;};

    /**
     * @brief Attempt to clear the platform-specific latched error.
     * @return true when the clear operation completed successfully.
     */
    virtual bool clearError() {return true;};
};
    
} // namespace HAL
