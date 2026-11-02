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
    /* enable 并清除 Fault, 为启动做准备 */
    virtual void prepareGateDriver() {};
    /* 处于启动期间或者 Fault 状态需要返回 false */
    virtual bool isGateDriverReady() {return true;};
    virtual inline void disableGateDriver() = 0;

    virtual inline void enableLowSideMosfet() = 0;
    virtual inline void enableHighSideMosfet() = 0;
    virtual inline void setPWMDutyCycle(float dutyCycleA, float dutyCycleB, float dutyCycleC) = 0;

    virtual inline void enterCurrentCalibrationMode() {};
    virtual inline void exitCurrentCalibrationMode() {};

};
    
} // namespace HAL
