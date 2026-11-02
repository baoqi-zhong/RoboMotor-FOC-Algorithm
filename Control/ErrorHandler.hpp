/**
 * @file ErrorHandler.hpp
 * @brief Error handling configuration and threshold definitions.
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
#include "ADC.hpp"
#include "MotorHardware.hpp"
#include "Debouncer.hpp"

namespace Control::ErrorHandler
{
struct ErrorHandlerConfig
{
    uint8_t ignoreAllErrors                 = 0;
    uint8_t enableAutoRecovery              = 1;
    uint32_t autoRecoveryTimeout            = 100;

    float underVoltageThreshold             = 18.0f;
    float overVoltageThreshold              = 28.0f;
    float overCurrentThreshold              = 20.0f;

    uint32_t overUnderVoltageTimeout        = 500;
    uint32_t overCurrentTimeout             = 2;
};

class ErrorHandler
{
public:
    struct ErrorStatus
    {
        /* 需要累积的错误 */
        uint8_t overUnderVoltage        = 0;
        uint8_t overCurrent             = 0;
        uint8_t currentSensorError      = 0;

        /* 非累积的错误 */
        uint8_t adcCalibrationTimeout   = 0;
        uint8_t gateDriverInitTimeout   = 0;
        uint8_t gateDriverError         = 0;
    };

    ErrorHandler(const ErrorHandlerConfig& errorHandlerConfig_ = ErrorHandlerConfig());

    /**
     * @brief Called at 1KHz to check voltage and current-sensor errors.
     * @return 1 if caller should disable the motor.
     */
    bool checkError1KHz(const Sensor::ADC::AnalogValues& analogValues,
                           HAL::MotorHardware& motorHardware);

    /**
     * @brief Called synchronously with the current loop to check over-current.
     * @return 1 if caller should disable the motor.
     */
    bool checkErrorHighFreq(const Sensor::ADC::AnalogValues& analogValues);

    void clearError();

    ErrorHandlerConfig errorHandlerConfig;
    ErrorStatus errorStatus;
    ErrorStatus lastErrorStatus;

private:

    Utils::Debouncer overUnderVoltageDebouncer;
    Utils::Debouncer overCurrentDebouncer;
    Utils::Debouncer currentSensorErrorDebouncer;
}; // class ErrorHandler
} // namespace Control::ErrorHandler
