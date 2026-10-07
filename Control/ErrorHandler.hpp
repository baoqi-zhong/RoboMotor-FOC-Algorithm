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

#include "main.h"
#include "stdint.h"

#include "ADC.hpp"

namespace Control::ErrorHandler
{
struct ErrorHandlerConfig
{
    uint8_t ignoreAllErrors                 = 0;
    uint8_t enableAutoRecovery              = 1;
    uint32_t autoRecoveryTimeout            = 100;

    float underVoltageThreshold             = 16.0f;
    float overVoltageThreshold              = 30.0f;
    float overCurrentThreshold              = 12.0f;
    float underTemperatureThreshold         = 0.0f;
    float overTemperatureThreshold          = 100.0f;

    uint32_t underVoltageTriggerTimeout     = 500;
    uint32_t overVoltageTriggerTimeout      = 500;
    uint32_t overCurrentTriggerTimeout      = 10;
    uint32_t overTemperatureTriggerTimeout  = 10000;
};

class ErrorHandler
{
public:


    struct ErrorStatus
    {
        uint8_t underVoltage            = 0;
        uint8_t overVoltage             = 0;
        uint8_t overCurrent             = 0;
        uint8_t ADCDecoderError         = 0;
        uint8_t underTemperature        = 0;
        uint8_t overTemperature         = 0;

        uint8_t encoderError            = 0;
        uint8_t motorDisconnected       = 0;
    };

    struct ErrorCounter
    {
        uint32_t underVoltageCounter        = 0;
        uint32_t overVoltageCounter         = 0;
        uint32_t overCurrentCounter         = 0;
        uint32_t currnentSensorErrorCounter = 0;

        uint32_t noErrorCounter             = 0;
    };

    ErrorHandler(const ErrorHandlerConfig& errorHandlerConfig_ = ErrorHandlerConfig()) : errorHandlerConfig(errorHandlerConfig_) {}
    
    /**
     * @brief Check whether errorStatus contains any error.
     */
    uint8_t checkIfAnyErrorStatus();

    /**
     * @brief Check whether the measured values are still in an error state.
     */
    uint8_t checkIfStillInError(const Sensor::ADC::AnalogValues& analogValues);

    /**
     * @brief Called at 1KHz to check voltage, current-sensor and temperature errors.
     * @return 1 if caller should disable the motor.
     */
    uint8_t checkError1KHz(const Sensor::ADC::AnalogValues& analogValues);

    /**
     * @brief Called synchronously with the current loop to check over-current.
     * @return 1 if caller should disable the motor.
     */
    uint8_t checkErrorHighFreq(const Sensor::ADC::AnalogValues& analogValues);

    /**
     * @brief Called at 1KHz to check whether auto recovery can request reset.
     * @return 1 if caller should set triggerReset.
     */
    uint8_t checkIfCanAutoRecovery(const Sensor::ADC::AnalogValues& analogValues, uint8_t motorStopped, uint8_t triggerReset);

    /**
     * @brief Clear all errors.
     */
    void clearAllError();

    ErrorHandlerConfig errorHandlerConfig;
    ErrorStatus errorStatus;
    ErrorCounter errorCounter;
}; // class ErrorHandler
} // namespace Control::ErrorHandler
