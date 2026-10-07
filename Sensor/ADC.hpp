/**
 * @file ADC.hpp
 * @brief ADC configuration structures and calibration data.
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
#include "adc.h"
#include "stdint.h"

#include "statisticsCalculator.hpp"

namespace Sensor
{
class ADC
{
public:
    struct AnalogValues
    {
        float measuredIA            = 0.0f;
        float measuredIB            = 0.0f;
        float measuredIC            = 0.0f;
        float measuredIphaseSum     = 0.0f;

        float measuredVA            = 0.0f;
        float measuredVB            = 0.0f;
        float measuredVC            = 0.0f;

        float Vbus                  = 0.0f;
        float NTCTemperature        = 0.0f;

        float encoderA              = 0.0f;
        float encoderB              = 0.0f;
    };

    AnalogValues analogValues;

private:

    void resetADCCalibrationData();

    void addADCCalibrationData();

    uint8_t checkADCCalibrationSuccess();

    Utils::StatisticsCalculator IAStatisticsCalculator;
    Utils::StatisticsCalculator IBStatisticsCalculator;
    Utils::StatisticsCalculator ICStatisticsCalculator;
    Utils::StatisticsCalculator VbusStatisticsCalculator;
};

} // namespace Sensor
