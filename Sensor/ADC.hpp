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

namespace Sensor::ADC
{
struct ADCCalibrationData
{
    uint16_t IAOffset           = 0;
    uint16_t IBOffset           = 0;
    uint16_t ICOffset           = 0;

    float IAGain                = 1.0f;
    float IBGain                = 1.0f;
    float ICGain                = 1.0f;
    float VAGain                = 1.0f;
    float VBGain                = 1.0f;
    float VCGain                = 1.0f;
    float VbusGain              = 1.0f;
};

struct AnalogValues
{
    uint16_t rawIA              = 0;
    uint16_t rawIB              = 0;
    uint16_t rawIC              = 0;

    float measuredIA            = 0.0f;
    float measuredIB            = 0.0f;
    float measuredIC            = 0.0f;
    float measuredIphaseSum     = 0.0f;

    float measuredVA            = 0.0f;
    float measuredVB            = 0.0f;
    float measuredVC            = 0.0f;

    float Vbus                  = 0.0f;
    float NTCTemperature        = 0.0f;
};

class ADC
{
public:
    AnalogValues analogValues;
    ADCCalibrationData adcCalibrationData;

    ADC(ADCCalibrationData adcCalibrationData_ = ADCCalibrationData()) : adcCalibrationData(adcCalibrationData_) {}

    void updatePhaseCurrent(uint16_t rawIA, uint16_t rawIB, uint16_t rawIC);
    void updatePhaseVoltage(uint16_t rawVA, uint16_t rawVB, uint16_t rawVC);
    void updateVbus(uint16_t rawVbus);

    void resetADCCalibrationData();
    void addADCCalibrationData();
    uint8_t checkADCCalibrationSuccess();

private:
    Utils::StatisticsCalculator IAStatisticsCalculator;
    Utils::StatisticsCalculator IBStatisticsCalculator;
    Utils::StatisticsCalculator ICStatisticsCalculator;
};

} // namespace Sensor::ADC
