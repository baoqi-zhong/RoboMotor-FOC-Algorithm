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

#include "stdint.h"
#include "statisticsCalculator.hpp"

namespace Sensor::ADC
{
struct ADCCalibrationData
{
    uint16_t IAOffset   = 0;
    uint16_t IBOffset   = 0;
    uint16_t ICOffset   = 0;

    float IAGain        = 1.0f;
    float IBGain        = 1.0f;
    float ICGain        = 1.0f;
    float VAGain        = 1.0f;
    float VBGain        = 1.0f;
    float VCGain        = 1.0f;
    float VbusGain      = 1.0f;

    /* 稳定性判定, 默认值为 24V 小功率电机建议值 */
    float vbusStabilityMin                  = 18.0f;   /* VBUS 稳定性判定时, 任意时刻数值必须大于该值 */
    float vbusStabilityMax                  = 28.0f;   /* VBUS 稳定性判定时, 任意时刻数值必须小于该值 */
    float currentStabilityMinMax            = 0.5f;    /* 电流稳定性判定时, 任意时刻数值必须在正负该值内 */
    float currentStabilityVarianceThreshold = 0.05f;   /* 电流稳定性判定时, 方差小于该值则认为采样稳定 */
    float vbusStabilityVarianceThreshold    = 0.05f;   /* VBUS 稳定性判定时, 方差小于该值则认为采样稳定 */
};

struct AnalogValues
{
    uint16_t rawIA      = 0;
    uint16_t rawIB      = 0;
    uint16_t rawIC      = 0;
    uint16_t rawVA      = 0;
    uint16_t rawVB      = 0;
    uint16_t rawVC      = 0;
    uint16_t rawVbus    = 0;

    float measuredIA    = 0.0f;
    float measuredIB    = 0.0f;
    float measuredIC    = 0.0f;
    float measuredISum  = 0.0f;

    float measuredVA    = 0.0f;
    float measuredVB    = 0.0f;
    float measuredVC    = 0.0f;

    float Vbus          = 0.0f;
};

class ADC
{
public:
    AnalogValues analogValues;
    ADCCalibrationData adcCalibrationData;

    ADC(ADCCalibrationData adcCalibrationData_ = ADCCalibrationData()) : adcCalibrationData(adcCalibrationData_) {}

    void updateRawPhaseCurrent(uint16_t rawIA, uint16_t rawIB, uint16_t rawIC);
    void updateRawPhaseVoltage(uint16_t rawVA, uint16_t rawVB, uint16_t rawVC);
    void updateRawVbus(uint16_t rawVbus);

    void updatePhaseCurrent(float IA, float IB, float IC);
    void updatePhaseVoltage(float VA, float VB, float VC);
    void updateVbus(float Vbus);

    bool checkStability();
    void resetStabilityChecker();

private:
    Utils::StatisticsCalculator IAStatisticsCalculator;
    Utils::StatisticsCalculator IBStatisticsCalculator;
    Utils::StatisticsCalculator ICStatisticsCalculator;
    Utils::StatisticsCalculator VbusStatisticsCalculator;
};

} // namespace Sensor::ADC
