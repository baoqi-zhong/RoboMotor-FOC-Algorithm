/**
 * @file ADC.cpp
 * @brief ADC configuration and calibration logic.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "ADC.hpp"
#include "statisticsCalculator.hpp"
#include "Math.hpp"

/**
 * 所有电流和 hall 传感器的采样都使用 Injected Simultaneous Mode, 并分布在 ADC1 和 ADC2 上, 以保证采样的同步性.
 * 其他数据可以通过 ADC1/2 剩余的 Injected 或者任意 adc 的 Regular Channel 采样. 
 * 
 * Regular Channel 和 Injected Channel 同时使用时, 可能触发勘误表 
 * ADC channel 0 converted instead of the required ADC channel 错误.
 * 
 * 因此 Regular Conversion 不能使用一个独立的 timer 触发. 而是必须在 Injected Conversion 完成后, 
 * 由 Injected Conversion 的中断回调函数触发 Regular Conversion.
 */

namespace Sensor::ADC
{
void ADC::updatePhaseCurrent(float IA, float IB, float IC)
{
    analogValues.measuredIA = IA;
    analogValues.measuredIB = IB;
    analogValues.measuredIC = IC;
    analogValues.measuredISum = IA + IB + IC;
}

void ADC::updatePhaseVoltage(float VA, float VB, float VC)
{
    analogValues.measuredVA = VA;
    analogValues.measuredVB = VB;
    analogValues.measuredVC = VC;
}

void ADC::updateVbus(float Vbus)
{
    analogValues.Vbus = Vbus;
}

void ADC::updateRawPhaseCurrent(uint16_t rawIA, uint16_t rawIB, uint16_t rawIC)
{
    analogValues.rawIA = rawIA;
    analogValues.rawIB = rawIB;
    analogValues.rawIC = rawIC;
    analogValues.measuredIA = (float)((int16_t)rawIA - adcCalibrationData.IAOffset) * adcCalibrationData.IAGain;
    analogValues.measuredIB = (float)((int16_t)rawIB - adcCalibrationData.IBOffset) * adcCalibrationData.IBGain;
    analogValues.measuredIC = (float)((int16_t)rawIC - adcCalibrationData.ICOffset) * adcCalibrationData.ICGain;
    analogValues.measuredISum = analogValues.measuredIA + analogValues.measuredIB + analogValues.measuredIC;
}

void ADC::updateRawPhaseVoltage(uint16_t rawVA, uint16_t rawVB, uint16_t rawVC)
{
    analogValues.measuredVA = (float)rawVA * adcCalibrationData.VAGain;
    analogValues.measuredVB = (float)rawVB * adcCalibrationData.VBGain;
    analogValues.measuredVC = (float)rawVC * adcCalibrationData.VCGain;
}

void ADC::updateRawVbus(uint16_t rawVbus)
{
    analogValues.Vbus = (float)rawVbus * adcCalibrationData.VbusGain;
}



/**
 * 检查 ADC 数据的稳定性. 判定条件:
 * 1. VBUS 电压的方差小于稳定性判定阈值, 且电压在稳定性判定范围内.
 * 2. 三相电流的方差小于稳定性判定阈值, 且三相电流在稳定性判定范围内.
 */
bool ADC::checkStability()
{
    /* 首先要求电压和电流在范围内 */
    if (
        FABS(analogValues.measuredIA) > adcCalibrationData.currentStabilityMinMax ||
        FABS(analogValues.measuredIB) > adcCalibrationData.currentStabilityMinMax ||
        FABS(analogValues.measuredIC) > adcCalibrationData.currentStabilityMinMax ||
        analogValues.Vbus < adcCalibrationData.vbusStabilityMin ||
        analogValues.Vbus > adcCalibrationData.vbusStabilityMax
    )
    {
        resetStabilityChecker();
        return false;
    }
    
    IAStatisticsCalculator.addData(analogValues.measuredIA);
    IBStatisticsCalculator.addData(analogValues.measuredIB);
    ICStatisticsCalculator.addData(analogValues.measuredIC);
    VbusStatisticsCalculator.addData(analogValues.Vbus);

    if (IAStatisticsCalculator.getCount() != 50)
    {
        return false;
    }

    if (
        IAStatisticsCalculator.getVariance() > adcCalibrationData.currentStabilityVarianceThreshold ||
        IBStatisticsCalculator.getVariance() > adcCalibrationData.currentStabilityVarianceThreshold ||
        ICStatisticsCalculator.getVariance() > adcCalibrationData.currentStabilityVarianceThreshold ||
        VbusStatisticsCalculator.getVariance() > adcCalibrationData.vbusStabilityVarianceThreshold
    )
    {
        resetStabilityChecker();
        return false;
    }

    adcCalibrationData.IAOffset += IAStatisticsCalculator.getMean() / adcCalibrationData.IAGain;
    adcCalibrationData.IBOffset += IBStatisticsCalculator.getMean() / adcCalibrationData.IBGain;
    adcCalibrationData.ICOffset += ICStatisticsCalculator.getMean() / adcCalibrationData.ICGain;
    return true;
}


void ADC::resetStabilityChecker()
{
    IAStatisticsCalculator.reset();
    IBStatisticsCalculator.reset();
    ICStatisticsCalculator.reset();
    VbusStatisticsCalculator.reset();
}

} // namespace Sensor::ADC
