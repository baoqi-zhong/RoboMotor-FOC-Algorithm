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
#include "Boards.hpp"
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
void ADC::updatePhaseCurrent(uint16_t rawIA, uint16_t rawIB, uint16_t rawIC)
{
    analogValues.rawIA = rawIA;
    analogValues.rawIB = rawIB;
    analogValues.rawIC = rawIC;
    analogValues.measuredIA = (float)((int16_t)rawIA - adcCalibrationData.IAOffset) * adcCalibrationData.IAGain;
    analogValues.measuredIB = (float)((int16_t)rawIB - adcCalibrationData.IBOffset) * adcCalibrationData.IBGain;
    analogValues.measuredIC = (float)((int16_t)rawIC - adcCalibrationData.ICOffset) * adcCalibrationData.ICGain;
}

void ADC::updatePhaseVoltage(uint16_t rawVA, uint16_t rawVB, uint16_t rawVC)
{
    analogValues.measuredVA = (float)rawVA * adcCalibrationData.VAGain;
    analogValues.measuredVB = (float)rawVB * adcCalibrationData.VBGain;
    analogValues.measuredVC = (float)rawVC * adcCalibrationData.VCGain;
}

void ADC::updateVbus(uint16_t rawVbus)
{
    analogValues.Vbus = (float)rawVbus * adcCalibrationData.VbusGain;
}

void ADC::resetADCCalibrationData()
{
    IAStatisticsCalculator.reset();
    IBStatisticsCalculator.reset();
    ICStatisticsCalculator.reset();
}

void ADC::addADCCalibrationData()
{
    IAStatisticsCalculator.addData(analogValues.rawIA);
    IBStatisticsCalculator.addData(analogValues.rawIB);
    ICStatisticsCalculator.addData(analogValues.rawIC);
}

// 返回 1 代表校准成功, 0 代表校准失败. 校准成功的条件是 ADC 数据的标准差小于某个阈值, 且平均值在某个范围内.
uint8_t ADC::checkADCCalibrationSuccess()
{
    // 检查平均方差, 判断 ADC 数据是否稳定.
    float averageVariance = (
        IAStatisticsCalculator.getVariance() + 
        IBStatisticsCalculator.getVariance() + 
        ICStatisticsCalculator.getVariance()
    ) / 4.0f;

    if(averageVariance > 5.0f)
        return 0;

    if(
        FABS(IAStatisticsCalculator.getMean() - adcCalibrationData.IAOffset) > (adcCalibrationData.IAOffset >> 4) ||
        FABS(IBStatisticsCalculator.getMean() - adcCalibrationData.IBOffset) > (adcCalibrationData.IBOffset >> 4) ||
        FABS(ICStatisticsCalculator.getMean() - adcCalibrationData.ICOffset) > (adcCalibrationData.ICOffset >> 4)
    )
    {
        return 0;
    }

    adcCalibrationData.IAOffset = (uint16_t)IAStatisticsCalculator.getMean();
    adcCalibrationData.IBOffset = (uint16_t)IBStatisticsCalculator.getMean();
    adcCalibrationData.ICOffset = (uint16_t)ICStatisticsCalculator.getMean();
    return 1;
}

} // namespace Sensor::ADC
