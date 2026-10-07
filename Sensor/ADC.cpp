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

namespace Sensor
{

void ADC::resetADCCalibrationData()
{
    IAStatisticsCalculator.reset();
    IBStatisticsCalculator.reset();
    ICStatisticsCalculator.reset();
    VbusStatisticsCalculator.reset();
}

void ADC::addADCCalibrationData()
{
    IAStatisticsCalculator.addData(analogValues.measuredIA);
    IBStatisticsCalculator.addData(analogValues.measuredIB);
    ICStatisticsCalculator.addData(analogValues.measuredIC);
    VbusStatisticsCalculator.addData(analogValues.Vbus);
}

// 返回 1 代表校准成功, 0 代表校准失败. 校准成功的条件是 ADC 数据的标准差小于某个阈值, 且平均值在某个范围内.
uint8_t ADC::checkADCCalibrationSuccess()
{
    // 检查平均方差, 判断 ADC 数据是否稳定.
    float averageVariance = (
        IAStatisticsCalculator.getVariance() + 
        IBStatisticsCalculator.getVariance() + 
        ICStatisticsCalculator.getVariance() + 
        VbusStatisticsCalculator.getVariance()
    ) / 4.0f;

    if(averageVariance > 0.5f)
        return 0;

    if(
        FABS(IAStatisticsCalculator.getMean()) > 0.1f ||
        FABS(IBStatisticsCalculator.getMean()) > 0.1f ||
        FABS(ICStatisticsCalculator.getMean()) > 0.1f ||
        VbusStatisticsCalculator.getMean() < 10.0f
    )
    {
        return 0;
    }

    return 1;
}

} // namespace Sensor
