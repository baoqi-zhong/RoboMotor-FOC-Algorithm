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
#include "opamp.h"
#include "statisticsCalculator.hpp"
#include "Math.hpp"
#include "ErrorHandler.hpp"

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
namespace ADC
{
constexpr uint8_t MaxADCRegularChannelNum = 4;

bool isInjectedChannel(ADCChannel channel)
{
    return channel >= ADCChannel::INJECTED_CHANNEL_1 && channel <= ADCChannel::INJECTED_CHANNEL_4;
}

bool isRegularChannel(ADCChannel channel)
{
    return channel >= ADCChannel::REGULAR_CHANNEL_1 && channel <= ADCChannel::REGULAR_CHANNEL_4;
}

bool channelEnabled(ADCIndex adcIndex, ADCChannel adcChannel)
{
    return adcIndex != ADCIndex::DISABLED && adcChannel != ADCChannel::DISABLED;
}

bool regularChannelEnabled(ADCIndex adcIndex, ADCChannel adcChannel)
{
    return channelEnabled(adcIndex, adcChannel) && isRegularChannel(adcChannel);
}

bool hasRegularChannels()
{
    return regularChannelEnabled(Boards::staticADCConfig.Vbus_hadc, Boards::staticADCConfig.Vbus_channel) ||
           regularChannelEnabled(Boards::staticADCConfig.VA_hadc, Boards::staticADCConfig.VA_channel) ||
           regularChannelEnabled(Boards::staticADCConfig.VB_hadc, Boards::staticADCConfig.VB_channel) ||
           regularChannelEnabled(Boards::staticADCConfig.VC_hadc, Boards::staticADCConfig.VC_channel) ||
           regularChannelEnabled(Boards::staticADCConfig.NTC_hadc, Boards::staticADCConfig.NTC_channel) ||
           regularChannelEnabled(Boards::staticADCConfig.user_hadc1, Boards::staticADCConfig.user_channel1) ||
           regularChannelEnabled(Boards::staticADCConfig.user_hadc2, Boards::staticADCConfig.user_channel2);
}

bool usesInjectedADC(ADCIndex adcIndex)
{
    return (Boards::staticADCConfig.IA_hadc == adcIndex && isInjectedChannel(Boards::staticADCConfig.IA_channel)) ||
           (Boards::staticADCConfig.IB_hadc == adcIndex && isInjectedChannel(Boards::staticADCConfig.IB_channel)) ||
           (Boards::staticADCConfig.IC_hadc == adcIndex && isInjectedChannel(Boards::staticADCConfig.IC_channel));
}

const ADCConfig& adcConfig = Boards::staticADCConfig;
ADCCalibrationData adcCalibrationData = Boards::staticADCCalibrationData;

uint16_t adcRegularChannelBuffer[MaxADCRegularChannelNum * 2];
AnalogValues analogValues;

Utils::StatisticsCalculator IAStatisticsCalculator;
Utils::StatisticsCalculator IBStatisticsCalculator;
Utils::StatisticsCalculator ICStatisticsCalculator;
Utils::StatisticsCalculator VbusStatisticsCalculator;

namespace
{
uint8_t regularBufferIndex(ADCIndex adcIndex, ADCChannel adcChannel)
{
    return static_cast<uint8_t>((static_cast<uint8_t>(adcChannel) - static_cast<uint8_t>(ADCChannel::REGULAR_CHANNEL_1)) * 2U +
                                (adcIndex == ADCIndex::ADC_2 ? 1U : 0U));
}

uint16_t readADCChannel(ADCIndex adcIndex, ADCChannel adcChannel)
{
    if(!channelEnabled(adcIndex, adcChannel))
        return 0;

    if(isInjectedChannel(adcChannel))
    {
        ADC_HandleTypeDef* hadc = nullptr;
        if(adcIndex == ADCIndex::ADC_1)
            hadc = &hadc1;
        else if(adcIndex == ADCIndex::ADC_2)
            hadc = &hadc2;
        else
            return 0;

        if(adcChannel == ADCChannel::INJECTED_CHANNEL_1)
            return static_cast<uint16_t>(hadc->Instance->JDR1);
        if(adcChannel == ADCChannel::INJECTED_CHANNEL_2)
            return static_cast<uint16_t>(hadc->Instance->JDR2);
        if(adcChannel == ADCChannel::INJECTED_CHANNEL_3)
            return static_cast<uint16_t>(hadc->Instance->JDR3);
        return static_cast<uint16_t>(hadc->Instance->JDR4);
    }

    if(!hasRegularChannels() || !isRegularChannel(adcChannel))
        return 0;

    return adcRegularChannelBuffer[regularBufferIndex(adcIndex, adcChannel)];
}
} // namespace

void start()
{
    Boards::startAnalog();

}

void triggerRegularConversion()
{
    if(hasRegularChannels())
    {
        HAL_ADC_Start(&hadc2);
        HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)adcRegularChannelBuffer, Boards::staticADCConfig.regularChannelNum);
    }
}

void decodeInjectedBuffer()
{
    if(channelEnabled(Boards::staticADCConfig.IA_hadc, Boards::staticADCConfig.IA_channel))
    {
        analogValues.measuredIA = (static_cast<float>(readADCChannel(Boards::staticADCConfig.IA_hadc, Boards::staticADCConfig.IA_channel)) -
                                   adcCalibrationData.IA_BIAS) * adcCalibrationData.IA_GAIN;
    }

    if(channelEnabled(Boards::staticADCConfig.IB_hadc, Boards::staticADCConfig.IB_channel))
    {
        analogValues.measuredIB = (static_cast<float>(readADCChannel(Boards::staticADCConfig.IB_hadc, Boards::staticADCConfig.IB_channel)) -
                                   adcCalibrationData.IB_BIAS) * adcCalibrationData.IB_GAIN;
    }

    if(channelEnabled(Boards::staticADCConfig.IC_hadc, Boards::staticADCConfig.IC_channel))
    {
        analogValues.measuredIC = (static_cast<float>(readADCChannel(Boards::staticADCConfig.IC_hadc, Boards::staticADCConfig.IC_channel)) -
                                   adcCalibrationData.IC_BIAS) * adcCalibrationData.IC_GAIN;
    }

    analogValues.measuredIphaseSum = analogValues.measuredIA + analogValues.measuredIB + analogValues.measuredIC;
}

void decodeRegularBuffer()
{
    if(channelEnabled(Boards::staticADCConfig.Vbus_hadc, Boards::staticADCConfig.Vbus_channel))
    {
        analogValues.Vbus = (static_cast<float>(readADCChannel(Boards::staticADCConfig.Vbus_hadc, Boards::staticADCConfig.Vbus_channel)) -
                             adcCalibrationData.Vbus_BIAS) * adcCalibrationData.Vbus_GAIN;
    }

    if(channelEnabled(Boards::staticADCConfig.VA_hadc, Boards::staticADCConfig.VA_channel))
    {
        analogValues.measuredVA = (static_cast<float>(readADCChannel(Boards::staticADCConfig.VA_hadc, Boards::staticADCConfig.VA_channel)) -
                                   adcCalibrationData.VA_BIAS) * adcCalibrationData.VA_GAIN;
    }

    if(channelEnabled(Boards::staticADCConfig.VB_hadc, Boards::staticADCConfig.VB_channel))
    {
        analogValues.measuredVB = (static_cast<float>(readADCChannel(Boards::staticADCConfig.VB_hadc, Boards::staticADCConfig.VB_channel)) -
                                   adcCalibrationData.VB_BIAS) * adcCalibrationData.VB_GAIN;
    }

    if(channelEnabled(Boards::staticADCConfig.VC_hadc, Boards::staticADCConfig.VC_channel))
    {
        analogValues.measuredVC = (static_cast<float>(readADCChannel(Boards::staticADCConfig.VC_hadc, Boards::staticADCConfig.VC_channel)) -
                                   adcCalibrationData.VC_BIAS) * adcCalibrationData.VC_GAIN;
    }

    if(channelEnabled(Boards::staticADCConfig.NTC_hadc, Boards::staticADCConfig.NTC_channel))
    {
        analogValues.NTCTemperature = (static_cast<float>(readADCChannel(Boards::staticADCConfig.NTC_hadc, Boards::staticADCConfig.NTC_channel)) -
                                       adcCalibrationData.NTC_BIAS) * adcCalibrationData.NTC_GAIN;
    }

    if(channelEnabled(Boards::staticADCConfig.user_hadc1, Boards::staticADCConfig.user_channel1))
    {
        analogValues.userValue1 = static_cast<float>(readADCChannel(Boards::staticADCConfig.user_hadc1, Boards::staticADCConfig.user_channel1));
    }

    if(channelEnabled(Boards::staticADCConfig.user_hadc2, Boards::staticADCConfig.user_channel2))
    {
        analogValues.userValue2 = static_cast<float>(readADCChannel(Boards::staticADCConfig.user_hadc2, Boards::staticADCConfig.user_channel2));
    }
}


void resetADCCalibrationData()
{
    IAStatisticsCalculator.reset();
    IBStatisticsCalculator.reset();
    ICStatisticsCalculator.reset();
    VbusStatisticsCalculator.reset();
}

void addADCCalibrationData()
{
    if(channelEnabled(Boards::staticADCConfig.IA_hadc, Boards::staticADCConfig.IA_channel))
        IAStatisticsCalculator.addData(static_cast<float>(readADCChannel(Boards::staticADCConfig.IA_hadc, Boards::staticADCConfig.IA_channel)));
    if(channelEnabled(Boards::staticADCConfig.IB_hadc, Boards::staticADCConfig.IB_channel))
        IBStatisticsCalculator.addData(static_cast<float>(readADCChannel(Boards::staticADCConfig.IB_hadc, Boards::staticADCConfig.IB_channel)));
    if(channelEnabled(Boards::staticADCConfig.IC_hadc, Boards::staticADCConfig.IC_channel))
        ICStatisticsCalculator.addData(static_cast<float>(readADCChannel(Boards::staticADCConfig.IC_hadc, Boards::staticADCConfig.IC_channel)));
    if(channelEnabled(Boards::staticADCConfig.Vbus_hadc, Boards::staticADCConfig.Vbus_channel))
        VbusStatisticsCalculator.addData(static_cast<float>(readADCChannel(Boards::staticADCConfig.Vbus_hadc, Boards::staticADCConfig.Vbus_channel)));
}

// 返回 1 代表校准成功, 0 代表校准失败. 校准成功的条件是 ADC 数据的标准差小于某个阈值, 且平均值在某个范围内.
uint8_t checkADCCalibrationSuccess()
{
    // 检查平均方差, 判断 ADC 数据是否稳定.
    float averageVariance = (
        IAStatisticsCalculator.getVariance() + 
        IBStatisticsCalculator.getVariance() + 
        ICStatisticsCalculator.getVariance() + 
        VbusStatisticsCalculator.getVariance()
    ) / 4.0f;

    if(averageVariance > 10.0f)
        return 0;

    if(
        FABS(IAStatisticsCalculator.getMean() - 2048.0f)    > 100.0f ||
        FABS(IBStatisticsCalculator.getMean() - 2048.0f)    > 100.0f ||
        FABS(ICStatisticsCalculator.getMean() - 2048.0f)    > 100.0f
    )
    {
        return 0;
    }

    if(channelEnabled(Boards::staticADCConfig.Vbus_hadc, Boards::staticADCConfig.Vbus_channel))
    {
        float VbusMean = (VbusStatisticsCalculator.getMean() - adcCalibrationData.Vbus_BIAS) * adcCalibrationData.Vbus_GAIN;
        if(VbusMean < Control::ErrorHandler::errorHandlerConfig.underVoltageThreshold ||
           VbusMean > Control::ErrorHandler::errorHandlerConfig.overVoltageThreshold)
        {
            return 0;
        }
    }

    adcCalibrationData.IA_BIAS = (uint16_t)IAStatisticsCalculator.getMean();
    adcCalibrationData.IB_BIAS = (uint16_t)IBStatisticsCalculator.getMean();
    adcCalibrationData.IC_BIAS = (uint16_t)ICStatisticsCalculator.getMean();
    return 1;
}

} // namespace ADC
} // namespace Sensor
