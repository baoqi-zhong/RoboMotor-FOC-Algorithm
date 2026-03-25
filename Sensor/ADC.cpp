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
#include "opamp.h"
#include "statisticsCalculator.hpp"
#include "Math.hpp"
#include "ErrorHandler.hpp"

/**
 * 约定: 三相电流必须在 Injected Channel, 电压和温度等必须在 Regular Channel. 
 * 因为本 FOC 不支持需要用到相电压的算法. 母线电压短时间波动校准没有太大意义, 目前母线电压仅用于保护逻辑.
 */

namespace Sensor
{
namespace ADC
{
#define MAX_ADC_REGULAR_CHANNEL_NUM     4

ADCConfig adcConfig;
ADCCalibrationData adcCalibrationData;

uint16_t adcRegularChannelBuffer[MAX_ADC_REGULAR_CHANNEL_NUM * 2];
const uint16_t ZERO_VALUE_VARIABLE = 0;

AnalogValuePointers analogValuePointers;
AnalogValues analogValues;

Utils::StatisticsCalculator IAStatisticsCalculator;
Utils::StatisticsCalculator IBStatisticsCalculator;
Utils::StatisticsCalculator ICStatisticsCalculator;
Utils::StatisticsCalculator VbusStatisticsCalculator;

uint16_t* getBufferPointer(ADCIndex adcIndex, ADCChannel adcChannel)
{
    if(adcIndex == ADCIndex::DISABLED)
        return (uint16_t*)&ZERO_VALUE_VARIABLE;
    if(adcChannel == ADCChannel::DISABLED)
        return (uint16_t*)&ZERO_VALUE_VARIABLE;
    
    ADC_HandleTypeDef* hadc = nullptr;
    if(adcIndex == ADCIndex::ADC_1)
        hadc = &hadc1;
    else if(adcIndex == ADCIndex::ADC_2)
        hadc = &hadc2;

    // Injected Channel
    if(adcChannel == ADCChannel::INJECTED_CHANNEL_1)
        return (uint16_t*)&hadc->Instance->JDR1;
    if(adcChannel == ADCChannel::INJECTED_CHANNEL_2)
        return (uint16_t*)&hadc->Instance->JDR2;
    if(adcChannel == ADCChannel::INJECTED_CHANNEL_3)
        return (uint16_t*)&hadc->Instance->JDR3;
    if(adcChannel == ADCChannel::INJECTED_CHANNEL_4)
        return (uint16_t*)&hadc->Instance->JDR4;
    
    if(adcConfig.enableRegularChannels == 0)
        return (uint16_t*)&ZERO_VALUE_VARIABLE;

    // Regular Channel
    uint8_t bufferIndex = ((uint8_t)adcChannel - (uint8_t)ADCChannel::REGULAR_CHANNEL_1) * 2;
    if(adcIndex == ADCIndex::ADC_2)
    {
        bufferIndex += 1;
    }

    return &adcRegularChannelBuffer[bufferIndex];
}

/**
 * ADC 写死电压电流信号是两个 ADC 的同时注入通道, 
 * 电压温度等别的是写死两个 ADC dual 的 regular 通道.
 */
void setConfig(const ADCConfig* config, const ADCCalibrationData* calibrationData)
{
    assert_param(config != nullptr);
    assert_param(config->IA_hadc != ADCIndex::DISABLED);
    assert_param(config->IB_hadc != ADCIndex::DISABLED);
    assert_param(config->IC_hadc != ADCIndex::DISABLED);
    assert_param(config->IA_channel >= ADCChannel::INJECTED_CHANNEL_1 && config->IA_channel <= ADCChannel::INJECTED_CHANNEL_4);
    assert_param(config->IB_channel >= ADCChannel::INJECTED_CHANNEL_1 && config->IB_channel <= ADCChannel::INJECTED_CHANNEL_4);
    assert_param(config->IC_channel >= ADCChannel::INJECTED_CHANNEL_1 && config->IC_channel <= ADCChannel::INJECTED_CHANNEL_4);

    if(config->enableRegularChannels == 1)
    {
        assert_param(config->regularChannelNum > 0 && config->regularChannelNum <= MAX_ADC_REGULAR_CHANNEL_NUM);
        if(config->VA_hadc != ADCIndex::DISABLED)
        {
            assert_param(config->VA_channel >= ADCChannel::REGULAR_CHANNEL_1 && config->VA_channel <= ADCChannel::REGULAR_CHANNEL_4);
        }
        if(config->VB_hadc != ADCIndex::DISABLED)
        {
            assert_param(config->VB_channel >= ADCChannel::REGULAR_CHANNEL_1 && config->VB_channel <= ADCChannel::REGULAR_CHANNEL_4);
        }
        if(config->VC_hadc != ADCIndex::DISABLED)
        {
            assert_param(config->VC_channel >= ADCChannel::REGULAR_CHANNEL_1 && config->VC_channel <= ADCChannel::REGULAR_CHANNEL_4);
        }
        if(config->Vbus_hadc != ADCIndex::DISABLED)
        {
            assert_param(config->Vbus_channel >= ADCChannel::REGULAR_CHANNEL_1 && config->Vbus_channel <= ADCChannel::REGULAR_CHANNEL_4);
        }
        if(config->NTC_hadc != ADCIndex::DISABLED)
        {
            assert_param(config->NTC_channel >= ADCChannel::REGULAR_CHANNEL_1 && config->NTC_channel <= ADCChannel::REGULAR_CHANNEL_4);
        }
    }

    adcConfig = *config;
    adcCalibrationData = *calibrationData;

    analogValuePointers.pIABuffer    = getBufferPointer(adcConfig.IA_hadc,     adcConfig.IA_channel);
    analogValuePointers.pIBBuffer    = getBufferPointer(adcConfig.IB_hadc,     adcConfig.IB_channel);
    analogValuePointers.pICBuffer    = getBufferPointer(adcConfig.IC_hadc,     adcConfig.IC_channel);
    analogValuePointers.pVbusBuffer  = getBufferPointer(adcConfig.Vbus_hadc,   adcConfig.Vbus_channel);
    analogValuePointers.pVABuffer    = getBufferPointer(adcConfig.VA_hadc,     adcConfig.VA_channel);
    analogValuePointers.pVBBuffer    = getBufferPointer(adcConfig.VB_hadc,     adcConfig.VB_channel);
    analogValuePointers.pVCBuffer    = getBufferPointer(adcConfig.VC_hadc,     adcConfig.VC_channel);
    analogValuePointers.pNTCBuffer   = getBufferPointer(adcConfig.NTC_hadc,    adcConfig.NTC_channel);
    analogValuePointers.pVREFINTBuffer = getBufferPointer(adcConfig.VREFINT_hadc, adcConfig.Vrefint_channel);
    analogValuePointers.pUserBuffer1 = getBufferPointer(adcConfig.user_hadc1, adcConfig.user_channel1);
    analogValuePointers.pUserBuffer2 = getBufferPointer(adcConfig.user_hadc2, adcConfig.user_channel2);
}

void start()
{
    adcCalibrationData.Vrefint_GAIN = *(uint16_t*)(0x1FFF75AA);

    if(adcConfig.enableOPAMP)
    {
        HAL_OPAMPEx_SelfCalibrateAll(&hopamp1, &hopamp2, &hopamp3);

        HAL_OPAMP_Start(&hopamp1);
        HAL_OPAMP_Start(&hopamp2);
        HAL_OPAMP_Start(&hopamp3);
    }

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    // Regular Channel 使用 DMA, 必须使用 Register Callback.
    extern void ADCRegularChannelCallback(ADC_HandleTypeDef *hadc);
    HAL_ADC_RegisterCallback(&hadc1, HAL_ADC_CONVERSION_COMPLETE_CB_ID, ADCRegularChannelCallback);

    // 先设置 Callback 再启动 ADC Injection
    HAL_ADCEx_InjectedStart_IT(&hadc2);
    HAL_ADCEx_InjectedStart_IT(&hadc1);
}

void triggerRegularConversion()
{
    if(adcConfig.enableRegularChannels == 1)
    {
        HAL_ADC_Start(&hadc2);
        HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)adcRegularChannelBuffer, adcConfig.regularChannelNum);
    }
}

uint16_t fuck = 0;
void decodeInjectedBuffer()
{
    fuck = *analogValuePointers.pIABuffer;
    analogValues.measuredIA = ((float)*analogValuePointers.pIABuffer    - adcCalibrationData.IA_BIAS)   * adcCalibrationData.IA_GAIN;
    analogValues.measuredIB = ((float)*analogValuePointers.pIBBuffer    - adcCalibrationData.IB_BIAS)   * adcCalibrationData.IB_GAIN;
    analogValues.measuredIC = ((float)*analogValuePointers.pICBuffer    - adcCalibrationData.IC_BIAS)   * adcCalibrationData.IC_GAIN;


    // measuredIC 似乎有问题, 在特定 setphrasevoltage 下会震荡.
    analogValues.measuredIphaseSum = analogValues.measuredIA + analogValues.measuredIB + analogValues.measuredIC;

    // // 可能会增大噪声
    // float measuredIphaseSumDiv3 = analogValues.measuredIphaseSum / 3.0f;
    // analogValues.measuredIA -= measuredIphaseSumDiv3;
    // analogValues.measuredIB -= measuredIphaseSumDiv3;
    // analogValues.measuredIC -= measuredIphaseSumDiv3;

}

void decodeRegularBuffer()
{
    if(adcConfig.enableRegularChannels == 1)
    {
        analogValues.Vbus           = ((float)*analogValuePointers.pVbusBuffer  - adcCalibrationData.Vbus_BIAS) * adcCalibrationData.Vbus_GAIN;
        analogValues.measuredVA     = ((float)*analogValuePointers.pVABuffer    - adcCalibrationData.VA_BIAS)   * adcCalibrationData.VA_GAIN;
        analogValues.measuredVB     = ((float)*analogValuePointers.pVBBuffer    - adcCalibrationData.VB_BIAS)   * adcCalibrationData.VB_GAIN;
        analogValues.measuredVC     = ((float)*analogValuePointers.pVCBuffer    - adcCalibrationData.VC_BIAS)   * adcCalibrationData.VC_GAIN;
        analogValues.NTCTemperature = ((float)*analogValuePointers.pNTCBuffer   - adcCalibrationData.NTC_BIAS)  * adcCalibrationData.NTC_GAIN;
        analogValues.Vref           = adcCalibrationData.Vrefint_GAIN / (float)*analogValuePointers.pVREFINTBuffer * VREFINT_CAL_VREF;
        analogValues.userValue1     = (float)*analogValuePointers.pUserBuffer1;
        analogValues.userValue2     = (float)*analogValuePointers.pUserBuffer2;
    }
}

void ADCRegularChannelCallback(ADC_HandleTypeDef *hadc)
{
    decodeRegularBuffer();
}

void resetADCCalibrationData()
{
    IAStatisticsCalculator.reset();
    IBStatisticsCalculator.reset();
    ICStatisticsCalculator.reset();
    VbusStatisticsCalculator.reset();
}

// 每一 ms 调用一次, 添加一个 ADC 数据点, 用于后续的校准计算. 采用 1k 刷新率是因为 Vbus 数据刷新率是 1K.
void addADCCalibrationData()
{
    // 电流校准目的是得到零偏
    IAStatisticsCalculator.addData((float)*analogValuePointers.pIABuffer);
    IBStatisticsCalculator.addData((float)*analogValuePointers.pIBBuffer);
    ICStatisticsCalculator.addData((float)*analogValuePointers.pICBuffer);
    // 电压校准目的是判断电压是否稳定和电压范围
    VbusStatisticsCalculator.addData((float)*analogValuePointers.pVbusBuffer);

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

    // 检查 ADC 数据范围和真实 Vbus 范围.
    float VbusMean = (VbusStatisticsCalculator.getMean() - adcCalibrationData.Vbus_BIAS) * adcCalibrationData.Vbus_GAIN;;
    if(
        FABS(IAStatisticsCalculator.getMean() - 2048.0f)    > 100.0f ||
        FABS(IBStatisticsCalculator.getMean() - 2048.0f)    > 100.0f ||
        FABS(ICStatisticsCalculator.getMean() - 2048.0f)    > 100.0f ||
        VbusMean < Control::ErrorHandler::errorHandlerConfig.underVoltageThreshold ||
        VbusMean > Control::ErrorHandler::errorHandlerConfig.overVoltageThreshold
    )
    {
        return 0;
    }


    // 保存校准结果
    adcCalibrationData.IA_BIAS = (uint16_t)IAStatisticsCalculator.getMean();
    adcCalibrationData.IB_BIAS = (uint16_t)IBStatisticsCalculator.getMean();
    adcCalibrationData.IC_BIAS = (uint16_t)ICStatisticsCalculator.getMean();
    return 1;
}


} // namespace ADC
} // namespace Sensor