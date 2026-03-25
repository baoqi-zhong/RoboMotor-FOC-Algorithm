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

namespace Sensor
{
namespace ADC
{
struct AnalogValuePointers
{
    uint16_t *pIABuffer         = nullptr;
    uint16_t *pIBBuffer         = nullptr;
    uint16_t *pICBuffer         = nullptr;
    uint16_t *pVbusBuffer       = nullptr;
    uint16_t *pVABuffer         = nullptr;
    uint16_t *pVBBuffer         = nullptr;
    uint16_t *pVCBuffer         = nullptr;
    uint16_t *pNTCBuffer        = nullptr;
    uint16_t *pVREFINTBuffer    = nullptr;

    uint16_t* pUserBuffer1      = nullptr;
    uint16_t* pUserBuffer2      = nullptr;
};

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
    float Vref                  = 0.0f;

    float userValue1             = 0.0f;
    float userValue2             = 0.0f;
};
extern AnalogValues analogValues;

enum class ADCIndex : uint8_t
{
    DISABLED = 0,
    ADC_1,
    ADC_2,
};

enum class ADCChannel : uint8_t
{
    DISABLED = 0,
    REGULAR_CHANNEL_1,
    REGULAR_CHANNEL_2,
    REGULAR_CHANNEL_3,
    REGULAR_CHANNEL_4,
    INJECTED_CHANNEL_1,
    INJECTED_CHANNEL_2,
    INJECTED_CHANNEL_3,
    INJECTED_CHANNEL_4
};

struct ADCConfig
{
    ADCIndex IA_hadc            = ADCIndex::DISABLED;
    ADCChannel IA_channel       = ADCChannel::DISABLED;
    ADCIndex IB_hadc            = ADCIndex::DISABLED;
    ADCChannel IB_channel       = ADCChannel::DISABLED;
    ADCIndex IC_hadc            = ADCIndex::DISABLED;
    ADCChannel IC_channel       = ADCChannel::DISABLED;
    ADCIndex Vbus_hadc          = ADCIndex::DISABLED;
    ADCChannel Vbus_channel     = ADCChannel::DISABLED;
    ADCIndex VA_hadc            = ADCIndex::DISABLED;
    ADCChannel VA_channel       = ADCChannel::DISABLED;
    ADCIndex VB_hadc            = ADCIndex::DISABLED;
    ADCChannel VB_channel       = ADCChannel::DISABLED;
    ADCIndex VC_hadc            = ADCIndex::DISABLED;
    ADCChannel VC_channel       = ADCChannel::DISABLED;
    ADCIndex NTC_hadc           = ADCIndex::DISABLED;
    ADCChannel NTC_channel      = ADCChannel::DISABLED;
    ADCIndex VREFINT_hadc       = ADCIndex::DISABLED;
    ADCChannel Vrefint_channel  = ADCChannel::DISABLED;
    ADCIndex user_hadc1         = ADCIndex::DISABLED;
    ADCChannel user_channel1    = ADCChannel::DISABLED;
    ADCIndex user_hadc2         = ADCIndex::DISABLED;
    ADCChannel user_channel2    = ADCChannel::DISABLED;

    uint8_t enableOPAMP             = 0;
    uint8_t enableRegularChannels   = 0;
    uint8_t regularChannelNum       = 0;
};

/**
 * 默认值: 
 * - Vref = 2.9V
 * - 电流 ADC 零偏为半量程, 增益为 1; 
 * - 电压 ADC 零偏为 0, 默认分压比 11:1;
 * - NTC 10K, 作为下电阻, 上电阻 10K.
 */
struct ADCCalibrationData
{
    uint16_t IA_BIAS    = 2048.0f;
    float IA_GAIN       = 0.005f;
    uint16_t IB_BIAS    = 2048.0f;
    float IB_GAIN       = 0.005f;
    uint16_t IC_BIAS    = 2048.0f;
    float IC_GAIN       = 0.005f;

    uint16_t VA_BIAS    = 0;
    float VA_GAIN       = 0.00779f;
    uint16_t VB_BIAS    = 0;
    float VB_GAIN       = 0.00779f;
    uint16_t VC_BIAS    = 0;
    float VC_GAIN       = 0.00779f;

    uint16_t Vbus_BIAS  = 0;
    float Vbus_GAIN     = 0.00779f;
    uint16_t NTC_BIAS   = 3000.0f;
    float NTC_GAIN      = -0.03f;

    uint16_t Vrefint_GAIN = 0;
};

extern ADCConfig adcConfig;
extern ADCCalibrationData adcCalibrationData;

void setConfig(const ADCConfig* adcConfig, const ADCCalibrationData* adcCalibrationData);

void start();

void triggerRegularConversion();

void decodeInjectedBuffer();

void decodeRegularBuffer();

void resetADCCalibrationData();

void addADCCalibrationData();

uint8_t checkADCCalibrationSuccess();
} // namespace ADC
} // namespace Sensor