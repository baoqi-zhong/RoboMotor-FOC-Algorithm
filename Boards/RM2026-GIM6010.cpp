/**
 * @file RM2026-GIM6010.cpp
 * @brief RM2026-GIM6010 board specific hardware configuration.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#if 0
#include "Boards.hpp"
#include "WS2812.hpp"

namespace Boards
{
const Sensor::ADC::ADCConfig adcConfig = {
    .IA_hadc    = &hadc1,   .IA_channel     = Sensor::ADC::ADCChannel::INJECTED_CHANNEL_1,
    .IB_hadc    = &hadc2,   .IB_channel     = Sensor::ADC::ADCChannel::INJECTED_CHANNEL_1,
    .IC_hadc    = &hadc1,   .IC_channel     = Sensor::ADC::ADCChannel::INJECTED_CHANNEL_2,
    .Vbus_hadc  = &hadc1,   .Vbus_channel   = Sensor::ADC::ADCChannel::INJECTED_CHANNEL_3,
    .VA_hadc    = &hadc2,   .VA_channel     = Sensor::ADC::ADCChannel::INJECTED_CHANNEL_2,
    .VB_hadc    = &hadc2,   .VB_channel     = Sensor::ADC::ADCChannel::INJECTED_CHANNEL_3,
    .VC_hadc    = &hadc2,   .VC_channel     = Sensor::ADC::ADCChannel::INJECTED_CHANNEL_4,
};


Drivers::LED::WS2812Group RGBGroup(6, &htim2, TIM_CHANNEL_3);
Drivers::LED::WS2812 IdLED(&RGBGroup, 0, Drivers::LED::LEDFunctionType::DISPLAY_ID);
Drivers::LED::WS2812 ErrorLED(&RGBGroup, 1, Drivers::LED::LEDFunctionType::DISPLAY_ERROR_ID);
Drivers::LED::WS2812 TestLED(&RGBGroup, 2, Drivers::LED::LEDFunctionType::DISPLAY_CUSTOM_1);

void init()
{
    Drivers::LED::registerLED(&IdLED);
    Drivers::LED::registerLED(&ErrorLED);
    Drivers::LED::registerLED(&TestLED);
}


const BoardLevelHardwareInitConfig boardLevelHardwareInitConfig = {
    .adcConfig = &adcConfig,
};
} // namespace Boards

#endif