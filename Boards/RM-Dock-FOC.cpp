/**
 * @file RM-Dock-FOC.cpp
 * @brief RM-Dock-FOC board specific hardware configuration.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#if 1

#include "Boards.hpp"
#include "WS2812.hpp"
#include "MotorControl.hpp"
#include "STSPIN32G4MosfetDriver.hpp"
#include "ErrorHandler.hpp"
#include "Encoder.hpp"
#include "InterBoard.hpp"
#include "ConfigLoader.hpp"
#include "FlashManager.hpp"

/**
 * 约定: 各种校准参数和硬件参数只在代码里出现一次, 其余驱动以指针形式引用. 
 * 初始化时必须调用对应的修改函数来修改参数, 否则会出现 nullptr 错误.
 * 校准过程中修改的数组实际上还是此处定义的数组.
 */

namespace Boards
{
#define CURRENT_LOOP_FREQ 20000.0f

/* 电机参数, 可以运行中校准 */
Control::FOC::MotorConfig motorConfig = {
    .REVERSE_DIRECTION              = 0,
    .POLE_PAIRS                     = 7,
    .shaftReductionRatio            = 1.0f,
    .electricAngleReductionRatio    = 7.0f,
    .phaseResistance                = 0.2f,
    .phaseInductance                = 0.0004f,
    .kv                             = 350.0f,
};

Control::FOC::FOCConfig focConfig = {
    .currentLoopFreq = CURRENT_LOOP_FREQ,
};

/* ADC 参数, 运行过程中不修改 */
const Sensor::ADC::ADCConfig adcConfig = {
    .IA_hadc    = Sensor::ADC::ADCIndex::DISABLED,   .IA_channel     = Sensor::ADC::ADCChannel::INJECTED_CHANNEL_1,
    .IB_hadc    = Sensor::ADC::ADCIndex::DISABLED,   .IB_channel     = Sensor::ADC::ADCChannel::INJECTED_CHANNEL_1,
    .IC_hadc    = Sensor::ADC::ADCIndex::DISABLED,   .IC_channel     = Sensor::ADC::ADCChannel::INJECTED_CHANNEL_2,

    .Vbus_hadc      = Sensor::ADC::ADCIndex::DISABLED,   .Vbus_channel       = Sensor::ADC::ADCChannel::REGULAR_CHANNEL_1,
    .VREFINT_hadc   = Sensor::ADC::ADCIndex::DISABLED,   .Vrefint_channel    = Sensor::ADC::ADCChannel::REGULAR_CHANNEL_2,
    .user_hadc1     = Sensor::ADC::ADCIndex::DISABLED,   .user_channel1      = Sensor::ADC::ADCChannel::REGULAR_CHANNEL_1,
    .user_hadc2     = Sensor::ADC::ADCIndex::DISABLED,   .user_channel2      = Sensor::ADC::ADCChannel::REGULAR_CHANNEL_2,

    .enableOPAMP = 1,
    .enableRegularChannels = 1,
    .regularChannelNum = 2
};

/* 需要在运行开始时重新校准 Bias */
Sensor::ADC::ADCCalibrationData adcCalibrationData = {
    .IA_BIAS    = 2048, .IA_GAIN   = -0.006679319f,
    .IB_BIAS    = 2048, .IB_GAIN   = -0.006679319f,
    .IC_BIAS    = 2048, .IC_GAIN   = -0.006679319f,
    .Vbus_BIAS  = 0,    .Vbus_GAIN = 0.00779f
};

Sensor::Encoder::EncoderConfig encoderConfig = {
    .driverType = Sensor::Encoder::EncoderDriverType::MA732,
    .encoderZeroOffset = 42960,
    .encoderCompensationTable = {0},
    .encoderCompensationGain = 1.0f,
    .enableEncoderCompensation = 1,
    .encoderDelayTime = 0.0f,
    .encoderDifferenceLPFAlpha = 0.05f
};

/* 控制器参数, 运行过程中不修改 */
const Control::MotorControl::MotorControlConfig motorControlConfig = {
    .enableSpeedCloseLoop   = 0,
    .enablePositionCloseLoop= 0,

    .defaultIqLimit          = 6.0f,
    .defaultVelocityLimit    = 0.0f,
    .openLoopRotateSpeed     = 10.0f,
    .openLoopDragVoltage     = 2.0f,

    .boardID                  = 1
};

const Control::ErrorHandler::ErrorHandlerConfig errorHandlerConfig = {
    .ignoreAllErrors = 0,

    .underVoltageThreshold = 12.0f,
    .overVoltageThreshold = 30.0f,
    .overCurrentThreshold = 12.0f,

    .underVoltageTriggerTimeout = 500,
    .overVoltageTriggerTimeout = 500,
    .overCurrentTriggerTimeout = 10,
    .overTemperatureTriggerTimeout = 10000
};

Control::PIDParameters_t positionToCurrentPIDParam = {
    .kPonError = 0.07f,
    .kIonError = 0.03f,
    .kDonMeasurement = 0.002f,
    .kPonMeasurement = 0.0f,
    .kDonTarget = 0.001f,
    .alpha = 0.1f,
    .outputLimit = motorControlConfig.defaultIqLimit,
    .updateFrequency = 1000.0f
};

Control::PIDParameters_t positionToVelocityPIDParam = {
    .kPonError = 0.1f,
    .kIonError = 1.0f,
    .kDonMeasurement = 0.05f,
    .kPonMeasurement = 0.0f,
    .kDonTarget = 0.0f,
    .alpha = 0.1f,
    .outputLimit = motorControlConfig.defaultVelocityLimit,
    .updateFrequency = 1000.0f
};

Control::PIDParameters_t velocityPIDParam = {
    .kPonError = 0.4f,
    .kIonError = 25.0f,
    .kDonMeasurement = 0.001f,
    .kPonMeasurement = 0.0f,
    .kDonTarget = 0.0f,
    .alpha = 0.02f,
    .outputLimit = motorControlConfig.defaultIqLimit,
    .updateFrequency = 4000.0f
};

Control::PIDParameters_t IqPIDParameters =
{
    .kPonError = 0.4f,
    .kIonError = 200.0f,
    .kDonMeasurement = 0.0f,
    .kPonMeasurement = 0.0f,
    .kDonTarget = 0.0f,
    .alpha = 0.0f,
    .outputLimit = 24.0f,
    .updateFrequency = CURRENT_LOOP_FREQ
};

/* 板载 LED 配置 */
Drivers::LED::WS2812Group   RGBGroup(6, &htim3, TIM_CHANNEL_2);
Drivers::LED::WS2812        IdLED(&RGBGroup, 0, Drivers::LED::LEDFunctionType::DISPLAY_ID);
Drivers::LED::WS2812        ErrorLED(&RGBGroup, 1, Drivers::LED::LEDFunctionType::DISPLAY_ERROR_ID);

Control::InterBoard::InterBoardConfig interBoardConfig = {
    .CANFilterMask                      = 0x7FF,
    .CANFilterID                        = 0x201,
    .interboardDisconnectTriggerTimeout = 200
};

void init()
{
    // Control::ConfigLoader::init();
    // Drivers::FlashManager::erasePageAsync(63, nullptr);
    // HAL_Delay(100);

    // if(Control::ConfigLoader::loadAllConfigFromFlash() != Control::ConfigLoader::ConfigLoaderError::NoError)
    {
        // 写入默认配置
        Sensor::ADC::setConfig(&adcConfig, &adcCalibrationData);
        Sensor::Encoder::setConfig(&encoderConfig);

        Control::FOC::setMotorConfig(&motorConfig);
        Control::FOC::setFOCConfig(&focConfig);
        Control::FOC::IqPID.setParameters(IqPIDParameters);
        Control::FOC::IdPID.setParameters(IqPIDParameters);

        Control::MotorControl::positionToCurrentPID.setParameters(positionToCurrentPIDParam);
        Control::MotorControl::positionToVelocityPID.setParameters(positionToVelocityPIDParam);
        Control::MotorControl::velocityPID.setParameters(velocityPIDParam);
        Control::MotorControl::setConfig(&motorControlConfig);

        Control::ErrorHandler::setConfig(&errorHandlerConfig);
        Control::InterBoard::setConfig(&interBoardConfig);


        // Control::ConfigLoader::saveAllConfigToFlashAsync();

        // Control::MotorControl::motorControlStatus.enableCalibration = 1; // 进入校准状态
    }


    Sensor::Encoder::MA732::init(&hspi1);

    if(Drivers::STSPIN32G4MosfetDriver::init())
    {
        Control::ErrorHandler::motorErrorStatus.STSPIN32G4MosfetDriverErrorStatus.I2C_CommunicationError = 1;
    }

    Drivers::LED::registerLED(&IdLED);
    Drivers::LED::registerLED(&ErrorLED);
}
} // namespace Boards

#endif