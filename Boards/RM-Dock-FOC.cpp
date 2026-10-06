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

#include "Boards.hpp"
#if (BOARD_RM_DOCK_FOC)

#include "main.h"
#include "adc.h"
#include "opamp.h"
#include "tim.h"

#include "WS2812.hpp"
#include "ThreePhaseFOC.hpp"
#include "MotorControl.hpp"
#include "STSPIN32G4MosfetDriver.hpp"
#include "ErrorHandler.hpp"
#include "Encoder.hpp"
#include "ADC.hpp"
#include "InterBoard.hpp"

namespace Boards
{
constexpr float CurrentLoopFreq = 20000.0f;

constexpr Control::FOC::MotorConfig motorConfig = {
    .REVERSE_DIRECTION              = 0,
    .POLE_PAIRS                     = 7,
    .shaftReductionRatio            = 1.0f,
    .electricAngleReductionRatio    = 7.0f,
    .phaseResistance                = 0.2f,
    .phaseInductance                = 0.0004f,
    .kv                             = 350.0f,
};

constexpr Control::FOC::FOCConfig focConfig = {
    .currentLoopFreq = CurrentLoopFreq,
};

extern const Sensor::ADC::ADCConfig staticADCConfig = {
    .IA_hadc    = Sensor::ADC::ADCIndex::ADC_1,         .IA_channel         = Sensor::ADC::ADCChannel::INJECTED_CHANNEL_1,
    .IB_hadc    = Sensor::ADC::ADCIndex::ADC_2,         .IB_channel         = Sensor::ADC::ADCChannel::INJECTED_CHANNEL_1,
    .IC_hadc    = Sensor::ADC::ADCIndex::ADC_1,         .IC_channel         = Sensor::ADC::ADCChannel::INJECTED_CHANNEL_2,
    .Vbus_hadc  = Sensor::ADC::ADCIndex::ADC_1,         .Vbus_channel       = Sensor::ADC::ADCChannel::REGULAR_CHANNEL_1,

    .regularChannelNum = 2
};

extern const Sensor::ADC::ADCCalibrationData staticADCCalibrationData = {
    .IA_BIAS    = 2048, .IA_GAIN   = -0.006679319f,
    .IB_BIAS    = 2048, .IB_GAIN   = -0.006679319f,
    .IC_BIAS    = 2048, .IC_GAIN   = -0.006679319f,
    .Vbus_BIAS  = 0,    .Vbus_GAIN = 0.00779f
};

constexpr Sensor::Encoder::EncoderConfig encoderConfig = {
    .driverType = Sensor::Encoder::EncoderDriverType::MA732,
    .encoderZeroOffset = 42960,
    .encoderCompensationTable = {0},
    .encoderCompensationGain = 1.0f,
    .enableEncoderCompensation = 1,
    .encoderDelayTime = 0.0f,
    .encoderDifferenceLPFAlpha = 0.05f
};

constexpr Control::MotorControl::MotorControlConfig motorControlConfig = {
    .enableSpeedCloseLoop   = 0,
    .enablePositionCloseLoop= 0,

    .defaultIqLimit          = 6.0f,
    .defaultVelocityLimit    = 0.0f,
    .openLoopRotateSpeed     = 10.0f,
    .openLoopDragVoltage     = 2.0f,

    .boardID                  = 1
};

constexpr Control::ErrorHandler::ErrorHandlerConfig errorHandlerConfig = {
    .ignoreAllErrors = 0,

    .underVoltageThreshold = 12.0f,
    .overVoltageThreshold = 30.0f,
    .overCurrentThreshold = 12.0f,

    .underVoltageTriggerTimeout = 500,
    .overVoltageTriggerTimeout = 500,
    .overCurrentTriggerTimeout = 10,
    .overTemperatureTriggerTimeout = 10000
};

constexpr Control::PIDParameters_t positionToCurrentPIDParam = {
    .kPonError = 0.07f,
    .kIonError = 0.03f,
    .kDonMeasurement = 0.002f,
    .kPonMeasurement = 0.0f,
    .kDonTarget = 0.001f,
    .alpha = 0.1f,
    .outputLimit = motorControlConfig.defaultIqLimit,
    .updateFrequency = 1000.0f
};

constexpr Control::PIDParameters_t positionToVelocityPIDParam = {
    .kPonError = 0.1f,
    .kIonError = 1.0f,
    .kDonMeasurement = 0.05f,
    .kPonMeasurement = 0.0f,
    .kDonTarget = 0.0f,
    .alpha = 0.1f,
    .outputLimit = motorControlConfig.defaultVelocityLimit,
    .updateFrequency = 1000.0f
};

constexpr Control::PIDParameters_t velocityPIDParam = {
    .kPonError = 0.4f,
    .kIonError = 25.0f,
    .kDonMeasurement = 0.001f,
    .kPonMeasurement = 0.0f,
    .kDonTarget = 0.0f,
    .alpha = 0.02f,
    .outputLimit = motorControlConfig.defaultIqLimit,
    .updateFrequency = 4000.0f
};

constexpr Control::PIDParameters_t IqPIDParameters =
{
    .kPonError = 0.4f,
    .kIonError = 200.0f,
    .kDonMeasurement = 0.0f,
    .kPonMeasurement = 0.0f,
    .kDonTarget = 0.0f,
    .alpha = 0.0f,
    .outputLimit = 24.0f,
    .updateFrequency = CurrentLoopFreq
};

Drivers::LED::WS2812Group   RGBGroup(6, &htim3, TIM_CHANNEL_2);
Drivers::LED::WS2812        IdLED(&RGBGroup, 0, Drivers::LED::LEDFunctionType::DISPLAY_ID);
Drivers::LED::WS2812        ErrorLED(&RGBGroup, 1, Drivers::LED::LEDFunctionType::DISPLAY_ERROR_ID);

constexpr Control::InterBoard::InterBoardConfig interBoardConfig = {
    .CANFilterMask                      = 0x7FF,
    .CANFilterID                        = 0x201,
    .interboardDisconnectTriggerTimeout = 200
};

void startTimerBase()
{
    // 只开 timer base, 不打开 PWM 输出, 需要在状态机里打开
    HAL_TIM_Base_Start_IT(&htim1);
    // 这样做的目的是修改TIM1 Update Event 的相位
    htim1.Instance->RCR = 1;
    HAL_TIMEx_ConfigDeadTime(&htim1, 8);
    HAL_TIMEx_ConfigAsymmetricalDeadTime(&htim1, 8);

    // 4KHz 定时器, 开始运行状态机
    HAL_TIM_Base_Start_IT(&htim16);
}

void startTimerPWMLowSide()
{

}

void startTimerPWMHighSide()
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

static void TIM_CCxNChannelCmd(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelNState)
{
  uint32_t tmp;

  tmp = TIM_CCER_CC1NE << (Channel & 0xFU); /* 0xFU = 15 bits max shift */

  /* Reset the CCxNE Bit */
  TIMx->CCER &=  ~tmp;

  /* Set or reset the CCxNE Bit */
  TIMx->CCER |= (uint32_t)(ChannelNState << (Channel & 0xFU)); /* 0xFU = 15 bits max shift */
}

void stopTimerPWM()
{
    htim1.Instance->CCR1 = 0;
    htim1.Instance->CCR2 = 0;
    htim1.Instance->CCR3 = 0;

    TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_1 | TIM_CHANNEL_2 | TIM_CHANNEL_3, TIM_CCx_DISABLE);
    TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_1 | TIM_CHANNEL_2 | TIM_CHANNEL_3, TIM_CCx_DISABLE);
}

void setTimerPWMDutyCycle(float dutyCycleA, float dutyCycleB, float dutyCycleC)
{
    float arr = htim1.Instance->ARR + 1;
    htim1.Instance->CCR1 = dutyCycleA * arr;
    htim1.Instance->CCR2 = dutyCycleB * arr;
    htim1.Instance->CCR3 = dutyCycleC * arr;
}

void startAnalog()
{
    HAL_OPAMPEx_SelfCalibrateAll(&hopamp1, &hopamp2, &hopamp3);

    HAL_OPAMP_Start(&hopamp1);
    HAL_OPAMP_Start(&hopamp2);
    HAL_OPAMP_Start(&hopamp3);

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    // HAL_ADC_RegisterCallback(&hadc1, HAL_ADC_CONVERSION_COMPLETE_CB_ID, ADCRegularChannelCallback);

    HAL_ADCEx_InjectedStart_IT(&hadc2);
    HAL_ADCEx_InjectedStart_IT(&hadc1);
}

void init()
{
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

    Sensor::Encoder::MA732::init(&hspi1);

    if(Drivers::STSPIN32G4MosfetDriver::init())
    {
        Control::ErrorHandler::motorErrorStatus.STSPIN32G4MosfetDriverErrorStatus.I2C_CommunicationError = 1;
    }

    Drivers::LED::registerLED(&IdLED);
    Drivers::LED::registerLED(&ErrorLED);
}



extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    Control::MotorControl::ADC_RegularConvCpltEntry();
}

extern "C" void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    Control::MotorControl::ADC_InjectedConvCpltEntry();
}

uint32_t Loop4KHzCounter = 0;
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM1)
    {
        Control::MotorControl::ADC_InjectedConvBeginEntry();
    }
    else if (htim->Instance == TIM16)
    {
        // 4KHz 中断
        Control::MotorControl::TIM_4KHzEntry();
        Loop4KHzCounter += 1;
        if(Loop4KHzCounter == 4)
        {
            Loop4KHzCounter = 0;
            Control::MotorControl::TIM_1KHzEntry();
        }
    }
}

} // namespace Boards

#endif
