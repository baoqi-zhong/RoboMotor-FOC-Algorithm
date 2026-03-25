/**
 * @file MotorControl.cpp
 * @brief Main motor control loop and SVPWM configuration.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
/*
Note: 在转速低的情况下, 较低的 SVPWM 频率能提供更好的电流-电压匹配效果, 电机低速转动也更平稳. 以 17KHz 为宜. 
SVPWM 频率过高 (80KHz) 采样会失真, 使得圆角的正弦波变成偏三角的波形.
经过测试, 缩短变长 ADC 采样时间无法改善高频 SVPWM 下的电流采样效果.

关于磁场方向的注解:
在校准零点的时候, 给定 Ialpha, 使磁铁对齐电角度0点, 此时 reset 磁编码器, 可以把磁编码器的 0 点对齐到电角度 0 点.
在电机运行的时候, 我们给定 Iq. 注意到, 在磁编码器 = 0 时, Id 才是和 alpha 轴共线的, 而 Iq 和 alpha 轴是垂直的. 
所以直接把 Uq 输入到 SVPWM 中, 就能产生与当前磁铁方向垂直的磁场了. 这里不需要手动加上 arctan2 的角度.
*/

// PWM 频率为 170MHz / 5 / 1000 = 34KHz
// 电流采样频率为 34KHz
// 电流闭环控制频率为 34KHz / 2 = 17KHz

// Drivers
#include "Encoder.hpp"
#include "STSPIN32G4MosfetDriver.hpp"
#include "FDCANManager.hpp"
#include "LED.hpp"

// Controls
#include "MotorControl.hpp"
#include "ThreePhaseFOC.hpp"
#include "IncrementalPID.hpp"
#include "PositionalPID.hpp"
#include "ErrorHandler.hpp"
#include "ErrorHandler.hpp"
#include "Calibrator.hpp"
#include "ConfigLoader.hpp"
#include "InterBoard.hpp"

// Utils
#include "Math.hpp"
#include "LPF.hpp"
#include "CordicHelper.hpp"

namespace Control
{
namespace MotorControl
{
MotorControlConfig motorControlConfig;  // 静态参数
MotorControlStatus motorControlStatus;  // 动态参数

Control::PositionalPID positionToCurrentPID;
Control::PositionalPID positionToVelocityPID;
Control::PositionalPID velocityPID;

uint32_t Loop4KHzCounter = 0;

void setConfig(const MotorControlConfig* config)
{
    motorControlConfig = *config;
}

uint32_t dead1 = 8;
uint32_t dead2 = 8;

void init()
{
    Control::ErrorHandler::init();

    // 配置硬件相关参数
    Boards::init();

    // 初始化各个模块
    Sensor::ADC::start();
    Sensor::Encoder::init();
    Utils::CordicHelper::cordic16_init();
    Control::Calibrator::init();
    Control::InterBoard::init();

    // 只开 timer base, 不打开 PWM 输出, 需要在状态机里打开
    HAL_TIM_Base_Start_IT(&htim1);
    // 这样做的目的是修改TIM1 Update Event 的相位
    htim1.Instance->RCR = 1;
    HAL_TIMEx_ConfigDeadTime(&htim1, dead1);
    HAL_TIMEx_ConfigAsymmetricalDeadTime(&htim1, dead2);

    // 等待 ADC 稳定和输入电压稳定.
    HAL_Delay(5);
    // 4KHz 定时器, 开始运行状态机
    HAL_TIM_Base_Start_IT(&htim16);

    // 触发复位, 自动启动
    motorControlStatus.triggerReset = 1;
}


void triggerResetHandler()
{
    if(motorControlStatus.triggerReset)
    {
        // 尝试手动触发复位, 清除故障标志
        // 仅当没有致命错误才 reset
        motorControlStatus.triggerReset = 0;

        Control::ErrorHandler::clearAllError();
        // 开始运行, 不退出校准模式
        motorControlStatus.state = MotorControlState::preADCCalibrating;
    }
}

void Loop4KHz()
{
    if(motorControlStatus.enableFOCOutput == 0)
    {
        return;
    }

    if(motorControlConfig.enableSpeedCloseLoop)
    {
        velocityPID.setOutputLimit(FABS(motorControlConfig.defaultIqLimit));
        motorControlStatus.targetIq = velocityPID(motorControlStatus.targetVelocity, Sensor::Encoder::encoderStatus.RAD_shaftAngularVelocity);
    }
}

void Loop1KHz()
{
    ErrorHandler::checkError1KHz();
    ErrorHandler::checkIfCanAutoRecovery();
    triggerResetHandler();

    // 更新配置和Flash状态机
    Control::ConfigLoader::update();

    // 可能会出现 Race Condition
    Sensor::ADC::triggerRegularConversion();

    // 更新状态机
    if(motorControlStatus.enableMotor == 0 && motorControlStatus.state != MotorControlState::Stop)
    {
        Control::FOC::disableFOC();
        return;
    }

    if (motorControlStatus.state == MotorControlState::Running)
    {
        if(motorControlStatus.enableFOCOutput == 1)
        {
            // 位置 PID
            if(motorControlConfig.enablePositionCloseLoop)
            {
                if(motorControlConfig.enableSpeedCloseLoop)
                {
                    positionToVelocityPID.setOutputLimit(FABS(motorControlConfig.defaultVelocityLimit));
                    motorControlStatus.targetVelocity = positionToVelocityPID(motorControlStatus.targetPosition, Sensor::Encoder::encoderStatus.RAD_accumulatedShaftAngle);
                }
                else
                {
                    positionToCurrentPID.setOutputLimit(FABS(motorControlConfig.defaultIqLimit));
                    motorControlStatus.targetIq = positionToCurrentPID(motorControlStatus.targetPosition, Sensor::Encoder::encoderStatus.RAD_accumulatedShaftAngle);
                }
            }
            else
            {
                // 如果开环速度, targetVelocity 由 Interboard 控制
                motorControlStatus.targetVelocity = motorControlConfig.defaultVelocityLimit;
            }
        }
    }

    else if (motorControlStatus.state == MotorControlState::Stop)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    }

    else if (motorControlStatus.state == MotorControlState::preADCCalibrating)
    {
        // 关闭 PWM, 校准 ADC 零偏.
        Drivers::LED::onOff(Drivers::LED::LEDFunctionType::DISPLAY_ID, 0);
        Drivers::LED::onOff(Drivers::LED::LEDFunctionType::DISPLAY_ERROR_ID, 0);

        // 阻塞等待电压
        if(Sensor::ADC::analogValues.Vbus < Control::ErrorHandler::errorHandlerConfig.underVoltageThreshold || Sensor::ADC::analogValues.Vbus > Control::ErrorHandler::errorHandlerConfig.overVoltageThreshold)
            return;
        
        Sensor::ADC::resetADCCalibrationData();

        motorControlStatus.state = MotorControlState::ADCCalibrating;
    }

    else if (motorControlStatus.state == MotorControlState::ADCCalibrating)
    {
        // 采样
        static uint32_t adcCalibrationCounter = 0;
        adcCalibrationCounter += 1;
        if(adcCalibrationCounter < 50)
        {
            Sensor::ADC::addADCCalibrationData();
            return;
        }
        adcCalibrationCounter = 0;
        if(Sensor::ADC::checkADCCalibrationSuccess() == 0)
        {
            // 校准失败, reset 重来.
            motorControlStatus.state = MotorControlState::preADCCalibrating;
            return;
        }

        // ADC 零偏校准成功, 继续后续流程
        motorControlStatus.state = MotorControlState::preChargingBootCap;
    }

    else if (motorControlStatus.state == MotorControlState::preChargingBootCap)
    {
        // 不 set 0 似乎会 hardfault.
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

        // 先使能下管充 bootstrap 电容
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
        motorControlStatus.state = MotorControlState::ChargingBootCap;
    }

    else if (motorControlStatus.state == MotorControlState::ChargingBootCap)
    {
        // 使能上管
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

        HAL_TIMEx_ConfigDeadTime(&htim1, dead1);
        HAL_TIMEx_ConfigAsymmetricalDeadTime(&htim1, dead2);

        // 已经充了 1ms, 直接开始正常运行
        Control::FOC::setPhraseVoltage(0.02f, 0.0f);
        // if(motorControlStatus.enableCalibration == 1)
        // {
        //     // 使能上管
        //     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        //     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        //     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
            
        //     /* 停止正常开 FOC 的状态机 */
        //     motorControlStatus.state = MotorControlState::Stop;
        //     motorControlStatus.calibrationState = MotorCalibrationState::preCalibrating;
        // }
        // else
        // {
        //     // 防止覆盖校准逻辑里对 targetIq 的设置
        //     motorControlStatus.targetIq = 0;
        //     motorControlStatus.targetId = 0;
        //     motorControlStatus.targetPosition = 0;
        //     motorControlStatus.targetVelocity = 0;
            
        //     motorControlStatus.state = MotorControlState::preRunning;
        // }
    }

    else if (motorControlStatus.state == MotorControlState::preRunning) 
    {
        Drivers::LED::blink(Drivers::LED::LEDFunctionType::DISPLAY_ID, motorControlConfig.boardID);
        Drivers::LED::onOff(Drivers::LED::LEDFunctionType::DISPLAY_ERROR_ID, 0);

        // 使能上管
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

        positionToCurrentPID.reset();
        positionToVelocityPID.reset();
        velocityPID.reset();
        Control::FOC::IqPID.reset();
        Control::FOC::IdPID.reset();
    
        motorControlStatus.enableFOCOutput = 1;
        motorControlStatus.state = MotorControlState::Running;
    }


    /**
     * 校准状态机和正常运行状态机是分开并行跑的, 校准的过程中可以开启 FOC 闭环.
     */
    if(motorControlStatus.enableCalibration == 1)
    {
        if (motorControlStatus.calibrationState == MotorCalibrationState::preCalibrating)
        {
            motorControlStatus.calibrationState = MotorCalibrationState::Calibrating;
        }

        else if (motorControlStatus.calibrationState == MotorCalibrationState::Calibrating)
        {
            Control::Calibrator::update();
        }
    }



    Drivers::LED::update();
}


extern "C" void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    Sensor::ADC::decodeInjectedBuffer();
    
    Control::ErrorHandler::checkErrorHighFreq();

    Control::FOC::currentLoop();
}


extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM1)
    {
        // 电流环同频率, 但是在 ADC 完成之前就能触发读取 Encoder.
        Sensor::Encoder::readBlocking();
    }
    else if (htim->Instance == TIM16)
    {
        // 4KHz 中断
        Control::MotorControl::Loop4KHz();
        Loop4KHzCounter += 1;
        if(Loop4KHzCounter == 4)
        {
            Loop4KHzCounter = 0;
            Control::MotorControl::Loop1KHz();
        }
    }
}

} // namespace MotorControl
} // namespace Control

