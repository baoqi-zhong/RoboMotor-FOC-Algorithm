/**
 * @file Motor.hpp
 * @brief Motor control state machine and configuration.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#pragma once

#include "stdint.h"

#include "ADC.hpp"
#include "Trigonometric.hpp"
#include "Encoder.hpp"
#include "ErrorHandler.hpp"
#include "FOC.hpp"
#include "Math.hpp"
#include "MotorHardware.hpp"
#include "PositionalPID.hpp"

namespace Control::Motor
{
struct MotorConfig
{
    bool enableSpeedCloseLoop       = false;
    bool enablePositionCloseLoop    = false;

    float defaultIqLimit            = 1.0f;
    float defaultVelocityLimit      = 0.0f;
    float openLoopRotateSpeed       = 0.0f;
    float openLoopDragVoltage       = 0.0f;
};

struct GenericConfigStatic
{
    /* Static Config: Used for Template instantiation */
    Sensor::EncoderConfigStatic encoderConfigStatic;
    FOC::FOCConfigStatic focConfigStatic;

    /* Dynamic Config: Used at runtime */
    Sensor::EncoderConfig encoderConfig;
    Sensor::ADC::ADCCalibrationData adcCalibrationData;
    ErrorHandler::ErrorHandlerConfig errorHandlerConfig;
    MotorConfig motorConfig;

    PIDParameters_t positionToCurrentPIDParameters;
    PIDParameters_t positionToVelocityPIDParameters;
    PIDParameters_t velocityPIDParameters;
};

template<GenericConfigStatic genericConfigStatic, typename MotorHardware>
class Motor
{
public:
    enum class MotorState : uint8_t
    {
        Stop = 0,           /* 如果不使能 Auto Recovery / 手动触发 Recovery, 则会卡在 Stop 状态 */
        ADCCalibrating,     /* 等待电压电流稳定与 ADC 校准成功 */
        GateDriverIniting,  /* 等待 MOS 驱动初始化完毕 */
        ChargingBootCap,    /* 给 BootCap 电容充电 */
        Running,

        CalibratingMotor
    };

    void init();
    void updateEncoder(uint16_t Q16_encoder_);
    void run1KhzLoop();
    void run4KhzLoop();
    void runCurrentLoop();

    void disableMotor();

    MotorHardware motorHardware;
    FOC::FOC<genericConfigStatic.focConfigStatic> foc;
    Sensor::Encoder<genericConfigStatic.encoderConfigStatic> encoder  
                                                    {genericConfigStatic.encoderConfig};
    Sensor::ADC::ADC adc                            {genericConfigStatic.adcCalibrationData};
    ErrorHandler::ErrorHandler errorHandler         {genericConfigStatic.errorHandlerConfig};

    Control::PositionalPID positionToCurrentPID     {genericConfigStatic.positionToCurrentPIDParameters};
    Control::PositionalPID positionToVelocityPID    {genericConfigStatic.positionToVelocityPIDParameters};
    Control::PositionalPID velocityPID              {genericConfigStatic.velocityPIDParameters};

    uint32_t delayCounter                           = 0;    /* Delay Ticks before switching to next state */
    int32_t timeoutCounter                          = 0;    /* Timeout Ticks for each state */
    MotorState state                                = MotorState::Stop;

    bool enableSpeedCloseLoop                       = genericConfigStatic.motorConfig.enableSpeedCloseLoop;
    bool enablePositionCloseLoop                    = genericConfigStatic.motorConfig.enablePositionCloseLoop;
    float defaultIqLimit                            = genericConfigStatic.motorConfig.defaultIqLimit;
    float defaultVelocityLimit                      = genericConfigStatic.motorConfig.defaultVelocityLimit;
    float openLoopRotateSpeed                       = genericConfigStatic.motorConfig.openLoopRotateSpeed;
    float openLoopDragVoltage                       = genericConfigStatic.motorConfig.openLoopDragVoltage;

    uint8_t triggerRecover                            = 0;

    float targetIq                                  = 0.0f;
    float targetId                                  = 0.0f;
    float targetVelocity                            = 0.0f;
    float targetPosition                            = 0.0f;

    float targetIqModulation                        = 0.0f; /* independent Iq ripple, A */
};

template<GenericConfigStatic genericConfigStatic, typename MotorHardware>
void Motor<genericConfigStatic, MotorHardware>::init()
{
    Utils::Trigonometric::init();
    // Control::Calibrator::init();
    // Control::InterBoard::init();

    triggerRecover = 1;
}

template<GenericConfigStatic genericConfigStatic, typename MotorHardware>
void Motor<genericConfigStatic, MotorHardware>::updateEncoder(uint16_t Q16_encoder_)
{
    encoder.update(Q16_encoder_);
}

template<GenericConfigStatic genericConfigStatic, typename MotorHardware>
void Motor<genericConfigStatic, MotorHardware>::disableMotor()
{
    motorHardware.disableGateDriver();
    state = MotorState::Stop;
}

template<GenericConfigStatic genericConfigStatic, typename MotorHardware>
void Motor<genericConfigStatic, MotorHardware>::run1KhzLoop()
{
    if (delayCounter > 0)
    {
        delayCounter -= 1;
        return;
    }

    if (timeoutCounter > 0)
    {
        timeoutCounter -= 1;
    }

    /* Running 保证最低延迟和最常见情况 */
    if (state == MotorState::Running)
    {
        if(errorHandler.checkError1KHz(adc.analogValues, motorHardware))
            disableMotor();
        
        if(enablePositionCloseLoop)
        {
            if(enableSpeedCloseLoop)
            {
                positionToVelocityPID.setOutputLimit(FABS(defaultVelocityLimit));
                targetVelocity = positionToVelocityPID(targetPosition, encoder.RAD_accumulatedShaftAngle);
            }
            else
            {
                positionToCurrentPID.setOutputLimit(FABS(defaultIqLimit));
                targetIq = positionToCurrentPID(targetPosition, encoder.RAD_accumulatedShaftAngle);
            }
        }
        else
        {
            targetVelocity = defaultVelocityLimit;
        }
    }

    else if (state == MotorState::Stop)
    {
        motorHardware.disableGateDriver();
        timeoutCounter = -1; /* 永不超时 */
        if (errorHandler.errorHandlerConfig.enableAutoRecovery || triggerRecover)
        {
            triggerRecover = 0;
            errorHandler.clearError();

            adc.resetStabilityChecker();
            motorHardware.enterCurrentCalibrationMode();

            /* 在真正进入下一个状态之前延迟 200 个周期 */
            delayCounter = errorHandler.errorHandlerConfig.autoRecoveryTimeout;
            timeoutCounter = 100; /* 等待 ADC 校准成功, 最多等待 100 个周期后重试 */
            state = MotorState::ADCCalibrating;
        }
    }

    else if (state == MotorState::ADCCalibrating)
    {
        if(adc.checkStability())
        {
            motorHardware.exitCurrentCalibrationMode();

            /* 开 MOS 前, enable 并 Clear Fault */
            motorHardware.setPWMDutyCycle(0.0f, 0.0f, 0.0f);
            motorHardware.prepareGateDriver();
            
            delayCounter = 10;      /* 等待 MOS 驱动初始化 */
            timeoutCounter = 500;   /* 等待 MOS 驱动初始化完毕, 最多等待 100 个周期后重试 */
            state = MotorState::GateDriverIniting;
        }

        /* 超时处理 */
        if (timeoutCounter == 0)
        {
            errorHandler.errorStatus.adcCalibrationTimeout = 1;
            state = MotorState::Stop;
        }
    }

    else if (state == MotorState::GateDriverIniting)
    {
        if(motorHardware.isGateDriverReady())
        {
            motorHardware.enableLowSideMosfet();
            state = MotorState::ChargingBootCap;
        }

        /* 超时处理 */
        if (timeoutCounter == 0)
        {
            motorHardware.disableGateDriver();
            errorHandler.errorStatus.gateDriverInitTimeout = 1;
            state = MotorState::Stop;
        }
    }

    else if (state == MotorState::ChargingBootCap)
    {
        motorHardware.enableHighSideMosfet();

        targetIq = 0;
        targetId = 0;
        targetPosition = 0;
        targetVelocity = 0;
        
        positionToCurrentPID.reset();
        positionToVelocityPID.reset();
        velocityPID.reset();
        foc.IdPID.reset();
        foc.IqPID.reset();

        state = MotorState::Running;
    }

}

template<GenericConfigStatic genericConfigStatic, typename MotorHardware>
void Motor<genericConfigStatic, MotorHardware>::run4KhzLoop()
{
    if(state != MotorState::Running)
        return;

    if(enableSpeedCloseLoop)
    {
        velocityPID.setOutputLimit(FABS(defaultIqLimit));
        targetIq = velocityPID(targetVelocity, encoder.RAD_shaftAngularVelocity);
    }
    else
    {
        targetIq = defaultIqLimit;
    }
}

template<GenericConfigStatic genericConfigStatic, typename MotorHardware>
void Motor<genericConfigStatic, MotorHardware>::runCurrentLoop()
{
    foc.focInput.measuredIA = adc.analogValues.measuredIA;
    foc.focInput.measuredIB = adc.analogValues.measuredIB;
    foc.focInput.measuredIC = adc.analogValues.measuredIC;
    foc.focInput.measuredVbus = adc.analogValues.Vbus;
    /* Speed PID output plus the independently injected torque ripple. */
    foc.focInput.targetIq = targetIq + targetIqModulation;
    foc.focInput.targetId = targetId;
    foc.focInput.Q16_electricAngle = encoder.Q16_electricAngle;
    foc.focInput.RAD_electricAngularVelocity = encoder.RAD_electricAngularVelocity;
    foc.focInput.RAD_shaftAngularVelocity = encoder.RAD_shaftAngularVelocity;

    foc.currentLoopUpdateMeasure();

    /* 只有在 Running 的情况下, 才尝试错误保护和控制 */
    if(state != MotorState::Running)
        return;

    if(errorHandler.checkErrorHighFreq(adc.analogValues))
    {
        disableMotor();
        return;
    }

    foc.currentLoopCalculateOutput();

    motorHardware.setPWMDutyCycle(foc.focOutput.dutyA, foc.focOutput.dutyB, foc.focOutput.dutyC);
}

} // namespace Control::Motor

