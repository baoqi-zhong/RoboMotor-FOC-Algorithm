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
#include "MotorHAL.hpp"
#include "PositionalPID.hpp"

namespace Control::Motor
{

/* 仅在初始化时用于修改 Status */
struct MotorConfigStatic
{
    Sensor::EncoderConfigStatic encoderConfigStatic;
    Sensor::EncoderConfig encoderConfig;
    FOC::FOCConfigStatic focConfigStatic;

    Sensor::ADC::ADCCalibrationData adcCalibrationData;
    ErrorHandler::ErrorHandlerConfig errorHandlerConfig;

    PIDParameters_t positionToCurrentPIDParameters;
    PIDParameters_t positionToVelocityPIDParameters;
    PIDParameters_t velocityPIDParameters;

    bool enableSpeedCloseLoop       = false;
    bool enablePositionCloseLoop    = false;

    float defaultIqLimit            = 1.0f;
    float defaultVelocityLimit      = 0.0f;
    float openLoopRotateSpeed       = 0.0f;
    float openLoopDragVoltage       = 0.0f;
};

template<MotorConfigStatic motorConfigStatic, typename MotorHAL>
class Motor
{
public:
    enum class MotorState : uint8_t
    {
        Stop = 0,
        preADCCalibrating,
        ADCCalibrating,
        preChargingBootCap,
        ChargingBootCap,
        preRunning,
        Running,

        preMusic,
        Music,
    };

    enum class MotorCalibrationState : uint8_t
    {
        Stop = 0,
        Calibrating,
    };

    void init();
    void updateEncoder(uint16_t Q16_encoder_);
    void run1KhzLoop();
    void run4KhzLoop();
    void runCurrentLoop();

    void disableMotor();

    MotorHAL motorHAL;
    FOC::FOC<motorConfigStatic.focConfigStatic> foc;
    Sensor::Encoder<motorConfigStatic.encoderConfigStatic> encoder  
                                                    {motorConfigStatic.encoderConfig};
    Sensor::ADC::ADC adc                            {motorConfigStatic.adcCalibrationData};
    ErrorHandler::ErrorHandler errorHandler         {motorConfigStatic.errorHandlerConfig};

    Control::PositionalPID positionToCurrentPID     {motorConfigStatic.positionToCurrentPIDParameters};
    Control::PositionalPID positionToVelocityPID    {motorConfigStatic.positionToVelocityPIDParameters};
    Control::PositionalPID velocityPID              {motorConfigStatic.velocityPIDParameters};

    MotorState state                                = MotorState::Stop;
    MotorCalibrationState calibrationState          = MotorCalibrationState::Stop;

    bool enableSpeedCloseLoop                       = motorConfigStatic.enableSpeedCloseLoop;
    bool enablePositionCloseLoop                    = motorConfigStatic.enablePositionCloseLoop;
    float defaultIqLimit                            = motorConfigStatic.defaultIqLimit;
    float defaultVelocityLimit                      = motorConfigStatic.defaultVelocityLimit;
    float openLoopRotateSpeed                       = motorConfigStatic.openLoopRotateSpeed;
    float openLoopDragVoltage                       = motorConfigStatic.openLoopDragVoltage;

    uint8_t triggerReset                            = 0;

    float targetIq                                  = 0.0f;
    float targetId                                  = 0.0f;
    float targetVelocity                            = 0.0f;
    float targetPosition                            = 0.0f;
};

template<MotorConfigStatic motorConfigStatic, typename MotorHAL>
void Motor<motorConfigStatic, MotorHAL>::init()
{
    Utils::Trigonometric::init();
    // Control::Calibrator::init();
    // Control::InterBoard::init();

    triggerReset = 1;

    motorHAL.startTimerBase();
}

template<MotorConfigStatic motorConfigStatic, typename MotorHAL>
void Motor<motorConfigStatic, MotorHAL>::updateEncoder(uint16_t Q16_encoder_)
{
    encoder.update(Q16_encoder_);
}

template<MotorConfigStatic motorConfigStatic, typename MotorHAL>
void Motor<motorConfigStatic, MotorHAL>::disableMotor()
{
    motorHAL.disableTimerPWMOutput();
    foc.enableFOCOutput = false;
    state = MotorState::Stop;
}

template<MotorConfigStatic motorConfigStatic, typename MotorHAL>
void Motor<motorConfigStatic, MotorHAL>::run1KhzLoop()
{
    if(errorHandler.checkError1KHz(adc.analogValues))
        disableMotor();

    if(errorHandler.checkIfCanAutoRecovery(adc.analogValues, state == MotorState::Stop, triggerReset))
        triggerReset = 1;

    if(triggerReset)
    {
        triggerReset = 0;
        errorHandler.clearAllError();
        state = MotorState::preADCCalibrating;
    }

    // Config/Flash state-machine update hook. ConfigLoader is currently disabled.
    // Control::ConfigLoader::update();

    if (state == MotorState::Running)
    {
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
        motorHAL.disableTimerPWMOutput();
    }

    else if (state == MotorState::preADCCalibrating)
    {
        motorHAL.disableTimerPWMOutput();
        adc.resetADCCalibrationData();
        state = MotorState::ADCCalibrating;
    }

    else if (state == MotorState::ADCCalibrating)
    {
        static uint32_t adcCalibrationCounter = 0;
        if(adcCalibrationCounter < 50)
        {
            if (
                adc.analogValues.Vbus < errorHandler.errorHandlerConfig.underVoltageThreshold || 
                adc.analogValues.Vbus > errorHandler.errorHandlerConfig.overVoltageThreshold
            )
            {
                adcCalibrationCounter = 0;
                state = MotorState::preADCCalibrating;
            }
            
            adc.addADCCalibrationData();
            adcCalibrationCounter += 1;
            return;
        }
        
        if(adc.checkADCCalibrationSuccess() == 0)
        {
            adcCalibrationCounter = 0;
            state = MotorState::preADCCalibrating;
            return;
        }

        state = MotorState::preChargingBootCap;
    }

    else if (state == MotorState::preChargingBootCap)
    {
        motorHAL.setPWMDutyCycle(0.0f, 0.0f, 0.0f);
        motorHAL.enableTimerPWMLowSideOutput();

        state = MotorState::ChargingBootCap;
    }

    else if (state == MotorState::ChargingBootCap)
    {
        if(calibrationState != MotorCalibrationState::Stop)
        {
            motorHAL.enableTimerPWMHighSideOutput();
            state = MotorState::Stop;
            calibrationState = MotorCalibrationState::Calibrating;
        }
        else
        {
            targetIq = 0;
            targetId = 0;
            targetPosition = 0;
            targetVelocity = 0;
            
            state = MotorState::preRunning;
        }
    }

    else if (state == MotorState::preRunning) 
    {
        positionToCurrentPID.reset();
        positionToVelocityPID.reset();
        velocityPID.reset();
        foc.IdPID.reset();
        foc.IqPID.reset();

        motorHAL.enableTimerPWMHighSideOutput();

        foc.enableFOCOutput = true;
        state = MotorState::Running;
    }

    if(calibrationState != MotorCalibrationState::Stop)
    {
        // Control::Calibrator::update();
    }
}

template<MotorConfigStatic motorConfigStatic, typename MotorHAL>
void Motor<motorConfigStatic, MotorHAL>::run4KhzLoop()
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

template<MotorConfigStatic motorConfigStatic, typename MotorHAL>
void Motor<motorConfigStatic, MotorHAL>::runCurrentLoop()
{
    if(errorHandler.checkErrorHighFreq(adc.analogValues))
        disableMotor();

    foc.focInput.measuredIA = adc.analogValues.measuredIA;
    foc.focInput.measuredIB = adc.analogValues.measuredIB;
    foc.focInput.measuredIC = adc.analogValues.measuredIC;
    foc.focInput.measuredVbus = adc.analogValues.Vbus;
    foc.focInput.targetIq = targetIq;
    foc.focInput.targetId = targetId;
    foc.focInput.Q16_electricAngle = encoder.Q16_electricAngle;
    foc.focInput.RAD_electricAngularVelocity = encoder.RAD_electricAngularVelocity;
    foc.focInput.RAD_shaftAngularVelocity = encoder.RAD_shaftAngularVelocity;

    foc.currentLoopUpdate();

    motorHAL.setPWMDutyCycle(foc.focOutput.dutyA, foc.focOutput.dutyB, foc.focOutput.dutyC);
}

} // namespace Control::Motor

