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

template<GenericConfigStatic genericConfigStatic, typename MotorHAL>
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
    FOC::FOC<genericConfigStatic.focConfigStatic> foc;
    Sensor::Encoder<genericConfigStatic.encoderConfigStatic> encoder  
                                                    {genericConfigStatic.encoderConfig};
    Sensor::ADC::ADC adc                            {genericConfigStatic.adcCalibrationData};
    ErrorHandler::ErrorHandler errorHandler         {genericConfigStatic.errorHandlerConfig};

    Control::PositionalPID positionToCurrentPID     {genericConfigStatic.positionToCurrentPIDParameters};
    Control::PositionalPID positionToVelocityPID    {genericConfigStatic.positionToVelocityPIDParameters};
    Control::PositionalPID velocityPID              {genericConfigStatic.velocityPIDParameters};

    MotorState state                                = MotorState::Stop;
    MotorCalibrationState calibrationState          = MotorCalibrationState::Stop;

    bool enableSpeedCloseLoop                       = genericConfigStatic.motorConfig.enableSpeedCloseLoop;
    bool enablePositionCloseLoop                    = genericConfigStatic.motorConfig.enablePositionCloseLoop;
    float defaultIqLimit                            = genericConfigStatic.motorConfig.defaultIqLimit;
    float defaultVelocityLimit                      = genericConfigStatic.motorConfig.defaultVelocityLimit;
    float openLoopRotateSpeed                       = genericConfigStatic.motorConfig.openLoopRotateSpeed;
    float openLoopDragVoltage                       = genericConfigStatic.motorConfig.openLoopDragVoltage;

    uint8_t triggerReset                            = 0;

    float targetIq                                  = 0.0f;
    float targetId                                  = 0.0f;
    float targetVelocity                            = 0.0f;
    float targetPosition                            = 0.0f;
};

template<GenericConfigStatic genericConfigStatic, typename MotorHAL>
void Motor<genericConfigStatic, MotorHAL>::init()
{
    Utils::Trigonometric::init();
    // Control::Calibrator::init();
    // Control::InterBoard::init();

    triggerReset = 1;

    motorHAL.startTimerBase();
}

template<GenericConfigStatic genericConfigStatic, typename MotorHAL>
void Motor<genericConfigStatic, MotorHAL>::updateEncoder(uint16_t Q16_encoder_)
{
    encoder.update(Q16_encoder_);
}

template<GenericConfigStatic genericConfigStatic, typename MotorHAL>
void Motor<genericConfigStatic, MotorHAL>::disableMotor()
{
    motorHAL.disableTimerPWMOutput();
    foc.enableFOCOutput = false;
    state = MotorState::Stop;
}

template<GenericConfigStatic genericConfigStatic, typename MotorHAL>
void Motor<genericConfigStatic, MotorHAL>::run1KhzLoop()
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

template<GenericConfigStatic genericConfigStatic, typename MotorHAL>
void Motor<genericConfigStatic, MotorHAL>::run4KhzLoop()
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

template<GenericConfigStatic genericConfigStatic, typename MotorHAL>
void Motor<genericConfigStatic, MotorHAL>::runCurrentLoop()
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

