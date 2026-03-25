/**
 * @file ErrorHandler.cpp
 * @brief System error handling and protection logic.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "ErrorHandler.hpp"
#include "Math.hpp"

#include "MotorControl.hpp"
#include "ThreePhaseFOC.hpp"
#include "IncrementalPID.hpp"
#include "PositionalPID.hpp"
#include "ADC.hpp"
#include "LED.hpp"
#include "InterBoard.hpp"
#include "ADC.hpp"
#include "Encoder.hpp"

namespace Control
{
namespace ErrorHandler
{
ErrorStatus motorErrorStatus;
ErrorCounter_t motorErrorCounter;
ErrorHandlerConfig errorHandlerConfig;

/*
  启动之后写入一个 magic number, 
  后续重启时检查这个 magic number, 如果读到了, 说明是不断电重启(热启动).
  定义为 noinit 是为了保证这个变量在软件重启时不会被清零.
*/
static volatile __attribute__((section (".noinit"))) uint32_t hotStartMagicNumber;
static volatile __attribute__((section (".noinit"))) uint32_t hardfaultCounter;

void setConfig(const ErrorHandlerConfig *config)
{
    errorHandlerConfig = *config;
}

void init()
{
    motorErrorStatus.underVoltage = 0;
    motorErrorStatus.overVoltage = 0;
    motorErrorStatus.overCurrent = 0;

    motorErrorStatus.overTemperature = 0;
    motorErrorStatus.underTemperautre = 0;
    motorErrorStatus.encoderError = 0;  
    motorErrorStatus.motorDisconnected = 0;

    motorErrorStatus.STSPIN32G4MosfetDriverErrorStatus.I2C_CommunicationError = 0;
    motorErrorStatus.STSPIN32G4MosfetDriverErrorStatus.VDSProtectionTriggered = 0;
    motorErrorStatus.STSPIN32G4MosfetDriverErrorStatus.overTemperature = 0;
    motorErrorStatus.STSPIN32G4MosfetDriverErrorStatus.underVoltage = 0;

    motorErrorCounter.underVoltageCounter = 0;
    motorErrorCounter.overVoltageCounter = 0;
    motorErrorCounter.overCurrentCounter = 0;
    motorErrorCounter.currnentSensorErrorCounter = 0;

    motorErrorCounter.noErrorCounter = 0;

    if(hotStartMagicNumber == 0xC0FFEE)
    {
        hardfaultCounter +=1 ;
    }
    else
    {
        hotStartMagicNumber = 0xC0FFEE;
        hardfaultCounter = 0;
    }
}


uint8_t checkIfAnyErrorStatus()
{
    return motorErrorStatus.underVoltage    || 
        motorErrorStatus.overVoltage        || 
        motorErrorStatus.overCurrent        ||
        motorErrorStatus.ADCDecoderError    ||
        #if USE_NTC
        motorErrorStatus.underTemperautre   ||
        motorErrorStatus.overTemperature    || 
        #endif
        motorErrorStatus.encoderError       ||
        motorErrorStatus.motorDisconnected  ||
        motorErrorStatus.STSPIN32G4MosfetDriverErrorStatus.I2C_CommunicationError   ||
        motorErrorStatus.STSPIN32G4MosfetDriverErrorStatus.VDSProtectionTriggered   ||
        motorErrorStatus.STSPIN32G4MosfetDriverErrorStatus.overTemperature          ||
        motorErrorStatus.STSPIN32G4MosfetDriverErrorStatus.underVoltage;
}


uint8_t checkIfStillInError()
{
    if(Sensor::ADC::analogValues.Vbus < errorHandlerConfig.underVoltageThreshold || Sensor::ADC::analogValues.Vbus > errorHandlerConfig.overVoltageThreshold)
        return 1;
    
    if(Sensor::ADC::analogValues.measuredIA > errorHandlerConfig.overCurrentThreshold || 
        Sensor::ADC::analogValues.measuredIB > errorHandlerConfig.overCurrentThreshold || 
        Sensor::ADC::analogValues.measuredIC > errorHandlerConfig.overCurrentThreshold ||
        Sensor::ADC::analogValues.measuredIA < -errorHandlerConfig.overCurrentThreshold ||
        Sensor::ADC::analogValues.measuredIB < -errorHandlerConfig.overCurrentThreshold ||
        Sensor::ADC::analogValues.measuredIC < -errorHandlerConfig.overCurrentThreshold
    )
        return 1;
    
    if(FABS(Sensor::ADC::analogValues.measuredIphaseSum) > errorHandlerConfig.overCurrentThreshold / 5.0f)
        return 1;

    #if USE_NTC
    if(NTCTemperature < errorHandlerConfig.underTemperatureThreshold || NTCTemperature > errorHandlerConfig.overTemperatureThreshold)
        return 1;
    #endif

    // 跑到这里代表电压电流已经正常, 如果有 driver fault 可以尝试触发复位
    // 下一次调用 CheckError1KHz 时会再次检查 driver fault
    if(HAL_GPIO_ReadPin(nFAULT_GPIO_Port, nFAULT_Pin) == GPIO_PIN_RESET)
    {
        if(Drivers::STSPIN32G4MosfetDriver::clearFault())
        {
            motorErrorStatus.STSPIN32G4MosfetDriverErrorStatus.I2C_CommunicationError = 1;
        }
        return 1;
    }

    
    return 0;
}


// 以 1KHz 频率调用
void checkError1KHz()
{
    if(errorHandlerConfig.ignoreAllErrors)
        return;

    if(Sensor::ADC::analogValues.Vbus < errorHandlerConfig.underVoltageThreshold)
    {
        if(motorErrorCounter.underVoltageCounter < errorHandlerConfig.underVoltageTriggerTimeout)
        {
            motorErrorCounter.underVoltageCounter ++;
        }
        else
        {
            Control::FOC::disableFOC();
            motorErrorStatus.underVoltage = 1;
            Drivers::LED::blink(Drivers::LED::LEDFunctionType::DISPLAY_ERROR_ID, 2);
        }
    }
    else if(Sensor::ADC::analogValues.Vbus > errorHandlerConfig.overVoltageThreshold)
    {
        if(motorErrorCounter.overVoltageCounter < errorHandlerConfig.overVoltageTriggerTimeout)
        {
            motorErrorCounter.overVoltageCounter ++;
        }
        else
        {
            Control::FOC::disableFOC();
            motorErrorStatus.overVoltage = 1;
            Drivers::LED::blink(Drivers::LED::LEDFunctionType::DISPLAY_ERROR_ID, 3);
        }
    }
    else
    {
        if(motorErrorCounter.underVoltageCounter)
            motorErrorCounter.underVoltageCounter --;
        if(motorErrorCounter.overVoltageCounter)
            motorErrorCounter.overVoltageCounter --;
    }

    // 三相电流和不为 0
    if(FABS(Sensor::ADC::analogValues.measuredIphaseSum) > errorHandlerConfig.overCurrentThreshold / 5.0f)
    {
        if(motorErrorCounter.currnentSensorErrorCounter < errorHandlerConfig.overCurrentTriggerTimeout)
        {
            motorErrorCounter.currnentSensorErrorCounter ++;
        }
        else
        {
            Control::FOC::disableFOC();
            motorErrorStatus.ADCDecoderError = 1;
            Drivers::LED::blink(Drivers::LED::LEDFunctionType::DISPLAY_ERROR_ID, 4);
        }
    }
    else
    {
        if(motorErrorCounter.currnentSensorErrorCounter)
            motorErrorCounter.currnentSensorErrorCounter --;
    }


    // 不需要累加的错误
    if(Sensor::ADC::analogValues.NTCTemperature < errorHandlerConfig.underTemperatureThreshold)
    {
        Control::FOC::disableFOC();
        motorErrorStatus.underTemperautre = 1;
        Drivers::LED::blink(Drivers::LED::LEDFunctionType::DISPLAY_ERROR_ID, 5);
    }
    else if(Sensor::ADC::analogValues.NTCTemperature > errorHandlerConfig.overTemperatureThreshold)
    {
        Control::FOC::disableFOC();
        motorErrorStatus.overTemperature = 1;
        Drivers::LED::blink(Drivers::LED::LEDFunctionType::DISPLAY_ERROR_ID, 5);
    }

    if(HAL_GPIO_ReadPin(nFAULT_GPIO_Port, nFAULT_Pin) == GPIO_PIN_RESET)
    {
        uint8_t status;
        if(Drivers::STSPIN32G4MosfetDriver::readStatus(&status))
        {
            motorErrorStatus.STSPIN32G4MosfetDriverErrorStatus.I2C_CommunicationError = 1;
        }
        else
        {
            motorErrorStatus.STSPIN32G4MosfetDriverErrorStatus.I2C_CommunicationError   = 0;
            motorErrorStatus.STSPIN32G4MosfetDriverErrorStatus.underVoltage             = status        & 0x01;
            motorErrorStatus.STSPIN32G4MosfetDriverErrorStatus.overTemperature          = (status >> 1) & 0x01;
            motorErrorStatus.STSPIN32G4MosfetDriverErrorStatus.VDSProtectionTriggered   = (status >> 2) & 0x01;
        }
        Drivers::LED::blink(Drivers::LED::LEDFunctionType::DISPLAY_ERROR_ID, 6);
    }
}

void checkErrorHighFreq()
{
    if(errorHandlerConfig.ignoreAllErrors)
        return;

    if(Sensor::ADC::analogValues.measuredIA > errorHandlerConfig.overCurrentThreshold || 
        Sensor::ADC::analogValues.measuredIB > errorHandlerConfig.overCurrentThreshold || 
        Sensor::ADC::analogValues.measuredIC > errorHandlerConfig.overCurrentThreshold ||
        Sensor::ADC::analogValues.measuredIA < -errorHandlerConfig.overCurrentThreshold ||
        Sensor::ADC::analogValues.measuredIB < -errorHandlerConfig.overCurrentThreshold ||
        Sensor::ADC::analogValues.measuredIC < -errorHandlerConfig.overCurrentThreshold
    )
    {
        if(motorErrorCounter.overCurrentCounter < errorHandlerConfig.overCurrentTriggerTimeout)
            motorErrorCounter.overCurrentCounter ++;
        else
        {
            motorErrorStatus.overCurrent = 1;
            Control::FOC::disableFOC();
            Drivers::LED::blink(Drivers::LED::LEDFunctionType::DISPLAY_ERROR_ID, 4);
        }
    }
    else if(motorErrorCounter.overCurrentCounter)
    {
        motorErrorCounter.overCurrentCounter --;
    }
}

void checkIfCanAutoRecovery()
{
    // 只有当有错误且不再处于错误状态时才进行 auto Recovery
    if( Control::MotorControl::motorControlStatus.state                == Control::MotorControl::MotorControlState::Stop && 
        errorHandlerConfig.enableAutoRecovery  == 1  && 
        Control::ErrorHandler::checkIfAnyErrorStatus()    == 1  &&
        Control::ErrorHandler::checkIfStillInError()      == 0  &&
        Control::MotorControl::motorControlStatus.triggerReset         == 0)
    {
        // 进行 auto Recovery
        // 这里只 set 变量而不进行真正的 Trigger reset 是希望能通过 CAN 调用触发 reset.
        motorErrorCounter.noErrorCounter ++;
        if(motorErrorCounter.noErrorCounter > errorHandlerConfig.autoRecoveryTimeout)
        {
            Control::MotorControl::motorControlStatus.triggerReset = 1;
            motorErrorCounter.noErrorCounter = 0;
        }
    }
}

void clearAllError()
{
    motorErrorStatus.underVoltage       = 0;
    motorErrorStatus.overVoltage        = 0;
    motorErrorStatus.overCurrent        = 0;
    motorErrorStatus.ADCDecoderError    = 0;
    motorErrorStatus.overTemperature    = 0;
    motorErrorStatus.underTemperautre   = 0;
    motorErrorStatus.encoderError       = 0;
    motorErrorStatus.motorDisconnected  = 0;
    motorErrorStatus.STSPIN32G4MosfetDriverErrorStatus.I2C_CommunicationError = 0;
    motorErrorStatus.STSPIN32G4MosfetDriverErrorStatus.VDSProtectionTriggered = 0;
    motorErrorStatus.STSPIN32G4MosfetDriverErrorStatus.overTemperature        = 0;
    motorErrorStatus.STSPIN32G4MosfetDriverErrorStatus.underVoltage           = 0;
}


// Hardfault / Watchdog 引起的严重错误处理.
void GGHandler()
{
    HAL_NVIC_SystemReset();
}

} // namespace ErrorHandler
} // namespace Control