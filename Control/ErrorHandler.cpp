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

#include "FOC.hpp"
#include "IncrementalPID.hpp"
#include "PositionalPID.hpp"
#include "ADC.hpp"
#include "InterBoard.hpp"
#include "ADC.hpp"
#include "Encoder.hpp"

namespace Control::ErrorHandler
{

/*
  启动之后写入一个 magic number, 
  后续重启时检查这个 magic number, 如果读到了, 说明是不断电重启(热启动).
  定义为 noinit 是为了保证这个变量在软件重启时不会被清零.
*/
// static volatile __attribute__((section (".noinit"))) uint32_t hotStartMagicNumber;
// static volatile __attribute__((section (".noinit"))) uint32_t hardfaultCounter;


uint8_t ErrorHandler::checkIfAnyErrorStatus()
{
    return errorStatus.underVoltage    || 
        errorStatus.overVoltage        || 
        errorStatus.overCurrent        ||
        errorStatus.ADCDecoderError    ||
        errorStatus.underTemperature   ||
        errorStatus.overTemperature    || 
        errorStatus.encoderError       ||
        errorStatus.motorDisconnected;
}


uint8_t ErrorHandler::checkIfStillInError(const Sensor::ADC::AnalogValues& analogValues)
{
    if(analogValues.Vbus < errorHandlerConfig.underVoltageThreshold || analogValues.Vbus > errorHandlerConfig.overVoltageThreshold)
        return 1;
    
    if(analogValues.measuredIA > errorHandlerConfig.overCurrentThreshold || 
        analogValues.measuredIB > errorHandlerConfig.overCurrentThreshold || 
        analogValues.measuredIC > errorHandlerConfig.overCurrentThreshold ||
        analogValues.measuredIA < -errorHandlerConfig.overCurrentThreshold ||
        analogValues.measuredIB < -errorHandlerConfig.overCurrentThreshold ||
        analogValues.measuredIC < -errorHandlerConfig.overCurrentThreshold
    )
        return 1;
    
    if(FABS(analogValues.measuredIphaseSum) > errorHandlerConfig.overCurrentThreshold / 5.0f)
        return 1;

    if(analogValues.NTCTemperature < errorHandlerConfig.underTemperatureThreshold || analogValues.NTCTemperature > errorHandlerConfig.overTemperatureThreshold)
        return 1;

    // 跑到这里代表电压电流已经正常, 如果有 driver fault 可以尝试触发复位
    // 下一次调用 CheckError1KHz 时会再次检查 driver fault
    // if(HAL_GPIO_ReadPin(nFAULT_GPIO_Port, nFAULT_Pin) == GPIO_PIN_RESET)
    // {
    //     if(Drivers::STSPIN32G4MosfetDriver::clearFault())
    //     {
    //         errorStatus.STSPIN32G4MosfetDriverErrorStatus.I2C_CommunicationError = 1;
    //     }
    //     return 1;
    // }

    
    return 0;
}


// 以 1KHz 频率调用
uint8_t ErrorHandler::checkError1KHz(const Sensor::ADC::AnalogValues& analogValues)
{
    uint8_t shouldDisableMotor = 0;

    if(errorHandlerConfig.ignoreAllErrors)
        return 0;

    if(analogValues.Vbus < errorHandlerConfig.underVoltageThreshold)
    {
        if(errorCounter.underVoltageCounter < errorHandlerConfig.underVoltageTriggerTimeout)
        {
            errorCounter.underVoltageCounter ++;
        }
        else
        {
            shouldDisableMotor = 1;
            errorStatus.underVoltage = 1;
        }
    }
    else if(analogValues.Vbus > errorHandlerConfig.overVoltageThreshold)
    {
        if(errorCounter.overVoltageCounter < errorHandlerConfig.overVoltageTriggerTimeout)
        {
            errorCounter.overVoltageCounter ++;
        }
        else
        {
            shouldDisableMotor = 1;
            errorStatus.overVoltage = 1;
        }
    }
    else
    {
        if(errorCounter.underVoltageCounter)
            errorCounter.underVoltageCounter --;
        if(errorCounter.overVoltageCounter)
            errorCounter.overVoltageCounter --;
    }

    // 三相电流和不为 0
    if(FABS(analogValues.measuredIphaseSum) > errorHandlerConfig.overCurrentThreshold / 5.0f)
    {
        if(errorCounter.currnentSensorErrorCounter < errorHandlerConfig.overCurrentTriggerTimeout)
        {
            errorCounter.currnentSensorErrorCounter ++;
        }
        else
        {
            shouldDisableMotor = 1;
            errorStatus.ADCDecoderError = 1;
        }
    }
    else
    {
        if(errorCounter.currnentSensorErrorCounter)
            errorCounter.currnentSensorErrorCounter --;
    }


    // 不需要累加的错误
    if(analogValues.NTCTemperature < errorHandlerConfig.underTemperatureThreshold)
    {
        shouldDisableMotor = 1;
        errorStatus.underTemperature = 1;
    }
    else if(analogValues.NTCTemperature > errorHandlerConfig.overTemperatureThreshold)
    {
        shouldDisableMotor = 1;
        errorStatus.overTemperature = 1;
    }

    return shouldDisableMotor;
}

uint8_t ErrorHandler::checkErrorHighFreq(const Sensor::ADC::AnalogValues& analogValues)
{
    if(errorHandlerConfig.ignoreAllErrors)
        return 0;

    if(analogValues.measuredIA > errorHandlerConfig.overCurrentThreshold || 
        analogValues.measuredIB > errorHandlerConfig.overCurrentThreshold || 
        analogValues.measuredIC > errorHandlerConfig.overCurrentThreshold ||
        analogValues.measuredIA < -errorHandlerConfig.overCurrentThreshold ||
        analogValues.measuredIB < -errorHandlerConfig.overCurrentThreshold ||
        analogValues.measuredIC < -errorHandlerConfig.overCurrentThreshold
    )
    {
        if(errorCounter.overCurrentCounter < errorHandlerConfig.overCurrentTriggerTimeout)
            errorCounter.overCurrentCounter ++;
        else
        {
            errorStatus.overCurrent = 1;
            return 1;
        }
    }
    else if(errorCounter.overCurrentCounter)
    {
        errorCounter.overCurrentCounter --;
    }

    return 0;
}

uint8_t ErrorHandler::checkIfCanAutoRecovery(const Sensor::ADC::AnalogValues& analogValues, uint8_t motorStopped, uint8_t triggerReset)
{
    // 只有当有错误且不再处于错误状态时才进行 auto Recovery
    if (motorStopped                                      == 1  && 
        errorHandlerConfig.enableAutoRecovery               == 1  && 
        checkIfAnyErrorStatus()                             == 1  &&
        checkIfStillInError(analogValues)                   == 0  &&
        triggerReset                                        == 0)
    {
        // 进行 auto Recovery
        // 这里只 set 变量而不进行真正的 Trigger reset 是希望提供另一条路径, 通过 CAN/Ozone 触发 reset.
        errorCounter.noErrorCounter ++;
        if(errorCounter.noErrorCounter > errorHandlerConfig.autoRecoveryTimeout)
        {
            errorCounter.noErrorCounter = 0;
            return 1;
        }
    }

    return 0;
}

void ErrorHandler::clearAllError()
{
    errorStatus.underVoltage       = 0;
    errorStatus.overVoltage        = 0;
    errorStatus.overCurrent        = 0;
    errorStatus.ADCDecoderError    = 0;
    errorStatus.overTemperature    = 0;
    errorStatus.underTemperature   = 0;
    errorStatus.encoderError       = 0;
    errorStatus.motorDisconnected  = 0;
}

} // namespace Control::ErrorHandler
