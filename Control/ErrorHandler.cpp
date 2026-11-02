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

ErrorHandler::ErrorHandler(const ErrorHandlerConfig& errorHandlerConfig_) 
    : errorHandlerConfig(errorHandlerConfig_)
{
    overUnderVoltageDebouncer.init(errorHandlerConfig.overUnderVoltageTimeout, errorHandlerConfig.overUnderVoltageTimeout);
    overCurrentDebouncer.init(errorHandlerConfig.overCurrentTimeout, errorHandlerConfig.overCurrentTimeout);
    currentSensorErrorDebouncer.init(errorHandlerConfig.overCurrentTimeout << 4, errorHandlerConfig.overCurrentTimeout << 4);
    clearError();
}

// 以 1KHz 频率调用
bool ErrorHandler::checkError1KHz(const Sensor::ADC::AnalogValues& analogValues,
                                     HAL::MotorHardware& motorHardware)
{
    bool anyErrorOccur = false;

    if(errorHandlerConfig.ignoreAllErrors)
        return false;

    if (overUnderVoltageDebouncer(
        analogValues.Vbus < errorHandlerConfig.underVoltageThreshold || 
        analogValues.Vbus > errorHandlerConfig.overVoltageThreshold
    ))
    {
        anyErrorOccur = true;
        errorStatus.overUnderVoltage = 1;
    }
    
    /* 三相电流和不为 0 */
    if (currentSensorErrorDebouncer(
        FABS(analogValues.measuredISum) > errorHandlerConfig.overCurrentThreshold / 5.0f
    ))
    {
        anyErrorOccur = true;
        errorStatus.currentSensorError = 1;
    }

    if(!motorHardware.isGateDriverReady())
    {
        anyErrorOccur = true;
        errorStatus.gateDriverError = 1;
    }

    return anyErrorOccur;
}

bool ErrorHandler::checkErrorHighFreq(const Sensor::ADC::AnalogValues& analogValues)
{
    if(errorHandlerConfig.ignoreAllErrors)
        return false;
    
    if (analogValues.Vbus < 1.0f)
    {
        errorStatus.overUnderVoltage = 1;
        return true;
    }

    if(overCurrentDebouncer(
        analogValues.measuredIA > errorHandlerConfig.overCurrentThreshold || 
        analogValues.measuredIB > errorHandlerConfig.overCurrentThreshold || 
        analogValues.measuredIC > errorHandlerConfig.overCurrentThreshold ||
        analogValues.measuredIA < -errorHandlerConfig.overCurrentThreshold ||
        analogValues.measuredIB < -errorHandlerConfig.overCurrentThreshold ||
        analogValues.measuredIC < -errorHandlerConfig.overCurrentThreshold
    ))
    {
        errorStatus.overCurrent = 1;
        return true;
    }

    return false;
}

void ErrorHandler::clearError()
{
    lastErrorStatus.overUnderVoltage        += errorStatus.overUnderVoltage;
    lastErrorStatus.overCurrent             += errorStatus.overCurrent;
    lastErrorStatus.currentSensorError      += errorStatus.currentSensorError;
    lastErrorStatus.adcCalibrationTimeout   += errorStatus.adcCalibrationTimeout;
    lastErrorStatus.gateDriverInitTimeout   += errorStatus.gateDriverInitTimeout;
    lastErrorStatus.gateDriverError         += errorStatus.gateDriverError;

    errorStatus.overUnderVoltage            = 0;
    errorStatus.overCurrent                 = 0;
    errorStatus.currentSensorError          = 0;
    errorStatus.adcCalibrationTimeout       = 0;
    errorStatus.gateDriverInitTimeout       = 0;
    errorStatus.gateDriverError             = 0;

    overUnderVoltageDebouncer.reset();
    currentSensorErrorDebouncer.reset();
}

} // namespace Control::ErrorHandler
