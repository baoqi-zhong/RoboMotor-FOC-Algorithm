/**
 * @file ConfigLoader.hpp
 * @brief Flash configuration loader interface and error definitions.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#pragma once

#include "main.h"
#include "stdint.h"

#include "ADC.hpp"
#include "Encoder.hpp"
#include "PositionalPID.hpp"
#include "ThreePhaseFOC.hpp"
#include "MotorControl.hpp"
#include "ErrorHandler.hpp"
#include "InterBoard.hpp"


namespace Control
{
namespace ConfigLoader
{
enum ConfigLoaderError : uint8_t
{
    NoError = 0,
    OperationStarted,
    FlashReadError,
    FlashWriteError,
    FlashEraseError,
    FlashBusy,
    FlashTimeout,
    ChecksumError,
    ConfigBufferOverflow,
    InvalidAddress,
};

enum PIDType : uint8_t
{
    IqPID = 0,
    IdPID,
    PositionToCurrentPID,
    PositionToVelocityPID,
    VelocityPID,
};

enum class ConfigLoaderState : uint8_t
{
    IDLE,
    PREPARE_BUFFER,
    START_ERASE,
    WAIT_ERASE,
    START_WRITE,
    WAIT_WRITE,
    COMPLETE
};

void init();
void update();

ConfigLoaderError loadAllConfigFromFlash();
ConfigLoaderError saveAllConfigToFlashAsync();

ConfigLoaderError loadADCConfigFromFlash();
ConfigLoaderError loadADCCalibrationDataFromFlash();
ConfigLoaderError loadEncoderConfigFromFlash();
ConfigLoaderError loadMotorConfigFromFlash();
ConfigLoaderError loadFOCConfigFromFlash();
ConfigLoaderError loadMotorControlFromFlash();
ConfigLoaderError loadErrorHandlerConfigFromFlash();
ConfigLoaderError loadPIDConfigFromFlash(PIDType pidType);

ConfigLoaderError waitForLastOperation(uint32_t timeout);

} // namespace ConfigLoader
} // namespace Control