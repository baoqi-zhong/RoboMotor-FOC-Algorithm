/**
 * @file ConfigLoader.cpp
 * @brief Flash configuration loading and saving implementation.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "ConfigLoader.hpp"
#include "FlashManager.hpp"
#include "crc.h"
#include <cstring>

namespace Control
{
namespace ConfigLoader
{
/**
 * Flash 定义: 最后一个 Bank (2KB). 所有结构 8 字节对齐. 最后两个 Byte 为校验. 自动计算 address.
 */

constexpr uint16_t FlashConfigPageSize            = 2048;
constexpr uint16_t FlashChecksumSize              = 4;
constexpr uint16_t FlashConfigPayloadMaxSize      = FlashConfigPageSize - FlashChecksumSize;
constexpr uint16_t FlashAddressAlignment          = 8;
constexpr uint8_t FlashConfigPage                 = 63;

constexpr uint16_t alignUp8(uint16_t value)
{
    return (uint16_t)((value + (FlashAddressAlignment - 1)) & ~(FlashAddressAlignment - 1));
}

constexpr uint16_t alignedSizeOf(uint16_t rawSize)
{
    return alignUp8(rawSize);
}

constexpr uint16_t ADCConfigMaxSize               = alignedSizeOf((uint16_t)(sizeof(Sensor::ADC::ADCConfig)));
constexpr uint16_t ADCCalibrationDataMaxSize      = alignedSizeOf((uint16_t)(sizeof(Sensor::ADC::ADCCalibrationData)));
constexpr uint16_t EncoderConfigMaxSize           = alignedSizeOf((uint16_t)(sizeof(Sensor::Encoder::EncoderConfig)));
constexpr uint16_t MotorConfigMaxSize             = alignedSizeOf((uint16_t)(sizeof(Control::FOC::MotorConfig)));
constexpr uint16_t encoderCompensationTableSize   = 128;
constexpr uint16_t FOCConfigMaxSize               = alignedSizeOf((uint16_t)(sizeof(Control::FOC::FOCConfig)));
constexpr uint16_t MotorControlConfigMaxSize      = alignedSizeOf((uint16_t)(sizeof(Control::MotorControl::MotorControlConfig)));
constexpr uint16_t ErrorHandlerConfigMaxSize      = alignedSizeOf((uint16_t)(sizeof(Control::ErrorHandler::ErrorHandlerConfig)));
constexpr uint16_t PositionToCurrentPIDParamSize  = alignedSizeOf((uint16_t)(sizeof(Control::PIDParameters_t)));
constexpr uint16_t PositionToVelocityPIDParamSize = alignedSizeOf((uint16_t)(sizeof(Control::PIDParameters_t)));
constexpr uint16_t VelocityPIDParamSize           = alignedSizeOf((uint16_t)(sizeof(Control::PIDParameters_t)));
constexpr uint16_t IqPIDParamSize                 = alignedSizeOf((uint16_t)(sizeof(Control::PIDParameters_t)));
constexpr uint16_t IdPIDParamSize                 = alignedSizeOf((uint16_t)(sizeof(Control::PIDParameters_t)));

constexpr uint16_t ADCConfigAddr                  = 0x00;
constexpr uint16_t ADCCalibrationDataAddr         = alignUp8(ADCConfigAddr + ADCConfigMaxSize);
constexpr uint16_t EncoderConfigAddr              = alignUp8(ADCCalibrationDataAddr + ADCCalibrationDataMaxSize);
constexpr uint16_t MotorConfigAddr                = alignUp8(EncoderConfigAddr + EncoderConfigMaxSize);
constexpr uint16_t encoderCompensationTableAddr   = alignUp8(MotorConfigAddr + MotorConfigMaxSize);
constexpr uint16_t FOCConfigAddr                  = alignUp8(encoderCompensationTableAddr + encoderCompensationTableSize);
constexpr uint16_t MotorControlConfigAddr         = alignUp8(FOCConfigAddr + FOCConfigMaxSize);
constexpr uint16_t ErrorHandlerConfigAddr         = alignUp8(MotorControlConfigAddr + MotorControlConfigMaxSize);
constexpr uint16_t PositionToCurrentPIDParamAddr  = alignUp8(ErrorHandlerConfigAddr + ErrorHandlerConfigMaxSize);
constexpr uint16_t PositionToVelocityPIDParamAddr = alignUp8(PositionToCurrentPIDParamAddr + PositionToCurrentPIDParamSize);
constexpr uint16_t VelocityPIDParamAddr           = alignUp8(PositionToVelocityPIDParamAddr + PositionToVelocityPIDParamSize);
constexpr uint16_t IqPIDParamAddr                 = alignUp8(VelocityPIDParamAddr + VelocityPIDParamSize);
constexpr uint16_t IdPIDParamAddr                 = alignUp8(IqPIDParamAddr + IqPIDParamSize);
constexpr uint16_t ConfigTotalSize                = alignUp8(IdPIDParamAddr + IdPIDParamSize);

// static_assert((ADCConfigAddr % FlashAddressAlignment) == 0);
// static_assert((ADCCalibrationDataAddr % FlashAddressAlignment) == 0);
// static_assert((EncoderConfigAddr % FlashAddressAlignment) == 0);
// static_assert((MotorConfigAddr % FlashAddressAlignment) == 0);
// static_assert((encoderCompensationTableAddr % FlashAddressAlignment) == 0);
// static_assert((FOCConfigAddr % FlashAddressAlignment) == 0);
// static_assert((MotorControlConfigAddr % FlashAddressAlignment) == 0);
// static_assert((ErrorHandlerConfigAddr % FlashAddressAlignment) == 0);
// static_assert((PositionToCurrentPIDParamAddr % FlashAddressAlignment) == 0);
// static_assert((PositionToVelocityPIDParamAddr % FlashAddressAlignment) == 0);
// static_assert((VelocityPIDParamAddr % FlashAddressAlignment) == 0);
// static_assert(ConfigTotalSize <= FlashConfigPayloadMaxSize);
// static_assert(ADCConfigMaxSize                  >= sizeof(Sensor::ADC::ADCConfig));
// static_assert(ADCCalibrationDataMaxSize         >= sizeof(Sensor::ADC::ADCCalibrationData));
// static_assert(EncoderConfigMaxSize              >= sizeof(Sensor::Encoder::EncoderConfig));
// static_assert(MotorConfigMaxSize                >= sizeof(Control::FOC::MotorConfig));
// static_assert(FOCConfigMaxSize                  >= sizeof(Control::FOC::FOCConfig));
// static_assert(MotorControlConfigMaxSize         >= sizeof(Control::MotorControl::MotorControlConfig));
// static_assert(ErrorHandlerConfigMaxSize         >= sizeof(Control::ErrorHandler::ErrorHandlerConfig));
// static_assert(PositionToCurrentPIDParamSize     >= sizeof(Control::PIDParameters_t));
// static_assert(PositionToVelocityPIDParamSize    >= sizeof(Control::PIDParameters_t));
// static_assert(VelocityPIDParamSize              >= sizeof(Control::PIDParameters_t));


alignas(8) uint8_t flashOperationBuffer[2048] = {0};

ConfigLoaderError readPageToBuffer()
{
    uint32_t baseAddress = 0;
    if(Drivers::FlashManager::getAddrFromPage(FlashConfigPage, &baseAddress) != Drivers::FlashManager::FlashErrorState::FLASH_SUCCESS)
    {
        return ConfigLoaderError::InvalidAddress;
    }

    if(Drivers::FlashManager::flashRead(baseAddress, flashOperationBuffer, FlashConfigPageSize) != Drivers::FlashManager::FlashErrorState::FLASH_SUCCESS)
    {
        return ConfigLoaderError::FlashReadError;
    }

    return ConfigLoaderError::NoError;
}

// Callback declarations
void onEraseComplete(Drivers::FlashManager::FlashErrorState state);
void onWriteComplete(Drivers::FlashManager::FlashErrorState state);

ConfigLoaderError writeBufferBackToFlashAsync()
{
    uint32_t baseAddress = 0;
    if(Drivers::FlashManager::getAddrFromPage(FlashConfigPage, &baseAddress) != Drivers::FlashManager::FlashErrorState::FLASH_SUCCESS)
    {
        return ConfigLoaderError::InvalidAddress;
    }

    const uint32_t calculatedCRC = HAL_CRC_Calculate(&hcrc, (uint32_t*)flashOperationBuffer, FlashConfigPayloadMaxSize / 4);
    *(uint32_t*)&flashOperationBuffer[FlashConfigPayloadMaxSize] = calculatedCRC;

    // Start with Erase
    Drivers::FlashManager::FlashErrorState eraseState = Drivers::FlashManager::erasePageAsync(FlashConfigPage, onEraseComplete);
    
    if(eraseState == Drivers::FlashManager::FlashErrorState::FLASH_BUSY)
    {
        return ConfigLoaderError::FlashBusy;
    }
    
    if(eraseState != Drivers::FlashManager::FlashErrorState::FLASH_SUCCESS)
    {
        return ConfigLoaderError::FlashEraseError;
    }

    return ConfigLoaderError::OperationStarted;
}

void onEraseComplete(Drivers::FlashManager::FlashErrorState state)
{
    if(state != Drivers::FlashManager::FlashErrorState::FLASH_SUCCESS)
    {
        // Erase failed
        // Handle error?
        return;
    }

    // Proceed to Write
    uint32_t baseAddress = 0;
    Drivers::FlashManager::getAddrFromPage(FlashConfigPage, &baseAddress);
    
    Drivers::FlashManager::flashWriteAsync(flashOperationBuffer, baseAddress, FlashConfigPageSize, onWriteComplete);
}

void onWriteComplete(Drivers::FlashManager::FlashErrorState state)
{
    if(state != Drivers::FlashManager::FlashErrorState::FLASH_SUCCESS)
    {
        // Write failed
        return;
    }
    // Success
}

ConfigLoaderError loadConfigToBufferFromFlash()
{
    ConfigLoaderError err = readPageToBuffer();
    if(err)
    {
        return err;
    }

    // check checksum
    const uint32_t calculatedCRC = HAL_CRC_Calculate(&hcrc, (uint32_t*)flashOperationBuffer, FlashConfigPayloadMaxSize / 4);
    
    // Checksum stored at the end of the payload
    const uint32_t storedCRC = *(uint32_t*)&flashOperationBuffer[FlashConfigPayloadMaxSize];

    if(calculatedCRC == storedCRC)
    {
        return ConfigLoaderError::NoError;
    }
    return ConfigLoaderError::ChecksumError;
}

void unpackConfigFromBuffer(uint16_t offset, void* destination, uint16_t destinationSize)
{
    if(destination == nullptr)
    {
        return;
    }

    for(uint16_t i = 0; i < destinationSize; i++)
    {
        ((uint8_t*)destination)[i] = flashOperationBuffer[offset + i];
    }
}

void packConfigToBuffer(uint16_t offset, const void* source, uint16_t sourceSize)
{
    if(source == nullptr)
    {
        return;
    }

    for(uint16_t i = 0; i < sourceSize; i++)
    {
        flashOperationBuffer[offset + i] = ((const uint8_t*)source)[i];
    }
}


void init()
{
    Drivers::FlashManager::init();
    for(uint16_t i = 0; i < FlashConfigPageSize; i++)
    {
        flashOperationBuffer[i] = 0xFF;
    }
}

ConfigLoaderState currentState = ConfigLoaderState::IDLE;
uint8_t flashOpDone = false; 
Drivers::FlashManager::FlashErrorState flashOpResult;


// Callback
void onFlashCallback(Drivers::FlashManager::FlashErrorState state)
{
    flashOpResult = state;
    flashOpDone = true;
}

/**
 * 状态机: 专门为写入 config 设计的状态机. 
 * 由外部调用 saveAllConfigToFlashAsync() 触发, 将当前配置打包到 buffer, 擦除 Flash, 写入 Flash, 等待完成.
 */
void update()
{
    // 1. Update Drivers
    Drivers::FlashManager::update();

    // 2. State Machine
    switch (currentState)
    {
    case ConfigLoaderState::IDLE:
        break;

    case ConfigLoaderState::PREPARE_BUFFER:
        {
            // Pack data
            const Control::PIDParameters_t positionToCurrentPIDParam = Control::MotorControl::positionToCurrentPID.getParameters();
            const Control::PIDParameters_t positionToVelocityPIDParam = Control::MotorControl::positionToVelocityPID.getParameters();
            const Control::PIDParameters_t velocityPIDParam = Control::MotorControl::velocityPID.getParameters();
            const Control::PIDParameters_t iqPIDParam = Control::FOC::IqPID.getParameters();
            const Control::PIDParameters_t idPIDParam = Control::FOC::IdPID.getParameters();

            packConfigToBuffer(ADCConfigAddr, &Sensor::ADC::adcConfig, sizeof(Sensor::ADC::ADCConfig));
            packConfigToBuffer(ADCCalibrationDataAddr, &Sensor::ADC::adcCalibrationData, sizeof(Sensor::ADC::ADCCalibrationData));
            packConfigToBuffer(EncoderConfigAddr, &Sensor::Encoder::encoderConfig, sizeof(Sensor::Encoder::EncoderConfig));
            packConfigToBuffer(MotorConfigAddr, &Control::FOC::motorConfig, sizeof(Control::FOC::MotorConfig));
            packConfigToBuffer(FOCConfigAddr, &Control::FOC::focConfig, sizeof(Control::FOC::FOCConfig));
            packConfigToBuffer(MotorControlConfigAddr, &Control::MotorControl::motorControlConfig, sizeof(Control::MotorControl::MotorControlConfig));
            packConfigToBuffer(ErrorHandlerConfigAddr, &Control::ErrorHandler::errorHandlerConfig, sizeof(Control::ErrorHandler::ErrorHandlerConfig));
            packConfigToBuffer(PositionToCurrentPIDParamAddr, &positionToCurrentPIDParam, sizeof(Control::PIDParameters_t));
            packConfigToBuffer(PositionToVelocityPIDParamAddr, &positionToVelocityPIDParam, sizeof(Control::PIDParameters_t));
            packConfigToBuffer(VelocityPIDParamAddr, &velocityPIDParam, sizeof(Control::PIDParameters_t));
            packConfigToBuffer(IqPIDParamAddr, &iqPIDParam, sizeof(Control::PIDParameters_t));
            packConfigToBuffer(IdPIDParamAddr, &idPIDParam, sizeof(Control::PIDParameters_t));
            
            // CRC
            const uint32_t calculatedCRC = HAL_CRC_Calculate(&hcrc, (uint32_t*)flashOperationBuffer, FlashConfigPayloadMaxSize / 4);
            *(uint32_t*)&flashOperationBuffer[FlashConfigPayloadMaxSize] = calculatedCRC;

            currentState = ConfigLoaderState::START_ERASE;
        }
        break;

    case ConfigLoaderState::START_ERASE:
        flashOpDone = false;
        if (Drivers::FlashManager::erasePageAsync(FlashConfigPage, onFlashCallback) == Drivers::FlashManager::FlashErrorState::FLASH_SUCCESS)
        {
            currentState = ConfigLoaderState::WAIT_ERASE;
        }
        else
        {
            currentState = ConfigLoaderState::IDLE; // Error trigger?
        }
        break;

    case ConfigLoaderState::WAIT_ERASE:
        if (flashOpDone)
        {
            if (flashOpResult == Drivers::FlashManager::FlashErrorState::FLASH_SUCCESS)
            {
                currentState = ConfigLoaderState::START_WRITE;
            }
            else
            {
                currentState = ConfigLoaderState::IDLE; // Failed
            }
        }
        break;
    
    case ConfigLoaderState::START_WRITE:
        flashOpDone = false;
        {
            uint32_t baseAddress = 0;
            Drivers::FlashManager::getAddrFromPage(FlashConfigPage, &baseAddress);
            if (Drivers::FlashManager::flashWriteAsync(flashOperationBuffer, baseAddress, FlashConfigPageSize, onFlashCallback) == Drivers::FlashManager::FlashErrorState::FLASH_SUCCESS)
            {
                currentState = ConfigLoaderState::WAIT_WRITE;
            }
            else
            {
                currentState = ConfigLoaderState::IDLE;
            }
        }
        break;

    case ConfigLoaderState::WAIT_WRITE:
        if (flashOpDone)
        {
            currentState = ConfigLoaderState::IDLE; // Complete
        }
        break;

    default:
        currentState = ConfigLoaderState::IDLE;
        break;
    }
}

ConfigLoaderError loadAllConfigFromFlash()
{
    ConfigLoaderError err = loadConfigToBufferFromFlash();
    if(err)
    {
        return err;
    }

    Sensor::ADC::ADCConfig adcConfig;
    Sensor::ADC::ADCCalibrationData adcCalibrationData;
    Sensor::Encoder::EncoderConfig encoderConfig;
    Control::FOC::MotorConfig motorConfig;
    Control::FOC::FOCConfig focConfig;
    Control::MotorControl::MotorControlConfig motorControlConfig;
    Control::ErrorHandler::ErrorHandlerConfig errorHandlerConfig;

    unpackConfigFromBuffer(ADCConfigAddr, &adcConfig, sizeof(Sensor::ADC::ADCConfig));
    unpackConfigFromBuffer(ADCCalibrationDataAddr, &adcCalibrationData, sizeof(Sensor::ADC::ADCCalibrationData));
    unpackConfigFromBuffer(EncoderConfigAddr, &encoderConfig, sizeof(Sensor::Encoder::EncoderConfig));
    unpackConfigFromBuffer(MotorConfigAddr, &motorConfig, sizeof(Control::FOC::MotorConfig));
    unpackConfigFromBuffer(FOCConfigAddr, &focConfig, sizeof(Control::FOC::FOCConfig));
    unpackConfigFromBuffer(MotorControlConfigAddr, &motorControlConfig, sizeof(Control::MotorControl::MotorControlConfig));
    unpackConfigFromBuffer(ErrorHandlerConfigAddr, &errorHandlerConfig, sizeof(Control::ErrorHandler::ErrorHandlerConfig));
    Control::PIDParameters_t positionToCurrentPIDParam = {};
    Control::PIDParameters_t positionToVelocityPIDParam = {};
    Control::PIDParameters_t velocityPIDParam = {};
    Control::PIDParameters_t iqPIDParam = {};
    Control::PIDParameters_t idPIDParam = {};

    unpackConfigFromBuffer(PositionToCurrentPIDParamAddr, &positionToCurrentPIDParam, sizeof(Control::PIDParameters_t));
    unpackConfigFromBuffer(PositionToVelocityPIDParamAddr, &positionToVelocityPIDParam, sizeof(Control::PIDParameters_t));
    unpackConfigFromBuffer(VelocityPIDParamAddr, &velocityPIDParam, sizeof(Control::PIDParameters_t));
    unpackConfigFromBuffer(IqPIDParamAddr, &iqPIDParam, sizeof(Control::PIDParameters_t));
    unpackConfigFromBuffer(IdPIDParamAddr, &idPIDParam, sizeof(Control::PIDParameters_t));

    Sensor::ADC::setConfig(&adcConfig, &adcCalibrationData);
    Sensor::Encoder::setConfig(&encoderConfig);
    Control::FOC::setMotorConfig(&motorConfig);
    Control::FOC::setFOCConfig(&focConfig);
    Control::MotorControl::setConfig(&motorControlConfig);
    Control::ErrorHandler::setConfig(&errorHandlerConfig);

    Control::MotorControl::positionToCurrentPID.setParameters(positionToCurrentPIDParam);
    Control::MotorControl::positionToVelocityPID.setParameters(positionToVelocityPIDParam);
    Control::MotorControl::velocityPID.setParameters(velocityPIDParam);
    Control::FOC::IqPID.setParameters(iqPIDParam);
    Control::FOC::IdPID.setParameters(idPIDParam);

    return ConfigLoaderError::NoError;
}

ConfigLoaderError saveAllConfigToFlashAsync()
{
    if(currentState != ConfigLoaderState::IDLE)
        return ConfigLoaderError::FlashBusy;

    currentState = ConfigLoaderState::PREPARE_BUFFER;
    return ConfigLoaderError::OperationStarted;
}

ConfigLoaderError loadADCConfigFromFlash()
{
    ConfigLoaderError err = loadConfigToBufferFromFlash();
    if(err)
    {
        return err;
    }

    Sensor::ADC::ADCConfig adcConfig;
    Sensor::ADC::ADCCalibrationData adcCalibrationData = Sensor::ADC::adcCalibrationData; // Keep existing calib data
    
    unpackConfigFromBuffer(ADCConfigAddr, &adcConfig, sizeof(Sensor::ADC::ADCConfig));
    Sensor::ADC::setConfig(&adcConfig, &adcCalibrationData);
    
    return ConfigLoaderError::NoError;
}

ConfigLoaderError loadADCCalibrationDataFromFlash()
{
    ConfigLoaderError err = loadConfigToBufferFromFlash();
    if(err)
    {
        return err;
    }

    Sensor::ADC::ADCConfig adcConfig = Sensor::ADC::adcConfig; // Keep existing config
    Sensor::ADC::ADCCalibrationData adcCalibrationData;

    unpackConfigFromBuffer(ADCCalibrationDataAddr, &adcCalibrationData, sizeof(Sensor::ADC::ADCCalibrationData));
    Sensor::ADC::setConfig(&adcConfig, &adcCalibrationData);
    
    return ConfigLoaderError::NoError;
}

ConfigLoaderError loadEncoderConfigFromFlash()
{
    ConfigLoaderError err = loadConfigToBufferFromFlash();
    if(err)
    {
        return err;
    }

    Sensor::Encoder::EncoderConfig encoderConfig;
    unpackConfigFromBuffer(EncoderConfigAddr, &encoderConfig, sizeof(Sensor::Encoder::EncoderConfig));
    Sensor::Encoder::setConfig(&encoderConfig);
    
    return ConfigLoaderError::NoError;
}

ConfigLoaderError loadMotorConfigFromFlash()
{
    ConfigLoaderError err = loadConfigToBufferFromFlash();
    if(err)
    {
        return err;
    }

    Control::FOC::MotorConfig motorConfig;
    unpackConfigFromBuffer(MotorConfigAddr, &motorConfig, sizeof(Control::FOC::MotorConfig));
    Control::FOC::setMotorConfig(&motorConfig);
    
    return ConfigLoaderError::NoError;
}

ConfigLoaderError loadFOCConfigFromFlash()
{
    ConfigLoaderError err = loadConfigToBufferFromFlash();
    if(err)
    {
        return err;
    }

    Control::FOC::FOCConfig focConfig;
    unpackConfigFromBuffer(FOCConfigAddr, &focConfig, sizeof(Control::FOC::FOCConfig));
    Control::FOC::setFOCConfig(&focConfig);
    
    return ConfigLoaderError::NoError;
}

ConfigLoaderError loadMotorControlFromFlash()
{
    ConfigLoaderError err = loadConfigToBufferFromFlash();
    if(err)
    {
        return err;
    }

    Control::MotorControl::MotorControlConfig motorControlConfig;
    unpackConfigFromBuffer(MotorControlConfigAddr, &motorControlConfig, sizeof(Control::MotorControl::MotorControlConfig));
    Control::MotorControl::setConfig(&motorControlConfig);
    
    return ConfigLoaderError::NoError;
}

ConfigLoaderError loadErrorHandlerConfigFromFlash()
{
    ConfigLoaderError err = loadConfigToBufferFromFlash();
    if(err)
    {
        return err;
    }

    Control::ErrorHandler::ErrorHandlerConfig errorHandlerConfig;
    unpackConfigFromBuffer(ErrorHandlerConfigAddr, &errorHandlerConfig, sizeof(Control::ErrorHandler::ErrorHandlerConfig));
    Control::ErrorHandler::setConfig(&errorHandlerConfig);
    
    return ConfigLoaderError::NoError;
}

ConfigLoaderError loadPIDConfigFromFlash(PIDType pidType)
{
    ConfigLoaderError err = loadConfigToBufferFromFlash();
    if(err)
    {
        return err;
    }

    Control::PIDParameters_t pidParam = {};

    switch(pidType)
    {
    case PIDType::PositionToCurrentPID:
        unpackConfigFromBuffer(PositionToCurrentPIDParamAddr, &pidParam, sizeof(Control::PIDParameters_t));
        Control::MotorControl::positionToCurrentPID.setParameters(pidParam);
        break;
    case PIDType::PositionToVelocityPID:
        unpackConfigFromBuffer(PositionToVelocityPIDParamAddr, &pidParam, sizeof(Control::PIDParameters_t));
        Control::MotorControl::positionToVelocityPID.setParameters(pidParam);
        break;
    case PIDType::VelocityPID:
        unpackConfigFromBuffer(VelocityPIDParamAddr, &pidParam, sizeof(Control::PIDParameters_t));
        Control::MotorControl::velocityPID.setParameters(pidParam);
        break;
    case PIDType::IqPID:
        unpackConfigFromBuffer(IqPIDParamAddr, &pidParam, sizeof(Control::PIDParameters_t));
        Control::FOC::IqPID.setParameters(pidParam);
        break;
    case PIDType::IdPID:
        unpackConfigFromBuffer(IdPIDParamAddr, &pidParam, sizeof(Control::PIDParameters_t));
        Control::FOC::IdPID.setParameters(pidParam);
        break;
    default:
        break;
    }

    return ConfigLoaderError::NoError;
}


ConfigLoaderError waitForLastOperation(uint32_t timeout)
{
    if(Drivers::FlashManager::waitForLastOperation(timeout) == Drivers::FlashManager::FlashErrorState::FLASH_SUCCESS)
    {
        return ConfigLoaderError::NoError;
    }
    else
    {
        return ConfigLoaderError::FlashTimeout;
    }
}

} // namespace ConfigLoader
} // namespace Control