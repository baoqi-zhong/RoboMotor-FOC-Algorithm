/**
 * @file STSPIN32G4MosfetDriver.hpp
 * @brief Generic STSPIN32G4 MOSFET driver interface.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#pragma once

#include <stdint.h>

namespace Drivers
{
namespace STSPIN32G4MosfetDriver
{
constexpr uint8_t DEVICE_ADDRESS = 0x8E;

constexpr uint8_t REGISTER_POWMNG = 0x01;
constexpr uint8_t REGISTER_LOGIC = 0x02;
constexpr uint8_t REGISTER_READY = 0x03;
constexpr uint8_t REGISTER_NFAULT = 0x08;
constexpr uint8_t REGISTER_CLEAR = 0x09;
constexpr uint8_t REGISTER_STBY = 0x0A;
constexpr uint8_t REGISTER_LOCK = 0x0B;
constexpr uint8_t REGISTER_RESET = 0x0C;
constexpr uint8_t REGISTER_STATUS = 0x80;

struct ErrorStatus
{
    uint8_t I2C_CommunicationError;
    uint8_t VDSProtectionTriggered;
    uint8_t overTemperature;
    uint8_t underVoltage;
};

uint8_t init();
uint8_t readStatus(uint8_t* status);
uint8_t clearFault();

} // namespace STSPIN32G4MosfetDriver
} // namespace Drivers
