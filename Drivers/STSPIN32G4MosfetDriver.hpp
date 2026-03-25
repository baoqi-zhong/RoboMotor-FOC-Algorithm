/**
 * @file STSPIN32G4MosfetDriver.hpp
 * @brief Register definitions for STSPIN32G4 driver.
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
#include "i2c.h"

namespace Drivers
{
namespace STSPIN32G4MosfetDriver
{
#define STSPIN32G4_MOS_DRIVER_I2C               hi2c3

#define STSPIN32G4_MOS_DRIVER_DEVICE_ADDRESS    0x8E

#define STSPIN32G4_MOS_DRIVER_REGISTER_POWMNG   0x01
#define STSPIN32G4_MOS_DRIVER_REGISTER_LOGIC    0x02
#define STSPIN32G4_MOS_DRIVER_REGISTER_READY    0x03
#define STSPIN32G4_MOS_DRIVER_REGISTER_NFAULT   0x08
#define STSPIN32G4_MOS_DRIVER_REGISTER_CLEAR    0x09
#define STSPIN32G4_MOS_DRIVER_REGISTER_STBY     0x0A
#define STSPIN32G4_MOS_DRIVER_REGISTER_LOCK     0x0B
#define STSPIN32G4_MOS_DRIVER_REGISTER_RESET    0x0C
#define STSPIN32G4_MOS_DRIVER_REGISTER_STATUS   0x80

struct ErrorStatus
{
    uint8_t I2C_CommunicationError;
    uint8_t VDSProtectionTriggered;
    uint8_t overTemperature;
    uint8_t underVoltage;
};

uint8_t readStatus(uint8_t *status);

uint8_t init();

uint8_t clearFault();

} // namespace STSPIN32G4MosfetDriver
} // namespace Drivers