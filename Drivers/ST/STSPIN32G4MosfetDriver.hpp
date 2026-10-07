/**
 * @file STSPIN32G4MosfetDriver.hpp
 * @brief STM32 STSPIN32G4 MOSFET driver declarations.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#pragma once

#include "../Generic/STSPIN32G4MosfetDriver.hpp"

#if (PLATFORM_ST)
#include "i2c.h"
#include "main.h"

#ifndef STSPIN32G4_MOS_DRIVER_I2C
#define STSPIN32G4_MOS_DRIVER_I2C hi2c1
#endif

#ifndef STSPIN32G4_MOS_DRIVER_ENABLE_GPIO_Port
#define STSPIN32G4_MOS_DRIVER_ENABLE_GPIO_Port PS_EN_GPIO_Port
#endif

#ifndef STSPIN32G4_MOS_DRIVER_ENABLE_Pin
#define STSPIN32G4_MOS_DRIVER_ENABLE_Pin PS_EN_Pin
#endif

#define STSPIN32G4_MOS_DRIVER_DEVICE_ADDRESS Drivers::STSPIN32G4MosfetDriver::DEVICE_ADDRESS
#define STSPIN32G4_MOS_DRIVER_REGISTER_POWMNG Drivers::STSPIN32G4MosfetDriver::REGISTER_POWMNG
#define STSPIN32G4_MOS_DRIVER_REGISTER_LOGIC Drivers::STSPIN32G4MosfetDriver::REGISTER_LOGIC
#define STSPIN32G4_MOS_DRIVER_REGISTER_READY Drivers::STSPIN32G4MosfetDriver::REGISTER_READY
#define STSPIN32G4_MOS_DRIVER_REGISTER_NFAULT Drivers::STSPIN32G4MosfetDriver::REGISTER_NFAULT
#define STSPIN32G4_MOS_DRIVER_REGISTER_CLEAR Drivers::STSPIN32G4MosfetDriver::REGISTER_CLEAR
#define STSPIN32G4_MOS_DRIVER_REGISTER_STBY Drivers::STSPIN32G4MosfetDriver::REGISTER_STBY
#define STSPIN32G4_MOS_DRIVER_REGISTER_LOCK Drivers::STSPIN32G4MosfetDriver::REGISTER_LOCK
#define STSPIN32G4_MOS_DRIVER_REGISTER_RESET Drivers::STSPIN32G4MosfetDriver::REGISTER_RESET
#define STSPIN32G4_MOS_DRIVER_REGISTER_STATUS Drivers::STSPIN32G4MosfetDriver::REGISTER_STATUS

#endif // PLATFORM_ST
