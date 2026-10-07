/**
 * @file AT24Cxx.hpp
 * @brief Generic AT24Cxx EEPROM driver interface.
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
namespace AT24Cxx
{
constexpr uint16_t DEVICE_ADDRESS = 0xA0;

uint8_t read(void* i2cHandle, uint16_t addr, uint8_t* data, uint16_t len);
uint8_t write(void* i2cHandle, uint16_t addr, uint8_t* data, uint16_t len);

} // namespace AT24Cxx
} // namespace Drivers
