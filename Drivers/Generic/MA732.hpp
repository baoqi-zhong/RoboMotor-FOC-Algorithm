/**
 * @file MA732.hpp
 * @brief Generic MA732 magnetic encoder driver interface.
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

namespace Sensor
{
namespace Encoder
{
namespace MA732
{
constexpr uint8_t READ_CMD = 0x40;
constexpr uint8_t WRITE_CMD = 0x80;

constexpr uint8_t ROTATION_DIRECTION_CW = 0x00;
constexpr uint8_t ROTATION_DIRECTION_CCW = 0x80;

constexpr uint8_t FILTER_CUTOFF_FREQ_6000 = 51;
constexpr uint8_t FILTER_CUTOFF_FREQ_3000 = 68;
constexpr uint8_t FILTER_CUTOFF_FREQ_1500 = 85;
constexpr uint8_t FILTER_CUTOFF_FREQ_740 = 102;
constexpr uint8_t FILTER_CUTOFF_FREQ_370 = 119;
constexpr uint8_t FILTER_CUTOFF_FREQ_185 = 136;
constexpr uint8_t FILTER_CUTOFF_FREQ_93 = 153;
constexpr uint8_t FILTER_CUTOFF_FREQ_46 = 170;
constexpr uint8_t FILTER_CUTOFF_FREQ_23 = 187;

void init(uint32_t spiIndex);
void writeZeroOffset(uint16_t offset);
uint16_t readZeroOffset();
void setZeroHardware();
uint16_t readBlocking();

} // namespace MA732
} // namespace Encoder
} // namespace Sensor
