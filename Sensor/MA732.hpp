/**
 * @file MA732.hpp
 * @brief MA732 magnetic encoder driver interface.
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
#include "spi.h"
#include "stdint.h"

namespace Sensor
{
namespace Encoder
{
namespace MA732
{
#define READ_CMD      0x40
#define WRITE_CMD     0x80


#define ROTATION_DIRECTION_CW   0x00
#define ROTATION_DIRECTION_CCW  0x80

#define FILTER_CUTOFF_FREQ_6000 51
#define FILTER_CUTOFF_FREQ_3000 68
#define FILTER_CUTOFF_FREQ_1500 85
#define FILTER_CUTOFF_FREQ_740  102
#define FILTER_CUTOFF_FREQ_370  119
#define FILTER_CUTOFF_FREQ_185  136
#define FILTER_CUTOFF_FREQ_93   153
#define FILTER_CUTOFF_FREQ_46   170
#define FILTER_CUTOFF_FREQ_23   187


/**
 * @brief Writes the zero offset to the MA732 sensor.
 */
void writeZeroOffset(uint16_t offset);

/**
 * @brief Reads the zero offset from the MA732 sensor.
 * 
 * @return The zero offset value.
 */
uint16_t readZeroOffset();

/**
 * @brief Sets the hardware zero position for the MA732 sensor.
 */
void setZeroHardware();

/**
 * @brief Initializes the MA732 sensor.
 * 
 * @param hspi Pointer to the SPI handle.
 */
void init(SPI_HandleTypeDef *hspi_);

/**
 * @brief Reads data from the MA732 sensor in a blocking manner.
 */
uint16_t readBlocking();

} // namespace MA732
} // namespace Encoder
} // namespace Sensor