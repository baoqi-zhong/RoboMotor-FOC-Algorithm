/**
 * @file MA732.hpp
 * @brief STM32 MA732 magnetic encoder driver declarations.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#pragma once

#include "../Generic/MA732.hpp"

#if (PLATFORM_ST)
#include "main.h"
#include "stm32g4xx_hal_spi.h"

#ifndef MA732_CS_GPIO_Port
#define MA732_CS_GPIO_Port ((GPIO_TypeDef*)0)
#endif

#ifndef MA732_CS_Pin
#define MA732_CS_Pin 0U
#endif

namespace Sensor
{
namespace Encoder
{
namespace MA732
{
void init(SPI_HandleTypeDef* hspi_);

} // namespace MA732
} // namespace Encoder
} // namespace Sensor

#endif // PLATFORM_ST
