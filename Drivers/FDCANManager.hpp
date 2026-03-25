/**
 * @file FDCANManager.hpp
 * @brief FDCAN Manager Interface and definitions.
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
#include "fdcan.h"

namespace Drivers
{
namespace FDCANManager
{
/**
 * @brief Initializes the FDCAN Manager.
 * 
 * @param hfdcan_ Pointer to the FDCAN handle struct ure.
 */
void init(FDCAN_HandleTypeDef* hfdcan_, uint32_t filterMask, uint32_t filterId);

/**
 * @brief Transmits data over FDCAN.
 * 
 * @param CANId The CAN message identifier.
 * @param buffer Pointer to the data buffer to be transmitted.
 */
void transmit(uint32_t CANId, uint8_t* buffer);

/**
 * @brief Registers a callback function for FDCAN message reception.
 * 
 * @param callback Pointer to the callback function that will be invoked upon message reception.
 * @param CANId The CAN message identifier.
 * @param rxBuffer Pointer to the receive buffer.
 */
void registerCallback(void (*callback)(uint32_t CANId, uint8_t* rxBuffer));

} // namespace FDCANManager
} // namespace Drivers