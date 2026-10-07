/**
 * @file CANManager.hpp
 * @brief CAN Manager Interface and definitions.
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
namespace CANManager
{
using RxCallback = void (*)(uint32_t CANId, const uint8_t* rxBuffer);
inline RxCallback rxCallback = nullptr;

/**
 * @brief Initializes the CAN manager with the specified parameters.
 */
void init(uint32_t filterMask, uint32_t filterId);

/**
 * @brief Transmits data over CAN.
 * 
 * @param CANId The CAN message identifier.
 * @param buffer Pointer to the data buffer to be transmitted. 8 Bytes.
 */
void transmit(uint32_t CANId, const uint8_t* buffer);

/**
 * @brief Registers a callback function for CAN message reception.
 * 
 * @param callback Pointer to the callback function that will be invoked upon message reception.
 * @param CANId The CAN message identifier.
 * @param rxBuffer Pointer to the receive buffer. 8 Bytes.
 */
void registerCallback(RxCallback callback) {rxCallback = callback;}

} // namespace CANManager
} // namespace Drivers
