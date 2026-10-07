/**
 * @file InternalFlashManager.hpp
 * @brief Flash Manager definitions and interface.
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
namespace InternalFlashManager
{

enum class ErrorState : uint8_t
{
    FLASH_SUCCESS   = 0,
    FLASH_ERROR,
    FLASH_BUSY,
    FLASH_TIMEOUT
};

typedef void (*AsyncCallback)(ErrorState state);

void init(void);

/**
 * @brief Update the flash manager state machine. Call this in the main loop (e.g. 1kHz).
 * 
 */
void update(void);

/**
 * @brief Read data from flash memory.
 *
 * @param src uint32_t value, 32 bit address to read from
 * @param dest pointer to the result buffer
 * @param len number of bytes to read
 * @return uint8_t
 */
ErrorState read(uint32_t src, uint8_t* dest, uint32_t len);

/**
 * @brief Erase a section of flash memory asynchronously. The operation will be performed in the background, and the provided callback function will be called when the operation is complete.
 *
 * @param startAddress the starting address of the flash page to erase
 * @param size the size of the data to erase
 * @param callback the callback function to be called when the operation is complete
 */
ErrorState eraseAsync(uint32_t startAddress, uint32_t size, AsyncCallback callback = nullptr);

/**
 * @brief Write data to flash memory asynchronously. The operation will be performed in the background, and the provided callback function will be called when the operation is complete.
 *
 * @param src pointer to the source buffer, better to be 32 bit aligned
 * @param dest uint32_t value, 32 bit address to write to
 * @param len number of bytes to write, must be a multiple of 16 bytes
 * @param callback the callback function to be called when the operation is complete
 * @return ErrorState
 */
ErrorState writeAsync(uint8_t* src, uint32_t dest, uint32_t len, AsyncCallback callback = nullptr);

ErrorState waitForLastOperation(uint32_t timeout);

}
}  // namespace Drivers
