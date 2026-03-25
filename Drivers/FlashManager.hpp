/**
 * @file FlashManager.hpp
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
#include "main.h"
#include "string.h"
#include "stdint.h"


namespace Drivers
{
namespace FlashManager
{
// Start address of each flash Page
#define FLASH_PAGE_1_BASE ((uint32_t)0x08000000)    /* Base address of Page 1, 2KBytes each page */

enum class FlashErrorState : uint8_t
{
    FLASH_SUCCESS   = 0,
    FLASH_ERROR,
    FLASH_NOT_MATCH,
    FLASH_BUSY,
    FLASH_TIMEOUT
};


/**
 * @brief Get the page number from the address
 *
 * @param addr
 * @param page 0 ~ 255, depending on the total flash size
 * @return uint8_t
 */
FlashErrorState getPageFromAddr(uint32_t addr, uint8_t* page);


/**
 * @brief Get the address from the page number
 *
 * @param page 0 ~ 255, depending on the total flash size
 * @param addr
 * @return uint8_t
 */
FlashErrorState getAddrFromPage(uint8_t page, uint32_t* addr);

/**
 * @brief init the flash module and unlock the flash
 *
 */
void init(void);

/**
 * @brief Update the flash manager state machine. Call this in the main loop (e.g. 1kHz).
 * 
 */
void update(void);

typedef void (*FlashCallback)(FlashErrorState state);

/**
 * @brief
 *
 * @param page 0 ~ 255, depending on the total flash size
 */
FlashErrorState erasePageAsync(uint8_t page, FlashCallback callback = nullptr);

/**
 * @brief
 *
 * @param src pointer to the source buffer, better to be 32 bit aligned
 * @param dest uint32_t value, 32 bit address to write to
 * @param len number of bytes to write, must be a multiple of 16 bytes
 * @return FlashErrorState
 */
FlashErrorState flashWriteAsync(uint8_t* src, uint32_t dest, uint32_t len, FlashCallback callback = nullptr);

/**
 * @brief
 *
 * @param src uint32_t value, 32 bit address to read from
 * @param dest pointer to the result buffer
 * @param len number of bytes to read
 * @return uint8_t
 */
FlashErrorState flashRead(uint32_t src, uint8_t* dest, uint32_t len);

FlashErrorState waitForLastOperation(uint32_t timeout);

}  // namespace FlashManager
}  // namespace Drivers