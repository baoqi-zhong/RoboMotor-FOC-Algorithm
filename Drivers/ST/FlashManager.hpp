/**
 * @file FlashManager.hpp
 * @brief STM32 internal flash driver declarations.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#pragma once

#include "../Generic/InternalFlashManager.hpp"

#if (PLATFORM_ST)

namespace Drivers
{
namespace FlashManager
{
#define FLASH_PAGE_1_BASE ((uint32_t)0x08000000)

using FlashErrorState = InternalFlashManager::ErrorState;
using FlashCallback = InternalFlashManager::AsyncCallback;

FlashErrorState getPageFromAddr(uint32_t addr, uint8_t* page);
FlashErrorState getAddrFromPage(uint8_t page, uint32_t* addr);
void init(void);
void update(void);
FlashErrorState erasePageAsync(uint8_t page, FlashCallback callback = nullptr);
FlashErrorState flashWriteAsync(uint8_t* src, uint32_t dest, uint32_t len, FlashCallback callback = nullptr);
FlashErrorState flashRead(uint32_t src, uint8_t* dest, uint32_t len);
FlashErrorState waitForLastOperation(uint32_t timeout);

} // namespace FlashManager
} // namespace Drivers

#endif // PLATFORM_ST
