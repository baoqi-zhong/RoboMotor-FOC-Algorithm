/**
 * @file FlashManager.cpp
 * @brief Internal Flash management for STM32G4 implementation.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "FlashManager.hpp"


#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_flash_ex.h"

// STM32G431VE
// Single Bank, 2KB Bank 64Pages = 128KB total flash size
#define USER_FLASH_SIZE             0x00020000  // 128KB
#define USER_FLASH_PAGE_NB          64          // 128KB / 2KB per page
#define USER_FLASH_PAGE_SIZE        0x800       // 2KB per page

namespace Drivers
{
namespace FlashManager
{
FlashErrorState getPageFromAddr(uint32_t addr, uint8_t* page)
{
    if(FLASH_PAGE_1_BASE <= addr && addr < (FLASH_PAGE_1_BASE + (USER_FLASH_SIZE >> 1))) // Bank 1
    {
        *page = (addr - FLASH_PAGE_1_BASE) / USER_FLASH_PAGE_SIZE; // Calculate page number
    }
    else
    {
        return FlashErrorState::FLASH_ERROR;  // Invalid address
    }

    return FlashErrorState::FLASH_SUCCESS; // Success
}

FlashErrorState getAddrFromPage(uint8_t page, uint32_t* addr)
{
    if(
        page >= USER_FLASH_PAGE_NB ||
        addr == nullptr
    )
    {
        return FlashErrorState::FLASH_ERROR; // Invalid page
    }

    *addr = FLASH_PAGE_1_BASE + (page * USER_FLASH_PAGE_SIZE);
    return FlashErrorState::FLASH_SUCCESS;
}

FlashCallback currentCallback = nullptr;
uint8_t* writeSrc = nullptr;
uint32_t writeDest = 0;
uint32_t writeRemaining = 0;

enum class FlashState : uint8_t
{
    IDLE,
    ERASE_BUSY,     // Waiting for Hardware Erase
    ERASE_COMPLETE, // Erase finished, waiting for callback
    WRITE_BUSY,     // Waiting for Hardware Program
    WRITE_READY,    // Ready to program next DoubleWord or finish
    ERROR           // Hardware error occurred
};

volatile FlashState flashState = FlashState::IDLE;

void init(void)
{
    flashState = FlashState::IDLE;
}

uint8_t isBusy()
{
    // Consider ERROR state as not busy for new operations? 
    // Usually if error, we want to clear it or report it and go back to IDLE.
    // In update(), ERROR transitions to IDLE after callback.
    // So here we just check if IDLE.
    return (flashState != FlashState::IDLE) ? 1 : 0;
}

/**
 * 专门为写入多个 DOUBLEWORD 设计的异步写入状态机
 */
void update(void)
{
    switch(flashState)
    {
        case FlashState::IDLE:
        case FlashState::ERASE_BUSY:
        case FlashState::WRITE_BUSY:
            // Waiting for hardware or idle
            return;
        
        case FlashState::ERROR:
            flashState = FlashState::IDLE;
            if(currentCallback) currentCallback(FlashErrorState::FLASH_ERROR);
            break;

        case FlashState::ERASE_COMPLETE:
            flashState = FlashState::IDLE;
            if(currentCallback) currentCallback(FlashErrorState::FLASH_SUCCESS);
            break;

        case FlashState::WRITE_READY:
            if (writeRemaining > 0)
            {
                HAL_FLASH_Unlock();
                
                flashState = FlashState::WRITE_BUSY;
                
                // G4 uses 64-bit (Double Word) programming
                if (HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_DOUBLEWORD, writeDest, *(uint64_t*)writeSrc) != HAL_OK)
                {
                    // Error
                    HAL_FLASH_Lock();
                    flashState = FlashState::IDLE;
                    if(currentCallback) currentCallback(FlashErrorState::FLASH_ERROR);
                }
                else
                {
                    // Success starting, advance pointers for NEXT time
                    writeDest += 8;
                    writeSrc += 8;
                    if (writeRemaining >= 8) writeRemaining -= 8;
                    else writeRemaining = 0;
                }
            }
            else
            {
                // All done
                flashState = FlashState::IDLE;
                if(currentCallback) currentCallback(FlashErrorState::FLASH_SUCCESS);
            }
            break;
    }
}

FlashErrorState erasePageAsync(uint8_t page, FlashCallback callback)
{
    if(flashState != FlashState::IDLE)
    {
        return FlashErrorState::FLASH_BUSY;
    }

    if(page >= USER_FLASH_PAGE_NB)
    {
        return FlashErrorState::FLASH_ERROR; // Invalid page
    }

    FLASH_EraseInitTypeDef pEraseInit;
    
    pEraseInit.TypeErase    = FLASH_TYPEERASE_PAGES;
    pEraseInit.Banks        = FLASH_BANK_1;
    pEraseInit.Page         = page;
    pEraseInit.NbPages      = 1;

    currentCallback = callback;
    flashState = FlashState::ERASE_BUSY;
    
    HAL_FLASH_Unlock();

    if (HAL_FLASHEx_Erase_IT(&pEraseInit) != HAL_OK)
    {
        // lock the flash when error occurs
        HAL_FLASH_Lock();
        flashState = FlashState::IDLE;
        return FlashErrorState::FLASH_ERROR;
    }
    return FlashErrorState::FLASH_SUCCESS;
}

FlashErrorState flashWriteAsync(uint8_t* src, uint32_t dest, uint32_t len, FlashCallback callback)
{
    if(flashState != FlashState::IDLE)
    {
        return FlashErrorState::FLASH_BUSY;
    }
    
    // G4 DoubleWord Write (8 bytes)
    if (len % 8 != 0) return FlashErrorState::FLASH_ERROR;

    currentCallback = callback;
    writeSrc = src;
    writeDest = dest;
    writeRemaining = len;
    
    // Set to READY so update() picks it up
    flashState = FlashState::WRITE_READY;

    return FlashErrorState::FLASH_SUCCESS;
}

FlashErrorState flashRead(uint32_t src, uint8_t* dest, uint32_t len)
{
    // Return a valid address to avoid HardFault
    uint32_t i    = 0;
    uint8_t *psrc = (uint8_t *)src;

    if(flashState != FlashState::IDLE)
    {
        return FlashErrorState::FLASH_BUSY;
    }

    for (i = 0; i < len; i++)
    {
        ((uint8_t *)dest)[i] = *psrc++;
    }
    return FlashErrorState::FLASH_SUCCESS;
}

FlashErrorState waitForLastOperation(uint32_t timeout)
{
    uint32_t tickstart = HAL_GetTick();
    while (flashState != FlashState::IDLE)
    {
        // Allow state machine to run if waiting in a loop (though risk of reentrancy if called from ISR?)
        // waitForLastOperation should probably not be used with async logic mixed, but if user calls it:
        update(); 

        if ((HAL_GetTick() - tickstart) > timeout)
        {
            return FlashErrorState::FLASH_TIMEOUT;
        }
    }

    return FlashErrorState::FLASH_SUCCESS;
}

}  // namespace FlashManager
}  // namespace Drivers

// Implement these callbacks for Flash Interrupts
extern "C" void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue)
{
    using namespace Drivers::FlashManager;

    // Hardware operation finished
    HAL_FLASH_Lock();
    
    if (flashState == FlashState::ERASE_BUSY)
    {
        flashState = FlashState::ERASE_COMPLETE;
    }
    else if (flashState == FlashState::WRITE_BUSY)
    {
        flashState = FlashState::WRITE_READY;
    }
}

extern "C" void HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue)
{
    using namespace Drivers::FlashManager;
    
    HAL_FLASH_Lock();
    flashState = FlashState::ERROR;
}