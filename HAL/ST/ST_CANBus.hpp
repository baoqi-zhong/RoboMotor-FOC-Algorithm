/**
 * @file ST_CANBus.hpp
 * @brief STM32 classic CAN hardware abstraction implementation.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#pragma once

#include "Config.hpp"

#if (HARDWARE_PLATFORM == HARDWARE_PLATFORM_STM32) && defined(HAL_CAN_MODULE_ENABLED)

#include <stdint.h>
#include "main.h"

namespace HAL
{

class ST_CANBus
{
public:
    using Handle = CAN_HandleTypeDef*;
    using RxCallback = void (*)(uint32_t canId, const uint8_t* data);

    static bool init(Handle handle, uint32_t filterMask, uint32_t filterId)
    {
        if (handle == nullptr)
        {
            return false;
        }

        CAN_FilterTypeDef filter {};
        filter.FilterBank = 0;
        filter.FilterMode = CAN_FILTERMODE_IDMASK;
        filter.FilterScale = CAN_FILTERSCALE_32BIT;
        filter.FilterIdHigh = static_cast<uint16_t>((filterId << 5) & 0xFFFF);
        filter.FilterIdLow = 0;
        filter.FilterMaskIdHigh = static_cast<uint16_t>((filterMask << 5) & 0xFFFF);
        filter.FilterMaskIdLow = 0;
        filter.FilterFIFOAssignment = CAN_RX_FIFO0;
        filter.FilterActivation = CAN_FILTER_ENABLE;
        filter.SlaveStartFilterBank = 14;

        if (HAL_CAN_ConfigFilter(handle, &filter) != HAL_OK)
        {
            return false;
        }
        if (HAL_CAN_ActivateNotification(handle, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        {
            return false;
        }
        if (HAL_CAN_Start(handle) != HAL_OK)
        {
            return false;
        }

        return true;
    }

    static bool transmit(Handle handle, uint32_t canId, const uint8_t* data)
    {
        if (handle == nullptr || data == nullptr || HAL_CAN_GetTxMailboxesFreeLevel(handle) == 0)
        {
            return false;
        }

        CAN_TxHeaderTypeDef txHeader {};
        txHeader.StdId = canId;
        txHeader.ExtId = 0;
        txHeader.IDE = CAN_ID_STD;
        txHeader.RTR = CAN_RTR_DATA;
        txHeader.DLC = 8;
        txHeader.TransmitGlobalTime = DISABLE;

        uint32_t txMailbox = 0;
        return HAL_CAN_AddTxMessage(handle, &txHeader, data, &txMailbox) == HAL_OK;
    }

    static void registerCallback(RxCallback callback)
    {
        rxCallback = callback;
    }

    static void handleRxFifo0Irq(Handle handle)
    {
        CAN_RxHeaderTypeDef rxHeader {};
        uint8_t rxBuffer[8] {};

        if (HAL_CAN_GetRxMessage(handle, CAN_RX_FIFO0, &rxHeader, rxBuffer) != HAL_OK)
        {
            return;
        }

        if (rxCallback != nullptr)
        {
            const uint32_t canId = rxHeader.IDE == CAN_ID_STD ? rxHeader.StdId : rxHeader.ExtId;
            rxCallback(canId, rxBuffer);
        }
    }

private:
    inline static RxCallback rxCallback = nullptr;
};

} // namespace HAL

extern "C" inline void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    HAL::ST_CANBus::handleRxFifo0Irq(hcan);
}

#endif // HARDWARE_PLATFORM_STM32 && HAL_CAN_MODULE_ENABLED
