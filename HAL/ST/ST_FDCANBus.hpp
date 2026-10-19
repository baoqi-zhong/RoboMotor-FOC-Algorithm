/**
 * @file ST_FDCANBus.hpp
 * @brief STM32 FDCAN hardware abstraction implementation.
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

#if (HARDWARE_PLATFORM == HARDWARE_PLATFORM_STM32) && defined(HAL_FDCAN_MODULE_ENABLED)

#include <stdint.h>
#include "main.h"

namespace HAL
{

class ST_FDCANBus
{
public:
    using Handle = FDCAN_HandleTypeDef*;
    using RxCallback = void (*)(uint32_t canId, const uint8_t* data);

    static bool init(Handle handle, uint32_t filterMask, uint32_t filterId)
    {
        if (handle == nullptr)
        {
            return false;
        }

        FDCAN_FilterTypeDef filter {};
        filter.IdType = FDCAN_STANDARD_ID;
        filter.FilterIndex = 0;
        filter.FilterType = FDCAN_FILTER_MASK;
        filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        filter.FilterID1 = filterId;
        filter.FilterID2 = filterMask;

        if (HAL_FDCAN_ConfigFilter(handle, &filter) != HAL_OK)
        {
            return false;
        }
        if (HAL_FDCAN_ConfigGlobalFilter(handle,
                                         FDCAN_REJECT,
                                         FDCAN_REJECT,
                                         FDCAN_REJECT_REMOTE,
                                         FDCAN_REJECT_REMOTE) != HAL_OK)
        {
            return false;
        }
        if (HAL_FDCAN_ActivateNotification(handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        {
            return false;
        }
        if (HAL_FDCAN_Start(handle) != HAL_OK)
        {
            return false;
        }

        return true;
    }

    static bool transmit(Handle handle, uint32_t canId, const uint8_t* data)
    {
        if (handle == nullptr || data == nullptr || HAL_FDCAN_GetTxFifoFreeLevel(handle) == 0)
        {
            return false;
        }

        FDCAN_TxHeaderTypeDef txHeader {};
        txHeader.Identifier = canId;
        txHeader.IdType = FDCAN_STANDARD_ID;
        txHeader.TxFrameType = FDCAN_DATA_FRAME;
        txHeader.DataLength = FDCAN_DLC_BYTES_8;
        txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        txHeader.BitRateSwitch = FDCAN_BRS_OFF;
        txHeader.FDFormat = FDCAN_CLASSIC_CAN;
        txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        txHeader.MessageMarker = 0;

        return HAL_FDCAN_AddMessageToTxFifoQ(handle, &txHeader, data) == HAL_OK;
    }

    static void registerCallback(RxCallback callback)
    {
        rxCallback = callback;
    }

    static void handleRxFifo0Irq(Handle handle)
    {
        FDCAN_RxHeaderTypeDef rxHeader {};
        uint8_t rxBuffer[8] {};

        if (HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &rxHeader, rxBuffer) != HAL_OK)
        {
            return;
        }

        if (rxCallback != nullptr)
        {
            rxCallback(rxHeader.Identifier, rxBuffer);
        }
    }

private:
    inline static RxCallback rxCallback = nullptr;
};

} // namespace HAL

extern "C" inline void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;
    HAL::ST_FDCANBus::handleRxFifo0Irq(hfdcan);
}

#endif // HARDWARE_PLATFORM_STM32 && HAL_FDCAN_MODULE_ENABLED
