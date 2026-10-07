/**
 * @file FDCANManager.cpp
 * @brief STM32 FDCAN driver implementation.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "CANManager.hpp"

#if (PLATFORM_ST && defined(HAL_FDCAN_MODULE_ENABLED))
#include "fdcan.h"


namespace Drivers
{
namespace CANManager
{
FDCAN_TxHeaderTypeDef FDCANTxHeader {};

void init(uint32_t filterMask, uint32_t filterId)
{
    FDCANTxHeader.IdType = FDCAN_STANDARD_ID;
    FDCANTxHeader.TxFrameType = FDCAN_DATA_FRAME;
    FDCANTxHeader.DataLength = FDCAN_DLC_BYTES_8;
    FDCANTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    FDCANTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    FDCANTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    FDCANTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    FDCANTxHeader.MessageMarker = 0;

    FDCAN_FilterTypeDef filter;
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = filterId;
    filter.FilterID2 = filterMask;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan1);
}

void transmit(uint32_t CANId, const uint8_t* txBuffer)
{
    FDCANTxHeader.Identifier = CANId;

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0)
        return;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FDCANTxHeader, txBuffer);
}

} // namespace CANManager
} // namespace Drivers

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;
    FDCAN_RxHeaderTypeDef FDCANRxHeader;
    uint8_t rxBuffer[8];

    if (hfdcan != &hfdcan1)
        return;

    if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCANRxHeader, rxBuffer) != HAL_OK)
        return;

    if (Drivers::CANManager::rxCallback)
    {
        Drivers::CANManager::rxCallback(FDCANRxHeader.Identifier, rxBuffer);
    }
}

#endif // PLATFORM_ST