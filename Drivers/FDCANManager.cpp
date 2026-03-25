/**
 * @file FDCANManager.cpp
 * @brief FDCAN initialization and transmission implementation.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "FDCANManager.hpp"

namespace Drivers
{
namespace FDCANManager
{
FDCAN_HandleTypeDef* hfdcan;

void init(FDCAN_HandleTypeDef* hfdcan_, uint32_t filterMask, uint32_t filterId)
{
    hfdcan = hfdcan_;

    FDCAN_FilterTypeDef filter;
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = filterMask;
    filter.FilterID2 = filterId;  
    HAL_FDCAN_ConfigFilter(hfdcan, &filter);

    HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    HAL_FDCAN_Start(hfdcan);
}

// Controller
FDCAN_TxHeaderTypeDef FDCANTxHeader;

void setTxHeader(uint32_t CANId)
{
    FDCANTxHeader.Identifier = CANId;

    FDCANTxHeader.IdType = FDCAN_STANDARD_ID;
    FDCANTxHeader.TxFrameType = FDCAN_DATA_FRAME;
    FDCANTxHeader.DataLength = FDCAN_DLC_BYTES_8;
    FDCANTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    FDCANTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    FDCANTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    FDCANTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    FDCANTxHeader.MessageMarker = 0;
}

void transmit(uint32_t CANId, uint8_t* txBuffer)
{
    setTxHeader(CANId);
    // FIFO will only be cleaned when the transeiver is powered on.
    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0)
        return;
    if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &FDCANTxHeader, txBuffer) != HAL_OK)
        return;
}

// Decoder
FDCAN_RxHeaderTypeDef FDCANRxHeader;
uint8_t FDCANRxData[8];

void (*DecodeMessage)(uint32_t CANId, uint8_t* rxBuffer) = NULL;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan_, uint32_t RxFifo0ITs)
{
    HAL_FDCAN_GetRxMessage(hfdcan_, FDCAN_RX_FIFO0, &FDCANRxHeader, FDCANRxData);
    if(DecodeMessage != NULL)
        DecodeMessage(FDCANRxHeader.Identifier, FDCANRxData);
}

void registerCallback(void (*callback)(uint32_t CANId, uint8_t* rxBuffer))
{
    DecodeMessage = callback;
}

} // namespace FDCANManager
} // namespace Drivers