/**
 * @file InterBoard.hpp
 * @brief Inter-board communication configuration and status definitions.
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

namespace Control
{
namespace InterBoard
{
struct InterBoardConfig
{
    uint32_t CANFilterMask                      = 0x7FF;    /* FDCAN 接收的 ID 范围 */
    uint32_t CANFilterID                        = 0x200;    /* FDCAN 接收的 ID 范围 */
    uint32_t interboardDisconnectTriggerTimeout = 1000;     /* Interboard 断连触发时间, Tick */
};

struct InterBoardStatus 
{
    uint16_t disconnectCounter;
    uint8_t connectionStatus;

    // For transmit
    int32_t accumulatedEncoder;
    uint32_t accumulatedEncoderCounter;
    float accumulatedRotorSpeed;
    uint32_t accumulatedRotorSpeedCounter;
    float accumulatedIq;
    uint32_t accumulatedIqCounter;
};

extern InterBoardConfig interBoardConfig;
extern InterBoardStatus interBoardStatus;
extern uint8_t InterBoardTxBuffer[8];


/**
 * @brief  设置 InterBoard 配置, 应该在 init 之前调用
 */
void setConfig(const InterBoardConfig* config);

/**
 * @brief  初始化 InterBoard
 */
void init();

/**
 * @brief  发送 InterBoard 反馈, 应该被 1ms 调用 1 次
 */
void transmitFeedback();

/**
 * @brief  最高频率记录 Interboard 需要发送的反馈, 用于向用户返回平均值
 */
void recordDataHighFreq();

/**
 * @brief  1KHz 频率调用, 用于发送 Feedback 和检查控制包连接状态
 */
void handler1KHz();

} // namespace InterBoard
} // namespace Control