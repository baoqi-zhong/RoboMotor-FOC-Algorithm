/**
 * @file InterBoard.cpp
 * @brief Inter-board communication protocol handlling.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "InterBoard.hpp"

#include "FDCANManager.hpp"
#include "MotorControl.hpp"
#include "ThreePhaseFOC.hpp"
#include "PositionalPID.hpp"
#include "IncrementalPID.hpp"
#include "Encoder.hpp"

namespace Control
{
namespace InterBoard
{
InterBoardConfig interBoardConfig;
InterBoardStatus interBoardStatus;

uint8_t InterBoardTxBuffer[8] = {0};

extern void decode(uint32_t CANId, uint8_t* rxBuffer);

void setConfig(const InterBoardConfig* config)
{
    interBoardConfig = *config;
}

void init()
{
    Drivers::FDCANManager::registerCallback(decode);
    Drivers::FDCANManager::init(&hfdcan1, interBoardConfig.CANFilterMask, interBoardConfig.CANFilterID);

    interBoardStatus.disconnectCounter              = 0;
    interBoardStatus.connectionStatus               = 0;
    interBoardStatus.accumulatedEncoder             = 0;
    interBoardStatus.accumulatedEncoderCounter      = 0;
    interBoardStatus.accumulatedRotorSpeed          = 0;
    interBoardStatus.accumulatedRotorSpeedCounter   = 0;
    interBoardStatus.accumulatedIq                  = 0;
    interBoardStatus.accumulatedIqCounter           = 0;
}

void recordDataHighFreq()
{
    // 用 signed 类型, 防止过零点求平均的bug
    interBoardStatus.accumulatedEncoder += (int16_t)Sensor::Encoder::encoderStatus.Q16_encoder;
    interBoardStatus.accumulatedEncoderCounter ++;

    interBoardStatus.accumulatedRotorSpeed += Sensor::Encoder::encoderStatus.RAD_shaftAngularVelocity;
    interBoardStatus.accumulatedRotorSpeedCounter ++;

    interBoardStatus.accumulatedIq += Control::FOC::measuredIq;
    interBoardStatus.accumulatedIqCounter ++;
}

void handler1KHz()
{
    Control::InterBoard::transmitFeedback();

    if(interBoardStatus.disconnectCounter > interBoardConfig.interboardDisconnectTriggerTimeout)
    {
        interBoardStatus.connectionStatus = 0;
        // 不应该直接 disableFOC. 应该保证电流环运行, 其他环关闭
        // motorControlStatus.targetIq = 0;
        // motorControlStatus.enablePositionCloseLoop = 0;
        // motorControlStatus.enableSpeedCloseLoop = 0;
    }
    else
    {
        interBoardStatus.disconnectCounter ++;
    }
}

} // namespace InterBoard
} // namespace Control