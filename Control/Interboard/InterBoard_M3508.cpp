/**
 * @file InterBoard_M3508.cpp
 * @brief M3508 motor protocol decoding for InterBoard.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "InterBoard.hpp"

#include "ErrorHandler.hpp"
#include "MotorControl.hpp"
#include "Encoder.hpp"
#include "FDCANManager.hpp"
#include "Math.hpp"

namespace Control
{
namespace InterBoard
{
void decode(uint32_t CANId, uint8_t* rxBuffer)
{
    if(CANId != 0x200)
        return;

    int16_t receiveIq = (int16_t)(rxBuffer[(Control::MotorControl::motorControlConfig.boardID - 1) * 2] << 8 | rxBuffer[(Control::MotorControl::motorControlConfig.boardID - 1) * 2 + 1]);
    Control::MotorControl::motorControlStatus.targetIq = receiveIq * 20.0f / 16384.0f;

    interBoardStatus.disconnectCounter = 0;
    interBoardStatus.connectionStatus = 1;
}

uint16_t transmitEncoder;
int16_t transmitRotorSpeed;
int16_t transmitIq;
extern float measuredIq;

void transmitFeedback()
{
    // encoder 范围为 [0, 8191]
    transmitEncoder    = (uint16_t)(Sensor::Encoder::encoderStatus.RAD_accumulatedShaftAngle / TWO_PI * 8191.0f) % 8192;
    // 转速: RPM
    transmitRotorSpeed  = (int32_t)(interBoardStatus.accumulatedRotorSpeed * 9.54930f) / (int32_t)interBoardStatus.accumulatedRotorSpeedCounter;
    // 扭矩电流 [-16384, 16384]
    transmitIq         = (int32_t)(interBoardStatus.accumulatedIq * 16384.0f / 20.0f) / (int32_t)interBoardStatus.accumulatedIqCounter;

    // 单位: 度
    #if USE_NTC
    uint16_t transmitTemperature = Sensor::ADC::analogValues.NTCTemperature;
    #else
    uint16_t transmitTemperature = 0;
    #endif
    InterBoardTxBuffer[0] = transmitEncoder >> 8;
    InterBoardTxBuffer[1] = transmitEncoder & 0xFF;
    InterBoardTxBuffer[2] = transmitRotorSpeed >> 8;
    InterBoardTxBuffer[3] = transmitRotorSpeed & 0xFF;
    InterBoardTxBuffer[4] = transmitIq >> 8;
    InterBoardTxBuffer[5] = transmitIq & 0xFF;
    InterBoardTxBuffer[6] = transmitTemperature;
    InterBoardTxBuffer[7] = 0;

    interBoardStatus.accumulatedEncoder             = 0;
    interBoardStatus.accumulatedEncoderCounter      = 0;
    interBoardStatus.accumulatedRotorSpeed          = 0;
    interBoardStatus.accumulatedRotorSpeedCounter   = 0;
    interBoardStatus.accumulatedIq                  = 0;
    interBoardStatus.accumulatedIqCounter           = 0;
    Drivers::FDCANManager::transmit(0x200 + Control::MotorControl::motorControlConfig.boardID, InterBoardTxBuffer);
}

} // namespace InterBoard
} // namespace Control