/**
 * @file InterBoard_USER.cpp
 * @brief User-defined InterBoard communication logic.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "InterBoard.hpp"

#if USER_INTERBOARD
namespace Control
{
namespace InterBoard
{
#include "ErrorHandler.hpp"
#include "MotorControl.hpp"
#include "PositionalPID.hpp"
#include "IncrementalPID.hpp"
#include "Encoder.hpp"
#if USE_NTC
#include "ADC.hpp"
#endif

/*
* 通信协议

* 所有数据都是 int16_t 类型.

* 主控板发送控制包:
* CAN ID: 0x100 + ID = 0x101~0x108
*         | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
* Byte 0: | 0 |      Control MASK         |
* Byte 1: |          Control Data         |
* Byte 2: |             Data              |
* Byte 3: |             Data              |
* Byte 4: |             Data              |
* Byte 5: |             Data              |
* Byte 6: |             Data              |
* Byte 7: |             Data              |


    * Byte 0:   Control Data
        * Bit 7: 手动触发复位?                          0: 无效, 1: 复位    推荐: 0
        * Bit 6: 位置 Byte 是否有效(是否闭环位置)?      0: 无效, 1: 有效    推荐: 1
        * Bit 5: 速度 Byte 是否有效(是否闭环速度)?      0: 无效, 1: 有效    推荐: 1
        * Bit 4: 扭矩 Byte 使用电流闭环还是电压控制?    0: 电压, 1: 电流    推荐: 1
        * Bit 3: 强制忽略所有错误?                      0: 否,   1: 是      推荐: 0
        * Bit 2: 自动复位?                              0: 否,   1: 自动复位推荐: 0
        * Bit 1:
        * Bit 0: 是否使能输出?                          0: 禁用, 1: 启用    推荐: 1

    * Byte 2: 最大扭矩电流      高 8 位
    * Byte 3: 最大扭矩电流      低 8 位
    * Byte 4: 目标速度          高 8 位
    * Byte 5: 目标速度          低 8 位
    * Byte 6: 目标位置(增量) 高 8 位
    * Byte 7: 目标位置(增量) 低 8 位



* 电机返回状态包:
* CAN ID: 0x200 + ID = 0x201~0x208

* Byte 0: 电机运行状态
    * Bit 7: 0
    * Bit 6: 位置闭环?                              0: 电压, 1: 电流
    * Bit 5: 速度闭环?                              0: 否,   1: 是
    * Bit 4: 电压/电流控制?                         0: 电压, 1: 电流
    * Bit 3: 强制忽略所有错误?                      0: 否,   1: 是
    * Bit 2: 是否会自动复位?                        0: 否,   1: 自动复位

    * Bit 0: 电机是否使能                           0: Dis   1: Enable

* Byte 1: 错误状态
    * Bit 7: 控制包解码错误?                        0: 无,   1: 有
    * Bit 6: 过温 Warning?                          0: 无,   1: 有
    * Bit 5: 过温错误?                              0: 无,   1: 过温
    * Bit 4: 过流错误?                              0: 无,   1: 过流
    * Bit 3: 欠压 过压错误 ?                        0: 正常, 1: 欠压或过压
    * Bit 2: 是否堵转?                              0: 无,   1: 有
    * Bit 1: 驱动器 FAULT?                          0: 无,   1: 有
    * Bit 0: 电机是否有 Error?                      0: 无,   1: 有

* Byte 2: 实际转矩电流 高 8 位
* Byte 3: 实际转矩电流 低 8 位
* Byte 4: 编码器(输出轴单圈位置) 高 8 位
* Byte 5: 编码器(输出轴单圈位置) 低 8 位
* Byte 6: 到目标位置的执行进度(0~100%)

*/

// 返回包
#define RESPONSE_PACKET_ENABLE_MOTOR_MASK       0x01
#define RESPONSE_PACKET_AUTO_RESET_MASK         0x04
#define RESPONSE_PACKET_IGNORE_ERROR_MASK       0x08
#define RESPONSE_PACKET_TORQUE_CONTROL_MASK     0x10
#define RESPONSE_PACKET_SPEED_CONTROL_MASK      0x20
#define RESPONSE_PACKET_POSITION_CONTROL_MASK   0x40

#define RESPONSE_PACKET_ANY_ERROR_MASK          0x01
#define RESPONSE_PACKET_DRIVER_FAULT_MASK       0x02
#define RESPONSE_PACKET_STALL_MASK              0x04
#define RESPONSE_PACKET_UNDER_VOLTAGE_MASK      0x08
#define RESPONSE_PACKET_OVER_VOLTAGE_MASK       0x10

#define RESPONSE_PACKET_OVER_CURRENT_MASK       0x10
#define RESPONSE_PACKET_OVER_TEMPERATURE_MASK   0x20
#define RESPONSE_PACKET_OVER_TEMPERATURE_WARNING_MASK 0x40
#define RESPONSE_PACKET_ERROR_MASK              0x80

// 控制包
#define CONTROL_PACKET_ENABLE_MOTOR_MASK        0x01
#define CONTROL_PACKET_AUTO_RESET_MASK          0x04
#define CONTROL_PACKET_IGNORE_ERROR_MASK        0x08
#define CONTROL_PACKET_TORQUE_CONTROL_MASK      0x10
#define CONTROL_PACKET_SPEED_CONTROL_MASK       0x20
#define CONTROL_PACKET_POSITION_CONTROL_MASK    0x40
#define CONTROL_PACKET_TRIGGER_RESET_MASK       0x80


extern PositionalPID_t positionToCurrentPID, positionToVelocityPID, velocityPID;
extern IncrementalPID_t IqPID, IdPID;



void InterBoard_Decode(uint32_t CANId, uint8_t* rxBuffer)
{
    if(CANId != 0x100 + CAN_ID)
        return;
    
    interBoardStatus.disconnectCounter = 0;
    interBoardStatus.connectionStatus = 1;

    if((rxBuffer[0] & 0x80) == 0)
    {
        if(rxBuffer[0] & CONTROL_PACKET_ENABLE_MOTOR_MASK)
        {
            motorControlStatus.enableMotor = rxBuffer[1] & 0x01;
        }

        if(rxBuffer[0] & CONTROL_PACKET_AUTO_RESET_MASK)
            motorControlStatus.enableAutoRecovery = (rxBuffer[1] & CONTROL_PACKET_AUTO_RESET_MASK) >> 2;
  
        if(rxBuffer[0] & CONTROL_PACKET_IGNORE_ERROR_MASK)
            Control::ErrorHandler::errorHandlerConfig.ignoreAllErrors = (rxBuffer[1] & CONTROL_PACKET_IGNORE_ERROR_MASK) >> 3;

        if(rxBuffer[0] & CONTROL_PACKET_TORQUE_CONTROL_MASK)
        {
            if(rxBuffer[1] & CONTROL_PACKET_TORQUE_CONTROL_MASK)
            {
                IncrementalPIDreset(&IqPID);
                IncrementalPIDreset(&IdPID);
                motorControlConfig.FOCControlMode = CURRENT_TOURQUE_CONTROL;
            }
            else
            {
                motorControlConfig.FOCControlMode = VOLTAGE_TOURQUE_CONTROL;     
            }
        }

        if(rxBuffer[0] & CONTROL_PACKET_SPEED_CONTROL_MASK)
        {
            if(rxBuffer[1] & CONTROL_PACKET_SPEED_CONTROL_MASK)
            {
                PositionalPIDreset(&velocityPID);
                PositionalPIDreset(&positionToVelocityPID);
                motorControlStatus.enableSpeedCloseLoop = 1;            
            }
            else
            {
                PositionalPIDreset(&positionToCurrentPID);
                motorControlStatus.enableSpeedCloseLoop = 0;
            }
        }

        if(rxBuffer[0] & CONTROL_PACKET_POSITION_CONTROL_MASK)
        {
            if(rxBuffer[1] & CONTROL_PACKET_POSITION_CONTROL_MASK)
            {
                if(motorControlStatus.enableSpeedCloseLoop)
                    PositionalPIDreset(&positionToVelocityPID);
                else
                    PositionalPIDreset(&positionToCurrentPID);
                motorControlStatus.enablePositionCloseLoop = 1;
            }
            else
            {
                if(motorControlStatus.enableSpeedCloseLoop)
                motorControlStatus.enablePositionCloseLoop = 0;
            }
        }
            
        motorControlConfig.IqLimit = (float)((int16_t)(rxBuffer[2] << 8 | rxBuffer[3])) / 10000.0f;
        motorControlConfig.velocityLimit = (float)((int16_t)(rxBuffer[4] << 8 | rxBuffer[5])) / 1000.0f;
        motorControlStatus.targetPosition += (float)((int16_t)(rxBuffer[6] << 8 | rxBuffer[7])) / 10.0f;
        // 最后触发复位
        if(rxBuffer[1] & CONTROL_PACKET_TRIGGER_RESET_MASK)
        {
            motorControlStatus.triggerReset = 1;
        }
    }
}

void InterBoard_TransmitFeedback()
{
    for(uint8_t i = 0; i < 8; i++)
        InterBoardTxBuffer[i] = 0;
    
    uint8_t temp = 0;
    if(motorControlStatus.enablePositionCloseLoop)
        temp |= RESPONSE_PACKET_POSITION_CONTROL_MASK;
    if(motorControlStatus.enableSpeedCloseLoop)
        temp |= RESPONSE_PACKET_SPEED_CONTROL_MASK;
    if(motorControlConfig.FOCControlMode == VOLTAGE_TOURQUE_CONTROL)
        temp |= RESPONSE_PACKET_TORQUE_CONTROL_MASK;
    if(Control::ErrorHandler::errorHandlerConfig.ignoreAllErrors)
        temp |= RESPONSE_PACKET_IGNORE_ERROR_MASK;
    if(motorControlStatus.enableAutoRecovery)
        temp |= RESPONSE_PACKET_AUTO_RESET_MASK;
    if(motorControlStatus.enableFOCOutput)
        temp |= RESPONSE_PACKET_ENABLE_MOTOR_MASK;
    InterBoardTxBuffer[0] = temp;

    temp = 0;
    if(motorErrorStatus.overTemperature)
        temp |= RESPONSE_PACKET_OVER_TEMPERATURE_MASK;
    if(motorErrorStatus.overCurrent)
        temp |= RESPONSE_PACKET_OVER_CURRENT_MASK;
    if(motorErrorStatus.underVoltage)
        temp |= RESPONSE_PACKET_UNDER_VOLTAGE_MASK;
    if(motorErrorStatus.overVoltage)
        temp |= RESPONSE_PACKET_OVER_VOLTAGE_MASK;
    if(ErrorHandler_CheckIfAnyErrorStatus())
        temp |= RESPONSE_PACKET_ANY_ERROR_MASK;
    InterBoardTxBuffer[1] = temp;

    InterBoardTxBuffer[2] = (int16_t)(measuredIq * 10000) >> 8;
    InterBoardTxBuffer[3] = (int16_t)(measuredIq * 10000) & 0xFF;
    InterBoardTxBuffer[4] = Sensor::Encoder::encoderStatus.encoder >> 8;
    InterBoardTxBuffer[5] = Sensor::Encoder::encoderStatus.encoder & 0xFF;
    FDCANManager_Transmit(0x200 + CAN_ID, InterBoardTxBuffer);
}

} // namespace InterBoard
} // namespace Control
#endif // USER_INTERBOARD