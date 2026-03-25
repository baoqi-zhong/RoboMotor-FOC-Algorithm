/**
 * @file MotorControl.hpp
 * @brief Motor control state machine and configuration.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#pragma once

// STM32 必要外设
#include "main.h"
#include "stm32g4xx_hal.h"
#include "adc.h"
#include "tim.h"

// Neccessary Libraries when Initializing
#include "stdint.h"

#include "ADC.hpp"
#include "Boards.hpp"

namespace Control
{
namespace MotorControl
{

/* 仅在初始化时用于修改 Status */
struct MotorControlConfig
{
    uint8_t enableSpeedCloseLoop    = 0;
    uint8_t enablePositionCloseLoop = 0;

    float defaultIqLimit            = 5.0f;     /* A */
    float defaultVelocityLimit      = 40.0f;    /* RAD/S */
    float openLoopRotateSpeed       = 10.0f;    /* RAD/S */
    float openLoopDragVoltage       = 2.0f;     /* V */

    uint8_t boardID                 = 1;        /* 1 - 4 */
};

struct CPULoad_t
{
    float IRQLoad;
    float free;
};

enum class MotorControlState : uint8_t
{
    Stop = 0,
    preADCCalibrating,
    ADCCalibrating,
    preChargingBootCap,
    ChargingBootCap,
    preRunning,
    Running,

    preMusic,
    Music,
};

enum class MotorCalibrationState : uint8_t
{
    Stop = 0,
    preCalibrating,
    Calibrating,
};

struct MotorControlStatus
{
    CPULoad_t CPULoad;
    MotorControlState state     = MotorControlState::Stop;
    MotorCalibrationState calibrationState = MotorCalibrationState::preCalibrating;

    uint8_t enableMotor         = 1;
    uint8_t enableFOCOutput     = 0;
    uint8_t enableCalibration   = 0;
    uint8_t triggerReset        = 0;

    float targetIq              = 0.0f;
    float targetId              = 0.0f;
    float targetVelocity        = 0.0f;
    float targetPosition        = 0.0f;
};

extern MotorControlConfig motorControlConfig;
extern MotorControlStatus motorControlStatus;

extern uint32_t Loop4KHzCounter;
extern uint32_t Loop1KHzCounter;

extern Control::PositionalPID positionToCurrentPID;
extern Control::PositionalPID positionToVelocityPID;
extern Control::PositionalPID velocityPID;

void setConfig(const MotorControlConfig* config);

void init();

/**
 * @brief 处理 motorControlStatus.triggerReset == 1 的情况
 */
void triggerResetHandler();

} // namespace MotorControl
} // namespace Control