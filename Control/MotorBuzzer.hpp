/**
 * @file MotorBuzzer.hpp
 * @brief Motor buzzer status and note definitions.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#if 0

#pragma once

#include "main.h"
#include "tim.h"

namespace Control
{
namespace MotorBuzzer
{
#define MAX_MOTOR_BUZZER_QUEUE_SIZE 16
struct Note
{
    uint16_t frequency  = 0;        /* Hz */
    uint16_t duration   = 0;        /* ms */
    float intensity     = 0.0f;     /* 0.0f - 1.0f, 这个 note 的音量, 通过调整 FOC 电压的幅值来实现 */
};

struct MotorBuzzerStatus
{
    Note noteQueue[MAX_MOTOR_BUZZER_QUEUE_SIZE];
    uint8_t head = 0;
    uint8_t tail = 0;
    uint8_t isPlaying = 0;
};

extern MotorBuzzerStatus motorBuzzerStatus;

void addNote(uint16_t frequency, uint16_t duration, float intensity);

void stop();

void update();

} // namespace MotorBuzzer
} // namespace Control

#endif