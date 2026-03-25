/**
 * @file ThreePhaseFOC.hpp
 * @brief FOC algorithm configuration, constants and interface.
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

#include "IncrementalPID.hpp"

namespace Control
{
namespace FOC
{
enum class FOCControlMode_t : uint8_t
{
    CURRENT_TOURQUE_CONTROL = 0,
    VOLTAGE_TOURQUE_CONTROL,
};

struct MotorConfig
{
    uint8_t REVERSE_DIRECTION   = 0;            /* 是否反转方向, 0 不反转, 1 反转. 正方向为 A-B-C 依次高电平旋转的方向 */
    uint8_t POLE_PAIRS          = 7;
    float shaftReductionRatio   = 1.0f;         /* 输出减速比, 输出轴角度 : 编码器角度 */
    float electricAngleReductionRatio = 7.0f;   /* 编码器减速比, 电角度 : 编码器角度 */
    float phaseResistance       = 0.0f;         /* 相电阻, Ω */
    float phaseInductance       = 0.0f;         /* 相电感, H */
    float kv                    = 300.0f;       /* 电机速度常数, RPM/V */
};

struct FOCConfig
{
    float currentLoopFreq       = 20000.0f;     /* 电流环频率, Hz */
    uint16_t timerPeriod        = 4250;         /* PWM 周期, 定时器计数上限 */

    FOCControlMode_t FOCControlMode = FOCControlMode_t::CURRENT_TOURQUE_CONTROL;
};


extern float measuredIalpha;
extern float measuredIbeta;
extern float measuredIq;
extern float measuredId;
extern float outputUq;
extern float outputUd;
extern float outputUqWithFeedForward;
extern float outputUdWithFeedForward;
extern float outputUalpha;
extern float outputUbeta;
extern int16_t outputAngle;

extern Control::IncrementalPID IqPID;
extern Control::IncrementalPID IdPID;
extern MotorConfig motorConfig;
extern FOCConfig focConfig;

void setPhraseVoltage(float Ualpha, float Ubeta);


void setMotorConfig(MotorConfig* config);

void setFOCConfig(FOCConfig* config);

/**
 * @brief 初始化三相 FOC (电流环)
 */
void init();

/**
 * @brief 关闭 FOC 输出
 */
void disableFOC();

/**
 * @brief 电流环
 */
void currentLoop();

} // namespace FOC
} // namespace Control