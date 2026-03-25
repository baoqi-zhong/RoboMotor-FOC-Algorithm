/**
 * @file Calibrator.hpp
 * @brief Calibration status and step management definitions.
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
namespace Calibrator
{
struct CalibratorStatus
{
    /* Step Management */
    int32_t currentAngle            = 0;        /* 定点数, 0~65535 对应 0~360度 */
    int32_t actionStartAngle        = 0;        /* 定点数, 0~65535 对应 0~360度 */
    float currentVoltage            = 0.0f;
    float actionStartVoltage        = 0.0f;
    uint8_t currentStepIndex        = 0;
    uint16_t currentStepElapsedTime = 0;

    /* 用于记录校准过程中采样的状态 */
    float currentError              = 0.0f;
    uint8_t encoderRecordingStatus  = 0;   /* 0: 没有记录, 1: 记录正圈, 2: 记录反圈 */
};

struct M3508_calibrationResult_t
{
    uint16_t currentRawHallA;
    uint16_t currentRawHallB;

    uint16_t maxRawHallA;
    uint16_t minRawHallA;
    uint16_t maxRawHallB;
    uint16_t minRawHallB;

    uint32_t accumulatedHallA;
    uint32_t accumulatedHallB;
    uint32_t numberOfSamples;

    uint32_t averageRawHallA;
    uint32_t averageRawHallB;
    float HallAGain;
    float HallBGain;
};
extern CalibratorStatus calibratorStatus;


struct CalibrationResult
{
    /* 用于判断 TIM1_CH1/2/3 对应的电流传感器是否正确 */
    uint8_t phaseXADCindex[3]      = {0};       /* 记录每个 TIM CH 对应的 ADC index, 用于判断 ADC 对应关系是否正确 */
    int8_t phaseXCurrentSign[3]    = {0};       /* 1:极性正确, 不需要翻转; -1: 极性错误, 需要翻转. 记录每个 TIM CH 采样到的电流值的符号, 用于判断是否需要翻转电流采样极性 */
    float phaseInconsistency        = 0.0f;     /* 百分比, 电机内阻与电流采样硬件误差造成的三通道不一致性, 用于判断采样电路和电机是否正常 */

    float estimatedMotorResistance  = 0.0f;
    float estimatedMotorKV          = 0.0f;
    float estimatedEncoderRatio     = 0.0f;     /* 估计的编码器减速比, 电角度 : 编码器角度 */

    int8_t encoderSign              = 0;        /* 1:编码器方向正确, 不需要翻转; -1: 编码器方向错误, 需要翻转 */
    uint16_t encoderZeroOffset      = 0;        /* 编码器零点偏移 */
    int16_t encoderBias[14]         = {0};
};


enum class CalibrateActionType : uint8_t
{
    /* Generic */
    SLEEP = 0,
    START,
    MOVE_TO_POSITION,
    START_FOC,
    STOP_FOC,
    END,

    /* Step 1*/
    RECORD_CURRENT_VALUES,              /* 记录三相电流 */
    RECORD_ENCODER_VALUES,              /* 记录编码器原始值, 累计值等 */
    CALCULATION_1,                      /* 根据 3 个记录的电流值计算电机相电阻 */

    /* Step 2 */
    FULL_CYCLE_ENCODER_CALIBRATION,     /* 移动到每个极对的位置记录磁编码器角度 */
    RECORD_ENCODER_ERROR_START,         /* 记录编码器误差 */
    RECORD_ENCODER_ERROR_START_REVERSE, /* 反转后记录编码器误差 */
    RECORD_ENCODER_ERROR_END,           /* 记录编码器误差 */
    CALCULATION_2,                      /* 根据正反转数据, 计算磁编码器校准值 */
    SHOW_ENCODER_RESULT,
};

struct CalibrateAction
{
    CalibrateActionType action;
    int32_t angle;     // 角度单位: 定点数
    float voltage;      // 0~24V
    uint16_t duration;  // ms
};

void init();

void update();

} // namespace Calibrator
} // namespace Control