/**
 * @file Encoder.hpp
 * @brief Encoder configuration and status definitions.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#pragma once



#include "MA732.hpp"
#include "M3508_LinerHallEncoder.hpp"
#include "LPF.hpp"

namespace Sensor
{
namespace Encoder
{
enum class EncoderDriverType : uint8_t
{
    None = 0,
    MA732 = 1,
    M3508_LinerHallEncoder = 2
};

struct EncoderStatus
{
    // Private
    // Encoder 本身无量纲, 不应在外部使用
    uint16_t    Q16_encoder                     = 0;
    uint16_t    Q16_lastEncoder                 = 0;
    int32_t     Q16_accumulatedEncoder          = 0;

    int16_t     Q16_encoderDifference           = 0;
    float       F16_encoderDifferenceLPF   = 0.0f;

    // Public
    int16_t     Q16_electricAngle               = 0;
    int16_t     Q16_deltaElectricAngleLPF       = 0;    // 电角度每采样周期
    int32_t     Q16_electricAngularVelocity     = 0;    // 电角度每秒
    float       RAD_electricAngularVelocity     = 0.0f; // 电角速度, rad/s

    float       RAD_accumulatedShaftAngle       = 0.0f; // 输出轴累积角度, rad
    float       RAD_shaftAngularVelocity        = 0.0f; // 输出轴角速度, rad/s

    float       RPM_shaftAngularVelocity        = 0.0f; // 输出轴角速度, rpm
};

struct EncoderConfig
{
    EncoderDriverType driverType        = EncoderDriverType::None;
    uint16_t encoderZeroOffset          = 0;        // 编码器零点偏移
    uint8_t encoderDirection            = 1;        // 编码器方向
    int8_t encoderCompensationTable[64] = {0};      // 编码器偏移表, 用于校正编码器误差
    float encoderCompensationGain       = 1.0f;     // 编码器偏移增益, 用于放大或缩小校正值
    uint8_t enableEncoderCompensation   = 1;

    float encoderDelayTime              = 0.0f;     // 单位 us
    float encoderDifferenceLPFAlpha      = 0.01f;    // 编码器差分的低通滤波系数, 用于滤除编码器差分的高频噪声
};

extern EncoderConfig encoderConfig;
extern EncoderStatus encoderStatus;

/**
 * @brief Normalizes an angle to be within the range [0, 2*pi).
 * 
 * @param _angle The angle to be normalized.
 * @return The normalized angle.
 */
float normalizeAngleZeroToTwoPi(float _angle);

/**
 * @brief Normalizes an angle to be within the range [-pi, pi).
 * 
 * @param _angle The angle to be normalized.
 * @return The normalized angle.
 */
float normalizeAngleNegPiToPi(float _angle);


int16_t getCompensation(uint16_t rawAngle);
uint16_t getEncoderAfterCompensation(uint16_t rawAngle);

void setConfig(EncoderConfig* config);

/**
 * @brief Initializes the encoder.
 */
void init();

/**
 * @brief Reads data from the encoder in a blocking manner.
 */
void readBlocking();

/**
 * @brief Sets the electric angle to 0.
 */
void setZeroSoftware();

/**
 * @brief Write encoder offset to sensor registers (if possible).
 */
void setZeroHardware();

} // namespace Encoder
} // namespace Sensor