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

#include "LPF.hpp"
#include "Math.hpp"

#include "stdint.h"

namespace Sensor
{
struct EncoderConfigStatic
{
    float updateFrequency               = 1000.0f;
    float shaftReductionRatio           = 1.0f;
    float electricAngleReductionRatio   = 1.0f;
};

struct EncoderConfig
{
    uint16_t zeroOffset             = 0;
    uint8_t direction               = 1;
    int8_t compensationTable[64]    = {0};
    float compensationGain          = 1.0f;
    uint8_t enableCompensation      = 1;
    float delayTime                 = 0.0f;
    float LPFAlpha                  = 0.01f;
};

template<EncoderConfigStatic encoderConfigStatic>
class Encoder
{
public:
    Encoder(const EncoderConfig& encoderConfig_ = EncoderConfig()) : encoderConfig(encoderConfig_), encoderDifferenceLPF(encoderConfig.LPFAlpha) {}
    
    int16_t getCompensation(uint16_t rawAngle);
    uint16_t getEncoderAfterCompensation(uint16_t rawAngle);
    void init(uint16_t Q16_encoder_);
    void update(uint16_t Q16_encoder_);
    void setZeroSoftware();

    EncoderConfig encoderConfig;
    Utils::LPF encoderDifferenceLPF;

    int16_t     Q16_electricAngle               = 0;
    int16_t     Q16_deltaElectricAngleLPF       = 0;
    int32_t     Q16_electricAngularVelocity     = 0;
    float       RAD_electricAngularVelocity     = 0.0f;

    float       RAD_accumulatedShaftAngle       = 0.0f;
    float       RAD_shaftAngularVelocity        = 0.0f;
    float       RPM_shaftAngularVelocity        = 0.0f;

private:
    bool        initialized                     = false;

    uint16_t    Q16_encoder                     = 0;
    uint16_t    Q16_lastEncoder                 = 0;
    int32_t     Q16_accumulatedEncoder          = 0;

    int16_t     Q16_encoderDifference           = 0;
    float       F16_encoderDifferenceLPF        = 0.0f;
};

template<EncoderConfigStatic encoderConfigStatic>
int16_t Encoder<encoderConfigStatic>::getCompensation(uint16_t rawAngle)
{
    uint8_t index = rawAngle >> 10;
    uint8_t nextIndex = (index + 1) % 64;
    uint16_t remainder = rawAngle & 0x3FF;

    int16_t compensation = encoderConfig.compensationTable[index];
    int16_t nextCompensation = encoderConfig.compensationTable[nextIndex];
    compensation += (int32_t)(nextCompensation - compensation) * (int32_t)remainder / 1024;
    return compensation * encoderConfig.compensationGain;
}

template<EncoderConfigStatic encoderConfigStatic>
uint16_t Encoder<encoderConfigStatic>::getEncoderAfterCompensation(uint16_t rawAngle)
{
    if(encoderConfig.enableCompensation == 0)
        return rawAngle;

    return rawAngle - getCompensation(rawAngle);
}

template<EncoderConfigStatic encoderConfigStatic>
void Encoder<encoderConfigStatic>::init(uint16_t Q16_encoder_)
{
    uint16_t compensatedEncoder = getEncoderAfterCompensation(Q16_encoder_ - encoderConfig.zeroOffset);
    Q16_encoder = encoderConfig.direction ? compensatedEncoder : -compensatedEncoder;
    Q16_lastEncoder = Q16_encoder;
    Q16_accumulatedEncoder = Q16_encoder;
    initialized = true;

    Q16_encoderDifference = 0;
    F16_encoderDifferenceLPF = 0.0f;
    Q16_electricAngle = 0;
    Q16_deltaElectricAngleLPF = 0;
    Q16_electricAngularVelocity = 0;
    RAD_electricAngularVelocity = 0.0f;
    RAD_accumulatedShaftAngle = 0.0f;
    RAD_shaftAngularVelocity = 0.0f;
    RPM_shaftAngularVelocity = 0.0f;
}

template<EncoderConfigStatic encoderConfigStatic>
void Encoder<encoderConfigStatic>::update(uint16_t Q16_encoder_)
{
    uint16_t compensatedEncoder = getEncoderAfterCompensation(Q16_encoder_ - encoderConfig.zeroOffset);
    uint16_t currentEncoder = encoderConfig.direction ? compensatedEncoder : -compensatedEncoder;

    if(!initialized)
    {
        Q16_encoder = currentEncoder;
        Q16_lastEncoder = currentEncoder;
        Q16_accumulatedEncoder = currentEncoder;
        initialized = true;
    }
    else
    {
        Q16_lastEncoder = Q16_encoder;
        Q16_encoder = currentEncoder;
    }

    Q16_encoderDifference = Q16_encoder - Q16_lastEncoder;
    Q16_accumulatedEncoder += Q16_encoderDifference;

    F16_encoderDifferenceLPF = encoderDifferenceLPF(Q16_encoderDifference);

    float RAD_encoderDifferenceLPFFloat = F16_encoderDifferenceLPF / 65536.0f * TWO_PI * encoderConfigStatic.updateFrequency;
    RAD_accumulatedShaftAngle = Q16_accumulatedEncoder / 65536.0f * TWO_PI * encoderConfigStatic.shaftReductionRatio;
    RAD_shaftAngularVelocity = RAD_encoderDifferenceLPFFloat * encoderConfigStatic.shaftReductionRatio;
    RPM_shaftAngularVelocity = RAD_shaftAngularVelocity * RAD_PER_S_TO_RPM_RATIO;
    RAD_electricAngularVelocity = RAD_encoderDifferenceLPFFloat * encoderConfigStatic.electricAngleReductionRatio;
    Q16_electricAngularVelocity = RAD_electricAngularVelocity / TWO_PI * 65536.0f;

    Q16_deltaElectricAngleLPF = F16_encoderDifferenceLPF * encoderConfigStatic.electricAngleReductionRatio;
    int32_t estimateAccumulatedEncoder = Q16_accumulatedEncoder + (int32_t)(F16_encoderDifferenceLPF * (encoderConfig.delayTime * encoderConfigStatic.updateFrequency / 1000000.0f));
    int32_t Q16_electricAngle_ = (int32_t)((estimateAccumulatedEncoder % 65536) * encoderConfigStatic.electricAngleReductionRatio) % 65536;
    if(Q16_electricAngle_ > 32767)
        Q16_electricAngle_ -= 65536;
    else if(Q16_electricAngle_ < -32768)
        Q16_electricAngle_ += 65536;
    Q16_electricAngle = Q16_electricAngle_;
}

template<EncoderConfigStatic encoderConfigStatic>
void Encoder<encoderConfigStatic>::setZeroSoftware()
{
    Q16_accumulatedEncoder        = 0;
    Q16_electricAngle             = 0;
    F16_encoderDifferenceLPF      = 0.0f;
    Q16_encoderDifference         = 0;
    Q16_deltaElectricAngleLPF     = 0;
    Q16_electricAngularVelocity   = 0;
    RAD_electricAngularVelocity   = 0.0f;
    RAD_accumulatedShaftAngle     = 0.0f;
    RAD_shaftAngularVelocity      = 0.0f;
    RPM_shaftAngularVelocity      = 0.0f;
}

} // namespace Sensor
