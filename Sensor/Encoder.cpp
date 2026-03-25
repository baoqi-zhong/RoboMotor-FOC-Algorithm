/**
 * @file Encoder.cpp
 * @brief Encoder processing logic and lookup table compensation.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "Encoder.hpp"
#include "ThreePhaseFOC.hpp"
#include "Math.hpp"

namespace Sensor
{
namespace Encoder
{
EncoderConfig encoderConfig;
EncoderStatus encoderStatus;
Utils::LPF encoderDifferenceLPF;


/**
 * 经由 Calibrator::CALCULATION_2 处理后写入的补偿表.
 * 已从"真实位置索引"重映射为"测量位置索引", 可直接用有误差的原始编码器值查表.
 * 长度 64, 单位为编码器 Q16 原始值 (一圈 = 65536).
 */

uint16_t returnZero() { return 0; }

uint16_t (*readEncoderBlocking)() = returnZero;  // 读取编码器的函数指针, 返回值为定点数


// 归一化角度到 [0,2PI]
float normalizeAngleZeroToTwoPi(float _angle)
{
  _angle = FMOD(_angle, TWO_PI);
  return _angle >= 0 ? _angle : (_angle + TWO_PI);  
}

// 归一化角度到 [-PI, PI]
float normalizeAngleNegPiToPi(float _angle)
{
    _angle = FMOD(_angle, TWO_PI);
    if(_angle > PI)
        return _angle - TWO_PI;
    else if(_angle < -PI)
        return _angle + TWO_PI;
    return _angle;  
}

/**
 * 补偿表格: 长度 64 方便计算.
 * 使用 encoderConfig.encoderCompensationTable (由 Calibrator::CALCULATION_2 写入).
 * 表已按测量位置索引排列, 可直接用有误差的原始编码器值 rawAngle 查询.
 * @param rawAngle 减去零点偏移后的原始编码器读取值 (有误差的测量值)
 * @return 需要减去的误差补偿值 (编码器 Q16 原始单位)
 */

float compensationGain = 1.0f;
int16_t compensationPhase = 0;

int16_t getCompensation(uint16_t rawAngle)
{
    rawAngle += compensationPhase;
    uint8_t index = rawAngle >> 10;         /* Index range: [0-63], 65536/64=1024 */
    uint8_t nextIndex = (index + 1) % 64;
    uint16_t remainder = rawAngle & 0x3FF;  /* Remainder range: [0-1023] */

    int16_t compensation = encoderConfig.encoderCompensationTable[index];
    int16_t nextCompensation = encoderConfig.encoderCompensationTable[nextIndex];
    compensation += (int32_t)(nextCompensation - compensation) * (int32_t)remainder / 1024;
    return compensation * compensationGain;
}

uint16_t getEncoderAfterCompensation(uint16_t rawAngle)
{
    if(encoderConfig.enableEncoderCompensation == 0)
        return rawAngle;
    
    // 实测一次迭代就够了
    uint16_t firstCompensationAngle = rawAngle - getCompensation(rawAngle);
    // uint16_t secondCompensationAngle = rawAngle - getCompensation(firstCompensationAngle);

    return firstCompensationAngle;
}

void setConfig(EncoderConfig* config)
{
    encoderConfig = *config;
    assert_param(encoderConfig.driverType >= EncoderDriverType::MA732 && encoderConfig.driverType <= EncoderDriverType::M3508_LinerHallEncoder);
    if(encoderConfig.driverType == EncoderDriverType::MA732)
    {
        readEncoderBlocking = MA732::readBlocking;
    }
    else if(encoderConfig.driverType == EncoderDriverType::M3508_LinerHallEncoder)
    {
        // readEncoderBlocking = M3508_LinerHallEncoder::readBlocking;
    }
    encoderDifferenceLPF.setAlpha(encoderConfig.encoderDifferenceLPFAlpha);
}

void init()
{
    encoderStatus.Q16_encoder = getEncoderAfterCompensation(readEncoderBlocking() - encoderConfig.encoderZeroOffset);
    encoderStatus.Q16_accumulatedEncoder = encoderStatus.Q16_encoder;
}

// 提前于电流环调用, 在 Update Event 触发了 ADC 之后就可以调用.
void readBlocking()
{
    // encoder 本身是无量纲的, 不应该在除了此处之外的任何地方使用.
    encoderStatus.Q16_lastEncoder = encoderStatus.Q16_encoder;
    uint16_t encoder = getEncoderAfterCompensation(readEncoderBlocking() - encoderConfig.encoderZeroOffset);

    if(encoderConfig.encoderDirection == 1)
    {
        encoderStatus.Q16_encoder = encoder;
    }
    else
    {
        encoderStatus.Q16_encoder = - encoder;
    }

    encoderStatus.Q16_encoderDifference = encoderStatus.Q16_encoder - encoderStatus.Q16_lastEncoder;
    encoderStatus.Q16_accumulatedEncoder += encoderStatus.Q16_encoderDifference;

    // encoderDifference 经过 LPF 后, 用作速度估计
    encoderStatus.F16_encoderDifferenceLPF = encoderDifferenceLPF(encoderStatus.Q16_encoderDifference);

    // 引入量纲
    // Shaft
    float RAD_encoderDifferenceLPFFloat = encoderStatus.F16_encoderDifferenceLPF / 65536.0f * TWO_PI * Control::FOC::focConfig.currentLoopFreq;
    encoderStatus.RAD_shaftAngularVelocity = RAD_encoderDifferenceLPFFloat * Control::FOC::motorConfig.shaftReductionRatio;
    encoderStatus.RPM_shaftAngularVelocity = encoderStatus.RAD_shaftAngularVelocity * RAD_PER_S_TO_RPM_RATIO;
    encoderStatus.RAD_electricAngularVelocity = RAD_encoderDifferenceLPFFloat * Control::FOC::motorConfig.electricAngleReductionRatio;

    // Electric
    encoderStatus.Q16_deltaElectricAngleLPF = encoderStatus.F16_encoderDifferenceLPF * Control::FOC::motorConfig.electricAngleReductionRatio;
    // 延迟补偿
    int32_t estimateAccumulatedEncoder = encoderStatus.Q16_accumulatedEncoder + (int32_t)(encoderStatus.F16_encoderDifferenceLPF * (encoderConfig.encoderDelayTime * Control::FOC::focConfig.currentLoopFreq / 1000000.0f));
    uint32_t Q16_electricAngle_ = (int32_t)((estimateAccumulatedEncoder % 65536) * Control::FOC::motorConfig.electricAngleReductionRatio) % 65536;
    if(Q16_electricAngle_ > 32767)
        Q16_electricAngle_ -= 65536;
    encoderStatus.Q16_electricAngle = Q16_electricAngle_;
}

/*
作用: 将当前的 accumulatedEncoder, electricAngle 值设为 0, 断电后不能保存, 是软件虚拟位置回零
*/
void setZeroSoftware()
{
    encoderStatus.Q16_accumulatedEncoder        = 0;
    encoderStatus.Q16_electricAngle             = 0;
    encoderStatus.F16_encoderDifferenceLPF = 0.0f;

    encoderStatus.Q16_encoderDifference         = 0;
    encoderStatus.Q16_deltaElectricAngleLPF     = 0;
    encoderStatus.RAD_accumulatedShaftAngle     = 0.0f;
    encoderStatus.RAD_shaftAngularVelocity      = 0.0f;
}

void setZeroHardware()
{
    #if USE_MA732
        MA732::setZeroHardware();
    #endif
}

} // namespace Encoder
} // namespace Sensor