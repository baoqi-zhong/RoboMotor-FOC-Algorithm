/**
 * @file ThreePhaseFOC.cpp
 * @brief Three-phase Field Oriented Control (FOC) algorithm implementation.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "ThreePhaseFOC.hpp"
#include "MotorControl.hpp"
#include "ADC.hpp"
#include "CordicHelper.hpp"
#include "Encoder.hpp"
#include "CordicHelper.hpp"
#include "tim.h"
#include "Math.hpp"
#include "stdint.h"

namespace Control
{
namespace FOC
{
MotorConfig motorConfig;
FOCConfig focConfig;

float Q_rsqrt( float number )
{
	uint32_t i;
	float x2, y;
	constexpr float threehalfs = 1.5f;

	x2 = number * 0.5f;
	y  = number;
	__builtin_memcpy(&i, &y, sizeof(i));
	i  = 0x5f3759dfU - ( i >> 1 );
	__builtin_memcpy(&y, &i, sizeof(y));
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
//      y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

	return y;
}

uint8_t sector = 0;
float X = 0, Y = 0;
uint32_t dutyCycleA = 0, dutyCycleB = 0, dutyCycleC = 0;
void setPhraseVoltage(float Ualpha, float Ubeta)
{
    float dutyCycleA = 0.0f;
    float dutyCycleB = 0.0f;
    float dutyCycleC = 0.0f;

    float scaleSquare = Ualpha * Ualpha + Ubeta * Ubeta;
    if(scaleSquare > 1)
    {
        float scaleRatio = Q_rsqrt(scaleSquare);
        Ualpha *= scaleRatio;
        Ubeta  *= scaleRatio;
    }
    // TODO? alpha beta 限幅

    // 六边形内接圆, 保证 X + Y <= 1
    Ualpha *= SQRT3_OVER_2;
    Ubeta *= SQRT3_OVER_2;
    
    float BETA_MUL_2_OVER_SQRT3 =                            Ubeta * TWO_OVER_SQRT3;
    float ALPHA_PLUS_BETA_MUL_1_OVER_SQRT3 =        Ualpha + Ubeta * ONE_OVER_SQRT3;
    float MINUS_ALPHA_PLUS_BETA_MUL_1_OVER_SQRT3 =- Ualpha + Ubeta * ONE_OVER_SQRT3;
    
    // 顺序: 123456
    if(Ubeta >= 0.0f)
    {
        // 1, 2, 3 象限
        if(Ubeta * ONE_OVER_SQRT3 < Ualpha)
            sector = 1;
        else if (-Ubeta * ONE_OVER_SQRT3 < Ualpha)
            sector = 2;
        else
            sector = 3;
    }
    else
    {
        // 4, 5, 6 象限
        if(Ubeta * ONE_OVER_SQRT3 > Ualpha)
            sector = 4;
        else if (-Ubeta * ONE_OVER_SQRT3 > Ualpha)
            sector = 5;
        else
            sector = 6;
    }

    
    switch (sector)
    {
    case 1:
        X = - MINUS_ALPHA_PLUS_BETA_MUL_1_OVER_SQRT3;
        Y =   BETA_MUL_2_OVER_SQRT3;
        dutyCycleA = (1 + X + Y) / 2;
        dutyCycleB = (1 - X + Y) / 2;
        dutyCycleC = (1 - X - Y) / 2;
        break;
    case 2:
        X =   ALPHA_PLUS_BETA_MUL_1_OVER_SQRT3;
        Y =   MINUS_ALPHA_PLUS_BETA_MUL_1_OVER_SQRT3;
        dutyCycleA = (1 + X - Y) / 2;
        dutyCycleB = (1 + X + Y) / 2;
        dutyCycleC = (1 - X - Y) / 2;
        break;
    case 3:
        X =   BETA_MUL_2_OVER_SQRT3;
        Y = - ALPHA_PLUS_BETA_MUL_1_OVER_SQRT3;
        dutyCycleA = (1 - X - Y) / 2;
        dutyCycleB = (1 + X + Y) / 2;
        dutyCycleC = (1 - X + Y) / 2;
        break;
    case 4:
        X =   MINUS_ALPHA_PLUS_BETA_MUL_1_OVER_SQRT3;
        Y = - BETA_MUL_2_OVER_SQRT3;
        dutyCycleA = (1 - X - Y) / 2;
        dutyCycleB = (1 + X - Y) / 2;
        dutyCycleC = (1 + X + Y) / 2;
        break;
    case 5:
        X = - ALPHA_PLUS_BETA_MUL_1_OVER_SQRT3;
        Y = - MINUS_ALPHA_PLUS_BETA_MUL_1_OVER_SQRT3;
        dutyCycleA = (1 - X + Y) / 2;
        dutyCycleB = (1 - X - Y) / 2;
        dutyCycleC = (1 + X + Y) / 2;
        break;
    case 6:
        X = - BETA_MUL_2_OVER_SQRT3;
        Y =   ALPHA_PLUS_BETA_MUL_1_OVER_SQRT3;
        dutyCycleA = (1 + X + Y) / 2;
        dutyCycleB = (1 - X - Y) / 2;
        dutyCycleC = (1 + X - Y) / 2;
        break;
    }

    Boards::setTimerPWMDutyCycle(dutyCycleA, dutyCycleB, dutyCycleC);
}

// 电流环
float measuredIalpha = 0.0f;
float measuredIbeta = 0.0f;
float measuredIq = 0.0f;
float measuredId = 0.0f;
float outputUq = 0.0f;
float outputUd = 0.0f;
float outputUqWithFeedForward = 0;
float outputUdWithFeedForward = 0;
float outputUalpha = 0;
float outputUbeta = 0;
int16_t outputAngle = 0;

// 前馈项
float backwardEMF = 0;
float outputUqFeedForward = 0.0f;
float outputUdFeedForward = 0.0f;
Utils::LPF measuredIqLPF(0.01f);
Utils::LPF measuredIdLPF(0.01f);
float measuredIqFiltered = 0.0f;
float measuredIdFiltered = 0.0f;

IncrementalPID IqPID;
IncrementalPID IdPID;

// 开环速度
float openloopPosition = 0.0f;
float openloopVelocity = 0.0f;
float openloopAcceleration = 80.0f;

void disableFOC()
{
    Boards::stopTimerPWM();

    MotorControl::motorControlStatus.enableFOCOutput = 0;
    MotorControl::motorControlStatus.state = MotorControl::MotorControlState::Stop;
}

// 基准电流, 在计算 Park 变换的时候调用 cordic 前的缩放值. 最大电流不应超过此数
namespace
{
constexpr float CurrentCordicBase = 25.0f;
constexpr uint32_t TimerInputClockHz = 170000000U;
}

void setMotorConfig(const MotorConfig* config)
{
    assert_param(config != nullptr);
    motorConfig = *config;
}

void setFOCConfig(const FOCConfig* config)
{
    assert_param(config != nullptr);
    focConfig = *config;
    focConfig.timerPeriod = TimerInputClockHz / (uint32_t)(focConfig.currentLoopFreq) / 2;
    __HAL_TIM_SetAutoreload(&htim1, focConfig.timerPeriod);
}

// 必须在下一次 ADC 采样触发前跑完电流环
// 暂时不考虑 ADC 的 latency 问题
void currentLoop()
{
    // Clarke 变换
    // measuredIalpha = measuredIA - 0.5f * measuredIB - 0.5f * measuredIphaseC;
    // measuredIbeta = SQRT3_OVER_2 * (measuredIB - measuredIphaseC);
    // 等幅值形式
    measuredIalpha = Sensor::ADC::analogValues.measuredIA;
    measuredIbeta = ONE_OVER_SQRT3 * (Sensor::ADC::analogValues.measuredIA + 2.0f * Sensor::ADC::analogValues.measuredIB);

    // Park 变换
    // measuredId = measuredIalpha * cosf(realAngle) + measuredIbeta * sinf(realAngle);
    // measuredIq = measuredIalpha * sinf(realAngle) - measuredIbeta * cosf(realAngle);
    float cordicOutputSinMulIalpha;
    float cordicOutputCosMulIalpha;
    hcordic.Instance->WDATA = (Utils::CordicHelper::singleFloatToCordic15(measuredIalpha / CurrentCordicBase) << 16) | (Sensor::Encoder::encoderStatus.Q16_electricAngle & 0xFFFF);
    Utils::CordicHelper::cordic15ToDualFloat((int32_t)(hcordic.Instance->RDATA), &cordicOutputSinMulIalpha, &cordicOutputCosMulIalpha);

    float cordicOutputSinMulIbeta;
    float cordicOutputCosMulIbeta;
    hcordic.Instance->WDATA = (Utils::CordicHelper::singleFloatToCordic15(measuredIbeta / CurrentCordicBase) << 16) | (Sensor::Encoder::encoderStatus.Q16_electricAngle & 0xFFFF);
    Utils::CordicHelper::cordic15ToDualFloat((int32_t)(hcordic.Instance->RDATA), &cordicOutputSinMulIbeta, &cordicOutputCosMulIbeta);

    measuredId = (cordicOutputCosMulIalpha + cordicOutputSinMulIbeta) * CurrentCordicBase;
    measuredIq = (-cordicOutputSinMulIalpha + cordicOutputCosMulIbeta) * CurrentCordicBase;

    if(MotorControl::motorControlStatus.enableFOCOutput == 0)
    {
        // 有待斟酌 到底是 set 个 0 的电压还是切换到高阻态.
        // setPhraseVoltage(0, 0);
        openloopPosition = 0.0f;
        openloopVelocity = 0.0f;
        return;
    }

    float outputLimitVoltage = Sensor::ADC::analogValues.Vbus;
    if(focConfig.FOCControlMode == FOCControlMode_t::CURRENT_TOURQUE_CONTROL)
    {
        // 电流 PI
        outputUq = IqPID(MotorControl::motorControlStatus.targetIq, measuredIq);
        outputUd = IdPID(MotorControl::motorControlStatus.targetId, measuredId);

        /*
        float v_d_ff = (1.0f * controller->i_d_ref * R_PHASE - controller->dtheta_elec * L_Q * controller->i_q); // feed-forward voltages
        float v_q_ff = (1.0f * controller->i_q_ref * R_PHASE + controller->dtheta_elec * (L_D * controller->i_d + 1.0f * WB));
        */
        // 反电动势前馈
        backwardEMF = Sensor::Encoder::encoderStatus.RAD_shaftAngularVelocity * RAD_PER_S_TO_RPM_RATIO / motorConfig.kv;
        // DQ 解耦前馈要滤波
        measuredIqFiltered = measuredIqLPF(measuredIq);
        measuredIdFiltered = measuredIdLPF(measuredId);

        outputUqFeedForward = motorConfig.phaseResistance * MotorControl::motorControlStatus.targetIq + measuredIdFiltered * Sensor::Encoder::encoderStatus.RAD_electricAngularVelocity * motorConfig.phaseInductance + backwardEMF;
        outputUdFeedForward = motorConfig.phaseResistance * MotorControl::motorControlStatus.targetId - measuredIqFiltered * Sensor::Encoder::encoderStatus.RAD_electricAngularVelocity * motorConfig.phaseInductance;

        if(outputUqFeedForward > outputLimitVoltage)
            outputUqFeedForward = outputLimitVoltage;
        else if(outputUqFeedForward < -outputLimitVoltage)
            outputUqFeedForward = -outputLimitVoltage;
        if(outputUdFeedForward > outputLimitVoltage)
            outputUdFeedForward = outputLimitVoltage;
        else if(outputUdFeedForward < -outputLimitVoltage)
            outputUdFeedForward = -outputLimitVoltage;

        outputUqWithFeedForward = outputUq + outputUqFeedForward;
        outputUdWithFeedForward = outputUd + outputUdFeedForward;

        // // 挤压限幅
        // if(outputUqWithFeedForward > outputLimitVoltage)
        // {
        //     IqPID.setOutput(outputUq - (outputUqWithFeedForward - outputLimitVoltage));
        //     outputUqWithFeedForward = outputLimitVoltage;
        // }
        // else if(outputUqWithFeedForward < -outputLimitVoltage)
        // {
        //     IqPID.setOutput(outputUq - (outputUqWithFeedForward + outputLimitVoltage));
        //     outputUqWithFeedForward = -outputLimitVoltage;
        // }
        
        // if(outputUdWithFeedForward > outputLimitVoltage)
        // {
        //     IdPID.setOutput(outputUd - (outputUdWithFeedForward - outputLimitVoltage));
        //     outputUdWithFeedForward = outputLimitVoltage;
        // }
        // else if(outputUdWithFeedForward < -outputLimitVoltage)
        // {
        //     IdPID.setOutput(outputUd - (outputUdWithFeedForward + outputLimitVoltage));
        //     outputUdWithFeedForward = -outputLimitVoltage;
        // }
        outputAngle = Sensor::Encoder::encoderStatus.Q16_electricAngle;
    }

    else if(focConfig.FOCControlMode == FOCControlMode_t::VOLTAGE_TOURQUE_CONTROL)
    {
        // 不使用电流采样, 直接使用前馈电压控制

        // 反电动势前馈
        backwardEMF = Sensor::Encoder::encoderStatus.RAD_shaftAngularVelocity * RAD_PER_S_TO_RPM_RATIO / motorConfig.kv;
        // DQ 解耦前馈要滤波
        measuredIqFiltered = measuredIqLPF(measuredIq);
        measuredIdFiltered = measuredIdLPF(measuredId);

        outputUqWithFeedForward = motorConfig.phaseResistance * MotorControl::motorControlStatus.targetIq + measuredIdFiltered * Sensor::Encoder::encoderStatus.RAD_electricAngularVelocity * motorConfig.phaseInductance + backwardEMF;
        outputUdWithFeedForward = motorConfig.phaseResistance * MotorControl::motorControlStatus.targetId - measuredIqFiltered * Sensor::Encoder::encoderStatus.RAD_electricAngularVelocity * motorConfig.phaseInductance;

        if(outputUqWithFeedForward > outputLimitVoltage)
            outputUqWithFeedForward = outputLimitVoltage;
        else if(outputUqWithFeedForward < -outputLimitVoltage)
            outputUqWithFeedForward = -outputLimitVoltage;
        if(outputUdWithFeedForward > outputLimitVoltage)
            outputUdWithFeedForward = outputLimitVoltage;
        else if(outputUdWithFeedForward < -outputLimitVoltage)
            outputUdWithFeedForward = -outputLimitVoltage;

        outputAngle = Sensor::Encoder::encoderStatus.Q16_electricAngle;
    }

    outputUqWithFeedForward = CLAMP(outputUqWithFeedForward, -outputLimitVoltage, outputLimitVoltage);
    outputUdWithFeedForward = CLAMP(outputUdWithFeedForward, -outputLimitVoltage, outputLimitVoltage);

    // Circular Limitation. 防止 Uq Ud 过大, 超出 PWM 可以提供的电压

    // 电角度补偿. 因为会先写入 shadow register, PWM 结果真正生效是在 1.5 周期之后. 所以此处需要补偿 1.5 周期
    outputAngle += (int16_t)(Sensor::Encoder::encoderStatus.Q16_deltaElectricAngleLPF * 1.5f);

    // Inverse Park Transform
    // outputUalpha = outputUd * cosf(outputAngle) - outputUq * sinf(outputAngle);
    // outputUbeta = outputUd * sinf(outputAngle) + outputUq * cosf(outputAngle);
    static float cordicOutputSinMulUq;
    static float cordicOutputCosMulUq;
    hcordic.Instance->WDATA = (Utils::CordicHelper::singleFloatToCordic15(outputUqWithFeedForward / Sensor::ADC::analogValues.Vbus) << 16) | (outputAngle & 0xFFFF);
    Utils::CordicHelper::cordic15ToDualFloat((int32_t)(hcordic.Instance->RDATA), &cordicOutputSinMulUq, &cordicOutputCosMulUq);

    static float cordicOutputSinMulUd;
    static float cordicOutputCosMulUd;
    hcordic.Instance->WDATA = (Utils::CordicHelper::singleFloatToCordic15(outputUdWithFeedForward / Sensor::ADC::analogValues.Vbus) << 16) | (outputAngle & 0xFFFF);
    Utils::CordicHelper::cordic15ToDualFloat((int32_t)(hcordic.Instance->RDATA), &cordicOutputSinMulUd, &cordicOutputCosMulUd);

    outputUalpha = (cordicOutputCosMulUd - cordicOutputSinMulUq);
    outputUbeta = (cordicOutputSinMulUd + cordicOutputCosMulUq);

    setPhraseVoltage(outputUalpha, outputUbeta);
}

} // namespace FOC
} // namespace Control
