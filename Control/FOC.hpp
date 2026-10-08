/**
 * @file FOC.hpp
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

#include "Trigonometric.hpp"
#include "IncrementalPID.hpp"
#include "LPF.hpp"
#include "Math.hpp"

#include "stdint.h"
#include "string.h"

namespace Control::FOC
{
enum class TorqueControlMode : uint8_t
{
    CURRENT_TOURQUE_CONTROL = 0,
    VOLTAGE_TOURQUE_CONTROL,
};

struct FOCConfigStatic
{
    float currentLoopFreq;                  /* 电流环频率, Hz */
    TorqueControlMode torqueControlMode;    /* 电流环控制模式 */
    float CurrentCordicBase;                /* 电流环计算时的基准电流, A */

    float phaseResistance;                  /* 相电阻, Ohm */
    float phaseInductance;                  /* 相电感, H */
    float kv;                               /* 电机速度常数, RPM/V */

    PIDParameters_t IqPIDParameters;        /* q 轴电流环 PID 参数 */
    PIDParameters_t IdPIDParameters;        /* d 轴电流环 PID 参数 */
};

template<FOCConfigStatic focConfigStatic>
class FOC
{
public:
    /* 高频跑电流环时的输入 */
    struct FOCInput
    {
        float measuredIA    = 0.0f;     /* 测量的 A 相电流, A */
        float measuredIB    = 0.0f;     /* 测量的 B 相电流, A */
        float measuredIC    = 0.0f;     /* 测量的 C 相电流, A */
        float measuredVbus  = 0.0f;     /* 测量的总线电压, V */

        float targetIq      = 0.0f;     /* 目标的 q 轴电流, A */
        float targetId      = 0.0f;     /* 目标的 d 轴电流, A */

        uint16_t Q16_electricAngle          = 0;        /* 测量角度转换为电角度, Q16 定点数 */
        float RAD_electricAngularVelocity   = 0.0f;     /* 测量的电机角速度, RAD/S */
        float RAD_shaftAngularVelocity      = 0.0f;     /* 测量的输出轴角速度, RAD/S */
    };

    struct FOCOutput
    {
        float dutyA         = 0.0f;     /* 输出的 A 相占空比, 0~1 */
        float dutyB         = 0.0f;     /* 输出的 B 相占空比, 0~1 */
        float dutyC         = 0.0f;     /* 输出的 C 相占空比, 0~1 */
    };

    void setPhaseVoltage(float Ualpha, float Ubeta);

    /**
     * @brief 电流环更新调用前需要外部填充 focInput，运行后会更新 focOutput
     */
    void currentLoopUpdate();

    bool enableFOCOutput            = true;     /* 是否使能 FOC 输出，失能时仍会计算相关参数 */
    FOCInput focInput;
    FOCOutput focOutput;

    Control::IncrementalPID IqPID   {focConfigStatic.IqPIDParameters};
    Control::IncrementalPID IdPID   {focConfigStatic.IdPIDParameters};

private:        
    float measuredIalpha            = 0.0f;
    float measuredIbeta             = 0.0f;
    float measuredIq                = 0.0f;
    float measuredId                = 0.0f;

    /* 前馈项 */
    float backwardEMF = 0;
    float outputUqFeedForward       = 0.0f;
    float outputUdFeedForward       = 0.0f;
    Utils::LPF measuredIqLPF        {0.01f};
    Utils::LPF measuredIdLPF        {0.01f};
    float measuredIqFiltered        = 0.0f;
    float measuredIdFiltered        = 0.0f;

    float outputUq                  = 0.0f;
    float outputUd                  = 0.0f;
    float outputUqWithFeedForward   = 0.0f;
    float outputUdWithFeedForward   = 0.0f;
    float outputUalpha              = 0.0f;
    float outputUbeta               = 0.0f;
    int16_t outputAngle             = 0;
};

inline float q16ToRadians(uint16_t q16Angle)
{
    return static_cast<float>(q16Angle) * TWO_PI / 65536.0f;
}

inline float fastInvSquareRoot(float number)
{
    uint32_t i;
    float x2, y;
    constexpr float threehalfs = 1.5f;

    x2 = number * 0.5f;
    y  = number;
    memcpy(&i, &y, sizeof(i));
    i  = 0x5f3759dfU - (i >> 1);
    memcpy(&y, &i, sizeof(y));
    y  = y * (threehalfs - (x2 * y * y));

    return y;
}

template<FOCConfigStatic focConfigStatic>
void FOC<focConfigStatic>::setPhaseVoltage(float Ualpha, float Ubeta)
{
    uint32_t sector = 0;
    float X = 0, Y = 0;

    float scaleSquare = Ualpha * Ualpha + Ubeta * Ubeta;
    if(scaleSquare > 1)
    {
        float scaleRatio = fastInvSquareRoot(scaleSquare);
        Ualpha *= scaleRatio;
        Ubeta  *= scaleRatio;
    }
    // TODO? alpha beta 闄愬箙

    // 鍏竟褰㈠唴鎺ュ渾, 淇濊瘉 X + Y <= 1
    Ualpha *= SQRT3_OVER_2;
    Ubeta *= SQRT3_OVER_2;
    
    float BETA_MUL_2_OVER_SQRT3 =                            Ubeta * TWO_OVER_SQRT3;
    float ALPHA_PLUS_BETA_MUL_1_OVER_SQRT3 =        Ualpha + Ubeta * ONE_OVER_SQRT3;
    float MINUS_ALPHA_PLUS_BETA_MUL_1_OVER_SQRT3 =- Ualpha + Ubeta * ONE_OVER_SQRT3;
    
    // 椤哄簭: 123456
    if(Ubeta >= 0.0f)
    {
        // 1, 2, 3 璞￠檺
        if(Ubeta * ONE_OVER_SQRT3 < Ualpha)
            sector = 1;
        else if (-Ubeta * ONE_OVER_SQRT3 < Ualpha)
            sector = 2;
        else
            sector = 3;
    }
    else
    {
        // 4, 5, 6 璞￠檺
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
        focOutput.dutyA = (1 + X + Y) / 2;
        focOutput.dutyB = (1 - X + Y) / 2;
        focOutput.dutyC = (1 - X - Y) / 2;
        break;
    case 2:
        X =   ALPHA_PLUS_BETA_MUL_1_OVER_SQRT3;
        Y =   MINUS_ALPHA_PLUS_BETA_MUL_1_OVER_SQRT3;
        focOutput.dutyA = (1 + X - Y) / 2;
        focOutput.dutyB = (1 + X + Y) / 2;
        focOutput.dutyC = (1 - X - Y) / 2;
        break;
    case 3:
        X =   BETA_MUL_2_OVER_SQRT3;
        Y = - ALPHA_PLUS_BETA_MUL_1_OVER_SQRT3;
        focOutput.dutyA = (1 - X - Y) / 2;
        focOutput.dutyB = (1 + X + Y) / 2;
        focOutput.dutyC = (1 - X + Y) / 2;
        break;
    case 4:
        X =   MINUS_ALPHA_PLUS_BETA_MUL_1_OVER_SQRT3;
        Y = - BETA_MUL_2_OVER_SQRT3;
        focOutput.dutyA = (1 - X - Y) / 2;
        focOutput.dutyB = (1 + X - Y) / 2;
        focOutput.dutyC = (1 + X + Y) / 2;
        break;
    case 5:
        X = - ALPHA_PLUS_BETA_MUL_1_OVER_SQRT3;
        Y = - MINUS_ALPHA_PLUS_BETA_MUL_1_OVER_SQRT3;
        focOutput.dutyA = (1 - X + Y) / 2;
        focOutput.dutyB = (1 - X - Y) / 2;
        focOutput.dutyC = (1 + X + Y) / 2;
        break;
    case 6:
        X = - BETA_MUL_2_OVER_SQRT3;
        Y =   ALPHA_PLUS_BETA_MUL_1_OVER_SQRT3;
        focOutput.dutyA = (1 + X + Y) / 2;
        focOutput.dutyB = (1 - X - Y) / 2;
        focOutput.dutyC = (1 + X - Y) / 2;
        break;
    }
}

template<FOCConfigStatic focConfigStatic>
void FOC<focConfigStatic>::currentLoopUpdate()
{
    // Clarke 变换
    // measuredIalpha = measuredIA - 0.5f * measuredIB - 0.5f * measuredIphaseC;
    // measuredIbeta = SQRT3_OVER_2 * (measuredIB - measuredIphaseC);
    // 等幅值形式
    measuredIalpha = focInput.measuredIA;
    measuredIbeta = ONE_OVER_SQRT3 * (focInput.measuredIA + 2.0f * focInput.measuredIB);

    // Park 变换
    // measuredId = measuredIalpha * cosf(realAngle) + measuredIbeta * sinf(realAngle);
    // measuredIq = measuredIalpha * sinf(realAngle) - measuredIbeta * cosf(realAngle);
    float electricAngle = q16ToRadians(focInput.Q16_electricAngle);
    float cordicOutputSinMulIalpha;
    float cordicOutputCosMulIalpha;
    Utils::Trigonometric::sinCosMultiply(measuredIalpha / focConfigStatic.CurrentCordicBase, electricAngle, &cordicOutputSinMulIalpha, &cordicOutputCosMulIalpha);

    float cordicOutputSinMulIbeta;
    float cordicOutputCosMulIbeta;
    Utils::Trigonometric::sinCosMultiply(measuredIbeta / focConfigStatic.CurrentCordicBase, electricAngle, &cordicOutputSinMulIbeta, &cordicOutputCosMulIbeta);

    measuredId = (cordicOutputCosMulIalpha + cordicOutputSinMulIbeta) * focConfigStatic.CurrentCordicBase;
    measuredIq = (-cordicOutputSinMulIalpha + cordicOutputCosMulIbeta) * focConfigStatic.CurrentCordicBase;

    if(!enableFOCOutput)
    {
        // 有待斟酌：到底是 set 为 0 电压还是切换到高阻态
        // setPhaseVoltage(0, 0);
        return;
    }

    float outputLimitVoltage = focInput.measuredVbus;
    if(focConfigStatic.torqueControlMode == TorqueControlMode::CURRENT_TOURQUE_CONTROL)
    {
        // 电流 PI
        outputUq = IqPID(focInput.targetIq, measuredIq);
        outputUd = IdPID(focInput.targetId, measuredId);

        /*
        float v_d_ff = (1.0f * controller->i_d_ref * R_PHASE - controller->dtheta_elec * L_Q * controller->i_q); // feed-forward voltages
        float v_q_ff = (1.0f * controller->i_q_ref * R_PHASE + controller->dtheta_elec * (L_D * controller->i_d + 1.0f * WB));
        */
        // 反电动势前馈
        backwardEMF = focInput.RAD_shaftAngularVelocity * RAD_PER_S_TO_RPM_RATIO / focConfigStatic.kv;
        // DQ 解耦前馈要滤波
        measuredIqFiltered = measuredIqLPF(measuredIq);
        measuredIdFiltered = measuredIdLPF(measuredId);

        outputUqFeedForward = focConfigStatic.phaseResistance * focInput.targetIq + measuredIdFiltered * focInput.RAD_electricAngularVelocity * focConfigStatic.phaseInductance + backwardEMF;
        outputUdFeedForward = focConfigStatic.phaseResistance * focInput.targetId - measuredIqFiltered * focInput.RAD_electricAngularVelocity * focConfigStatic.phaseInductance;

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

        // // 鎸ゅ帇闄愬箙
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
        outputAngle = focInput.Q16_electricAngle;
    }

    else if(focConfigStatic.torqueControlMode == TorqueControlMode::VOLTAGE_TOURQUE_CONTROL)
    {
        // 不使用电流采样，直接使用前馈电压控制

        // 反电动势前馈
        backwardEMF = focInput.RAD_shaftAngularVelocity * RAD_PER_S_TO_RPM_RATIO / focConfigStatic.kv;
        // DQ 解耦前馈要滤波
        measuredIqFiltered = measuredIqLPF(measuredIq);
        measuredIdFiltered = measuredIdLPF(measuredId);

        outputUqWithFeedForward = focConfigStatic.phaseResistance * focInput.targetIq + measuredIdFiltered * focInput.RAD_electricAngularVelocity * focConfigStatic.phaseInductance + backwardEMF;
        outputUdWithFeedForward = focConfigStatic.phaseResistance * focInput.targetId - measuredIqFiltered * focInput.RAD_electricAngularVelocity * focConfigStatic.phaseInductance;

        if(outputUqWithFeedForward > outputLimitVoltage)
            outputUqWithFeedForward = outputLimitVoltage;
        else if(outputUqWithFeedForward < -outputLimitVoltage)
            outputUqWithFeedForward = -outputLimitVoltage;
        if(outputUdWithFeedForward > outputLimitVoltage)
            outputUdWithFeedForward = outputLimitVoltage;
        else if(outputUdWithFeedForward < -outputLimitVoltage)
            outputUdWithFeedForward = -outputLimitVoltage;

        outputAngle = focInput.Q16_electricAngle;
    }

    outputUqWithFeedForward = CLAMP(outputUqWithFeedForward, -outputLimitVoltage, outputLimitVoltage);
    outputUdWithFeedForward = CLAMP(outputUdWithFeedForward, -outputLimitVoltage, outputLimitVoltage);

    // Circular Limitation. 防止 Uq Ud 过大，超出 PWM 可以提供的电压
    // 电角度补偿：因为会先写入 shadow register，PWM 结果真正生效是在 1.5 周期之后，所以此处需要补偿 1.5 周期
    // outputAngle += (int16_t)(focInput.Q16_deltaElectricAngleLPF * 1.5f);

    // Inverse Park Transform
    // outputUalpha = outputUd * cosf(outputAngle) - outputUq * sinf(outputAngle);
    // outputUbeta = outputUd * sinf(outputAngle) + outputUq * cosf(outputAngle);
    float outputAngleRadians = q16ToRadians(outputAngle);
    static float cordicOutputSinMulUq;
    static float cordicOutputCosMulUq;
    Utils::Trigonometric::sinCosMultiply(outputUqWithFeedForward / focInput.measuredVbus, outputAngleRadians, &cordicOutputSinMulUq, &cordicOutputCosMulUq);

    static float cordicOutputSinMulUd;
    static float cordicOutputCosMulUd;
    Utils::Trigonometric::sinCosMultiply(outputUdWithFeedForward / focInput.measuredVbus, outputAngleRadians, &cordicOutputSinMulUd, &cordicOutputCosMulUd);

    outputUalpha = (cordicOutputCosMulUd - cordicOutputSinMulUq);
    outputUbeta = (cordicOutputSinMulUd + cordicOutputCosMulUq);

    setPhaseVoltage(outputUalpha, outputUbeta);
}
} // namespace Control::FOC


