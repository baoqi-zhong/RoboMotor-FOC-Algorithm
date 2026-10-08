/**
 * @file Trigonometric.cpp
 * @brief Trigonometric backend abstraction.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "Trigonometric.hpp"

#include "Math.hpp"

#if TRIGONOMETRIC_BACKEND == TRIGONOMETRIC_BACKEND_ST_CORDIC
#include "main.h"
#include "cordic.h"
#elif TRIGONOMETRIC_BACKEND == TRIGONOMETRIC_BACKEND_LIBM
extern "C" float sinf(float x);
extern "C" float cosf(float x);
extern "C" float atan2f(float y, float x);
#endif

namespace Utils::Trigonometric
{

#if TRIGONOMETRIC_BACKEND == TRIGONOMETRIC_BACKEND_ST_CORDIC
namespace
{
constexpr float CORDIC_MAX_FLOAT = 0.999969482421875f;
constexpr float CORDIC_MIN_FLOAT = -1.0f;

float clampCordicInput(float value)
{
    return CLAMP(value, CORDIC_MIN_FLOAT, CORDIC_MAX_FLOAT);
}

uint16_t radiansToQ16(float angle)
{
    return static_cast<uint16_t>(static_cast<int32_t>(angle * 65536.0f / TWO_PI));
}

int32_t singleFloatToCordic15(float value)
{
    return static_cast<int32_t>(clampCordicInput(value) * 0x8000) & 0xFFFF;
}

int32_t dualFloatToCordic15(float lowValue, float highValue)
{
    int32_t high = static_cast<int32_t>(clampCordicInput(highValue) * 0x8000) << 16;
    int32_t low = static_cast<int32_t>(clampCordicInput(lowValue) * 0x8000) & 0xFFFF;
    return high | low;
}

void cordic15ToDualFloat(int32_t cordic15, float* lowValue, float* highValue)
{
    if(cordic15 & 0x8000)
        *lowValue = (static_cast<float>(cordic15 & 0x7FFF) - 0x8000) / 0x8000;
    else
        *lowValue = static_cast<float>(cordic15 & 0xFFFF) / 0x8000;

    if(cordic15 & 0x80000000)
        *highValue = (static_cast<float>((cordic15 >> 16) & 0x7FFF) - 0x8000) / 0x8000;
    else
        *highValue = static_cast<float>((cordic15 >> 16) & 0xFFFF) / 0x8000;
}

void setFunction(uint32_t function)
{
    MODIFY_REG(hcordic.Instance->CSR, CORDIC_CSR_FUNC, function);
}
} // namespace
#endif

void init()
{
#if TRIGONOMETRIC_BACKEND == TRIGONOMETRIC_BACKEND_ST_CORDIC
    CORDIC_ConfigTypeDef cordicConfig;
    cordicConfig.Function = CORDIC_FUNCTION_SINE;
    cordicConfig.Scale = CORDIC_SCALE_0;
    cordicConfig.InSize = CORDIC_INSIZE_16BITS;
    cordicConfig.OutSize = CORDIC_OUTSIZE_16BITS;
    cordicConfig.NbWrite = CORDIC_NBWRITE_1;
    cordicConfig.NbRead = CORDIC_NBREAD_1;
    cordicConfig.Precision = CORDIC_PRECISION_8CYCLES;
    HAL_CORDIC_Configure(&hcordic, &cordicConfig);
#endif
}

void sinCos(float angle, float* sinValue, float* cosValue)
{
    sinCosMultiply(1.0f, angle, sinValue, cosValue);
}

void sinCosMultiply(float value, float angle, float* sinMulValue, float* cosMulValue)
{
#if TRIGONOMETRIC_BACKEND == TRIGONOMETRIC_BACKEND_ST_CORDIC
    setFunction(CORDIC_FUNCTION_SINE);
    hcordic.Instance->WDATA = (singleFloatToCordic15(value) << 16) | radiansToQ16(angle);
    cordic15ToDualFloat(static_cast<int32_t>(hcordic.Instance->RDATA), sinMulValue, cosMulValue);
#elif TRIGONOMETRIC_BACKEND == TRIGONOMETRIC_BACKEND_LIBM
    *sinMulValue = value * sinf(angle);
    *cosMulValue = value * cosf(angle);
#endif
}

uint16_t phaseQ16(float y, float x)
{
#if TRIGONOMETRIC_BACKEND == TRIGONOMETRIC_BACKEND_ST_CORDIC
    setFunction(CORDIC_FUNCTION_PHASE);
    hcordic.Instance->WDATA = dualFloatToCordic15(y, x);
    uint16_t angle = static_cast<uint16_t>(hcordic.Instance->RDATA);
    setFunction(CORDIC_FUNCTION_SINE);
    return angle;
#elif TRIGONOMETRIC_BACKEND == TRIGONOMETRIC_BACKEND_LIBM
    float angle = atan2f(y, x);
    if(angle < 0.0f)
        angle += TWO_PI;
    return static_cast<uint16_t>(angle * 65536.0f / TWO_PI);
#endif
}

} // namespace Utils::Trigonometric
