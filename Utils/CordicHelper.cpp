/**
 * @file CordicHelper.cpp
 * @brief CORDIC helper utility for fixed-point conversion and CORDIC peripheral setup.
 * @authorbaoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */

#include "CordicHelper.hpp"

namespace Utils
{
namespace CordicHelper
{
void initCordic16()
{
    CORDIC_ConfigTypeDef cordicConfig;
    cordicConfig.Function = CORDIC_FUNCTION_SINE;
    cordicConfig.Scale = CORDIC_SCALE_0;
    cordicConfig.InSize = CORDIC_INSIZE_16BITS;
    cordicConfig.OutSize = CORDIC_OUTSIZE_16BITS;
    cordicConfig.NbWrite = CORDIC_NBWRITE_1;
    cordicConfig.NbRead = CORDIC_NBREAD_1;
    cordicConfig.Precision = CORDIC_PRECISION_8CYCLES;
    HAL_CORDIC_Configure(&hcordic, &cordicConfig);
}

void initCordic32()
{
    CORDIC_ConfigTypeDef cordicConfig;
    cordicConfig.Function = CORDIC_FUNCTION_SINE;
    cordicConfig.Scale = CORDIC_SCALE_0;
    cordicConfig.InSize = CORDIC_INSIZE_32BITS;
    cordicConfig.OutSize = CORDIC_OUTSIZE_32BITS;
    cordicConfig.NbWrite = CORDIC_NBWRITE_2;
    cordicConfig.NbRead = CORDIC_NBREAD_2;
    cordicConfig.Precision = CORDIC_PRECISION_8CYCLES;
    HAL_CORDIC_Configure(&hcordic, &cordicConfig);
}

void setFunction(uint32_t function)
{
    MODIFY_REG(hcordic.Instance->CSR, CORDIC_CSR_FUNC, function);
}

int32_t floatToCordic31(float floatingValue)
{
    if (floatingValue > CORDIC_MAX_FLOAT)
    {
        floatingValue = CORDIC_MAX_FLOAT;
    }
    else if (floatingValue < CORDIC_MIN_FLOAT)
    {
        floatingValue = CORDIC_MIN_FLOAT;
    }
    return (int32_t)(floatingValue * 0x80000000);
    ;
}

float cordic31ToFloat(int32_t cordic31)
{
    if (cordic31 & 0x80000000) /* Negative */
    {
        cordic31 = cordic31 & 0x7fffffff;
        return ((float)(cordic31)-0x80000000) / 0x80000000;
    }
    else /* Positive */
    {
        return (float)(cordic31) / 0x80000000;
    }
}

int32_t dualFloatToCordic15(float valueA, float valueB)
{
    if (valueA > CORDIC_MAX_FLOAT)
    {
        valueA = CORDIC_MAX_FLOAT;
    }
    else if (valueA < CORDIC_MIN_FLOAT)
    {
        valueA = CORDIC_MIN_FLOAT;
    }
    if (valueB > CORDIC_MAX_FLOAT)
    {
        valueB = CORDIC_MAX_FLOAT;
    }
    else if (valueB < CORDIC_MIN_FLOAT)
    {
        valueB = CORDIC_MIN_FLOAT;
    }
    int32_t CORDIC15;
    CORDIC15 = (int32_t)(valueB * 0x8000) << 16;
    CORDIC15 = CORDIC15 | ((int32_t)(valueA * 0x8000) & 0xFFFF);
    return CORDIC15;
}

int32_t singleFloatToCordic15(float valueA)
{
    if (valueA > CORDIC_MAX_FLOAT)
    {
        valueA = CORDIC_MAX_FLOAT;
    }
    else if (valueA < CORDIC_MIN_FLOAT)
    {
        valueA = CORDIC_MIN_FLOAT;
    }
    return (int32_t)(valueA * 0x8000) & 0xFFFF;
}

void cordic15ToDualFloat(int32_t CORDIC15, float *valueA, float *valueB)
{
    /* Handle high 16 bits */
    if (CORDIC15 & 0x80000000) /* Negative */
    {
        *valueB = ((float)((CORDIC15 >> 16) & 0x7FFF) - 0x8000) / 0x8000;
    }
    else /* Positive */
    {
        *valueB = (float)((CORDIC15 >> 16) & 0xFFFF) / 0x8000;
    }

    /* Handle low 16 bits */
    if (CORDIC15 & 0x8000) /* Negative */
    {
        *valueA = ((float)(CORDIC15 & 0x7FFF) - 0x8000) / 0x8000;
    }
    else /* Positive */
    {
        *valueA = (float)(CORDIC15 & 0xFFFF) / 0x8000;
    }
}

float cordic15ToSingleFloat(int32_t CORDIC15)
{
    if (CORDIC15 & 0x8000) /* Negative */
    {
        return ((float)(CORDIC15 & 0x7FFF) - 0x8000) / 0x8000;
    }
    else /* Positive */
    {
        return (float)(CORDIC15 & 0xFFFF) / 0x8000;
    }
}
} // namespace CordicHelper
} // namespace Utils