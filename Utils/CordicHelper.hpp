/**
 * @file CordicHelper.hpp
 * @brief CORDIC helper utility for fixed-point conversion and CORDIC peripheral setup.
 * @authorbaoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */

#pragma once

#include "main.h"
#include "cordic.h"
#include "stdint.h"

namespace Utils
{
namespace CordicHelper
{
#define CORDIC_MAX_FLOAT 0.999969482421875f
#define CORDIC_MIN_FLOAT -1.0f

/**
 * @brief Initialize the CORDIC algorithm for 16-bit operations
 */
void initCordic16();

/**
 * @brief Initialize the CORDIC algorithm for 32-bit operations
 */
void initCordic32();


/**
 * @brief Set the CORDIC function to be used
 * @param function The CORDIC function to set in CORDIC_Function (e.g., CORDIC_FUNCTION_SINE, CORDIC_FUNCTION_COSINE, etc.)
 */
void setFunction(uint32_t function);

/**
 * @brief Convert a floating-point value to a 31-bit CORDIC fixed-point representation
 * @param floatingValue Input floating-point value to convert
 * @return Converted 31-bit CORDIC fixed-point representation
 */
int32_t floatToCordic31(float floatingValue);

/**
 * @brief Convert a 31-bit CORDIC fixed-point representation to a floating-point value
 * @param cordic31 Input 31-bit CORDIC fixed-point value to convert
 * @return Converted floating-point value
 */
float cordic31ToFloat(int32_t cordic31);

/**
 * @brief Convert two floating-point values to a 15-bit CORDIC fixed-point representation
 * @param valueA First input floating-point value (High 16 bits)
 * @param valueB Second input floating-point value (Low 16 bits)
 * @return Combined 15-bit CORDIC fixed-point representation
 */
int32_t dualFloatToCordic15(float valueA, float valueB);

/**
 * @brief Convert a floating-point value to a 15-bit CORDIC fixed-point representation
 * @param valueA Input floating-point value
 * @return Converted 15-bit CORDIC fixed-point representation
 */
int32_t singleFloatToCordic15(float valueA);

/**
 * @brief Convert a 15-bit CORDIC fixed-point representation to two floating-point values
 * @param CORDIC15 Input 15-bit CORDIC fixed-point value to convert
 * @param valueA Pointer to store the first output floating-point value (High 16 bits)
 * @param valueB Pointer to store the second output floating-point value (Low 16 bits)
 */
void cordic15ToDualFloat(int32_t CORDIC15, float *valueA, float *valueB);

/**
 * @brief Convert a 15-bit CORDIC fixed-point representation to a floating-point value
 * @param CORDIC15 Input 15-bit CORDIC fixed-point value to convert
 * @return Converted floating-point value
 */
float cordic15ToSingleFloat(int32_t CORDIC15);

} // namespace CordicHelper
} // namespace Utils