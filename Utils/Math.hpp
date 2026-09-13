/**
 * @file Math.hpp
 * @brief Mathematical utility functions and constants.
 * @authorbaoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */

#pragma once

inline constexpr float PI                          = 3.1415926f;
inline constexpr float TWO_PI                      = 6.2831853f;
inline constexpr float ONE_OVER_SQRT3              = 0.5773503f;
inline constexpr float TWO_OVER_SQRT3              = 1.1547005f;
inline constexpr float SQRT3                       = 1.7320508f;
inline constexpr float SQRT3_OVER_2                = 0.8660254f;
inline constexpr float RPM_TO_RAD_PER_S_RATIO      = 0.1047198f;
inline constexpr float RAD_PER_S_TO_RPM_RATIO      = 9.549296f;

template<typename T>
constexpr T CLAMP(T value, T minimum, T maximum)
{
    return value > maximum ? maximum : (value < minimum ? minimum : value);
}

template<typename T>
constexpr T FABS(T value)
{
    return value >= T{} ? value : -value;
}

template<typename T>
constexpr T MIN(T a, T b)
{
    return a < b ? a : b;
}

template<typename T>
constexpr T MAX(T a, T b)
{
    return a > b ? a : b;
}

constexpr float FMOD(float value, float divisor)
{
    return value - static_cast<int>(value / divisor) * divisor;
}
