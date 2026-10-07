/**
 * @file StatisticsCalculator.hpp
 * @brief Statistics calculator for calculating mean and variance.
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

namespace Utils
{

class StatisticsCalculator
{
public:
    StatisticsCalculator(float dropRate = 0);
    void reset();
    void setDropRate(float dropRate);
    void addData(float data);
    float getMean();
    float getVariance();

private:
    /* data */
    float n;
    float correctedSumOfSquares;
    float mean;
    float variance;
    uint32_t dropPeriod;
    uint32_t dropCounter;

};

} // namespace Utils