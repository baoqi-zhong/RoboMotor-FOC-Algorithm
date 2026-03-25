/**
 * @file IncrementalPID.hpp
 * @brief Incremental PID controller class definition.
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
#include "PositionalPID.hpp"

namespace Control
{

class IncrementalPID
{
public:
    IncrementalPID();
    void setParameters(PIDParameters_t parameters_);
    float operator()(float target, float measurement);
    void reset();
    PIDParameters_t getParameters();
    void setOutput(float output_);

private:
    PIDParameters_t parameters;

    uint8_t firstUpdate         = 1;
    float lastError             = 0.0f;
    float lastMeasurement       = 0.0f;
    float lastLastMeasurement   = 0.0f;

    float pOut                  = 0.0f;
    float iOut                  = 0.0f;
    float dOut                  = 0.0f;
    
    float deltaOutput           = 0.0f;
    float output                = 0.0f;
};

} // namespace Control