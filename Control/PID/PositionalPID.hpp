/**
 * @file PositionalPID.hpp
 * @brief Positional PID controller class definition and PID parameters.
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
#include "tim.h"
#include "stdint.h"


namespace Control
{
struct PIDParameters_t
{
    float kPonError         = 0.0f;
    float kIonError         = 0.0f;
    float kDonMeasurement   = 0.0f;
    float kPonMeasurement   = 0.0f;
    float kDonTarget        = 0.0f;
    float alpha             = 1.0f;
    float outputLimit       = 0.0f;
    float updateFrequency   = 1000.0f;
};

class PositionalPID
{
public:
    PositionalPID();
    void setParameters(PIDParameters_t parameters_);
    PIDParameters_t getParameters() const { return this->parameters; }
    void setOutputLimit(float outputLimit_) { this->parameters.outputLimit = outputLimit_; }
    float operator()(float target, float measurement);
    void reset();
    
private:
    PIDParameters_t parameters;

    uint8_t firstUpdate     = 1;
    float lastTarget        = 0.0f;
    float lastMeasurement   = 0.0f;
    float pOut              = 0.0f;
    float iOut              = 0.0f;
    float dOut              = 0.0f;

    float output            = 0.0f;
};

} // namespace Control