/**
 * @file PositionalPID.cpp
 * @brief Positional PID controller implementation.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "PositionalPID.hpp"
#include "Math.hpp"

namespace Control
{
PositionalPID::PositionalPID()
{
    this->reset();
}

void PositionalPID::setParameters(PIDParameters_t parameters_)
{
    assert_param(outputLimit >= 0.0f);
    assert_param(updateFrequency > 0.0f);
    assert_param(alpha >= 0.0f && alpha <= 1.0f);
    
    this->parameters = parameters_;
    this->reset();
}


float PositionalPID::operator()(float target, float measurement)
{
    if(this->firstUpdate)
    {
        this->lastTarget         = target;
        this->lastMeasurement    = measurement;
        this->firstUpdate = 0;
    }
    float error             = target - measurement;
    float deltaTarget       = target - this->lastTarget;
    float deltaMeasurement  = measurement - this->lastMeasurement;
    this->lastTarget         = target;
    this->lastMeasurement    = measurement;


    float p = this->parameters.kPonError * error;
    float i = this->iOut + this->parameters.kIonError * error / this->parameters.updateFrequency;
    float d = (-this->parameters.kDonMeasurement * deltaMeasurement + this->parameters.kDonTarget * deltaTarget) * this->parameters.updateFrequency;
    this->dOut =  d * this->parameters.alpha + this->dOut * (1 - this->parameters.alpha);

    // apply p on measurement
    i -= this->parameters.kPonMeasurement * deltaMeasurement;

    // clamp p on error output; d 项会挤压 p 项
    float out = CLAMP(p + this->dOut, - this->parameters.outputLimit, this->parameters.outputLimit);
    this->pOut = out - this->dOut;

    // clamp integral output; pd 项会挤压 i 项
    this->iOut = CLAMP(CLAMP(i, - this->parameters.outputLimit - out, this->parameters.outputLimit - out), - this->parameters.outputLimit, this->parameters.outputLimit);

    // calculate output
    this->output = CLAMP(out + i, - this->parameters.outputLimit, this->parameters.outputLimit);

    return this->output;
}


void PositionalPID::reset()
{
    this->firstUpdate       = 1;
    this->lastMeasurement   = 0.0f;
    this->lastTarget        = 0.0f;
    this->pOut              = 0.0f;
    this->iOut              = 0.0f;
    this->dOut              = 0.0f;
    this->output            = 0.0f;
}
} // namespace Control