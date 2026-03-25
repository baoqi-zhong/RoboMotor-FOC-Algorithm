/**
 * @file IncrementalPID.cpp
 * @brief Incremental PID controller implementation.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "IncrementalPID.hpp"
#include "Math.hpp"

namespace Control
{
IncrementalPID::IncrementalPID()
{
    this->reset();
}

void IncrementalPID::setParameters(PIDParameters_t parameters_)
{
    assert_param(parameters.outputLimit >= 0.0f);
    assert_param(parameters.updateFrequency > 0.0f);
    assert_param(parameters.alpha >= 0.0f && parameters.alpha <= 1.0f);

    this->parameters = parameters_;
    this->reset();
}

float IncrementalPID::operator()(float target, float measurement)
{
    if(this->firstUpdate)
    {
        this->lastLastMeasurement = measurement;
        this->lastMeasurement = measurement;
        this->firstUpdate = 0;
    }
    float error = target - measurement;

    float p = this->parameters.kPonError * (error - this->lastError);
    this->pOut = p - this->parameters.kPonMeasurement * (measurement - this->lastMeasurement);

    this->iOut = this->parameters.kIonError * error / this->parameters.updateFrequency;

    float d = this->parameters.kDonMeasurement * (measurement - 2.0f * this->lastMeasurement + this->lastLastMeasurement) * this->parameters.updateFrequency;
    this->dOut = d * this->parameters.alpha + this->dOut * (1 - this->parameters.alpha);


    this->deltaOutput = this->pOut + this->iOut + this->dOut;

    this->lastError = error;
    this->lastLastMeasurement = this->lastMeasurement;
    this->lastMeasurement = measurement;

    this->output += this->deltaOutput;
    this->output = CLAMP(this->output, -this->parameters.outputLimit, this->parameters.outputLimit);
    return this->output;
}

void IncrementalPID::reset()
{
    this->firstUpdate           = 1;
    this->lastError             = 0.0f;
    this->lastMeasurement       = 0.0f;
    this->lastLastMeasurement   = 0.0f;
    this->deltaOutput           = 0.0f;
    this->output                = 0.0f;
}

PIDParameters_t IncrementalPID::getParameters()
{
    return this->parameters;
}

void IncrementalPID::setOutput(float output_)
{
    this->output = output_;
}
} // namespace Control
