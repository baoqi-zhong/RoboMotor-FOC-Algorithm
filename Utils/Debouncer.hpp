 /**
 * @file Debouncer.hpp
 * @brief A utility class for debouncing signals, providing a delay before confirming/clearing a state.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */

#pragma once

#include "stdint.h"
#include "stdbool.h"

namespace Utils
{
class Debouncer
{
public:
    void init(uint16_t activeTimeout_ = 10, uint16_t inactiveTimeout_ = 10)
    {
        activeTimeout = activeTimeout_ > 0 ? activeTimeout_ : 1;
        inactiveTimeout = inactiveTimeout_ > 0 ? inactiveTimeout_ : 1;
        activeThreshold = (uint32_t)activeTimeout * (uint32_t)inactiveTimeout;
    }

    Debouncer(uint16_t timeout = 10) { init(timeout, timeout); }
    Debouncer(uint16_t activeTimeout_, uint16_t inactiveTimeout_) { init(activeTimeout_, inactiveTimeout_); }

    bool operator()(bool currentStatus)
    {
        if (currentStatus)
        {
            if (counter < activeThreshold)
            {
                counter += inactiveTimeout;
            }

            if (counter >= activeThreshold)
            {
                counter = activeThreshold;
                outputActive = true;
            }
        }
        else
        {
            if (counter > 0)
            {
                if (counter > activeTimeout)
                {
                    counter -= activeTimeout;
                }
                else
                {
                    counter = 0;
                }
            }

            if (counter == 0)
            {
                outputActive = false;
            }
            
        }

        return outputActive;
    }

    void reset()
    {
        counter    = 0;
        outputActive = false;
    }

private:
    uint16_t activeTimeout      = 1;
    uint16_t inactiveTimeout    = 1;
    uint32_t activeThreshold    = 1;

    uint32_t counter            = 0;
    bool outputActive           = false;
};

}  // namespace Utils