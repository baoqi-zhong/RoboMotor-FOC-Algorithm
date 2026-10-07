/**
 * @file LED.hpp
 * @brief Generic LED interface and definitions.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#pragma once

#include <stdint.h>

#ifndef MAX_LED_NUM
#define MAX_LED_NUM 8
#endif

namespace Drivers
{
namespace LED
{
enum class LEDDriverType : uint8_t
{
    GPIO,
    TIM,
    WS2812
};

enum class LEDFunctionType : uint8_t
{
    DISPLAY_ID,
    DISPLAY_ERROR_ID,
    DISPLAY_CONNECTION_STATUS,
};

struct BlinkControlBlock
{
    uint8_t blinking;
    uint8_t value;
    uint16_t onDuration;
    uint16_t offDuration;
    uint16_t waitDuration;
    uint16_t time;
};

class GenericLED
{
public:
    GenericLED(LEDFunctionType functionType_, LEDDriverType driverType_)
        : functionType(functionType_), driverType(driverType_)
    {
    }

    virtual ~GenericLED() = default;

    void onOff(uint8_t on);
    void blink(uint8_t value, uint16_t onDuration = 100, uint16_t offDuration = 200, uint16_t waitDuration = 300);
    virtual void update() = 0;

    virtual void setBrightness(uint8_t brightness_) { brightness = brightness_; }

    const LEDFunctionType functionType;
    const LEDDriverType driverType;

protected:
    uint8_t brightness = 255;
    BlinkControlBlock blinkCB = {0, 0, 0, 0, 0, 0};
};

void registerLED(GenericLED* led);
GenericLED* getLEDByFunction(LEDFunctionType function);
void blink(LEDFunctionType function, uint8_t value);
void onOff(LEDFunctionType function, uint8_t on);
void update();

} // namespace LED
} // namespace Drivers
