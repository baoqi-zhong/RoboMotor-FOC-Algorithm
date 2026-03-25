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


#include "main.h"
#include "stdint.h"

namespace Drivers
{
namespace LED
{
#define MAX_LED_NUM     4

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
    DISPLAY_CUSTOM_1,
    DISPLAY_CUSTOM_2,
    DISPLAY_CUSTOM_3,
    DISPLAY_CUSTOM_4
};

struct BlinkControlBlock
{
    uint8_t blinking;       // 是否在 blink 过程中
    uint8_t value;          // 0-255
    uint16_t onDuration;    // ms
    uint16_t offDuration;   // ms
    uint16_t waitDuration;  // ms
    uint16_t time;          // blink 过程中计时 ms
};

class GenericLED
{
public:
    GenericLED(LEDFunctionType functionType_, LEDDriverType driverType_) : functionType(functionType_), driverType(driverType_) {}
    void setBrightness(uint8_t brightness_) { this->brightness = brightness_; }
    void onOff(uint8_t on);
    void blink(uint8_t value, uint16_t onDuration = 100, uint16_t offDuration = 200, uint16_t waitDuration = 300);
    virtual void update() = 0;

    uint8_t brightness = 255;   // 动画过程中最大亮度
    BlinkControlBlock blinkCB = {0, 0, 0, 0, 0, 0};

    const LEDFunctionType functionType;
    const LEDDriverType driverType;
};

void registerLED(GenericLED* led);

void blink(LEDFunctionType function, uint8_t value);

void onOff(LEDFunctionType function, uint8_t on);

void update();

} // namespace LED
} // namespace Drivers