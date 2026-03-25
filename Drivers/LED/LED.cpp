/**
 * @file LED.cpp
 * @brief Generic LED management and registration.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "LED.hpp"
#include "WS2812.hpp"

namespace Drivers
{
namespace LED
{
/*
    无论对于 GPIO 与 TIMER, WS2812, 一个 GenericLED 就对应一个 LED
    对于 WS2812, 在 transmit 的时候注意不要多次调用
*/
GenericLED* LEDPool[MAX_LED_NUM] = {nullptr};


void registerLED(GenericLED* led)
{
    for (int i = 0; i < MAX_LED_NUM; i++)
    {
        if (LEDPool[i] == nullptr)
        {
            LEDPool[i] = led;
            return;
        }
        else if (LEDPool[i]->functionType == led->functionType)
        {
            // 每种功能只允许注册一个
            return;
        }
    }
}

GenericLED* getLEDByFunction(LEDFunctionType function)
{
    for (int i = 0; i < MAX_LED_NUM; i++)
    {
        if (LEDPool[i] != nullptr && LEDPool[i]->functionType == function)
        {
            return LEDPool[i];
        }
    }
    return nullptr;
}


void blink(LEDFunctionType function, uint8_t value)
{
    GenericLED* led = getLEDByFunction(function);
    if (led == nullptr)
        return;

    led->blink(value);
}

void onOff(LEDFunctionType function, uint8_t on)
{
    GenericLED* led = getLEDByFunction(function);
    if (led == nullptr)
        return;

    led->onOff(on);
}

void update()
{
    WS2812Group* ws2812Group[MAX_LED_NUM] = {nullptr};
    for (int i = 0; i < MAX_LED_NUM; i++)
    {
        if (LEDPool[i] != nullptr)
        {
            LEDPool[i]->update();
            if(LEDPool[i]->driverType == LEDDriverType::WS2812)
            {
                WS2812* ws2812 = static_cast<WS2812*>(LEDPool[i]);
                for(int j = 0; j < MAX_LED_NUM; j++)
                {
                    if(ws2812Group[j] == nullptr)
                    {
                        ws2812Group[j] = ws2812->group;
                        break;
                    }
                    else if(ws2812Group[j] == ws2812->group)
                    {
                        break;
                    }
                }
            }
        }
    }

    for(int j = 0; j < MAX_LED_NUM; j++)
    {
        if(ws2812Group[j] != nullptr)
        {
            ws2812Group[j]->transmit();
        }
    }
}

void GenericLED::blink(uint8_t value, uint16_t onDuration, uint16_t offDuration, uint16_t waitDuration)
{
    this->blinkCB.blinking = 1;
    this->blinkCB.value = value;
    this->blinkCB.onDuration = onDuration;
    this->blinkCB.offDuration = offDuration;
    this->blinkCB.waitDuration = waitDuration;
    this->blinkCB.time = 0;
}

void GenericLED::onOff(uint8_t on)
{
    this->blinkCB.blinking = 0;
    if (on)
        this->blinkCB.value = 1;
    else
        this->blinkCB.value = 0;
}


} // namespace LED
} // namespace Drivers