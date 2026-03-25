/**
 * @file WS2812.hpp
 * @brief WS2812 LED definitions and structs.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "LED.hpp"
#include "tim.h"
#include "stdint.h"

namespace Drivers
{
namespace LED
{
/**
 * @brief The struct ure that contains the rgb value of a single ws2812 unit
 */
struct RGB
{
    uint8_t green;
    uint8_t red;
    uint8_t blue;
};


class WS2812Group;

class WS2812 : public GenericLED
{
public:
    WS2812(WS2812Group* group_, uint8_t index_, LEDFunctionType functionType_);
    
    void packOnData(uint8_t* buf);
    void packOffData(uint8_t* buf);
    void setColor(uint8_t red, uint8_t green, uint8_t blue);
    void setBrightness(uint8_t brightness_);
    void update();
    uint8_t updatedFlag = 1;

    WS2812Group* group;
    const uint8_t index;    // 第几个 WS2812 LED
    RGB color = {0, 0, 0};  // 颜色, 发送的时候会乘上亮度作为显示的颜色
};

class WS2812Group
{
public:
    WS2812Group(uint8_t LEDNum_, TIM_HandleTypeDef* htim_, uint32_t channel_) : LEDNum(LEDNum_), htim(htim_), channel(channel_) {};
    void transmit();        // 传输当前的 LED 状态到 LED 灯带上
    uint8_t* getCCRDMABuff() { return CCRDMABuff; }

private:
    const uint8_t LEDNum;
    TIM_HandleTypeDef* const htim;
    const uint32_t channel;

    uint8_t CCRDMABuff[MAX_LED_NUM * 24 + 1];
};

} // namespace LED
} // namespace Drivers