/**
 * @file WS2812.cpp
 * @brief WS2812 LED driver implementation.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "WS2812.hpp"

namespace Drivers
{
namespace LED
{
#define BIT1_WIDTH 106
#define BIT0_WIDTH 52


WS2812::WS2812(WS2812Group* group_, uint8_t index_, LEDFunctionType functionType_) : 
    GenericLED(functionType_, LEDDriverType::WS2812), group(group_), index(index_) 
{
    if(index_ >= MAX_LED_NUM)
        return;

    // 默认颜色
    if(functionType_ == LEDFunctionType::DISPLAY_ID)
        this->setColor(0, 255, 0); 
    else if(functionType_ == LEDFunctionType::DISPLAY_ERROR_ID)
        this->setColor(255, 0, 0);
    else if(functionType_ == LEDFunctionType::DISPLAY_CONNECTION_STATUS)
        this->setColor(255, 255, 255);
    
    this->setBrightness(64);    // 默认亮度
};
    
void WS2812::setColor(uint8_t red, uint8_t green, uint8_t blue)
{
    this->color.red = red;
    this->color.green = green;
    this->color.blue = blue;
    this->updatedFlag = 1;
}

void WS2812::setBrightness(uint8_t brightness_)
{
    this->brightness = brightness_;
    this->updatedFlag = 1;
}

void WS2812::packOnData(uint8_t* buf)
{
    RGB colorWithBrightness = {
        (uint8_t)(uint16_t(this->color.green) * this->brightness / 255),
        (uint8_t)(uint16_t(this->color.red) * this->brightness / 255),
        (uint8_t)(uint16_t(this->color.blue) * this->brightness / 255)
    };

    for(uint8_t i = 0; i < 8; i++)
    {
        buf[this->index * 24 + i] = colorWithBrightness.green & 0x80 ? BIT1_WIDTH : BIT0_WIDTH;
        colorWithBrightness.green <<= 1;
        buf[this->index * 24 + 8 + i] = colorWithBrightness.red & 0x80 ? BIT1_WIDTH : BIT0_WIDTH;
        colorWithBrightness.red <<= 1;
        buf[this->index * 24 + 16 + i] = colorWithBrightness.blue & 0x80 ? BIT1_WIDTH : BIT0_WIDTH;
        colorWithBrightness.blue <<= 1;
    }
}

void WS2812::packOffData(uint8_t* buf)
{
    for(int i = 0; i < 24; i++)
    {
        buf[this->index * 24 + i] = BIT0_WIDTH;
    }
}

void WS2812::update()
{
    uint8_t* buf = this->group->getCCRDMABuff();
    if(this->blinkCB.blinking)
    {
        this->blinkCB.time++;
        if(this->blinkCB.time <= (this->blinkCB.onDuration + this->blinkCB.offDuration) * this->blinkCB.value)
        {
            if(this->blinkCB.time % (this->blinkCB.onDuration + this->blinkCB.offDuration) == 1)
            {
                this->packOnData(buf);
            }
            else if(this->blinkCB.time % (this->blinkCB.onDuration + this->blinkCB.offDuration) == this->blinkCB.onDuration)
            {
                this->packOffData(buf);
            }
        }
        else if(this->blinkCB.time >= (this->blinkCB.onDuration + this->blinkCB.offDuration) * this->blinkCB.value + this->blinkCB.waitDuration)
        {
            this->blinkCB.time = 0;
        }
        return;
    }

    // 常亮模式
    if(this->updatedFlag == 0)
        return;

    if(this->blinkCB.value)
    {
        this->packOnData(buf);
    }
    else
    {
        this->packOffData(buf);
    }
}

void WS2812Group::transmit()
{
    CCRDMABuff[LEDNum * 24] = 0;
    HAL_TIM_PWM_Start_DMA(this->htim, this->channel, (uint32_t *)CCRDMABuff, this->LEDNum * 24 + 1);
}

} // namespace LED
} // namespace Drivers