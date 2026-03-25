/**
 * @file ErrorHandler.hpp
 * @brief Error handling configuration and threshold definitions.
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

#include "STSPIN32G4MosfetDriver.hpp"
#include "stdint.h"

namespace Control
{
namespace ErrorHandler
{
struct ErrorHandlerConfig
{
    uint8_t ignoreAllErrors                 = 0;
    uint8_t enableAutoRecovery              = 1;
    uint32_t autoRecoveryTimeout            = 100;      /* 自动恢复时间, Tick */

    float underVoltageThreshold             = 16.0f;    /* 欠压阈值, V */
    float overVoltageThreshold              = 30.0f;    /* 过压阈值, V */
    float overCurrentThreshold              = 12.0f;    /* 过流阈值, A */
    float underTemperatureThreshold         = 0.0f;     /* 欠温阈值, °C */
    float overTemperatureThreshold          = 100.0f;   /* 过温阈值, °C */

    uint32_t underVoltageTriggerTimeout     = 500;      /* 欠压触发时间, Tick */
    uint32_t overVoltageTriggerTimeout      = 500;      /* 过压触发时间, Tick */
    uint32_t overCurrentTriggerTimeout      = 10;       /* 过流触发时间, Tick */
    uint32_t overTemperatureTriggerTimeout  = 10000;    /* 过温触发时间, Tick */
};


struct ErrorStatus 
{
    // 需要累加
    uint8_t underVoltage;
    uint8_t overVoltage;
    uint8_t overCurrent;
    uint8_t ADCDecoderError;

    // 不需要累加 直接触发
    uint8_t overTemperature;
    uint8_t underTemperautre;
    uint8_t encoderError;               // 磁编码器/霍尔编码器通信失败 / 非法数值
    uint8_t motorDisconnected;          // 电机三相线断开, 不是没收到 CAN 消息.
#if USE_M3508_LINER_HALL_ENCODER
    uint8_t EEPROMDisconnect;           // EEPROM 通信失败
#endif

    Drivers::STSPIN32G4MosfetDriver::ErrorStatus STSPIN32G4MosfetDriverErrorStatus;
};

struct ErrorCounter_t 
{
    uint16_t underVoltageCounter;
    uint16_t overVoltageCounter;
    uint16_t overCurrentCounter;
    uint16_t currnentSensorErrorCounter;

    uint16_t noErrorCounter;
};

extern ErrorHandlerConfig errorHandlerConfig;
extern ErrorStatus motorErrorStatus;


void setConfig(const ErrorHandlerConfig* config);


void init();

/**
 * @brief 检查 motorErrorStatus 里是否有错误
 */
uint8_t checkIfAnyErrorStatus();

/**
 * @brief 检查是否仍然处于 error 状态(可否进行自动恢复)
 */
uint8_t checkIfStillInError();

/**
 * @brief 以 1KHz 频率调用, 检查过压欠压, 温度, 电流和, mos 驱动
 */
void checkError1KHz();

/**
 * @brief 与电流环同步调用, 最高频率. 检查过流.
 */
void checkErrorHighFreq();

/**
 * @brief 以 1KHz 频率调用, 用于自动恢复错误
 */
void checkIfCanAutoRecovery();


/**
 * @brief 清除所有错误
 */
void clearAllError();

} // namespace ErrorHandler
} // namespace Control