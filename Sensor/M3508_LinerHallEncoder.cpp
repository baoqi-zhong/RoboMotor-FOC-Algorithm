/**
 * @file M3508_LinerHallEncoder.cpp
 * @brief M3508 Linear Hall Encoder driver implementation.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "M3508_LinerHallEncoder.hpp"
#include "CordicHelper.hpp"
#include "IncrementalPID.hpp"
#include "LPF.hpp"
#include "AT24Cxx.hpp"
#if 0
namespace Sensor
{
namespace Encoder
{
namespace M3508_LinerHallEncoder
{
uint16_t M3508_HALL_A_BAIS = DEFAULT_M3508_HALL_A_BAIS;     /* M3508 Hall A 通道 ADC 中点值 */
uint16_t M3508_HALL_B_BAIS = DEFAULT_M3508_HALL_B_BAIS;     /* M3508 Hall B 通道 ADC 中点值 */
float M3508_HALL_A_GAIN = DEFAULT_M3508_HALL_A_GAIN;        /* M3508 Hall A 通道 ADC 增益 */
float M3508_HALL_B_GAIN = DEFAULT_M3508_HALL_B_GAIN;        /* M3508 Hall B 通道 ADC 增益 */

uint16_t* pHallABuffer;
uint16_t* pHallBBuffer;

float hallA, hallB;
uint16_t estimatedHallEncoder;
uint16_t estimatedHallEncoderCordic;
float estimatedHallEncoderFloat;
float deltaEstimatedEncoderFloat;

IncrementalPID_t hallEstimatorPID;

// 需要速度反馈做自适应截止频率
#include "Encoder.hpp"

uint8_t M3508_EEPROMBuffer[16];

void init()
{
    extern uint16_t adcBuffer[ADC_CHANNEL_NUM * 2];
    pHallABuffer = adcBuffer + M3508_HALL_A_BUFFER_OFFSET;
    pHallBBuffer = adcBuffer + M3508_HALL_B_BUFFER_OFFSET;

    estimatedHallEncoder = 0;
    estimatedHallEncoderFloat = 0;
    deltaEstimatedEncoderFloat = 0;

    IncrementalPIDsetParameters(&hallEstimatorPID,
                                0.0f,           // kPOnTarget
                                200.0f,         // kPOnMeasurement
                                80000.0f,      // kI
                                0.0f,           // kD
                                70000.0f,       // outputLimit
                                CURRENT_LOOP_FREQ    // updateFrequency
                                );

    #if USE_AT24CXX
    AT24Cxx_Read(0x70, M3508_EEPROMBuffer, 16);
    uint8_t xor = 0;
    for(uint8_t i = 0; i < 16; i++)
    {
        xor ^= M3508_EEPROMBuffer[i];
    }
    if(xor)
    {
        // EEPROM 数据不合法, 重新写入默认值
        M3508_EEPROMBuffer[0] =  DEFAULT_ENCODER_ZERO_OFFSET >> 8;
        M3508_EEPROMBuffer[1] =  DEFAULT_ENCODER_ZERO_OFFSET & 0xFF;
        M3508_EEPROMBuffer[2] =  (int8_t)((int16_t)(DEFAULT_M3508_HALL_A_BAIS) - 1280);
        M3508_EEPROMBuffer[3] =  (int8_t)((int16_t)(1 / DEFAULT_M3508_HALL_A_GAIN) - 900);
        M3508_EEPROMBuffer[4] =  (int8_t)((int16_t)(DEFAULT_M3508_HALL_B_BAIS) - 1280);
        M3508_EEPROMBuffer[5] =  (int8_t)((int16_t)(1 / DEFAULT_M3508_HALL_B_GAIN) - 900);
        for(uint8_t i = 6; i < 15; i++)
        {
            M3508_EEPROMBuffer[i] = 0;
        }
        // 计算校验和
        xor = 0;
        for(uint8_t i = 0; i < 5; i++)
        {
            xor ^= M3508_EEPROMBuffer[i];
        }
        M3508_EEPROMBuffer[15] = xor;
        AT24Cxx_Write(0x70, M3508_EEPROMBuffer, 16);

        // 使用默认值
        Sensor::Encoder::encoderStatus.ENCODER_ZERO_OFFSET = DEFAULT_ENCODER_ZERO_OFFSET;
        M3508_HALL_A_BAIS = DEFAULT_M3508_HALL_A_BAIS;
        M3508_HALL_A_GAIN = DEFAULT_M3508_HALL_A_GAIN;
        M3508_HALL_B_BAIS = DEFAULT_M3508_HALL_B_BAIS;
        M3508_HALL_B_GAIN = DEFAULT_M3508_HALL_B_GAIN;
    }
    else
    {
        // EEPROM 数据合法, 读取数据
        Sensor::Encoder::encoderStatus.ENCODER_ZERO_OFFSET = (M3508_EEPROMBuffer[0] << 8) | M3508_EEPROMBuffer[1];
        M3508_HALL_A_BAIS = (int16_t)(M3508_EEPROMBuffer[2] + 1280);
        M3508_HALL_A_GAIN = 1.0f / (float)((int16_t)(M3508_EEPROMBuffer[3]) + 900);
        M3508_HALL_B_BAIS = (int16_t)(M3508_EEPROMBuffer[4] + 1280);
        M3508_HALL_B_GAIN = 1.0f / (float)((int16_t)(M3508_EEPROMBuffer[5]) + 900);
    }
    #endif
}

float err;
uint16_t readBlocking()
{
    // hallA,B 归一化范围近似 [-1, 1]
    hallA = ((float)*pHallABuffer - M3508_HALL_A_BAIS) * M3508_HALL_A_GAIN;
    hallB = ((float)*pHallBBuffer - M3508_HALL_B_BAIS) * M3508_HALL_B_GAIN;


    // // 目前 PLL 存在急刹车跟不上的情况. 考虑在某些情况下使用 arctan 方法.
    // // PLL 方法
    // static float cordicOutputSin;
    // static float cordicOutputCos;
    // hcordic.Instance->WDATA = 0x80000000 | estimatedHallEncoder;
    // cordic15ToDualFloat((int32_t)(hcordic.Instance->RDATA) , &cordicOutputSin, &cordicOutputCos);

    // // PLL
    // err = - cordicOutputSin * hallB + cordicOutputCos * hallA;
    
    // // 自适应截止频率 PLL, 速度越大, 跟踪速度越快, 截止频率越高
    // // kI 最大值约为 300000
    // if(FABS(Sensor::Encoder::encoderStatus.RAD_shaftAngularVelocity) > 50.0f)
    //     hallEstimatorPID.kI = -10000.0f + FABS(Sensor::Encoder::encoderStatus.RAD_shaftAngularVelocity) * 400.0f;
    // else
    //     hallEstimatorPID.kI = 10000.0f;
    
    // // 非线性处理, err 越大, 跟踪速度越快
    // if(err >= 0)
    //     err = 0.5f * (err + 1.0f) * (err + 1.0f) * (err + 1.0f) - 0.5f;
    // else
    //     err = 0.5f * (err - 1.0f) * (err - 1.0f) * (err - 1.0f) + 0.5f;
    // // PLL 的输出结果是速度.
    // deltaEstimatedEncoderFloat = IncrementalPIDupdate(&hallEstimatorPID, 0, err);
    // estimatedHallEncoderFloat = FMOD(estimatedHallEncoderFloat + deltaEstimatedEncoderFloat + 65536.0f, 65536.0f);

    // estimatedHallEncoder = (uint16_t)estimatedHallEncoderFloat;

    // arctan 方法
    // cordic 内部有限幅.
    Utils::CordicHelper::cordic_setFunction(CORDIC_FUNCTION_PHASE);
    // 这里改成了 hallB, hallA
    hcordic.Instance->WDATA = dualFloatToCordic15(hallB, hallA);
    float estimatedHallEncoder_arctan = (int32_t)(hcordic.Instance->RDATA);
    Utils::CordicHelper::cordic_setFunction(CORDIC_FUNCTION_SINE);

    estimatedHallEncoder = (uint16_t)estimatedHallEncoder_arctan;
    // 我们的 ADC 只有 12 bit, 所以 estimatedHallEncoder 是以 128 为最小跳动单位的.
    return estimatedHallEncoder;
}

} // M3508_LinerHallEncoder
} // Encoder
} // Sensor

#endif