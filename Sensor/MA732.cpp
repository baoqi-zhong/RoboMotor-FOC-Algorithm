/**
 * @file MA732.cpp
 * @brief MA732 magnetic encoder driver implementation.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "MA732.hpp"
#include "Encoder.hpp"
#include "ThreePhaseFOC.hpp"

namespace Sensor
{
namespace Encoder
{
namespace MA732
{
SPI_HandleTypeDef *hspi = NULL;

uint8_t MA732Buffer[8] = {0};
uint8_t emptyBuffer[8] = {0};

/*
作用: 返回还原后的 Encoder 偏移值
*/
// int16_t getBias(uint16_t rawAngle)
// {
//     uint16_t rawAngleLow = rawAngle & 0xFF;
//     uint16_t rawAngleHigh = rawAngle >> 8;
//     return ENCODER_BIAS[rawAngleHigh] * 2 + (((ENCODER_BIAS[rawAngleHigh + 1] - ENCODER_BIAS[rawAngleHigh]) * rawAngleLow) >> 7);
// }

void writeReg(uint8_t addr, uint8_t data)
{
    MA732Buffer[0] = data;
    MA732Buffer[1] = WRITE_CMD | addr;
    HAL_GPIO_WritePin(MA732_CS_GPIO_Port, MA732_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, MA732Buffer, (uint8_t *)(&MA732Buffer[2]), 1, 100);  // 必须 Transmit 0x00, 否则会意外修改寄存器
    HAL_GPIO_WritePin(MA732_CS_GPIO_Port, MA732_CS_Pin, GPIO_PIN_SET);

    HAL_Delay(20);

    HAL_GPIO_WritePin(MA732_CS_GPIO_Port, MA732_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, emptyBuffer, (uint8_t *)(&MA732Buffer[2]), 1, 100);  // 必须 Transmit 0x00, 否则会意外修改寄存器
    HAL_GPIO_WritePin(MA732_CS_GPIO_Port, MA732_CS_Pin, GPIO_PIN_SET);

    // if(MA732Buffer[3] != data)
    //     __BKPT(0);
    HAL_Delay(5);
}

uint16_t readReg(uint8_t addr)
{
    MA732Buffer[0] = 0;
    MA732Buffer[1] = READ_CMD | addr;
    HAL_GPIO_WritePin(MA732_CS_GPIO_Port, MA732_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, MA732Buffer, (uint8_t *)(&MA732Buffer[2]), 1, 100);  // 必须 Transmit 0x00, 否则会意外修改寄存器
    HAL_GPIO_WritePin(MA732_CS_GPIO_Port, MA732_CS_Pin, GPIO_PIN_SET);

    HAL_Delay(1);
    HAL_GPIO_WritePin(MA732_CS_GPIO_Port, MA732_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, emptyBuffer, (uint8_t *)(&MA732Buffer[2]), 1, 100);  // 必须 Transmit 0x00, 否则会意外修改寄存器
    HAL_GPIO_WritePin(MA732_CS_GPIO_Port, MA732_CS_Pin, GPIO_PIN_SET);
    return MA732Buffer[3];
}


void writeZeroOffset(uint16_t offset)
{
    writeReg(0x00, offset & 0xFF);
    writeReg(0x01, offset >> 8);
}

uint16_t readZeroOffset()
{
    uint8_t zeroL = readReg(0x00);
    uint8_t zeroH = readReg(0x01);
    return (zeroH << 8) + zeroL;
}


/**
 * 写入偏移值, 使当前位置为 0. 断电可保存.
 * 未验证
 */
void setToZero()
{
    uint16_t offset = readZeroOffset();
    uint16_t currentAngle = readBlocking();
    writeZeroOffset(0xFFFF - currentAngle + offset);
}

void init(SPI_HandleTypeDef *hspi_)
{
    hspi = hspi_;
    HAL_GPIO_WritePin(MA732_CS_GPIO_Port, MA732_CS_Pin, GPIO_PIN_SET);
    __HAL_SPI_ENABLE(hspi);
    
    // writeReg(0x00, 0x00);
    // writeReg(0x01, 0x00);
    writeReg(0x02, 0x00);
    writeReg(0x03, 0x00);
    writeReg(0x04, 0xC0);
    writeReg(0x05, 0xFF);
    writeReg(0x06, 0x1C);
    writeReg(0x09, ROTATION_DIRECTION_CCW);
    writeReg(0x0E, FILTER_CUTOFF_FREQ_185);
    writeReg(0x10, 0x9C);
}

uint16_t readBlocking()
{
    // MA732 本身有 9 us 的 Latency, 但是需要加上从读取到 setPhaseVoltage 的时间 19 us
    // encoderDelayTime = 0.000009f + 0.000019;

    if(hspi->Lock == HAL_LOCKED)
        return encoderStatus.Q16_encoder; // SPI 正在被占用, 直接返回上次的值

    HAL_GPIO_WritePin(MA732_CS_GPIO_Port, MA732_CS_Pin, GPIO_PIN_RESET);

    hspi->Lock = HAL_LOCKED;
    /* Set fiforxthreshold according the reception data length: 16bit */
    CLEAR_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);
    while(!__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE)); // 阻塞等待直到 Tx 空闲
    hspi->Instance->DR = *((uint16_t *)emptyBuffer);

    while(!__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE)); // 阻塞等待直到 Rx 有数据
    *((uint16_t *)MA732Buffer) = (uint16_t)hspi->Instance->DR;

    // End transection
    while ((hspi->Instance->SR & SPI_FLAG_FTLVL) != SPI_FTLVL_EMPTY);         // Control if the TX fifo is empty
    while ((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_BSY) ? SET : RESET) != RESET);  // Control the BSY flag
    while ((hspi->Instance->SR & SPI_FLAG_FRLVL) != SPI_FRLVL_EMPTY);         // Control if the RX fifo is empty
    hspi->Lock = HAL_UNLOCKED;

    HAL_GPIO_WritePin(MA732_CS_GPIO_Port, MA732_CS_Pin, GPIO_PIN_SET);

    return *(uint16_t *)MA732Buffer;
}

} // namespace MA732
} // namespace Encoder
} // namespace Sensor