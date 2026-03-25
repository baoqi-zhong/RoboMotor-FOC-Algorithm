/**
 * @file STSPIN32G4MosfetDriver.cpp
 * @brief Driver for STSPIN32G4 internal registers via I2C.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "STSPIN32G4MosfetDriver.hpp"

namespace Drivers
{
namespace STSPIN32G4MosfetDriver
{
uint8_t buffer[2] = {0, 0};

uint8_t readRegister(uint8_t reg, uint8_t *data)
{
    if(HAL_I2C_Mem_Read(&STSPIN32G4_MOS_DRIVER_I2C,  STSPIN32G4_MOS_DRIVER_DEVICE_ADDRESS, reg, 1, data, 1, 100) != HAL_OK)
    {
        return -1;
    }
    return 0;
}

uint8_t writeRegister(uint8_t reg, uint8_t data)
{
    buffer[0] = data;
    if (HAL_I2C_Mem_Write(&STSPIN32G4_MOS_DRIVER_I2C, STSPIN32G4_MOS_DRIVER_DEVICE_ADDRESS, reg, 1, buffer, 1, 100) != HAL_OK)
        return -1;
    return 0;
}

uint8_t writeRegisterVerify(uint8_t reg, uint8_t data)
{
    buffer[0] = data;
    if (HAL_I2C_Mem_Write(&STSPIN32G4_MOS_DRIVER_I2C, STSPIN32G4_MOS_DRIVER_DEVICE_ADDRESS, reg, 1, buffer, 1, 100) != HAL_OK)
        return -1;
    if(HAL_I2C_Mem_Read(&STSPIN32G4_MOS_DRIVER_I2C,  STSPIN32G4_MOS_DRIVER_DEVICE_ADDRESS, reg, 1, buffer + 1, 1, 100) != HAL_OK)
        return -1;
    if(buffer[0] != buffer[1])
        return -1;
    return 0;
}


uint8_t readStatus(uint8_t *status)
{
    if(HAL_I2C_Mem_Read(&STSPIN32G4_MOS_DRIVER_I2C,  STSPIN32G4_MOS_DRIVER_DEVICE_ADDRESS, STSPIN32G4_MOS_DRIVER_REGISTER_STATUS, 1, status, 1, 100) != HAL_OK)
    {
        return -1;
    }
    return 0;
}


uint8_t init()
{
    // 必须 24v 上电, 才能读到 I2C. mos 驱动需要 24v 供电
    HAL_GPIO_WritePin(PS_WAKE_GPIO_Port, PS_WAKE_Pin, GPIO_PIN_SET);
    HAL_Delay(1);

    // Reset all register to default value
    // Command Register, no varify
    if(writeRegister(STSPIN32G4_MOS_DRIVER_REGISTER_RESET, 0xFF))
        return -1;


    // UNLOCK
    if(writeRegisterVerify(STSPIN32G4_MOS_DRIVER_REGISTER_LOCK, 0x0F))
        return -1;

    // Set VCC to 10V. Enable LDO
    if(writeRegisterVerify(STSPIN32G4_MOS_DRIVER_REGISTER_POWMNG, 0x01))
        return -1;

    // Enable Interlocking, Disable deadtime
    if(writeRegisterVerify(STSPIN32G4_MOS_DRIVER_REGISTER_LOGIC, 0x71))
        return -1;

    
    // Clear all fault
    if(writeRegister(STSPIN32G4_MOS_DRIVER_REGISTER_CLEAR, 0xFF))
        return -1;

    // LOCK
    if(writeRegisterVerify(STSPIN32G4_MOS_DRIVER_REGISTER_LOCK, 0x00))
        return -1;
    
    return 0;
}

uint8_t clearFault()
{
    // Clear all fault. 不用解锁
    if(writeRegister(STSPIN32G4_MOS_DRIVER_REGISTER_CLEAR, 0xFF))
        return -1;

    return 0;
}

} // namespace STSPIN32G4MosfetDriver
} // namespace Drivers