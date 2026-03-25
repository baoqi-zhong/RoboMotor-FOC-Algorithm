/**
 * @file AT24Cxx.hpp
 * @brief I2C EEPROM driver interface.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "main.h"


#if USE_AT24CXX
#define AT24Cxx_ADDRESS 0xA0
uint8_t AT24Cxx_Read(uint16_t addr, uint8_t *data, uint16_t len);

uint8_t AT24Cxx_Write(uint16_t addr, uint8_t *data, uint16_t len);

#endif // USE_AT24CXX