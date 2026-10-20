/**
 * @file AS5047P.hpp
 * @brief Generic AS5047P rotary encoder driver.
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

namespace Drivers
{

enum class AS5047PRegister : uint16_t
{
    Nop = 0x0000,
    ErrorFlags = 0x0001,
    Programming = 0x0003,
    Diagnostics = 0x3FFC,
    Magnitude = 0x3FFD,
    AngleUncompensated = 0x3FFE,
    AngleCompensated = 0x3FFF,
    ZeroPositionMsb = 0x0016,
    ZeroPositionLsb = 0x0017,
    Settings1 = 0x0018,
    Settings2 = 0x0019
};

struct AS5047PErrorFlags
{
    bool framingError = false;
    bool invalidCommand = false;
    bool parityError = false;
    uint16_t raw = 0;
};

struct AS5047PDiagnostics
{
    uint8_t agc = 0;
    bool offsetCompensationFinished = false;
    bool cordicOverflow = false;
    bool magneticFieldTooHigh = false;
    bool magneticFieldTooLow = false;
    uint16_t raw = 0;
};

struct AS5047PSettings1
{
    bool factorySetting = true;
    bool noiseSetting = false;
    bool inverted = false;
    bool uvwAbiSelection = false;
    bool daecDisabled = false;
    bool abiBinary = false;
    bool dataSelectUncompensated = false;
    bool pwmEnabled = false;

    constexpr uint16_t encode() const
    {
        return static_cast<uint16_t>(
            (factorySetting ? 0x01 : 0x00) |
            (noiseSetting ? 0x02 : 0x00) |
            (inverted ? 0x04 : 0x00) |
            (uvwAbiSelection ? 0x08 : 0x00) |
            (daecDisabled ? 0x10 : 0x00) |
            (abiBinary ? 0x20 : 0x00) |
            (dataSelectUncompensated ? 0x40 : 0x00) |
            (pwmEnabled ? 0x80 : 0x00));
    }

    static constexpr AS5047PSettings1 decode(uint16_t value)
    {
        return {
            (value & 0x01) != 0,
            (value & 0x02) != 0,
            (value & 0x04) != 0,
            (value & 0x08) != 0,
            (value & 0x10) != 0,
            (value & 0x20) != 0,
            (value & 0x40) != 0,
            (value & 0x80) != 0
        };
    }
};

struct AS5047PSettings2
{
    uint8_t uvwPolePairsMinusOne = 0;
    uint8_t hysteresis = 0;
    uint8_t abiResolution = 0;

    constexpr uint16_t encode() const
    {
        return static_cast<uint16_t>(
            (uvwPolePairsMinusOne & 0x07) |
            ((hysteresis & 0x03) << 3) |
            ((abiResolution & 0x07) << 5));
    }

    static constexpr AS5047PSettings2 decode(uint16_t value)
    {
        return {
            static_cast<uint8_t>(value & 0x07),
            static_cast<uint8_t>((value >> 3) & 0x03),
            static_cast<uint8_t>((value >> 5) & 0x07)
        };
    }
};

/**
 * @brief AS5047P driver using a 16-bit SPI HAL.
 *
 * SPI must use mode 1 (CPOL = 0, CPHA = 1), MSB first, and at most 10 MHz.
 */
template <typename SPIBus, typename Delay>
class AS5047P
{
public:
    using SPIHandle = typename SPIBus::Handle;

    explicit AS5047P(const SPIHandle& spiHandle_) : spiHandle(spiHandle_) {}

    bool readRegister(AS5047PRegister address, uint16_t& value, bool rejectErrorFlag = true) const
    {
        const uint16_t command = buildCommand(address, true);
        uint16_t ignored = 0;
        if (!transferFrame(command, ignored))
        {
            return false;
        }

        Delay::microseconds(INTER_FRAME_DELAY_US);

        uint16_t response = 0;
        if (!transferFrame(buildCommand(AS5047PRegister::Nop, true), response))
        {
            return false;
        }

        if (!hasEvenParity(response))
        {
            return false;
        }
        if (rejectErrorFlag && (response & ERROR_FLAG_MASK) != 0)
        {
            return false;
        }

        value = response & DATA_MASK;
        return true;
    }

    bool writeRegister(AS5047PRegister address, uint16_t value) const
    {
        uint16_t ignored = 0;
        if (!transferFrame(buildCommand(address, false), ignored))
        {
            return false;
        }

        Delay::microseconds(INTER_FRAME_DELAY_US);
        return transferFrame(buildDataFrame(value), ignored);
    }

    bool readAngleRaw(uint16_t& angle, bool withDaec = true) const
    {
        uint16_t value = 0;
        const AS5047PRegister address = withDaec
            ? AS5047PRegister::AngleCompensated
            : AS5047PRegister::AngleUncompensated;
        if (!readRegister(address, value))
        {
            return false;
        }

        angle = value & ANGLE_MASK;
        return true;
    }

    /**
     * @brief Read a blocking angle sample converted to Q16.
     */
    bool readBlocking(uint16_t& q16Angle) const
    {
        uint16_t angle14 = 0;
        if (!readAngleRaw(angle14))
        {
            return false;
        }

        q16Angle = static_cast<uint16_t>(angle14 << 2);
        return true;
    }

    /**
     * @brief Start a pipelined AngleUncompensated read.
     *
     * The command frame is built once and reused by fastReadContinue().
     * The response to this first command is returned by the first
     * fastReadContinue() call.
     */
    bool fastReadStart()
    {
        fastReadCommand = buildCommand(AS5047PRegister::AngleUncompensated, true);

        uint16_t ignored = 0;
        if (!transferFrame(fastReadCommand, ignored))
        {
            return false;
        }
        return true;
    }

    /**
     * @brief Receive the pending angle and immediately request the next one.
     *
     * The response received in this transfer corresponds to the command sent
     * by the previous fastReadStart() or fastReadContinue() call.
     */
    bool fastReadContinue(uint16_t& angle)
    {
        uint16_t response = 0;
        if (!transferFrame(fastReadCommand, response))
        {
            return false;
        }

        angle = static_cast<uint16_t>((response & ANGLE_MASK) << 2);
        return true;
    }

    bool readMagnitude(uint16_t& magnitude) const
    {
        return readRegister(AS5047PRegister::Magnitude, magnitude);
    }

    bool readErrorFlags(AS5047PErrorFlags& status) const
    {
        uint16_t value = 0;
        if (!readRegister(AS5047PRegister::ErrorFlags, value, false))
        {
            return false;
        }

        status.framingError = (value & 0x0001) != 0;
        status.invalidCommand = (value & 0x0002) != 0;
        status.parityError = (value & 0x0004) != 0;
        status.raw = value;
        return true;
    }

    bool readDiagnostics(AS5047PDiagnostics& diagnostics) const
    {
        uint16_t value = 0;
        if (!readRegister(AS5047PRegister::Diagnostics, value))
        {
            return false;
        }

        diagnostics.agc = static_cast<uint8_t>(value & 0x00FF);
        diagnostics.offsetCompensationFinished = (value & 0x0100) != 0;
        diagnostics.cordicOverflow = (value & 0x0200) != 0;
        diagnostics.magneticFieldTooHigh = (value & 0x0400) != 0;
        diagnostics.magneticFieldTooLow = (value & 0x0800) != 0;
        diagnostics.raw = value;
        return true;
    }

    bool readSettings(AS5047PSettings1& settings1, AS5047PSettings2& settings2) const
    {
        uint16_t value = 0;
        if (!readRegister(AS5047PRegister::Settings1, value))
        {
            return false;
        }
        settings1 = AS5047PSettings1::decode(value);

        if (!readRegister(AS5047PRegister::Settings2, value))
        {
            return false;
        }
        settings2 = AS5047PSettings2::decode(value);
        return true;
    }

    bool writeSettings(const AS5047PSettings1& settings1, const AS5047PSettings2& settings2) const
    {
        AS5047PSettings1 writableSettings1 = settings1;
        writableSettings1.factorySetting = true;
        writableSettings1.noiseSetting = false;
        return writeRegister(AS5047PRegister::Settings1, writableSettings1.encode()) &&
               writeRegister(AS5047PRegister::Settings2, settings2.encode());
    }

    bool setZeroPosition(uint16_t zeroPosition) const
    {
        return writeRegister(AS5047PRegister::ZeroPositionMsb,
                             static_cast<uint16_t>((zeroPosition >> 6) & 0xFF)) &&
               writeRegister(AS5047PRegister::ZeroPositionLsb,
                             static_cast<uint16_t>(zeroPosition & 0x3F)) &&
               sendNop();
    }

    static constexpr bool hasEvenParity(uint16_t frame)
    {
        frame ^= frame >> 8;
        frame ^= frame >> 4;
        frame ^= frame >> 2;
        frame ^= frame >> 1;
        return (frame & 0x0001) == 0;
    }

private:
    static constexpr uint16_t DATA_MASK = 0x3FFF;
    static constexpr uint16_t ERROR_FLAG_MASK = 0x4000;
    static constexpr uint16_t ANGLE_MASK = 0x3FFF;
    static constexpr uint32_t TRANSFER_TIMEOUT_MS = 100;
    static constexpr uint32_t INTER_FRAME_DELAY_US = 1;

    static constexpr uint16_t buildCommand(AS5047PRegister address, bool read)
    {
        return withEvenParity(static_cast<uint16_t>(
            (static_cast<uint16_t>(address) & 0x3FFF) |
            (read ? 0x4000 : 0x0000)));
    }

    static constexpr uint16_t buildDataFrame(uint16_t data)
    {
        return withEvenParity(static_cast<uint16_t>(data & 0x3FFF));
    }

    static constexpr uint16_t withEvenParity(uint16_t value)
    {
        return static_cast<uint16_t>(
            value | (hasEvenParity(value) ? 0x0000 : 0x8000));
    }

    bool transferFrame(uint16_t txFrame, uint16_t& rxFrame) const
    {
        return SPIBus::transferWord(spiHandle,
                                    txFrame,
                                    rxFrame,
                                    TRANSFER_TIMEOUT_MS);
    }

    bool sendNop() const
    {
        uint16_t ignored = 0;
        return transferFrame(buildCommand(AS5047PRegister::Nop, true), ignored);
    }

    SPIHandle spiHandle;
    uint16_t fastReadCommand = 0;
};

} // namespace Drivers
