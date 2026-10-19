/**
 * @file DRV8301.hpp
 * @brief Generic DRV8301 three-phase gate driver interface.
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

enum class DRV8301Register : uint8_t
{
    Status1 = 0x00,
    Status2 = 0x01,
    Control1 = 0x02,
    Control2 = 0x03
};

enum class DRV8301GateCurrent : uint16_t
{
    Peak1A7 = 0x0,
    Peak0A7 = 0x1,
    Peak0A25 = 0x2,
    Reserved = 0x3
};

enum class DRV8301PwmMode : uint16_t
{
    SixPwmInputs = 0x0,
    ThreePwmInputs = 0x1
};

enum class DRV8301OvercurrentMode : uint16_t
{
    CurrentLimit = 0x0,
    LatchShutdown = 0x1,
    ReportOnly = 0x2,
    Disabled = 0x3
};

enum class DRV8301OctwMode : uint16_t
{
    ReportOverTemperatureAndOvercurrent = 0x0,
    ReportOverTemperatureOnly = 0x1,
    ReportOvercurrentOnly = 0x2,
    Reserved = 0x3
};

enum class DRV8301ShuntGain : uint16_t
{
    Gain10 = 0x0,
    Gain20 = 0x1,
    Gain40 = 0x2,
    Gain80 = 0x3
};

enum class DRV8301OvercurrentOffTime : uint16_t
{
    CycleByCycle = 0x0,
    OffTimeControl = 0x1
};

enum class DRV8301AmplifierChannel : uint8_t
{
    Channel1,
    Channel2
};

struct DRV8301ControlRegister1
{
    DRV8301GateCurrent gateCurrent = DRV8301GateCurrent::Peak1A7;
    bool resetGate = false;
    DRV8301PwmMode pwmMode = DRV8301PwmMode::SixPwmInputs;
    DRV8301OvercurrentMode overcurrentMode = DRV8301OvercurrentMode::CurrentLimit;
    uint8_t overcurrentAdjustment = 0;

    constexpr uint16_t encode() const
    {
        return static_cast<uint16_t>(gateCurrent) |
               (resetGate ? 0x0004 : 0x0000) |
               static_cast<uint16_t>(static_cast<uint16_t>(pwmMode) << 3) |
               static_cast<uint16_t>(static_cast<uint16_t>(overcurrentMode) << 4) |
               static_cast<uint16_t>((overcurrentAdjustment & 0x1F) << 6);
    }

    static constexpr DRV8301ControlRegister1 decode(uint16_t value)
    {
        return {
            static_cast<DRV8301GateCurrent>(value & 0x0003),
            (value & 0x0004) != 0,
            static_cast<DRV8301PwmMode>((value >> 3) & 0x0001),
            static_cast<DRV8301OvercurrentMode>((value >> 4) & 0x0003),
            static_cast<uint8_t>((value >> 6) & 0x001F)
        };
    }
};

struct DRV8301ControlRegister2
{
    DRV8301OctwMode octwMode = DRV8301OctwMode::ReportOverTemperatureAndOvercurrent;
    DRV8301ShuntGain shuntGain = DRV8301ShuntGain::Gain10;
    bool dcCalibrationChannel1 = false;
    bool dcCalibrationChannel2 = false;
    DRV8301OvercurrentOffTime overcurrentOffTime = DRV8301OvercurrentOffTime::CycleByCycle;

    constexpr uint16_t encode() const
    {
        return static_cast<uint16_t>(octwMode) |
               static_cast<uint16_t>(static_cast<uint16_t>(shuntGain) << 2) |
               (dcCalibrationChannel1 ? 0x0010 : 0x0000) |
               (dcCalibrationChannel2 ? 0x0020 : 0x0000) |
               static_cast<uint16_t>(static_cast<uint16_t>(overcurrentOffTime) << 6);
    }

    static constexpr DRV8301ControlRegister2 decode(uint16_t value)
    {
        return {
            static_cast<DRV8301OctwMode>(value & 0x0003),
            static_cast<DRV8301ShuntGain>((value >> 2) & 0x0003),
            (value & 0x0010) != 0,
            (value & 0x0020) != 0,
            static_cast<DRV8301OvercurrentOffTime>((value >> 6) & 0x0001)
        };
    }
};

struct DRV8301StatusRegister1
{
    bool phaseCOvercurrentLowSide = false;
    bool phaseCOvercurrentHighSide = false;
    bool phaseBOvercurrentLowSide = false;
    bool phaseBOvercurrentHighSide = false;
    bool phaseAOvercurrentLowSide = false;
    bool phaseAOvercurrentHighSide = false;
    bool overTemperatureWarning = false;
    bool overTemperatureShutdown = false;
    bool powerSupplyUndervoltage = false;
    bool gateDriveSupplyUndervoltage = false;
    bool fault = false;
    uint16_t raw = 0;

    static constexpr DRV8301StatusRegister1 decode(uint16_t value)
    {
        return {
            (value & 0x0001) != 0,
            (value & 0x0002) != 0,
            (value & 0x0004) != 0,
            (value & 0x0008) != 0,
            (value & 0x0010) != 0,
            (value & 0x0020) != 0,
            (value & 0x0040) != 0,
            (value & 0x0080) != 0,
            (value & 0x0100) != 0,
            (value & 0x0200) != 0,
            (value & 0x0400) != 0,
            static_cast<uint16_t>(value & 0x07FF)
        };
    }

    constexpr bool hasFault() const
    {
        return raw != 0;
    }
};

struct DRV8301StatusRegister2
{
    bool gateDriveSupplyOvervoltage = false;
    uint8_t deviceId = 0;
    uint16_t raw = 0;

    static constexpr DRV8301StatusRegister2 decode(uint16_t value)
    {
        return {
            (value & 0x0080) != 0,
            static_cast<uint8_t>(value & 0x000F),
            static_cast<uint16_t>(value & 0x00FF)
        };
    }
};

struct DRV8301FaultStatus
{
    DRV8301StatusRegister1 status1;
    DRV8301StatusRegister2 status2;

    constexpr bool hasFault() const
    {
        return status1.hasFault() || status2.gateDriveSupplyOvervoltage;
    }
};

/**
 * @brief DRV8301 driver using an SPI HAL with software chip select.
 *
 * Configure the SPI peripheral for 16-bit frames, MSB first, mode 1
 * (CPOL = 0, CPHA = 1), and SCLK no faster than 10 MHz.
 */
template <typename SPIBus, typename GPIO, typename Delay>
class DRV8301
{
public:
    using SPIHandle = typename SPIBus::Handle;
    using GPIOHandle = typename GPIO::Handle;

    struct Pins
    {
        GPIOHandle enableGate;
        GPIOHandle nFault;
        GPIOHandle nOctw;
        GPIOHandle dcCal;
    };

    explicit DRV8301(const SPIHandle& spiHandle_, const Pins& pins_)
        : spiHandle(spiHandle_), pins(pins_) {}

    bool init(const DRV8301ControlRegister1& control1 = {},
              const DRV8301ControlRegister2& control2 = {})
    {
        if (!reset())
        {
            return false;
        }

        if (!configure(control1, control2))
        {
            return false;
        }

        DRV8301FaultStatus status;
        return readFaultStatus(status) && !status.hasFault();
    }

    bool configure(const DRV8301ControlRegister1& control1,
                   const DRV8301ControlRegister2& control2)
    {
        DRV8301ControlRegister1 writableControl1 = control1;
        writableControl1.resetGate = false;

        return writeRegisterVerify(DRV8301Register::Control1, writableControl1.encode()) &&
               writeRegisterVerify(DRV8301Register::Control2, control2.encode());
    }

    bool reset()
    {
        enabled = false;
        if (!GPIO::write(pins.enableGate, false))
        {
            return false;
        }

        // A low pulse longer than 20 us guarantees a full EN_GATE reset.
        Delay::milliseconds(1);

        if (!GPIO::write(pins.enableGate, true))
        {
            return false;
        }

        // tSPI_READY is at most 10 ms after EN_GATE goes high.
        Delay::milliseconds(10);
        enabled = true;
        return true;
    }

    bool enable()
    {
        if (!GPIO::write(pins.enableGate, true))
        {
            return false;
        }
        Delay::milliseconds(10);
        enabled = true;
        return true;
    }

    bool disable()
    {
        enabled = false;
        return GPIO::write(pins.enableGate, false);
    }

    bool isEnabled() const
    {
        return enabled;
    }

    bool isReady(bool& ready) const
    {
        bool nFaultHigh = false;
        if (!GPIO::read(pins.nFault, nFaultHigh))
        {
            return false;
        }
        ready = enabled && nFaultHigh;
        return true;
    }

    bool readFaultPin(bool& active) const
    {
        bool high = false;
        if (!GPIO::read(pins.nFault, high))
        {
            return false;
        }
        active = !high;
        return true;
    }

    bool readWarningPin(bool& active) const
    {
        bool high = false;
        if (!GPIO::read(pins.nOctw, high))
        {
            return false;
        }
        active = !high;
        return true;
    }

    bool setDcCalibrationPin(bool active)
    {
        return GPIO::write(pins.dcCal, active);
    }

    bool readRegister(DRV8301Register address, uint16_t& value) const
    {
        uint16_t ignored = 0;
        const uint16_t command = buildCommand(true, address, 0);
        if (!SPIBus::transferWord(spiHandle, command, ignored, TRANSFER_TIMEOUT_MS))
        {
            return false;
        }

        uint16_t response = 0;
        if (!SPIBus::transferWord(spiHandle, command, response, TRANSFER_TIMEOUT_MS))
        {
            return false;
        }

        if ((response & FRAME_FAULT_MASK) != 0 ||
            (response & ADDRESS_MASK) !=
                static_cast<uint16_t>((static_cast<uint8_t>(address) & 0x000F) << 11))
        {
            return false;
        }

        value = response & DATA_MASK;
        return true;
    }

    bool writeRegister(DRV8301Register address, uint16_t value) const
    {
        if (!isWritable(address))
        {
            return false;
        }

        uint16_t response = 0;
        return SPIBus::transferWord(spiHandle,
                                    buildCommand(false, address, value),
                                    response,
                                    TRANSFER_TIMEOUT_MS);
    }

    bool readControlRegister1(DRV8301ControlRegister1& control) const
    {
        uint16_t value = 0;
        if (!readRegister(DRV8301Register::Control1, value))
        {
            return false;
        }
        control = DRV8301ControlRegister1::decode(value);
        return true;
    }

    bool writeControlRegister1(const DRV8301ControlRegister1& control) const
    {
        return writeRegister(DRV8301Register::Control1, control.encode());
    }

    bool readControlRegister2(DRV8301ControlRegister2& control) const
    {
        uint16_t value = 0;
        if (!readRegister(DRV8301Register::Control2, value))
        {
            return false;
        }
        control = DRV8301ControlRegister2::decode(value);
        return true;
    }

    bool writeControlRegister2(const DRV8301ControlRegister2& control) const
    {
        return writeRegister(DRV8301Register::Control2, control.encode());
    }

    bool readStatusRegister1(DRV8301StatusRegister1& status) const
    {
        uint16_t value = 0;
        if (!readRegister(DRV8301Register::Status1, value))
        {
            return false;
        }
        status = DRV8301StatusRegister1::decode(value);
        return true;
    }

    bool readStatusRegister2(DRV8301StatusRegister2& status) const
    {
        uint16_t value = 0;
        if (!readRegister(DRV8301Register::Status2, value))
        {
            return false;
        }
        status = DRV8301StatusRegister2::decode(value);
        return true;
    }

    bool readFaultStatus(DRV8301FaultStatus& status) const
    {
        return readStatusRegister1(status.status1) &&
               readStatusRegister2(status.status2);
    }

    bool getDeviceId(uint8_t& deviceId) const
    {
        DRV8301StatusRegister2 status;
        if (!readStatusRegister2(status))
        {
            return false;
        }
        deviceId = status.deviceId;
        return true;
    }

    bool clearLatchedFaults()
    {
        DRV8301ControlRegister1 control;
        if (!readControlRegister1(control))
        {
            return false;
        }

        control.resetGate = true;
        if (!writeControlRegister1(control))
        {
            return false;
        }
        Delay::milliseconds(1);

        return readControlRegister1(control) && !control.resetGate;
    }

    bool setGateCurrent(DRV8301GateCurrent gateCurrent)
    {
        return updateControlRegister1(0x0003, static_cast<uint16_t>(gateCurrent));
    }

    bool setPwmMode(DRV8301PwmMode mode)
    {
        return updateControlRegister1(0x0008, static_cast<uint16_t>(static_cast<uint16_t>(mode) << 3));
    }

    bool setOvercurrentMode(DRV8301OvercurrentMode mode)
    {
        return updateControlRegister1(0x0030, static_cast<uint16_t>(static_cast<uint16_t>(mode) << 4));
    }

    bool setOvercurrentAdjustment(uint8_t code)
    {
        if (code > 31)
        {
            return false;
        }
        return updateControlRegister1(0x07C0, static_cast<uint16_t>(static_cast<uint16_t>(code) << 6));
    }

    bool setOvercurrentProtection(DRV8301OvercurrentMode mode, uint8_t adjustmentCode)
    {
        if (adjustmentCode > 31)
        {
            return false;
        }
        const uint16_t value = static_cast<uint16_t>(static_cast<uint16_t>(mode) << 4) |
                               static_cast<uint16_t>(static_cast<uint16_t>(adjustmentCode) << 6);
        return updateControlRegister1(0x07F0, value);
    }

    bool setOctwMode(DRV8301OctwMode mode)
    {
        return updateControlRegister2(0x0003, static_cast<uint16_t>(mode));
    }

    bool setShuntAmplifierGain(DRV8301ShuntGain gain)
    {
        return updateControlRegister2(0x000C, static_cast<uint16_t>(static_cast<uint16_t>(gain) << 2));
    }

    bool setDcCalibrationChannel(DRV8301AmplifierChannel channel, bool enableChannel)
    {
        const uint16_t mask = channel == DRV8301AmplifierChannel::Channel1 ? 0x0010 : 0x0020;
        return updateControlRegister2(mask, enableChannel ? mask : 0x0000);
    }

    bool setDcCalibration(bool channel1, bool channel2)
    {
        const uint16_t value = (channel1 ? 0x0010 : 0x0000) |
                               (channel2 ? 0x0020 : 0x0000);
        return updateControlRegister2(0x0030, value);
    }

    bool setOvercurrentOffTime(DRV8301OvercurrentOffTime mode)
    {
        return updateControlRegister2(0x0040, static_cast<uint16_t>(static_cast<uint16_t>(mode) << 6));
    }

    static constexpr float shuntGainValue(DRV8301ShuntGain gain)
    {
        switch (gain)
        {
            case DRV8301ShuntGain::Gain10: return 10.0f;
            case DRV8301ShuntGain::Gain20: return 20.0f;
            case DRV8301ShuntGain::Gain40: return 40.0f;
            case DRV8301ShuntGain::Gain80: return 80.0f;
        }
        return 0.0f;
    }

    static constexpr float overcurrentTripVoltage(uint8_t code)
    {
        return code < 32 ? OVERCURRENT_TRIP_VOLTAGES[code] : 0.0f;
    }

private:
    inline static constexpr float OVERCURRENT_TRIP_VOLTAGES[32] = {
        0.060f, 0.068f, 0.076f, 0.086f, 0.097f, 0.109f, 0.123f, 0.138f,
        0.155f, 0.175f, 0.197f, 0.222f, 0.250f, 0.282f, 0.317f, 0.358f,
        0.403f, 0.454f, 0.511f, 0.576f, 0.648f, 0.730f, 0.822f, 0.926f,
        1.043f, 1.175f, 1.324f, 1.491f, 1.679f, 1.892f, 2.131f, 2.400f
    };

    static constexpr uint32_t TRANSFER_TIMEOUT_MS = 100;
    static constexpr uint16_t READ_COMMAND_MASK = 0x8000;
    static constexpr uint16_t ADDRESS_MASK = 0x7800;
    static constexpr uint16_t DATA_MASK = 0x07FF;
    static constexpr uint16_t FRAME_FAULT_MASK = 0x8000;

    static constexpr uint16_t buildCommand(bool read, DRV8301Register address, uint16_t data)
    {
        return (read ? READ_COMMAND_MASK : 0x0000) |
               static_cast<uint16_t>((static_cast<uint8_t>(address) & 0x000F) << 11) |
               static_cast<uint16_t>(data & DATA_MASK);
    }

    static constexpr bool isWritable(DRV8301Register address)
    {
        return address == DRV8301Register::Control1 ||
               address == DRV8301Register::Control2;
    }

    bool writeRegisterVerify(DRV8301Register address, uint16_t value) const
    {
        for (uint8_t attempt = 0; attempt < 3; ++attempt)
        {
            uint16_t readback = 0;
            if (writeRegister(address, value) &&
                readRegister(address, readback) &&
                readback == value)
            {
                return true;
            }
        }
        return false;
    }

    bool updateControlRegister1(uint16_t mask, uint16_t value)
    {
        DRV8301ControlRegister1 control;
        if (!readControlRegister1(control))
        {
            return false;
        }

        const uint16_t updated = static_cast<uint16_t>((control.encode() & ~mask) | (value & mask));
        return writeRegisterVerify(DRV8301Register::Control1, updated);
    }

    bool updateControlRegister2(uint16_t mask, uint16_t value)
    {
        DRV8301ControlRegister2 control;
        if (!readControlRegister2(control))
        {
            return false;
        }

        const uint16_t updated = static_cast<uint16_t>((control.encode() & ~mask) | (value & mask));
        return writeRegisterVerify(DRV8301Register::Control2, updated);
    }

    SPIHandle spiHandle;
    Pins pins;
    bool enabled = false;
};

} // namespace Drivers
