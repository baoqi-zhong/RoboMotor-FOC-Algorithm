/**
 * @file HighResTimer.hpp
 * @brief High-resolution hardware timer with software overflow extension.
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the root directory for full license text.
 */
#pragma once

#include <stdint.h>

namespace Utils
{

/**
 * @brief Convert a hardware timer tick counter into elapsed seconds.
 *
 * The configured hardware timer must be free-running. The supplied callback
 * must return its current raw tick and must not reset or modify the timer.
 */
class HighResTimer
{
public:
    using TimerTickReader = uint32_t (*)();

    /**
     * @param readTimerTick_ Function returning the current hardware tick.
     * @param tickFrequencyHz Hardware timer tick frequency in hertz.
     * @param ticksPerOverflow Number of ticks in one hardware wrap. For a
     *        16-bit up-counter this is 65536; for a 32-bit counter it is 2^32.
     */
    explicit HighResTimer(
        TimerTickReader readTimerTick_,
        float tickFrequencyHz,
        uint64_t ticksPerOverflow = static_cast<uint64_t>(1) << 32) noexcept;

    /**
     * @brief Return elapsed time in seconds.
     *
     * The hardware tick and software overflow count are captured as one
     * interrupt-safe snapshot, so timerOverflow() may run concurrently with
     * callers on another interrupt priority level.
     */
    float getTime() const noexcept;

    /**
     * @brief Extend the timer by one overflow period.
     *
     * Call this exactly once from the hardware timer's overflow interrupt.
     * Keep the handler short; it should contain no blocking operations.
     */
    void timerOverflow() noexcept;

private:
    TimerTickReader readTimerTick;
    float tickFrequencyHz;
    uint64_t ticksPerOverflow;
    volatile uint64_t overflowCount;
};

} // namespace Utils
