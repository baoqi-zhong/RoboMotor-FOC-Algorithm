/**
 * @file HighResTimer.cpp
 * @brief High-resolution hardware timer with software overflow extension.
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the root directory for full license text.
 */
#include "HighResTimer.hpp"
# if 0
namespace Utils
{
namespace
{

class InterruptGuard
{
public:
    InterruptGuard() noexcept
        : previousPrimask(__get_PRIMASK())
    {
        __disable_irq();
    }

    ~InterruptGuard() noexcept
    {
        __set_PRIMASK(previousPrimask);
    }

    InterruptGuard(const InterruptGuard&) = delete;
    InterruptGuard& operator=(const InterruptGuard&) = delete;

private:
    uint32_t previousPrimask;
};

} // namespace

HighResTimer::HighResTimer(
    TimerTickReader readTimerTick_,
    float tickFrequencyHz_,
    uint64_t ticksPerOverflow_) noexcept
    : readTimerTick(readTimerTick_),
      tickFrequencyHz(tickFrequencyHz_),
      ticksPerOverflow(ticksPerOverflow_),
      overflowCount(0)
{
}

float HighResTimer::getTime() const noexcept
{
    if (readTimerTick == nullptr ||
        tickFrequencyHz <= 0.0f ||
        ticksPerOverflow == 0)
    {
        return 0.0f;
    }

    uint32_t timerTick = 0;
    uint64_t overflows = 0;

    {
        /*
         * The overflow counter is 64-bit, so its access is not atomic on the
         * Cortex-M4. Disable maskable interrupts while capturing both values.
         * PRIMASK is restored rather than unconditionally enabling interrupts,
         * which keeps this function safe when called from nested ISRs.
         */
        InterruptGuard guard;
        timerTick = readTimerTick();
        overflows = overflowCount;
    }

    return (static_cast<float>(overflows) *
                static_cast<float>(ticksPerOverflow) +
            static_cast<float>(timerTick)) /
           tickFrequencyHz;
}

void HighResTimer::timerOverflow() noexcept
{
    const uint64_t nextOverflowCount = overflowCount + 1U;
    overflowCount = nextOverflowCount;
}

} // namespace Utils
#endif