/**
 * @file MotorBuzzer.cpp
 * @brief Motor buzzer implementation using FOC voltage injection.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#if 0

#include "MotorBuzzer.hpp"
#include "ThreePhaseFOC.hpp"
#include "MotorControl.hpp"

namespace Control
{
namespace MotorBuzzer
{
/* 固定使用 Timer 17 */
MotorBuzzerStatus motorBuzzerStatus;
Control::FOC::FOCConfig focConfigBackUp;

void addNote(uint16_t frequency, uint16_t duration, float intensity)
{
    if(frequency < 0.0f || duration <= 0.0f)
    {
        return;
    }

    if((motorBuzzerStatus.tail + 1) % MAX_MOTOR_BUZZER_QUEUE_SIZE == motorBuzzerStatus.head)
    {
        // 队列满了, 丢掉这个 note
        return;
    }

    motorBuzzerStatus.noteQueue[motorBuzzerStatus.tail].frequency = frequency;
    motorBuzzerStatus.noteQueue[motorBuzzerStatus.tail].duration = duration;
    motorBuzzerStatus.noteQueue[motorBuzzerStatus.tail].intensity = intensity;
    motorBuzzerStatus.tail = (motorBuzzerStatus.tail + 1) % MAX_MOTOR_BUZZER_QUEUE_SIZE;
}

void playCurrentNote()
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    uint16_t timerPeriod = 170000000 / (uint32_t)(motorBuzzerStatus.noteQueue[motorBuzzerStatus.head].frequency) / 2;
    __HAL_TIM_SetAutoreload(&htim1, timerPeriod);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, timerPeriod / 10);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
}

void start()
{
    focConfigBackUp = Control::FOC::focConfig;
    playCurrentNote();
}

void stop()
{
    // 停止播放, 清空队列
    motorBuzzerStatus.head = 0;
    motorBuzzerStatus.tail = 0;
    Control::MotorControl::motorControlStatus.targetIq = 0.0f;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    Control::FOC::setFOCConfig(&focConfigBackUp);
}

void update()
{
    if(motorBuzzerStatus.head != motorBuzzerStatus.tail)
    {
        if(!motorBuzzerStatus.isPlaying)
        {
            motorBuzzerStatus.isPlaying = 1;
            start();
        }

        Note* currentNote = &motorBuzzerStatus.noteQueue[motorBuzzerStatus.head];
        currentNote->duration -= 1;
        if(currentNote->duration == 0)
        {
            // 这个 note 播放完了, 播放下一个 note
            motorBuzzerStatus.head = (motorBuzzerStatus.head + 1) % MAX_MOTOR_BUZZER_QUEUE_SIZE;
            if(motorBuzzerStatus.head == motorBuzzerStatus.tail)
            {
                // 没有 note 了, 停止播放
                stop();
            }
            else
            {
                playCurrentNote();
            }
        }
    }
}

} // namespace MotorBuzzer
} // namespace Control

#endif