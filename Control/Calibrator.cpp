/**
 * @file Calibrator.cpp
 * @brief Calibration logic for ADC offsets and motor parameters.
 * @author baoqi-zhong (zzhongas@connect.ust.hk)
 *
 * Part of RoboMotor-FOC-Algorithm.
 * Copyright (c) 2026 baoqi-zhong
 *
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for full license text.
 */
#include "Calibrator.hpp"

#include "MotorControl.hpp"
#include "ThreePhaseFOC.hpp"
#include "Encoder.hpp"
#include "CordicHelper.hpp"
#include "M3508_LinerHallEncoder.hpp"
#include "ADC.hpp"
#include "Math.hpp"
#include "ConfigLoader.hpp"

/**
 * 我们假定 ADC 电流和电压的 Gain 完全正确, 依赖此数据校准其他所有数据.
 * 校准流程: 
 *  1. 校准 ADC 零偏. 
 *      1.1 实施步骤: 关闭 MOS 情况下读取 ADC 的值.
 * 
 *  2. 校准 TIM CH 与电流 ADC 的对应关系. 判断电流感应顺序是否配置正确. 计算物理误差的比例供参考.
 *      2.1 实施步骤: 依次对每个相位施加一个固定电压, 读取 ADC 的值, 读取编码器值.
 *      2.2 判断 TIM CH 与电流 ADC 对应是否正确, 读取到的电流极性是否正确.
 *      2.3 计算电流和与 0 的相对偏差值, 计算三 ADC 通道最大电流值的一致性.
 *      2.4 计算电机相电阻
 *      [2.2-2.4]: CALCULATION_1
 * 
 *  3. 校准编码器零点.
 *      3.1 实施步骤: 施加某相位的固定电压, 读取编码器的值, 记录为零点位置.
 * 
 *  4. 校准编码器误差.
 *      4.1 实施步骤: 施加极对数个数的不同相位电压(目的是排除齿槽效应影响), 读取编码器的值, 记录误差.
 *      4.2 (可选的)旋转反向进行上述步骤
 * 
 *  5. 校准电机参数
 *      5.1 实施步骤: 给定固定 Uq, 记录旋转速度, 计算 KV.
 */

namespace Control
{
namespace Calibrator
{
// 开环电角度(不考虑 Pole Pairs)
CalibratorStatus calibratorStatus;
CalibrationResult calibrationResult;

/* 初始化时无法得知用户传入的, 因此默认一个 PolarPairs */
#define PP 7

float currentValues[3][3] = {0.0f};
int32_t encoderValues[28][2] = {0};
uint8_t nextCurrentSampleIndex = 0;
uint8_t nextEncoderSampleIndex = 0;


/*
 * 使用移动平均消除齿槽效应影响
 * 每个电角度采样 16 个点, 一圈一共采样 7 * 16 = 112 个点, 以 16 为 window 大小进行移动平均
 * 移动速度是每电角度 200ms.
 * 定义多一个 encoderErrors 是为了省内存, 用 encoderValues 太浪费了
 * PP + 1 是为了后续滤波方便计算
 */
#define ENCODER_SAMPLES_PER_ELECTRIC_CYCLE          128
#define ENCODER_CALIBRATE_TIME_PER_ELECTRIC_CYCLE   800
int16_t encoderErrors[ENCODER_SAMPLES_PER_ELECTRIC_CYCLE * (PP + 1)] = {0};
float filteredEncoderErrors[ENCODER_SAMPLES_PER_ELECTRIC_CYCLE * (PP + 1)] = {0};
float secondFilteredEncoderErrors[ENCODER_SAMPLES_PER_ELECTRIC_CYCLE * PP] = {0};
uint16_t nextEncoderErrorSampleIndex  = 0;

float display[5] = {0};

#define ALIGN_VOLTAGE   1.0f
// 所有的 Action 在其规定时间内线性插值执行, 在结束的一瞬间达到目标的 angle 和 voltage.
CalibrateAction calibrateSteps[] = {
    {.action = CalibrateActionType::START,                          .angle = 0,             .voltage = 0.0f,            .duration = 0},

    // Step 1
    {.action = CalibrateActionType::MOVE_TO_POSITION,               .angle = 0,             .voltage = ALIGN_VOLTAGE,   .duration = 200},   // 线性移动
    {.action = CalibrateActionType::MOVE_TO_POSITION,               .angle = 0,             .voltage = ALIGN_VOLTAGE,   .duration = 200},   // 停止 200ms 让振荡稳定
    {.action = CalibrateActionType::RECORD_CURRENT_VALUES,          .angle = 0,             .voltage = ALIGN_VOLTAGE,   .duration = 0},
    {.action = CalibrateActionType::RECORD_ENCODER_VALUES,          .angle = 0,             .voltage = ALIGN_VOLTAGE,   .duration = 0},

    {.action = CalibrateActionType::MOVE_TO_POSITION,               .angle = 21845,         .voltage = ALIGN_VOLTAGE,   .duration = 200},   // 线性移动
    {.action = CalibrateActionType::MOVE_TO_POSITION,               .angle = 21845,         .voltage = ALIGN_VOLTAGE,   .duration = 200},   // 停止 200ms 让振荡稳定
    {.action = CalibrateActionType::RECORD_CURRENT_VALUES,          .angle = 21845,         .voltage = ALIGN_VOLTAGE,   .duration = 0},
    {.action = CalibrateActionType::RECORD_ENCODER_VALUES,          .angle = 21845,         .voltage = ALIGN_VOLTAGE,   .duration = 0},


    {.action = CalibrateActionType::MOVE_TO_POSITION,               .angle = 43691,         .voltage = ALIGN_VOLTAGE,   .duration = 200},   // 线性移动
    {.action = CalibrateActionType::MOVE_TO_POSITION,               .angle = 43691,         .voltage = ALIGN_VOLTAGE,   .duration = 200},   // 停止 200ms 让振荡稳定
    {.action = CalibrateActionType::RECORD_CURRENT_VALUES,          .angle = 43691,         .voltage = ALIGN_VOLTAGE,   .duration = 0},
    {.action = CalibrateActionType::RECORD_ENCODER_VALUES,          .angle = 43691,         .voltage = ALIGN_VOLTAGE,   .duration = 0},
    {.action = CalibrateActionType::CALCULATION_1,                  .angle = 43691,         .voltage = ALIGN_VOLTAGE,   .duration = 0},


    // Step 2
    {.action = CalibrateActionType::MOVE_TO_POSITION,               .angle = -65536,        .voltage = ALIGN_VOLTAGE,   .duration = ENCODER_CALIBRATE_TIME_PER_ELECTRIC_CYCLE},         // 线性移动到负的一个极对点
    {.action = CalibrateActionType::MOVE_TO_POSITION,               .angle = 0,             .voltage = ALIGN_VOLTAGE,   .duration = ENCODER_CALIBRATE_TIME_PER_ELECTRIC_CYCLE},         // 线性移动到负的一个极对点
    {.action = CalibrateActionType::RECORD_ENCODER_ERROR_START,     .angle = 0,             .voltage = ALIGN_VOLTAGE,   .duration = 0},
    {.action = CalibrateActionType::MOVE_TO_POSITION,               .angle = 65536 * PP,    .voltage = ALIGN_VOLTAGE,   .duration = ENCODER_CALIBRATE_TIME_PER_ELECTRIC_CYCLE * PP},    // 线性移动
    {.action = CalibrateActionType::RECORD_ENCODER_ERROR_END,       .angle = 65536 * PP,    .voltage = ALIGN_VOLTAGE,   .duration = 0},

    {.action = CalibrateActionType::MOVE_TO_POSITION,               .angle = 65536 * (PP+1),.voltage = ALIGN_VOLTAGE,   .duration = ENCODER_CALIBRATE_TIME_PER_ELECTRIC_CYCLE},         // 线性移动
    {.action = CalibrateActionType::MOVE_TO_POSITION,               .angle = 65536 * PP,    .voltage = ALIGN_VOLTAGE,   .duration = ENCODER_CALIBRATE_TIME_PER_ELECTRIC_CYCLE },        // 线性移动
    {.action = CalibrateActionType::RECORD_ENCODER_ERROR_START_REVERSE,     .angle = 0,     .voltage = ALIGN_VOLTAGE,   .duration = 0},
    {.action = CalibrateActionType::MOVE_TO_POSITION,               .angle = 0,             .voltage = ALIGN_VOLTAGE,   .duration = ENCODER_CALIBRATE_TIME_PER_ELECTRIC_CYCLE * PP},    // 线性移动
    {.action = CalibrateActionType::RECORD_ENCODER_ERROR_END,       .angle = 0,             .voltage = ALIGN_VOLTAGE,   .duration = 0},
    {.action = CalibrateActionType::CALCULATION_2,                  .angle = 0,             .voltage = ALIGN_VOLTAGE,   .duration = 0},
    {.action = CalibrateActionType::SHOW_ENCODER_RESULT,            .angle = 0,             .voltage = ALIGN_VOLTAGE,   .duration = ENCODER_SAMPLES_PER_ELECTRIC_CYCLE * PP},

    // 再次校准看看, 可以确认校准很有效.
    // {.action = CalibrateActionType::MOVE_TO_POSITION,               .angle = -65536,        .voltage = ALIGN_VOLTAGE,   .duration = ENCODER_CALIBRATE_TIME_PER_ELECTRIC_CYCLE},         // 线性移动到负的一个极对点
    // {.action = CalibrateActionType::MOVE_TO_POSITION,               .angle = 0,             .voltage = ALIGN_VOLTAGE,   .duration = ENCODER_CALIBRATE_TIME_PER_ELECTRIC_CYCLE},         // 线性移动到负的一个极对点
    // {.action = CalibrateActionType::RECORD_ENCODER_ERROR_START,     .angle = 0,             .voltage = ALIGN_VOLTAGE,   .duration = 0},
    // {.action = CalibrateActionType::MOVE_TO_POSITION,               .angle = 65536 * PP,    .voltage = ALIGN_VOLTAGE,   .duration = ENCODER_CALIBRATE_TIME_PER_ELECTRIC_CYCLE * PP},    // 线性移动
    // {.action = CalibrateActionType::RECORD_ENCODER_ERROR_END,       .angle = 65536 * PP,    .voltage = ALIGN_VOLTAGE,   .duration = 0},

    // {.action = CalibrateActionType::MOVE_TO_POSITION,               .angle = 65536 * (PP+1),.voltage = ALIGN_VOLTAGE,   .duration = ENCODER_CALIBRATE_TIME_PER_ELECTRIC_CYCLE},         // 线性移动
    // {.action = CalibrateActionType::MOVE_TO_POSITION,               .angle = 65536 * PP,    .voltage = ALIGN_VOLTAGE,   .duration = ENCODER_CALIBRATE_TIME_PER_ELECTRIC_CYCLE },        // 线性移动
    // {.action = CalibrateActionType::RECORD_ENCODER_ERROR_START_REVERSE,     .angle = 0,     .voltage = ALIGN_VOLTAGE,   .duration = 0},
    // {.action = CalibrateActionType::MOVE_TO_POSITION,               .angle = 0,             .voltage = ALIGN_VOLTAGE,   .duration = ENCODER_CALIBRATE_TIME_PER_ELECTRIC_CYCLE * PP},    // 线性移动
    // {.action = CalibrateActionType::RECORD_ENCODER_ERROR_END,       .angle = 0,             .voltage = ALIGN_VOLTAGE,   .duration = 0},
    // {.action = CalibrateActionType::CALCULATION_2,                  .angle = 0,             .voltage = ALIGN_VOLTAGE,   .duration = 0},
    // {.action = CalibrateActionType::SHOW_ENCODER_RESULT,            .angle = 0,             .voltage = ALIGN_VOLTAGE,   .duration = ENCODER_SAMPLES_PER_ELECTRIC_CYCLE * PP},

    {.action = CalibrateActionType::MOVE_TO_POSITION,               .angle = 0,             .voltage = 0.0f,            .duration = 200},     // 释放


    // Step 3
    // {.action = CalibrateActionType::START_FOC,                      .angle = 0,             .voltage = 0,               .duration = 0},
    // {.action = CalibrateActionType::SLEEP,                          .angle = 0,             .voltage = 0,               .duration = 1000},


    {.action = CalibrateActionType::END ,                           .angle = 0,             .voltage = 0.0f,            .duration = 0},
};

void openLoopMoveTo(int32_t angle, float voltage)
{
    calibratorStatus.currentAngle = angle;
    calibratorStatus.currentVoltage = voltage;

    static float cordicOutputSinMulUd;
    static float cordicOutputCosMulUd;
    hcordic.Instance->WDATA = (Utils::CordicHelper::singleFloatToCordic15(voltage / 24.0f) << 16) | (angle & 0xFFFF);
    Utils::CordicHelper::cordic15ToDualFloat((int32_t)(hcordic.Instance->RDATA), &cordicOutputSinMulUd, &cordicOutputCosMulUd);
    
    Control::FOC::outputUalpha = cordicOutputCosMulUd;
    Control::FOC::outputUbeta  = cordicOutputSinMulUd;

    Control::FOC::setPhraseVoltage(Control::FOC::outputUalpha, Control::FOC::outputUbeta);
}

void init()
{
    calibratorStatus.currentError = 0.0f;
    calibratorStatus.currentAngle = 0;
    calibratorStatus.currentVoltage = 0.0f;
    calibratorStatus.currentStepIndex = 0;
    calibratorStatus.currentStepElapsedTime = 0;
}

void update()
{
    // 1KHz 调用
    CalibrateAction currentAction = calibrateSteps[calibratorStatus.currentStepIndex];

    if(currentAction.action == CalibrateActionType::END)
    {
        // End 之后不会再主动进来 update 函数
        Sensor::Encoder::encoderConfig.enableEncoderCompensation = 1;   // 启用编码器误差补偿

        // 进入正常运行模式
        Control::MotorControl::motorControlStatus.enableCalibration = 0;
        Control::MotorControl::motorControlStatus.calibrationState = Control::MotorControl::MotorCalibrationState::Stop;
        Control::MotorControl::motorControlStatus.state = Control::MotorControl::MotorControlState::preRunning;

        // 保存参数到 Flash
        Control::ConfigLoader::saveAllConfigToFlashAsync();
        return;
    }

    calibratorStatus.currentStepElapsedTime ++;
    /* General */
    if(currentAction.action == CalibrateActionType::SLEEP)
    {
    }

    else if(currentAction.action == CalibrateActionType::START)
    {
        // 消除旧的 Encoder Offset 和 Direction 影响
        Sensor::Encoder::encoderConfig.encoderZeroOffset = 0;
        Sensor::Encoder::encoderConfig.encoderDirection = 1;
        Sensor::Encoder::encoderConfig.enableEncoderCompensation = 0;

        calibratorStatus.currentAngle = 0;
        calibratorStatus.currentVoltage = 0.0f;
        nextCurrentSampleIndex = 0;
        nextEncoderSampleIndex = 0;
        Control::FOC::setPhraseVoltage(0, 0);
    }

    else if(currentAction.action == CalibrateActionType::MOVE_TO_POSITION)
    {
        calibratorStatus.currentError = (int16_t)(Sensor::Encoder::encoderStatus.Q16_accumulatedEncoder - calibratorStatus.currentAngle / PP);

        if(calibratorStatus.encoderRecordingStatus)
        {
            // uint16_t currentEncoderErrorIndex = FABS(currentAction.angle - calibratorStatus.actionStartAngle) * (float)(calibratorStatus.currentStepElapsedTime + 1) / (float)currentAction.duration / (65536 / ENCODER_SAMPLES_PER_ELECTRIC_CYCLE);
            uint16_t currentEncoderErrorIndex = FABS(calibratorStatus.currentAngle - calibratorStatus.actionStartAngle) / (65536 / ENCODER_SAMPLES_PER_ELECTRIC_CYCLE);
            if(currentEncoderErrorIndex > nextEncoderErrorSampleIndex && nextEncoderErrorSampleIndex < ENCODER_SAMPLES_PER_ELECTRIC_CYCLE * PP)
            {
                if(calibratorStatus.encoderRecordingStatus == 1)
                {
                    // 正转记录
                    encoderErrors[nextEncoderErrorSampleIndex] = calibratorStatus.currentError;
                }
                else if(calibratorStatus.encoderRecordingStatus == 2)
                {
                    // 反转记录
                    encoderErrors[ENCODER_SAMPLES_PER_ELECTRIC_CYCLE * PP - nextEncoderErrorSampleIndex - 1] += calibratorStatus.currentError;
                }
                nextEncoderErrorSampleIndex++;
            }
        }

        float voltage = calibratorStatus.actionStartVoltage + (currentAction.voltage - calibratorStatus.actionStartVoltage) * (float)calibratorStatus.currentStepElapsedTime / (float)currentAction.duration;
        int32_t angle = calibratorStatus.actionStartAngle + (currentAction.angle - calibratorStatus.actionStartAngle) * (float)calibratorStatus.currentStepElapsedTime / (float)currentAction.duration;
        openLoopMoveTo(angle, voltage);
    }

    else if (currentAction.action == CalibrateActionType::START_FOC)
    {
        /* 校准模式下 FOC State Machine 处于 Stop. 用于防止多次 triggerReset */
        if(Control::MotorControl::motorControlStatus.state == Control::MotorControl::MotorControlState::Stop)
        {
            // 进入正常运行模式
            Control::MotorControl::motorControlStatus.state = Control::MotorControl::MotorControlState::preRunning;

            Control::FOC::focConfig.FOCControlMode = Control::FOC::FOCControlMode_t::VOLTAGE_TOURQUE_CONTROL;
            Control::MotorControl::motorControlStatus.targetIq = 0.4f;
        }
    }
    
    else if (currentAction.action == CalibrateActionType::STOP_FOC)
    {
        Control::FOC::disableFOC();
    }


    /* Step 1 */
    else if(currentAction.action == CalibrateActionType::RECORD_CURRENT_VALUES)
    {
        currentValues[nextCurrentSampleIndex][0] = Sensor::ADC::analogValues.measuredIA;
        currentValues[nextCurrentSampleIndex][1] = Sensor::ADC::analogValues.measuredIB;
        currentValues[nextCurrentSampleIndex][2] = Sensor::ADC::analogValues.measuredIC;
        nextCurrentSampleIndex++;
    }

    else if(currentAction.action == CalibrateActionType::RECORD_ENCODER_VALUES)
    {
        encoderValues[nextEncoderSampleIndex][0] = Sensor::Encoder::encoderStatus.Q16_encoder;
        encoderValues[nextEncoderSampleIndex][1] = Sensor::Encoder::encoderStatus.Q16_accumulatedEncoder;
        nextEncoderSampleIndex++;
    }

    else if(currentAction.action == CalibrateActionType::CALCULATION_1)
    {
        // 判断 TIM CH 与 ADC 对应关系
        for(int i = 0; i < 3; i++)
        {
            if(FABS(currentValues[i][0]) > FABS(currentValues[i][1]) && FABS(currentValues[i][0]) > FABS(currentValues[i][2]))
            {
                calibrationResult.phaseXADCindex[i] = 0;
            }
            else if(FABS(currentValues[i][1]) > FABS(currentValues[i][0]) && FABS(currentValues[i][1]) > FABS(currentValues[i][2]))
            {
                calibrationResult.phaseXADCindex[i] = 1;
            }
            else if(FABS(currentValues[i][2]) > FABS(currentValues[i][0]) && FABS(currentValues[i][2]) > FABS(currentValues[i][1]))
            {
                calibrationResult.phaseXADCindex[i] = 2;
            }
        }

        // 判断电流极性
        calibrationResult.phaseXCurrentSign[0] = currentValues[0][calibrationResult.phaseXADCindex[0]] > 0 ? 1 : -1;
        calibrationResult.phaseXCurrentSign[1] = currentValues[1][calibrationResult.phaseXADCindex[1]] > 0 ? 1 : -1;
        calibrationResult.phaseXCurrentSign[2] = currentValues[2][calibrationResult.phaseXADCindex[2]] > 0 ? 1 : -1;

        // 估计相电阻
        float currentAbsSum[3] = {0.0f};   // 三相平均电流值
        for(int i = 0; i < 3; i++)
        {
            currentAbsSum[0] += FABS(currentValues[i][0]);
            currentAbsSum[1] += FABS(currentValues[i][1]);
            currentAbsSum[2] += FABS(currentValues[i][2]);
        }
        currentAbsSum[0] /= 3.0f;
        currentAbsSum[1] /= 3.0f;
        currentAbsSum[2] /= 3.0f;

        float averageCurrentAbsSum = (currentAbsSum[0] + currentAbsSum[1] + currentAbsSum[2]) / 3.0f;
        calibrationResult.estimatedMotorResistance = ALIGN_VOLTAGE / averageCurrentAbsSum / 1.5f;       // / 1.5f 后得到的是单相线圈的电阻

        float inconsistency = 0.0f;
        inconsistency += FABS(currentAbsSum[0] - averageCurrentAbsSum)
                        + FABS(currentAbsSum[1] - averageCurrentAbsSum)
                        + FABS(currentAbsSum[2] - averageCurrentAbsSum);
        calibrationResult.phaseInconsistency = inconsistency / averageCurrentAbsSum / 3.0f * 100.0f;   // 转换为百分比

        // 编码器
        calibrationResult.encoderSign = encoderValues[2][1] > encoderValues[0][1] ? 1 : -1;
        calibrationResult.encoderZeroOffset = (uint16_t)encoderValues[0][0] * calibrationResult.encoderSign;
        calibrationResult.estimatedEncoderRatio = (float)(currentAction.angle) / FABS((float)(encoderValues[2][1] - encoderValues[0][1]));

        // 将校准数据应用到校准参数
        Sensor::ADC::adcCalibrationData.IA_GAIN *= calibrationResult.phaseXCurrentSign[0];
        Sensor::ADC::adcCalibrationData.IB_GAIN *= calibrationResult.phaseXCurrentSign[1];
        Sensor::ADC::adcCalibrationData.IC_GAIN *= calibrationResult.phaseXCurrentSign[2];

        Control::FOC::motorConfig.phaseResistance = calibrationResult.estimatedMotorResistance;
        Control::FOC::motorConfig.electricAngleReductionRatio = (uint8_t)calibrationResult.estimatedEncoderRatio;
        Sensor::Encoder::encoderConfig.encoderDirection = calibrationResult.encoderSign;
        Sensor::Encoder::encoderConfig.encoderZeroOffset = calibrationResult.encoderZeroOffset;

        // 删除占用的记录
        nextCurrentSampleIndex = 0;
        nextEncoderSampleIndex = 0;
    }


    /* Step 2 */
    else if(currentAction.action == CalibrateActionType::RECORD_ENCODER_ERROR_START)
    {
        calibratorStatus.encoderRecordingStatus = 1;
    }

    else if(currentAction.action == CalibrateActionType::RECORD_ENCODER_ERROR_START_REVERSE)
    {
        calibratorStatus.encoderRecordingStatus = 2;
    }

    else if(currentAction.action == CalibrateActionType::RECORD_ENCODER_ERROR_END)
    {
        calibratorStatus.encoderRecordingStatus = 0;
        nextEncoderErrorSampleIndex = 0;
    }

    else if(currentAction.action == CalibrateActionType::CALCULATION_2)
    {
        // 平均
        for(uint16_t i = 0; i < ENCODER_SAMPLES_PER_ELECTRIC_CYCLE * (PP + 1); i++)
        {
            encoderErrors[i] = encoderErrors[i] / 2;
        }

        for(uint16_t i = 0; i < ENCODER_SAMPLES_PER_ELECTRIC_CYCLE; i++)
        {
            encoderErrors[ENCODER_SAMPLES_PER_ELECTRIC_CYCLE * PP + i] = encoderErrors[i];      /* 将正转的误差数据复制到后面, 方便后续滤波计算 */
        }
        for(uint16_t i = 0; i < ENCODER_SAMPLES_PER_ELECTRIC_CYCLE * PP; i++)
        {
            float sum = 0.0f;
            for(uint16_t j = 0; j < ENCODER_SAMPLES_PER_ELECTRIC_CYCLE; j++)
            {
                sum += encoderErrors[i + j];
            }
            filteredEncoderErrors[(i + ENCODER_SAMPLES_PER_ELECTRIC_CYCLE / 2) % (ENCODER_SAMPLES_PER_ELECTRIC_CYCLE * PP)] = sum / ENCODER_SAMPLES_PER_ELECTRIC_CYCLE;
        }

        for(uint16_t i = 0; i < ENCODER_SAMPLES_PER_ELECTRIC_CYCLE; i++)
        {
            filteredEncoderErrors[ENCODER_SAMPLES_PER_ELECTRIC_CYCLE * PP + i] = filteredEncoderErrors[i];      /* 将正转的误差数据复制到后面, 方便后续滤波计算 */
        }
        for(uint16_t i = 0; i < ENCODER_SAMPLES_PER_ELECTRIC_CYCLE * PP; i++)
        {
            float sum = 0.0f;
            for(uint16_t j = 0; j < ENCODER_SAMPLES_PER_ELECTRIC_CYCLE / 2; j++)
            {
                sum += filteredEncoderErrors[i + j];
            }
            secondFilteredEncoderErrors[(i + ENCODER_SAMPLES_PER_ELECTRIC_CYCLE / 4) % (ENCODER_SAMPLES_PER_ELECTRIC_CYCLE * PP)] = sum / ENCODER_SAMPLES_PER_ELECTRIC_CYCLE * 2;
        }

        float maxError = 0.0f;
        for(uint16_t i = 0; i < 64; i++)
        {
            float error = FABS(secondFilteredEncoderErrors[ENCODER_SAMPLES_PER_ELECTRIC_CYCLE * PP * i / 64]);
            if(error > maxError)
            {
                maxError = error;
            }
        }
        // 缩放到 int8 范围
        float encoderCompensationGain = maxError / 127.0f;
        float oneOverEncoderCompensationGain = 1.0f / encoderCompensationGain;

        // 将校准数据应用到校准参数:
        for(uint8_t i = 0; i < 64; i++)
        {
            Sensor::Encoder::encoderConfig.encoderCompensationTable[i] = secondFilteredEncoderErrors[ENCODER_SAMPLES_PER_ELECTRIC_CYCLE * PP * i / 64] * oneOverEncoderCompensationGain;
        }
        
        Sensor::Encoder::encoderConfig.encoderCompensationGain = encoderCompensationGain;
        Sensor::Encoder::encoderConfig.enableEncoderCompensation = 1;   // 启用编码器误差补偿
    }

    else if(currentAction.action == CalibrateActionType::SHOW_ENCODER_RESULT)
    {
        display[0] = encoderErrors[calibratorStatus.currentStepElapsedTime - 1];
        display[1] = secondFilteredEncoderErrors[calibratorStatus.currentStepElapsedTime - 1];
        display[2] = Sensor::Encoder::encoderConfig.encoderCompensationTable[64 * (uint32_t)(calibratorStatus.currentStepElapsedTime - 1) / (ENCODER_SAMPLES_PER_ELECTRIC_CYCLE * PP)];
        display[3] = Sensor::Encoder::getCompensation(65536 * (float)(calibratorStatus.currentStepElapsedTime - 1) / (ENCODER_SAMPLES_PER_ELECTRIC_CYCLE * PP));
    }


    /* Step 3 */

    if(calibratorStatus.currentStepElapsedTime >= currentAction.duration)
    {
        // 切换到下一个 Action
        calibratorStatus.currentStepIndex ++;
        calibratorStatus.currentStepElapsedTime = 0;
        calibratorStatus.actionStartAngle = calibratorStatus.currentAngle;
        calibratorStatus.actionStartVoltage = calibratorStatus.currentVoltage;
    }
}

} // namespace Calibrator
} // namespace Control