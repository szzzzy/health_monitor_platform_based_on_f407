/**
  ******************************************************************************
  * @file    app_motion.h
  * @brief   仅基于 PPG 的运动/接触扰动伪影检测接口。
  *
  * 不使用加速度计，纯从 PPG 信号特征推断。这里的“运动”也覆盖
  * 面包板静态测试中的手指压力变化、覆盖不充分和瞬态接触抖动。
 *
 * 三条检测信号：
 * 1. AC RMS 尖峰: RED 或 IR 当前 AC RMS 达到基线的 3.0x 且增量 >=80。
 * 2. RED/IR 平衡异常: AC 比值 x1000 落在 [350, 2800] 之外，或 R/BAL 已偏。
 * 3. SQ 骤降: 原始质量评分从 >=45 前值骤降 >=25 点。
 * 三条信号各自贡献分数，总分 >=60 确认运动，<=30 确认退出。
 *
 * 迟滞状态机：
 * - 进入: 评分 >= 60 连续 5 拍确认 → motion_artifact = 1。
 * - 退出: 评分 <= 30 连续 30 拍确认 → motion_artifact = 0。
 * - 运动期间 AC RMS 基线不更新（防止污染）。
 * - 非运动期间基线慢速跟踪（右移 6 位），维持自适应能力。
 *
 * 运动期间的测量策略：
 * - HR/SpO2/RR/IBI/HRV 的有效标志被清零，旧值保留。
 * - HRV/IBI 环形缓冲不清——运动结束可立即恢复。
 * - 流式脉冲检测状态重置（避免跨运动的假 IBI）。
  ******************************************************************************
  */
#ifndef __APP_MOTION_H__
#define __APP_MOTION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"
#include "max30102.h"

/* 清零内部状态与 app->motion_artifact / motion_score。 */
void app_motion_reset(AppState_t *app);
/*
 * 每拍更新运动评分与迟滞状态机。
 * metrics==NULL 或 finger_present==0: 退化为"无信号"路径，评分归零并计数退出。
 */
void app_motion_update_artifact(AppState_t *app,
                                const MAX30102_SignalMetrics_t *metrics,
                                uint8_t raw_quality);

#ifdef __cplusplus
}
#endif

#endif /* __APP_MOTION_H__ */
