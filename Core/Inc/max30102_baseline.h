/**
  ******************************************************************************
  * @file    max30102_baseline.h
  * @brief   MAX30102 背景基线统计与手指检测
  ******************************************************************************
  */

#ifndef __MAX30102_BASELINE_H__
#define __MAX30102_BASELINE_H__

#include <stddef.h>
#include <stdint.h>

/* 用于统计背景 IR 基线 */
typedef struct
{
  uint32_t ir_sum;       /* 启动背景窗口 IR 累加值 */
  uint32_t ir_min;       /* 启动背景窗口最小值 */
  uint32_t ir_max;       /* 启动背景窗口最大值 */
  uint32_t tracked_ir;   /* 运行期慢跟随背景基线 */
  uint32_t noise_ir;     /* 运行期背景变化幅度估计 */
  uint16_t sample_count; /* 已纳入启动统计的样本数 */
} MAX30102_Baseline_t;

/** @brief 清零启动统计和运行期跟踪状态。 */
void     max30102_baseline_reset(MAX30102_Baseline_t *baseline);
/** @brief 向启动背景窗口加入一个 IR 样本。 */
void     max30102_baseline_add_ir(MAX30102_Baseline_t *baseline, uint32_t ir_value);
/** @brief 判断启动窗口是否达到目标样本数。 */
uint8_t  max30102_baseline_is_ready(const MAX30102_Baseline_t *baseline, uint16_t target_samples);
/** @brief 返回启动窗口的 IR 均值与极差；无样本时返回 0。 */
uint32_t max30102_baseline_get_average_ir(const MAX30102_Baseline_t *baseline);
uint32_t max30102_baseline_get_range_ir(const MAX30102_Baseline_t *baseline);
/** @brief 判断启动窗口 IR 极差是否不大于 stable_range。 */
uint8_t  max30102_baseline_is_stable(const MAX30102_Baseline_t *baseline, uint32_t stable_range);
/** @brief 用启动均值和噪声估计初始化运行期背景跟踪器。 */
void     max30102_baseline_seed_tracking(MAX30102_Baseline_t *baseline,
                                         uint32_t baseline_ir,
                                         uint32_t noise_ir);
/** @brief 在无手指期间以慢跟随方式更新背景与噪声估计。 */
void     max30102_baseline_track_background(MAX30102_Baseline_t *baseline, uint32_t ir_value);
/** @brief 获取运行期跟踪基线和噪声估计。 */
uint32_t max30102_baseline_get_tracked_ir(const MAX30102_Baseline_t *baseline);
uint32_t max30102_baseline_get_noise_ir(const MAX30102_Baseline_t *baseline);
/**
 * @brief  使用独立的进入/退出增量执行手指状态滞回判定。
 * @return 更新后的状态：1 表示手指在位，0 表示离开。
 */
uint8_t  max30102_detect_finger(uint32_t ir_value,
                                uint32_t baseline_ir,
                                uint8_t current_state,
                                uint32_t finger_on_delta,
                                uint32_t finger_off_delta);

#endif /* __MAX30102_BASELINE_H__ */
