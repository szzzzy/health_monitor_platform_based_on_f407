/**
  ******************************************************************************
  * @file    app_ppg_pulse.c
  * @brief   PPG 脉搏波峰检测与 IBI 提取
  *
  * 在时间域对经过带通滤波的 PPG 信号进行逐点峰值检测，
  * 提取逐拍间隔 (IBI)，推送至 HRV 和 RR 模块。
  *
  * 检测算法：
  *   1. 维护 3 点滑动窗口 (previous2, previous1, current)
  *   2. 基于信号 AC RMS 计算自适应阈值
  *   3. 波峰/波谷检测 — 中间点需同时满足方向条件与幅值阈值
  *   4. 突出度检验 — 峰/谷相对于两侧邻域必须高出/低于指定阈值
  *   5. 极性交替 — 相邻有效峰必须交替（正向峰→负向谷→正向峰...）
  *   6. 间隔约束 — IBI 必须在 [250ms, 2000ms] 生理范围内
  *
  * 有效脉搏触发：
  *   - 标记 HRV 模块的峰值可见性窗口
  *   - 记录 IBI 到 HRV 环形缓冲区
  *   - 推送脉搏幅值到 RR 模块（仅在 SQ ≥ 阈值时）
  *   - 触发 OLED 波形上的脉搏标记
  ******************************************************************************
  */

#include "app_ppg_pulse.h"

#include <string.h>

#include "app_display.h"
#include "app_hrv.h"
#include "app_rr.h"

/* 峰值检测阈值参数 */
#define APP_STREAM_PULSE_MIN_RMS       4U    /* 绝对最小 AC RMS 阈值 */
#define APP_STREAM_PULSE_THRESHOLD_DIV 4U    /* 阈值 = AC_RMS / 4 */
#define APP_STREAM_PULSE_PROM_DIV      3U    /* 突出度 = 阈值 / 3 */

/* 流式峰值检测器内部状态 */
static struct
{
  int32_t previous2;          /* 倒数第 2 个样本 */
  int32_t previous1;          /* 倒数第 1 个样本（被检测的候选点） */
  uint8_t sample_count;       /* 已接收样本数（用于初始化窗口） */
  uint8_t last_peak_valid;    /* 上一次检测到有效峰/谷 */
  int8_t last_polarity;       /* 上一次峰/谷极性：+1=峰, -1=谷 */
  uint32_t last_peak_sample;  /* 上一次峰/谷的样本序号 */
} stream_pulse_state;

void app_ppg_pulse_reset(void)
{
  (void)memset(&stream_pulse_state, 0, sizeof(stream_pulse_state));
}

/*
 * 逐点脉搏检测主函数 — 每收到一个滤波后样本调用一次
 *
 * @param filtered_sample  带通滤波后的 PPG 样本值（AC 分量，已去 DC）
 * @param total_samples    传感器启动以来总样本数
 * @param pulse_info       输出：本次检测到的脉搏信息（若有）
 * @return 1 = 检测到有效脉搏，0 = 未检测到
 */
uint8_t app_ppg_pulse_update(AppState_t *app,
                             int32_t filtered_sample,
                             uint32_t total_samples,
                             MAX30102_PulseInfo_t *pulse_info)
{
  /* 生理 IBI 范围 → 样本数范围 */
  const uint32_t min_interval_samples =
      (((uint32_t)APP_HRV_IBI_MIN_MS * MAX30102_ALGO_SAMPLE_RATE_HZ) + 999U) / 1000U;
  const uint32_t max_interval_samples =
      (((uint32_t)APP_HRV_IBI_MAX_MS * MAX30102_ALGO_SAMPLE_RATE_HZ) + 999U) / 1000U;

  uint32_t current_sample_number;
  uint32_t candidate_sample_number;
  uint32_t interval_samples;
  uint32_t threshold;
  uint32_t prominence_threshold;
  uint32_t positive_prominence = 0U;
  uint32_t negative_prominence = 0U;
  uint32_t magnitude = 0U;
  int32_t neighbor_reference;
  int8_t polarity = 0;
  uint8_t pulse_detected = 0U;

  if (pulse_info != NULL)
  {
    (void)memset(pulse_info, 0, sizeof(*pulse_info));
  }

  if ((app == NULL) || (pulse_info == NULL) || (app->finger_present == 0U))
  {
    app_ppg_pulse_reset();
    return 0U;
  }

  /* 初始化 3 点窗口 */
  if (stream_pulse_state.sample_count < 2U)
  {
    if (stream_pulse_state.sample_count == 0U)
    {
      stream_pulse_state.previous1 = filtered_sample;
      stream_pulse_state.sample_count = 1U;
    }
    else
    {
      stream_pulse_state.previous2 = stream_pulse_state.previous1;
      stream_pulse_state.previous1 = filtered_sample;
      stream_pulse_state.sample_count = 2U;
    }
    return 0U;
  }

  /* 自适应阈值：基于 IR 通道 AC RMS */
  threshold = app->signal_ir_ac_rms / APP_STREAM_PULSE_THRESHOLD_DIV;
  if (threshold < APP_STREAM_PULSE_MIN_RMS)
  {
    threshold = APP_STREAM_PULSE_MIN_RMS;
  }

  /* 突出度阈值 = 幅值阈值 / 3，最小 2 LSB */
  prominence_threshold = threshold / APP_STREAM_PULSE_PROM_DIV;
  if (prominence_threshold < 2U)
  {
    prominence_threshold = 2U;
  }

  /*
   * 正向峰检测：
   *   prev1 > prev2 且 prev1 >= current 且 prev1 > threshold
   * 突出度 = prev1 - max(prev2, current)
   */
  if ((stream_pulse_state.previous1 > stream_pulse_state.previous2) &&
      (stream_pulse_state.previous1 >= filtered_sample) &&
      (stream_pulse_state.previous1 > (int32_t)threshold))
  {
    neighbor_reference = (stream_pulse_state.previous2 > filtered_sample) ?
                         stream_pulse_state.previous2 : filtered_sample;
    if (stream_pulse_state.previous1 > neighbor_reference)
    {
      positive_prominence = (uint32_t)(stream_pulse_state.previous1 - neighbor_reference);
    }
  }

  /*
   * 负向谷检测：
   *   prev1 < prev2 且 prev1 <= current 且 prev1 < -threshold
   * 突出度 = min(prev2, current) - prev1
   */
  if ((stream_pulse_state.previous1 < stream_pulse_state.previous2) &&
      (stream_pulse_state.previous1 <= filtered_sample) &&
      (stream_pulse_state.previous1 < -((int32_t)threshold)))
  {
    neighbor_reference = (stream_pulse_state.previous2 < filtered_sample) ?
                         stream_pulse_state.previous2 : filtered_sample;
    if (neighbor_reference > stream_pulse_state.previous1)
    {
      negative_prominence = (uint32_t)(neighbor_reference - stream_pulse_state.previous1);
    }
  }

  /* 选择突出度更大的极性方向作为本次检测结果 */
  if ((positive_prominence >= prominence_threshold) ||
      (negative_prominence >= prominence_threshold))
  {
    if (positive_prominence >= negative_prominence)
    {
      polarity = 1;
      magnitude = (uint32_t)stream_pulse_state.previous1;
    }
    else
    {
      polarity = -1;
      magnitude = (uint32_t)(-stream_pulse_state.previous1);
    }
  }

  /* 极性交替检验与 IBI 计算 */
  if ((polarity != 0) && (total_samples >= 2U))
  {
    current_sample_number = total_samples - 1U;
    candidate_sample_number = current_sample_number - 1U;

    if (stream_pulse_state.last_peak_valid == 0U)
    {
      /* 首个有效峰/谷：记录但不输出 IBI */
      stream_pulse_state.last_peak_valid = 1U;
      stream_pulse_state.last_polarity = polarity;
      stream_pulse_state.last_peak_sample = candidate_sample_number;
    }
    else if (polarity != stream_pulse_state.last_polarity)
    {
      /* 极性交替但间隔过长 → 可能漏峰，更新参考但不输出 */
      interval_samples = candidate_sample_number - stream_pulse_state.last_peak_sample;
      if (interval_samples > max_interval_samples)
      {
        stream_pulse_state.last_polarity = polarity;
        stream_pulse_state.last_peak_sample = candidate_sample_number;
      }
    }
    else
    {
      /* 同极性 → 可能是一个完整周期 */
      interval_samples = candidate_sample_number - stream_pulse_state.last_peak_sample;
      if (interval_samples > max_interval_samples)
      {
        /* 间隔过长 → 仅更新参考位置 */
        stream_pulse_state.last_peak_sample = candidate_sample_number;
      }
      else if (interval_samples >= min_interval_samples)
      {
        /* 有效脉搏！更新参考并输出 IBI */
        stream_pulse_state.last_peak_sample = candidate_sample_number;
        pulse_info->beat_valid = 1U;
        pulse_info->interval_samples = (uint16_t)interval_samples;

        /* 样本数 → 毫秒 (四舍五入) */
        pulse_info->latest_ibi_ms = (uint16_t)((interval_samples * 1000U +
                                                (MAX30102_ALGO_SAMPLE_RATE_HZ / 2U)) /
                                               MAX30102_ALGO_SAMPLE_RATE_HZ);
        pulse_info->latest_peak_sample = candidate_sample_number;
        pulse_info->beat_amplitude = magnitude;
        pulse_detected = 1U;
      }
    }
  }

  /* 滑动窗口前进 */
  stream_pulse_state.previous2 = stream_pulse_state.previous1;
  stream_pulse_state.previous1 = filtered_sample;
  return pulse_detected;
}

/*
 * 脉搏后处理：将检测到的脉搏信息推送至下游模块
 *
 * 流程：
 *   1. HRV 峰值可见性窗口检查（防同一脉搏被重复标记）
 *   2. 记录 IBI 到 HRV 环形缓冲区
 *   3. 触发 OLED 脉搏标记
 *   4. 推送振幅到 RR 模块（仅在信号质量达标时）
 */
void app_ppg_pulse_process_metrics(AppState_t *app,
                                   const MAX30102_PulseInfo_t *pulse_info,
                                   uint32_t total_samples)
{
  if ((app == NULL) || (pulse_info == NULL) || (pulse_info->beat_valid == 0U))
  {
    return;
  }

  /* HRV 峰值可见性窗口：防止同一峰被重复标记 */
  if (app_hrv_mark_peak_seen(pulse_info->latest_peak_sample) == 0U)
  {
    return;
  }

  /* 记录 IBI 到 HRV 缓冲区 */
  if (app_hrv_add_ibi(app, pulse_info->latest_ibi_ms) == 0U)
  {
    return;
  }

  /* 触发 OLED 波形脉搏标记点 */
  app_display_add_ir_pulse_marker();
  (void)total_samples;

  /* RR 呼吸率：仅在信号质量达标时推送振幅值 */
  if ((app->signal_quality >= APP_RR_SIGNAL_QUALITY_MIN) && (pulse_info->beat_amplitude != 0U))
  {
    app_rr_add_beat(pulse_info->latest_peak_sample, pulse_info->beat_amplitude);
    app_rr_update_output(app);
  }
  else if (app->signal_quality < APP_RR_SIGNAL_QUALITY_MIN)
  {
    app->rr_valid = 0U;
  }
}
