/**
  ******************************************************************************
  * @file    app_ptt.c
  * @brief   PTT（脉搏传导时间）计算
  *
  * PTT = PPG 脉搏波峰时刻 - ECG R 峰时刻
  *
  * 生理意义：
  *   PTT 与动脉硬度负相关，可作为无袖带血压估算的输入特征。
  *   正常成人 PTT 范围约 150-400 ms，本模块接受 60-600 ms。
  *
  * 实现要点：
  *   1. 维护最近 4 个 ECG R 峰时间戳的环形缓冲区
  *   2. PPG 波峰到达时，将 PPG 样本序号换算为绝对时间
  *   3. 在 ECG 历史中查找 PPG 波峰之前最近的 R 峰作为参考
  *   4. PTT = PPG 波峰时间 - ECG R 峰时间
  *   5. 超出 [MIN, MAX] 范围的结果标记为无效
  ******************************************************************************
  */

#include "app_ptt.h"

#include <string.h>

/* PTT 有效范围：60-600 ms */
#define APP_PTT_MIN_MS                60U
#define APP_PTT_MAX_MS                600U

/* ECG R 峰环形缓冲区大小 */
#define APP_PTT_ECG_PEAK_HISTORY_SIZE 4U

/* ECG 峰值历史环形缓冲区 */
static struct
{
  uint32_t r_peak_ms[APP_PTT_ECG_PEAK_HISTORY_SIZE]; /* 时间戳数组 */
  uint8_t count;       /* 已存储的有效条目数 (≤ HISTORY_SIZE) */
  uint8_t write_index; /* 下一次写入位置 */
} ptt_ecg_state;

/* === 内部辅助函数 ========================================================== */

/* 将 PPG 样本序号换算为绝对时间 (ms)
 * 基于当前传感器总样本数和最后采样时刻反推 */
static uint32_t app_ptt_ppg_peak_sample_to_ms(const AppState_t *app,
                                              uint32_t ppg_peak_sample,
                                              uint32_t ppg_total_samples);

/* 在 ECG R 峰历史中查找 PPG 波峰之前最近的一个 */
static uint8_t app_ptt_find_reference_r_peak(uint32_t ppg_peak_ms, uint32_t *r_peak_ms);

/* ========================================================================== */
/*  PTT 模块重置                                                               */
/* ========================================================================== */
void app_ptt_reset(AppState_t *app)
{
  (void)memset(&ptt_ecg_state, 0, sizeof(ptt_ecg_state));

  if (app == NULL)
  {
    return;
  }

  app->ptt_valid = 0U;
  app->ptt_ms = 0U;
}

/* ========================================================================== */
/*  记录新的 ECG R 峰到环形缓冲区                                               */
/*  由 app_ecg 模块在检测到有效 R 峰后调用                                      */
/* ========================================================================== */
void app_ptt_add_ecg_peak(uint32_t r_peak_ms)
{
  if (r_peak_ms == 0UL)
  {
    return;
  }

  /* 环形写入 */
  ptt_ecg_state.r_peak_ms[ptt_ecg_state.write_index] = r_peak_ms;
  ptt_ecg_state.write_index = (uint8_t)((ptt_ecg_state.write_index + 1U) %
                                        APP_PTT_ECG_PEAK_HISTORY_SIZE);
  if (ptt_ecg_state.count < APP_PTT_ECG_PEAK_HISTORY_SIZE)
  {
    ptt_ecg_state.count++;
  }
}

/* ========================================================================== */
/*  PPG 波峰到达时调用：查找匹配的 ECG R 峰，计算 PTT                           */
/*                                                                             */
/*  前置条件：                                                                  */
/*    - ECG 信号有效 (ecg_valid != 0)                                          */
/*    - 导联未脱落 (ecg_lead_off == 0)                                         */
/*    - 存在最近的 ECG R 峰时间戳                                               */
/*                                                                             */
/*  步骤：                                                                      */
/*    1. 将 PPG 样本序号转换为绝对时间戳                                        */
/*    2. 在 ECG 峰值历史中查找 PPG 波峰之前最近的 R 峰                          */
/*    3. 计算时间差并验证是否在 [60, 600] ms 范围内                             */
/* ========================================================================== */
void app_ptt_update_from_ppg_peak(AppState_t *app, uint32_t ppg_peak_sample, uint32_t ppg_total_samples)
{
  uint32_t ppg_peak_ms;
  uint32_t r_peak_ms;
  uint32_t ptt_ms;

  if (app == NULL)
  {
    return;
  }

  /* 前置条件检查 */
  if ((app->ecg_valid == 0U) ||
      (app->ecg_lead_off != 0U) ||
      (app->ecg_r_peak_ms == 0UL))
  {
    app->ptt_valid = 0U;
    return;
  }

  /* 1. PPG 样本序号 → 绝对时间 */
  ppg_peak_ms = app_ptt_ppg_peak_sample_to_ms(app, ppg_peak_sample, ppg_total_samples);

  /* 2. 查找 ECG 参考 R 峰 */
  if (app_ptt_find_reference_r_peak(ppg_peak_ms, &r_peak_ms) == 0U)
  {
    app->ptt_valid = 0U;
    return;
  }

  /* 3. 计算 PTT 并验证范围 */
  ptt_ms = ppg_peak_ms - r_peak_ms;
  if ((ptt_ms < APP_PTT_MIN_MS) || (ptt_ms > APP_PTT_MAX_MS))
  {
    app->ptt_valid = 0U;
    return;
  }

  app->ptt_ms = (uint16_t)ptt_ms;
  app->ptt_valid = 1U;
}

/* ========================================================================== */
/*  PPG 样本序号 → 绝对时间戳 (ms)                                              */
/*                                                                             */
/*  原理：                                                                      */
/*    已知最后采样时刻 sensor_last_sample_tick (HAL Tick)，                     */
/*    以及当前总样本数 ppg_total_samples。                                      */
/*    PPG 波峰样本与最后样本之间的时间差 = 样本数差 × 采样周期 (10 ms)。       */
/*    波峰时刻 = sensor_last_sample_tick - 时间差。                             */
/* ========================================================================== */
static uint32_t app_ptt_ppg_peak_sample_to_ms(const AppState_t *app,
                                              uint32_t ppg_peak_sample,
                                              uint32_t ppg_total_samples)
{
  uint32_t current_sample;
  uint32_t delta_samples;
  uint32_t delta_ms;

  if ((app == NULL) || (ppg_total_samples == 0U))
  {
    return HAL_GetTick();
  }

  /* 最后样本序号 = 总样本数 - 1 */
  current_sample = ppg_total_samples - 1U;

  /* 波峰样本序号不能超过最后样本 */
  if (ppg_peak_sample > current_sample)
  {
    return app->sensor_last_sample_tick;
  }

  delta_samples = current_sample - ppg_peak_sample;
  delta_ms = delta_samples * APP_SAMPLE_PERIOD_MS;

  /* 时间差不能超过传感器运行时间 */
  if (delta_ms > app->sensor_last_sample_tick)
  {
    return 0UL;
  }

  return app->sensor_last_sample_tick - delta_ms;
}

/* ========================================================================== */
/*  在 ECG R 峰历史中查找 PPG 波峰之前最近的 R 峰                               */
/*                                                                             */
/*  搜索策略：从最新到最旧遍历环形缓冲区，返回第一个 ≤ ppg_peak_ms 的峰。       */
/*  这保证 PTT = PPG 时刻 - 该 PPG 波之前的最后一个 ECG R 峰。                  */
/* ========================================================================== */
static uint8_t app_ptt_find_reference_r_peak(uint32_t ppg_peak_ms, uint32_t *r_peak_ms)
{
  uint8_t i;
  uint8_t index;
  uint32_t candidate_ms;

  if ((r_peak_ms == NULL) || (ptt_ecg_state.count == 0U))
  {
    return 0U;
  }

  /* 从最新条目 (write_index - 1) 向最旧遍历 */
  for (i = 0U; i < ptt_ecg_state.count; i++)
  {
    index = (uint8_t)((ptt_ecg_state.write_index + APP_PTT_ECG_PEAK_HISTORY_SIZE - 1U - i) %
                      APP_PTT_ECG_PEAK_HISTORY_SIZE);
    candidate_ms = ptt_ecg_state.r_peak_ms[index];

    /* 找到第一个 ≤ PPG 波峰时刻的 R 峰 */
    if ((candidate_ms != 0UL) && (candidate_ms <= ppg_peak_ms))
    {
      *r_peak_ms = candidate_ms;
      return 1U;
    }
  }

  return 0U;
}
