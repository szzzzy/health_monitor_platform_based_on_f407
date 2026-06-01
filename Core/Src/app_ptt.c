/**
  ******************************************************************************
  * @file    app_ptt.c
  * @brief   PTT 计算：PPG 脉搏波峰时刻 − ECG R 峰时刻
  *
  * 维护最近 4 个 ECG R 峰时间戳环形缓冲区。
  * PPG 波峰到达时查找其之前最近的 ECG R 峰，计算时间差。
  * PTT ∈ [60, 600] ms，越界结果标记无效并清零。
  * 导联脱落、手指离开、ECG/PPG 复位时清空 PTT 状态。
  ******************************************************************************
  */

#include "app_ptt.h"
#include <string.h>

#define APP_PTT_MIN_MS                60U
#define APP_PTT_MAX_MS                600U
#define APP_PTT_ECG_PEAK_HISTORY_SIZE 4U

static struct
{
  uint32_t r_peak_ms[APP_PTT_ECG_PEAK_HISTORY_SIZE];
  uint8_t  count;
  uint8_t  write_index;
} ptt_ecg_state;

/* PPG 样本序号 → 绝对 ms 时间戳 */
static uint32_t app_ptt_ppg_sample_to_ms(const AppState_t *app,
                                         uint32_t peak_sample,
                                         uint32_t total_samples);

/* 在 ECG 历史中查找 PPG 波峰之前最近的有效 R 峰 */
static uint8_t app_ptt_find_ref_r_peak(uint32_t ppg_peak_ms, uint32_t *r_peak_ms);

/* ========================================================================== */
/*  PTT 模块重置 — 清空 ECG 峰历史与 AppState PTT 字段                          */
/* ========================================================================== */
void app_ptt_reset(AppState_t *app)
{
  (void)memset(&ptt_ecg_state, 0, sizeof(ptt_ecg_state));
  if (app != NULL)
  {
    app->ptt_valid = 0U;
    app->ptt_ms    = 0U;
  }
}

/* ========================================================================== */
/*  ECG 有效 R 峰到达 → 写入环形缓冲区                                          */
/* ========================================================================== */
void app_ptt_add_ecg_peak(uint32_t r_peak_ms)
{
  if (r_peak_ms == 0UL) return;

  ptt_ecg_state.r_peak_ms[ptt_ecg_state.write_index] = r_peak_ms;
  ptt_ecg_state.write_index = (uint8_t)((ptt_ecg_state.write_index + 1U) %
                                        APP_PTT_ECG_PEAK_HISTORY_SIZE);
  if (ptt_ecg_state.count < APP_PTT_ECG_PEAK_HISTORY_SIZE)
  {
    ptt_ecg_state.count++;
  }
}

/* ========================================================================== */
/*  PPG 有效波峰到达 → 查找匹配 ECG R 峰 → 计算 PTT                             */
/*                                                                             */
/*  前置条件：ECG 有效、导联未脱落、已有 ECG R 峰记录                             */
/*  任一前置条件失败 → 标记 ptt_valid=0                                         */
/* ========================================================================== */
void app_ptt_update_from_ppg_peak(AppState_t *app,
                                  uint32_t ppg_peak_sample,
                                  uint32_t ppg_total_samples)
{
  uint32_t ppg_peak_ms, r_peak_ms, ptt_ms, ppg_age_ms;

  if (app == NULL) return;

  /* 前置条件 */
  if ((app->ecg_valid == 0U) ||
      (app->ecg_lead_off != 0U) ||
      (app->ecg_r_peak_ms == 0UL))
  {
    app->ptt_valid = 0U;
    return;
  }

  ppg_peak_ms = app_ptt_ppg_sample_to_ms(app, ppg_peak_sample, ppg_total_samples);

  /*
   * 时间基准可靠性检查：
   * PPG 样本在 FIFO 积压后批量补读时，sensor_last_sample_tick 是
   * 消费时刻而非真实采样时刻，导致 ppg_peak_ms 推算值偏晚。
   * 若推算的峰值年龄 > 30 ms，说明存在 FIFO 积压延迟，PTT 不可信。
   */
  ppg_age_ms = (HAL_GetTick() >= ppg_peak_ms) ? (HAL_GetTick() - ppg_peak_ms) : 0U;
  if (ppg_age_ms > 30U)
  {
    app->ptt_valid = 0U;
    return;
  }

  if (app_ptt_find_ref_r_peak(ppg_peak_ms, &r_peak_ms) == 0U)
  {
    app->ptt_valid = 0U;
    return;
  }

  ptt_ms = ppg_peak_ms - r_peak_ms;

  /* 越界检查：PTT 必须在 [MIN, MAX] ms 内 */
  if ((ptt_ms < APP_PTT_MIN_MS) || (ptt_ms > APP_PTT_MAX_MS))
  {
    app->ptt_valid = 0U;
    return;
  }

  app->ptt_ms    = (uint16_t)ptt_ms;
  app->ptt_valid = 1U;
}

/* ========================================================================== */
/*  PPG 样本序号 → ms 时间戳                                                    */
/*                                                                             */
/*  PPG 在 100 Hz 节拍下采样 (10 ms/样本)，最后样本时刻为                        */
/*  sensor_last_sample_tick (HAL Tick)。                                        */
/*  PPG 波峰时刻 = last_tick − (total − 1 − peak_sample) × 10 ms               */
/* ========================================================================== */
static uint32_t app_ptt_ppg_sample_to_ms(const AppState_t *app,
                                         uint32_t peak_sample,
                                         uint32_t total_samples)
{
  uint32_t last_sample, delta_samples, delta_ms;

  if ((app == NULL) || (total_samples == 0U)) return HAL_GetTick();

  last_sample = total_samples - 1U;
  if (peak_sample > last_sample) return app->sensor_last_sample_tick;

  delta_samples = last_sample - peak_sample;
  delta_ms = delta_samples * APP_SAMPLE_PERIOD_MS;

  if (delta_ms > app->sensor_last_sample_tick) return 0UL;

  return app->sensor_last_sample_tick - delta_ms;
}

/* ========================================================================== */
/*  在 ECG R 峰历史中查找 PPG 波峰之前最近的一个                                  */
/*                                                                             */
/*  从最新到最旧遍历环形缓冲区，返回第一个 ≤ ppg_peak_ms 的 R 峰。               */
/*  这使得 PTT = max(PPG 时间 − 该 PPG 波之前最近的 ECG R 峰)。                  */
/* ========================================================================== */
static uint8_t app_ptt_find_ref_r_peak(uint32_t ppg_peak_ms, uint32_t *r_peak_ms)
{
  uint8_t  i, index;
  uint32_t candidate;

  if ((r_peak_ms == NULL) || (ptt_ecg_state.count == 0U)) return 0U;

  for (i = 0U; i < ptt_ecg_state.count; i++)
  {
    index = (uint8_t)((ptt_ecg_state.write_index + APP_PTT_ECG_PEAK_HISTORY_SIZE - 1U - i) %
                      APP_PTT_ECG_PEAK_HISTORY_SIZE);
    candidate = ptt_ecg_state.r_peak_ms[index];

    if ((candidate != 0UL) && (candidate <= ppg_peak_ms))
    {
      *r_peak_ms = candidate;
      return 1U;
    }
  }
  return 0U;
}
