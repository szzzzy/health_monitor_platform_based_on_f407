/**
  ******************************************************************************
  * @file    app_bpm_filter.c
  * @brief   BPM 输出滤波器与候选确认状态机
  *
  * 核心功能：对逐拍原始心率值进行多级滤波与确认，输出稳定可靠的 BPM。
  *
  * 处理流程：
  *   1. 原始值中值滤波 — 3 样本滑动窗口中值，消除单拍野值
  *   2. 候选确认状态机 — 分层阈值判断心率变化幅度
  *      - 小幅变化 (≤8 bpm)：EMA 平滑 (7/8 旧 + 1/8 新)
  *      - 中等变化 (≤14 bpm)：需要 3 拍确认，混合平滑
  *      - 大幅变化 (>14 bpm，如骤升)：需要 5 拍确认，防止干扰误触发
  *   3. 首次锁定 — 需要 2 拍一致才输出首个有效 BPM
  *   4. 无效保持 — 连续 30 个无效周期后才清零输出
  *   5. 步进限幅 — 每拍最多改变 ±5 bpm，防止突变
  ******************************************************************************
  */

#include "app_bpm_filter.h"

#include <string.h>

/* 变化幅度分层阈值 (bpm) */
#define APP_BPM_CONFIRM_DELTA        8U    /* 小幅变化：快速 EMA 跟踪 */
#define APP_BPM_SWITCH_DELTA         14U   /* 中等变化：需 3 拍候选确认 */
#define APP_BPM_SPIKE_DELTA          24U   /* 大幅变化：需 5 拍候选确认（防野值） */

/* 候选确认所需连续拍数 */
#define APP_BPM_START_CONFIRM_COUNT  2U    /* 首次锁定 */
#define APP_BPM_SWITCH_CONFIRM_COUNT 3U    /* 中等变化确认 */
#define APP_BPM_SPIKE_CONFIRM_COUNT  5U    /* 大幅变化确认 */

/* 无效保持与更新限幅 */
#define APP_BPM_INVALID_HOLD_TICKS   30U   /* 连续无效周期数后清零 */
#define APP_BPM_MAX_STEP_PER_UPDATE  5U    /* 每拍最大 BPM 变化量 */

/* === 内部辅助函数 ========================================================== */
static uint8_t app_bpm_abs_diff_u8(uint8_t lhs, uint8_t rhs);
static uint8_t app_bpm_limit_step(uint8_t current_bpm, uint8_t target_bpm, uint8_t max_step);
static uint8_t app_bpm_median3_u8(uint8_t a, uint8_t b, uint8_t c);
static uint8_t app_bpm_filter_raw(AppState_t *app, uint8_t raw_bpm_value);

/* 重置所有 BPM 滤波状态 */
void app_bpm_filter_reset(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  app->bpm_valid = 0U;
  app->bpm_value = 0U;
  (void)memset(app->raw_bpm_history, 0, sizeof(app->raw_bpm_history));
  app->raw_bpm_history_count = 0U;
  app->raw_bpm_history_index = 0U;
  app->bpm_candidate_value = 0U;
  app->bpm_candidate_count = 0U;
  app->bpm_invalid_hold_count = 0U;
}

/**
  * BPM 滤波器主入口
  *
  * @param raw_bpm_valid  原始 BPM 是否有效
  * @param raw_bpm_value  原始 BPM 值 (0-255 bpm)
  * @return 0 = 正常（含平滑跟踪），1 = BPM 已因持续无效而被清除
  */
uint8_t app_bpm_filter_update(AppState_t *app, uint8_t raw_bpm_valid, uint8_t raw_bpm_value)
{
  uint8_t diff;
  uint8_t filtered_bpm;
  uint8_t required_confirm_count;
  uint16_t blended_value;

  if (app == NULL)
  {
    return 0U;
  }

  if (raw_bpm_valid != 0U)
  {
    /* === 原始值有效分支 === */

    /* 1. 中值滤波：消除单拍尖峰 */
    filtered_bpm = app_bpm_filter_raw(app, raw_bpm_value);
    app->bpm_invalid_hold_count = 0U;

    /* 2. 首次锁定：需要 2 拍候选值接近才输出首个有效 BPM */
    if (app->bpm_valid == 0U)
    {
      /* 候选值不匹配 → 重新开始计数 */
      if ((app->bpm_candidate_count == 0U) ||
          (app_bpm_abs_diff_u8(filtered_bpm, app->bpm_candidate_value) > APP_BPM_CONFIRM_DELTA))
      {
        app->bpm_candidate_value = filtered_bpm;
        app->bpm_candidate_count = 1U;
        return 0U;
      }

      if (app->bpm_candidate_count < 0xFFU)
      {
        app->bpm_candidate_count++;
      }

      /* 候选值平滑：新旧各 50% */
      blended_value = ((uint16_t)app->bpm_candidate_value + (uint16_t)filtered_bpm + 1U) / 2U;
      app->bpm_candidate_value = (uint8_t)blended_value;

      /* 达到确认次数 → 输出首帧有效 BPM */
      if (app->bpm_candidate_count >= APP_BPM_START_CONFIRM_COUNT)
      {
        app->bpm_valid = 1U;
        app->bpm_value = app->bpm_candidate_value;
        app->bpm_candidate_count = 0U;
      }

      return 0U;
    }

    /* 3. 已锁定状态下的变化处理 */
    diff = app_bpm_abs_diff_u8(filtered_bpm, app->bpm_value);

    /* 小幅变化 (≤8 bpm)：直接 EMA 跟踪 (α = 1/8) */
    if (diff <= APP_BPM_CONFIRM_DELTA)
    {
      blended_value = (((uint16_t)app->bpm_value * 7U) + (uint16_t)filtered_bpm + 4U) / 8U;
      app->bpm_value = app_bpm_limit_step(app->bpm_value,
                                          (uint8_t)blended_value,
                                          APP_BPM_MAX_STEP_PER_UPDATE);
      app->bpm_candidate_value = app->bpm_value;
      app->bpm_candidate_count = 0U;
      return 0U;
    }

    /* 大幅/中等变化 → 进入候选确认流程 */
    required_confirm_count = APP_BPM_SWITCH_CONFIRM_COUNT;
    if (diff > APP_BPM_SPIKE_DELTA)
    {
      required_confirm_count = APP_BPM_SPIKE_CONFIRM_COUNT;
    }

    /* 候选值不匹配 → 重新开始计数 */
    if ((app->bpm_candidate_count == 0U) ||
        (app_bpm_abs_diff_u8(filtered_bpm, app->bpm_candidate_value) > APP_BPM_SWITCH_DELTA))
    {
      app->bpm_candidate_value = filtered_bpm;
      app->bpm_candidate_count = 1U;
      return 0U;
    }

    if (app->bpm_candidate_count < 0xFFU)
    {
      app->bpm_candidate_count++;
    }

    blended_value = ((uint16_t)app->bpm_candidate_value + (uint16_t)filtered_bpm + 1U) / 2U;
    app->bpm_candidate_value = (uint8_t)blended_value;

    /* 候选确认完成 → 更新输出 */
    if (app->bpm_candidate_count >= required_confirm_count)
    {
      if (diff > APP_BPM_SPIKE_DELTA)
      {
        /* 尖峰变化：保守混合 (87.5% 旧 + 12.5% 新) */
        blended_value = (((uint16_t)app->bpm_value * 7U) + (uint16_t)app->bpm_candidate_value + 4U) / 8U;
      }
      else
      {
        /* 中等变化：均衡混合 (62.5% 旧 + 37.5% 新) */
        blended_value = (((uint16_t)app->bpm_value * 5U) +
                         ((uint16_t)app->bpm_candidate_value * 3U) + 4U) / 8U;
      }

      app->bpm_value = app_bpm_limit_step(app->bpm_value,
                                          (uint8_t)blended_value,
                                          APP_BPM_MAX_STEP_PER_UPDATE);
      app->bpm_candidate_value = app->bpm_value;
      app->bpm_candidate_count = 0U;
    }

    return 0U;
  }

  /* === 原始值无效分支 === */

  /* 已锁定的 BPM 在短暂无效期间保持，超时后清零 */
  if (app->bpm_valid != 0U)
  {
    if (app->bpm_invalid_hold_count < APP_BPM_INVALID_HOLD_TICKS)
    {
      app->bpm_invalid_hold_count++;
      return 0U;
    }
  }

  /* 持续无效超时 → 重置所有状态 */
  app_bpm_filter_reset(app);
  return 1U;
}

/* 无符号 8-bit 绝对值差 */
static uint8_t app_bpm_abs_diff_u8(uint8_t lhs, uint8_t rhs)
{
  if (lhs >= rhs)
  {
    return (uint8_t)(lhs - rhs);
  }

  return (uint8_t)(rhs - lhs);
}

/* 限幅步进：每拍最多改变 max_step bpm */
static uint8_t app_bpm_limit_step(uint8_t current_bpm, uint8_t target_bpm, uint8_t max_step)
{
  if (target_bpm > current_bpm)
  {
    if ((uint8_t)(target_bpm - current_bpm) > max_step)
    {
      return (uint8_t)(current_bpm + max_step);
    }
  }
  else if (current_bpm > target_bpm)
  {
    if ((uint8_t)(current_bpm - target_bpm) > max_step)
    {
      return (uint8_t)(current_bpm - max_step);
    }
  }

  return target_bpm;
}

/* 3 值中位数滤波器 */
static uint8_t app_bpm_median3_u8(uint8_t a, uint8_t b, uint8_t c)
{
  uint8_t temp;

  /* 排序网络：3 次比较 + 1 次交换实现对 a/b/c 的偏序 */
  if (a > b)
  {
    temp = a;
    a = b;
    b = temp;
  }

  if (b > c)
  {
    temp = b;
    b = c;
    c = temp;
  }

  if (a > b)
  {
    b = a;
  }

  return b;
}

/*
 * 原始 BPM 中值滤波
 *
 * 维护 3 样本环形缓冲区：
 * - 1 样本: 直接返回
 * - 2 样本: 返回均值
 * - 3 样本: 返回中位数（消除单拍尖峰干扰）
 */
static uint8_t app_bpm_filter_raw(AppState_t *app, uint8_t raw_bpm_value)
{
  uint8_t first_index;
  uint8_t second_index;

  if (app == NULL)
  {
    return raw_bpm_value;
  }

  /* 环形写入 */
  app->raw_bpm_history[app->raw_bpm_history_index] = raw_bpm_value;
  app->raw_bpm_history_index = (uint8_t)((app->raw_bpm_history_index + 1U) % 3U);

  if (app->raw_bpm_history_count < 3U)
  {
    app->raw_bpm_history_count++;
  }

  if (app->raw_bpm_history_count == 1U)
  {
    return app->raw_bpm_history[0];
  }

  if (app->raw_bpm_history_count == 2U)
  {
    return (uint8_t)(((uint16_t)app->raw_bpm_history[0] +
                      (uint16_t)app->raw_bpm_history[1] + 1U) / 2U);
  }

  /* 3 样本中值滤波：处理数据按环形索引排列，取中位数 */
  first_index = app->raw_bpm_history_index;
  second_index = (uint8_t)((app->raw_bpm_history_index + 1U) % 3U);

  return app_bpm_median3_u8(app->raw_bpm_history[first_index],
                            app->raw_bpm_history[second_index],
                            app->raw_bpm_history[(uint8_t)((app->raw_bpm_history_index + 2U) % 3U)]);
}
