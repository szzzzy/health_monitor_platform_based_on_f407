/**
  ******************************************************************************
  * @file    app_ecg.c
  * @brief   ECG QRS 检测 — 250 Hz 采样，简化 Pan-Tompkins 状态机
  *
  * 每 10 ms 主循环调用 app_ecg_process_samples()，从 DMA 环形缓冲消费
  * 全部待处理样本。单个样本的处理流水线：
  *   1. DMA 覆盖检测 → 丢弃旧数据 + 重置检测器
  *   2. 导联脱落检查 → 脱落则重置检测器并跳过后继续
  *   3. ADC 饱和检查 (>=4090 LSB) → 跳过该样本
  *   4. DC 漂移消除 (一阶高通 IIR, fc≈1.24 Hz)
  *   5. 滑动平均平滑 (一阶低通, fc≈13.3 Hz)
  *   6. 自适应阈值 QRS 状态机
  *   7. 有效 R 峰 → 至少 2 峰才更新 HR/RR → 通知 PTT
  *
  * 每个消费样本配有推算的 4 ms 间隔时间戳，用于 RR 与 PTT 计算。
  *
  * 仅为工程观测/趋势提示，不声称临床诊断能力。
  ******************************************************************************
  */

#include "app_ecg.h"
#include "app_ptt.h"
#include "app_display.h"
#include "adc.h"
#include <string.h>

/* === ECG 采样率 ============================================================ */
#define APP_ECG_SAMPLE_RATE_HZ      250U
#define APP_ECG_SAMPLE_PERIOD_MS    4U    /* 1000 / 250 = 4 ms */

/* === DC 漂移消除 — 一阶高通 IIR ============================================ */
/* cutoff ≈ fs / (2π·2^DC_SHIFT) ≈ 250 / (2π·32) ≈ 1.24 Hz */
#define APP_ECG_DC_SHIFT            5U

/* === 滑动平均平滑 — 一阶低通 ================================================ */
/* cutoff ≈ fs / (2π·SMOOTH_DIV) ≈ 250 / (2π·3) ≈ 13.3 Hz */
#define APP_ECG_SMOOTH_DIV          3

/* === QRS 检测器参数 ======================================================== */
#define APP_ECG_SETTLE_SAMPLES      125U /* 500 ms 启动稳定期，×250 Hz */
#define APP_ECG_MIN_RR_MS           300U /* 生理最小 RR (200 bpm 上限) */
#define APP_ECG_MAX_RR_MS           2000U/* 生理最大 RR (30 bpm 下限) */
#define APP_ECG_STALE_MS            3000U/* 无新 R 峰超时 */
#define APP_ECG_QRS_MIN_THRESHOLD   45U  /* QRS 检测绝对最小阈值 (LSB) */
#define APP_ECG_QRS_MAX_THRESHOLD   1000U/* QRS 检测绝对最大阈值 (LSB) */
#define APP_ECG_QRS_NOISE_GAIN      3U   /* 动态阈值 = 噪声基线 × 增益 */
#define APP_ECG_QRS_END_DIV         2U   /* 波群结束：幅值跌至候选峰值/2 */
#define APP_ECG_QRS_MAX_WIDTH_MS    160U /* QRS 波群最大宽度 */

/* === ADC 饱和检测阈值 ====================================================== */
#define APP_ECG_ADC_SAT_THRESHOLD   4090U

/* === 内部状态结构 ========================================================== */
typedef struct
{
  int32_t dc_estimate;
  int32_t smooth_value;
  uint32_t noise_level;
  uint32_t sample_count;
  uint32_t last_r_peak_ms;

  uint8_t initialized;
  uint8_t in_candidate;
  uint8_t peak_count;     /* 本次检出周期内的有效 R 峰计数 */

  uint32_t candidate_peak_abs;
  uint32_t candidate_peak_ms;
  uint32_t candidate_start_ms;
} AppEcgState_t;

static AppEcgState_t ecg_state;

/* === 前向声明 ============================================================== */
static void     app_ecg_reset_detector(void);
static uint32_t app_ecg_abs_i32(int32_t v);
static int16_t  app_ecg_clamp_i16(int32_t v);
static AppEcgUpdate_t app_ecg_process_sample(AppState_t *app,
                                             int32_t filtered,
                                             uint32_t timestamp_ms);

/* ========================================================================== */
/*  ECG 模块重置                                                               */
/* ========================================================================== */
void app_ecg_reset(AppState_t *app)
{
  app_ecg_reset_detector();
  if (app == NULL) return;

  app->ecg_raw       = 0U;
  app->ecg_filtered  = 0;
  app->ecg_lead_off  = 0U;
  app->ecg_valid     = 0U;
  app->ecg_hr        = 0U;
  app->ecg_rr_ms     = 0U;
  app->ecg_r_peak_ms = 0UL;
  app->ptt_valid     = 0U;
  app->ptt_ms        = 0U;
  app_ptt_reset(app);
  app_display_reset_ecg_waveform();
}

/* ========================================================================== */
/*  读取 AD8232 LO- / LO+ 引脚电平 → 导联脱落掩码                                */
/*  LO- → PE5, LO+ → PE6                                                        */
/* ========================================================================== */
uint8_t app_ecg_read_lead_off(void)
{
  uint8_t lead_off = 0U;

  if (HAL_GPIO_ReadPin(AD8232_LD_MINUS_GPIO_Port, AD8232_LD_MINUS_Pin) == GPIO_PIN_SET)
  {
    lead_off |= APP_ECG_LEAD_OFF_MINUS;
  }
  if (HAL_GPIO_ReadPin(AD8232_LD_PLUS_GPIO_Port, AD8232_LD_PLUS_Pin) == GPIO_PIN_SET)
  {
    lead_off |= APP_ECG_LEAD_OFF_PLUS;
  }
  return lead_off;
}

/* ========================================================================== */
/*  主入口：消费 DMA 缓冲中全部待处理样本                                         */
/*                                                                             */
/*  每个样本获得独立的推算时间戳（4 ms 间隔），解决批量消费时时间戳相同的问题。     */
/*  发生 DMA 覆盖时丢弃已损样本、重置检测器、清除 ecg_valid/ptt_valid。           */
/* ========================================================================== */
uint8_t app_ecg_process_samples(AppState_t *app)
{
  uint16_t raw_value;
  uint8_t  lead_off;
  int32_t  raw_i32, ac_value, delta;
  uint32_t now_ms;
  uint16_t avail;
  uint8_t  r_peak_found = 0U;
  uint16_t consumed = 0U;
  uint8_t  dma_overflow_occurred = 0U;

  if (app == NULL) return 0U;

  /* 进入批量消费前取一次 HAL Tick + 可用样本数 */
  now_ms = HAL_GetTick();
  avail = app_ecg_adc_get_available_count();

  /* DMA 覆盖检测：DMA 写入跑赢了软件消费，环形缓冲数据已丢失 */
  if (app_ecg_adc_had_overflow() != 0U)
  {
    dma_overflow_occurred = 1U;
    app->ecg_dma_overflow_count++;
    app_ecg_adc_clear_overflow();

    /* 丢弃已被覆盖的旧样本已在 adc.c read_sample 中处理，
     * 这里仅重置检测器状态和 ECG/PTT 输出 */
    app_ecg_reset_detector();
    app->ecg_valid = 0U;
    app->ecg_hr = 0U;
    app->ecg_rr_ms = 0U;
    app->ptt_valid = 0U;
    app->ptt_ms = 0U;
    app_ptt_reset(app);
    app_display_reset_ecg_waveform();

    /* 刷新可用样本数（丢弃旧数据后） */
    avail = app_ecg_adc_get_available_count();
  }

  /* 消费 DMA 环形缓冲中所有可用样本 */
  {
    uint32_t sample_ts;
    uint16_t remaining = avail;

    while (remaining > 0U)
    {
      /* 每个样本独立推算时间戳，间隔 4 ms */
      if (app_ecg_adc_read_sample(&raw_value, &sample_ts, now_ms, remaining) == 0U)
      {
        break;
      }
      remaining--;
      consumed++;
      app->ecg_sample_count++;

      /* 1. 导联脱落检查 */
      lead_off = app_ecg_read_lead_off();
      if ((lead_off != 0U) && (app->ecg_lead_off == 0U))
      {
        app_display_reset_ecg_waveform();
      }
      app->ecg_lead_off = lead_off;
      if (lead_off != 0U)
      {
        app->ecg_lead_off_count++;
        app->ecg_valid = 0U;
        app->ecg_hr = 0U;
        app->ecg_rr_ms = 0U;
        app->ptt_valid = 0U;
        app->ptt_ms = 0U;
        app_ecg_reset_detector();
        app_ptt_reset(app);
        app->ecg_filtered = 0;
        /* 导联脱落时不放弃剩余样本，但跳过进入下一次循环 */
        continue;
      }

      app->ecg_raw = raw_value;

      /* 2. ADC 饱和检测 — 饱和样本不可用于 QRS 检测 */
      if (raw_value >= APP_ECG_ADC_SAT_THRESHOLD)
      {
        app->ecg_adc_sat_count++;
        continue;
      }

      /* 3. 首次初始化 */
      raw_i32 = (int32_t)raw_value;
      if (ecg_state.initialized == 0U)
      {
        ecg_state.dc_estimate  = raw_i32;
        ecg_state.smooth_value = 0;
        ecg_state.noise_level  = APP_ECG_QRS_MIN_THRESHOLD / APP_ECG_QRS_NOISE_GAIN;
        ecg_state.sample_count = 0U;
        ecg_state.peak_count   = 0U;
        ecg_state.initialized  = 1U;
      }

      /* 4. DC 漂移消除 — 一阶高通 IIR */
      delta = raw_i32 - ecg_state.dc_estimate;
      ecg_state.dc_estimate += (delta / (int32_t)(1UL << APP_ECG_DC_SHIFT));
      ac_value = raw_i32 - ecg_state.dc_estimate;

      /* 5. 滑动平均平滑 — 一阶低通 */
      ecg_state.smooth_value += ((ac_value - ecg_state.smooth_value) / APP_ECG_SMOOTH_DIV);
      app->ecg_filtered = app_ecg_clamp_i16(ecg_state.smooth_value);
      app_display_add_ecg_sample(app->ecg_filtered);

      if (ecg_state.sample_count < 0xFFFFFFFFUL)
      {
        ecg_state.sample_count++;
      }

      /* 6. QRS 检测 — 传入每样本独立时间戳 */
      AppEcgUpdate_t update = app_ecg_process_sample(app, ecg_state.smooth_value, sample_ts);
      if (update.r_peak_detected != 0U)
      {
        r_peak_found = 1U;
        app_display_add_ecg_r_peak_marker();
      }
    }
  }

  /* 7. R 峰超时检查 (即使无新样本也不断激活) */
  if ((ecg_state.last_r_peak_ms != 0UL) &&
      ((now_ms - ecg_state.last_r_peak_ms) > APP_ECG_STALE_MS))
  {
    app->ecg_valid = 0U;
    app->ecg_hr = 0U;
    app->ecg_rr_ms = 0U;
    app->ptt_valid = 0U;
    app->ptt_ms = 0U;
    app_ptt_reset(app);
    app->ecg_no_r_peak_timeout_count++;
  }

  return r_peak_found;
}

/* ========================================================================== */
/*  QRS 检测状态机（简化 Pan-Tompkins）                                         */
/*                                                                             */
/*  空闲态：                                                                     */
/*    - 持续更新噪声基线（非对称：快升 1/32，慢降 1/16）                          */
/*    - 幅值 > 动态阈值 且 满足不应期 → 进入候选态                               */
/*                                                                             */
/*  候选态：                                                                     */
/*    - 追踪波群内最大峰值                                                      */
/*    - 波群结束条件：幅值 < 候选峰值/END_DIV 或 超时                            */
/*    - 候选峰值 ≥ 阈值 → 确认 R 峰                                             */
/*                                                                             */
/*  有效 RR 计算：                                                               */
/*    - 首个 R 峰只记录时间戳，ecg_valid 保持 0                                  */
/*    - 至少 2 个满足生理范围的峰才计算 HR/RR 并设 ecg_valid=1                   */
/*    - PTT 仅在 ecg_valid=1 后允许发布                                         */
/* ========================================================================== */
static AppEcgUpdate_t app_ecg_process_sample(AppState_t *app,
                                             int32_t filtered_value,
                                             uint32_t timestamp_ms)
{
  AppEcgUpdate_t update;
  uint32_t magnitude, threshold, noise_delta;
  uint32_t rr_ms, hr_bpm;

  (void)memset(&update, 0, sizeof(update));

  if (app == NULL) return update;

  /* 稳定期：让 DC 估计与平滑器收敛 */
  if (ecg_state.sample_count < APP_ECG_SETTLE_SAMPLES)
  {
    return update;
  }

  magnitude = app_ecg_abs_i32(filtered_value);

  /* 动态阈值 = 噪声基线 × 增益，钳位在 [MIN, MAX] */
  threshold = ecg_state.noise_level * APP_ECG_QRS_NOISE_GAIN;
  if (threshold < APP_ECG_QRS_MIN_THRESHOLD)
    threshold = APP_ECG_QRS_MIN_THRESHOLD;
  else if (threshold > APP_ECG_QRS_MAX_THRESHOLD)
    threshold = APP_ECG_QRS_MAX_THRESHOLD;

  /* ===== 空闲态 ===== */
  if (ecg_state.in_candidate == 0U)
  {
    /* 噪声基线自适应：快升慢降 */
    if (magnitude > ecg_state.noise_level)
    {
      noise_delta = magnitude - ecg_state.noise_level;
      ecg_state.noise_level += (noise_delta / 32U) ? (noise_delta / 32U) : 1U;
    }
    else
    {
      noise_delta = ecg_state.noise_level - magnitude;
      ecg_state.noise_level -= (noise_delta / 16U);
    }

    /* 触发：幅值超阈值 + 生理不应期满足 */
    if ((magnitude >= threshold) &&
        ((ecg_state.last_r_peak_ms == 0UL) ||
         ((timestamp_ms - ecg_state.last_r_peak_ms) >= APP_ECG_MIN_RR_MS)))
    {
      ecg_state.in_candidate       = 1U;
      ecg_state.candidate_peak_abs = magnitude;
      ecg_state.candidate_peak_ms  = timestamp_ms;
      ecg_state.candidate_start_ms = timestamp_ms;
    }
    return update;
  }

  /* ===== 候选态 ===== */

  /* 追踪波群最大峰值 */
  if (magnitude > ecg_state.candidate_peak_abs)
  {
    ecg_state.candidate_peak_abs = magnitude;
    ecg_state.candidate_peak_ms  = timestamp_ms;
  }

  /* 波群结束条件：幅值跌至候选峰值/END_DIV 以下，或宽度超限 */
  if ((magnitude > (ecg_state.candidate_peak_abs / APP_ECG_QRS_END_DIV)) &&
      ((timestamp_ms - ecg_state.candidate_start_ms) < APP_ECG_QRS_MAX_WIDTH_MS))
  {
    return update; /* 仍在波群内 */
  }

  /* 波群结束，退出候选态 */
  ecg_state.in_candidate = 0U;

  /* 候选峰值必须 ≥ 阈值 */
  if (ecg_state.candidate_peak_abs < threshold)
  {
    return update;
  }

  /* === 确认 R 峰 === */
  update.r_peak_detected = 1U;
  update.r_peak_ms       = ecg_state.candidate_peak_ms;
  app->ecg_r_peak_ms     = ecg_state.candidate_peak_ms;

  if (ecg_state.last_r_peak_ms != 0UL)
  {
    rr_ms = ecg_state.candidate_peak_ms - ecg_state.last_r_peak_ms;

    if ((rr_ms >= APP_ECG_MIN_RR_MS) && (rr_ms <= APP_ECG_MAX_RR_MS))
    {
      hr_bpm = (60000UL + (rr_ms / 2UL)) / rr_ms;
      if (hr_bpm > 255UL) hr_bpm = 255UL;

      app->ecg_rr_ms = (uint16_t)rr_ms;

      /* EMA 平滑, α=0.25 */
      if ((app->ecg_valid != 0U) && (app->ecg_hr != 0U))
      {
        app->ecg_hr = (uint8_t)((((uint16_t)app->ecg_hr * 3U) + (uint16_t)hr_bpm + 2U) / 4U);
      }
      else
      {
        app->ecg_hr = (uint8_t)hr_bpm;
      }

      app->ecg_valid = 1U;
      update.rr_ms   = (uint16_t)rr_ms;
      update.hr_bpm  = app->ecg_hr;

      /* PTT 链路：仅当 ecg_valid 已置位后，记录 ECG R 峰时间戳供 PTT 使用 */
      app_ptt_add_ecg_peak(ecg_state.candidate_peak_ms);
    }
    else
    {
      /* RR 超出生理范围 → 标无效，不清 peak_count（可能是偶发噪声） */
      app->ecg_valid = 0U;
      app->ecg_hr = 0U;
      app->ecg_rr_ms = 0U;
      app->ptt_valid = 0U;
      app->ptt_ms = 0U;
      app_ptt_reset(app);
    }
  }
  else
  {
    /* 首个 R 峰：只记录时间戳，不输出 ecg_hr / ecg_valid / PTT。
     * 需要第二个满足范围的峰才可验证 RR 是否合理。 */
    update.r_peak_detected = 1U;
    update.r_peak_ms       = ecg_state.candidate_peak_ms;
    /* ecg_valid 保持 0，ecg_hr 不变，ptt 不记录 */
  }

  ecg_state.last_r_peak_ms = ecg_state.candidate_peak_ms;
  return update;
}

/* ========================================================================== */
/*  辅助函数                                                                   */
/* ========================================================================== */
static void app_ecg_reset_detector(void)
{
  (void)memset(&ecg_state, 0, sizeof(ecg_state));
}

static uint32_t app_ecg_abs_i32(int32_t v)
{
  return (v < 0) ? (uint32_t)(-v) : (uint32_t)v;
}

static int16_t app_ecg_clamp_i16(int32_t v)
{
  if (v > 32767L)  return 32767;
  if (v < -32768L) return -32768;
  return (int16_t)v;
}
