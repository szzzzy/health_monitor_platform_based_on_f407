/**
  ******************************************************************************
  * @file    app_ecg.c
  * @brief   ECG 信号采集、预处理与 QRS 检测
  *
  * 信号处理流程（每拍执行一次）：
  *   1. 读取导联脱落状态 — 脱落时跳过本次处理
  *   2. 软件触发 ADC1 单次转换，获取 ECG 原始采样值
  *   3. DC 漂移消除：一阶高通 IIR，截止频率由 APP_ECG_DC_SHIFT 控制
  *   4. 滑动平均平滑：一阶低通 FIR，去除肌电高频噪声
  *   5. 自适应阈值 QRS 检测：基于 Pan-Tompkins 的简化状态机
  *      - 空闲态：持续更新噪声基线，幅值超过阈值进入候选态
  *      - 候选态：追踪峰值，波群结束后验证是否为有效 R 峰
  *   6. 有效 R 峰 → 计算 RR 间期与瞬时心率（EMA 平滑）
  *   7. R 峰超时检测：超过 APP_ECG_STALE_MS 无新峰则标记无效
  ******************************************************************************
  */

#include "app_ecg.h"

#include <string.h>

#include "adc.h"

/* === ADC 采样参数 ========================================================= */
#define APP_ECG_ADC_TIMEOUT_MS       2U   /* ADC 轮询超时 (ms) */

/* === DC 漂移消除 — 一阶高通 IIR =========================================== */
#define APP_ECG_DC_SHIFT             6U   /* DC 估计更新速率 (1/64 ≈ 0.016) */

/* === 滑动平均平滑 — 一阶低通 FIR =========================================== */
#define APP_ECG_SMOOTH_DIV           4L   /* 平滑系数 (1/4 = 0.25) */

/* === QRS 检测器参数 ======================================================= */
#define APP_ECG_SETTLE_SAMPLES       50U  /* 上电稳定样本数，此期间不检测 */
#define APP_ECG_MIN_RR_MS            300U /* 生理最小 RR 间期 (200 bpm 上限) */
#define APP_ECG_MAX_RR_MS            2000U/* 生理最大 RR 间期 (30 bpm 下限) */
#define APP_ECG_STALE_MS             3000U/* 无新 R 峰超时，标记信号丢失 */
#define APP_ECG_QRS_MIN_THRESHOLD    45U  /* QRS 检测绝对最小阈值 (ADC LSB) */
#define APP_ECG_QRS_MAX_THRESHOLD    1000U/* QRS 检测绝对最大阈值 (ADC LSB) */
#define APP_ECG_QRS_NOISE_GAIN       3U   /* 阈值 = 噪声基线 × 增益 */
#define APP_ECG_QRS_END_DIV          2U   /* 波群结束判定：幅值跌至峰值/2 以下 */
#define APP_ECG_QRS_MAX_WIDTH_MS     160U /* QRS 波群最大宽度 (ms) */

/* === 内部状态结构 =========================================================== */
typedef struct
{
  int32_t dc_estimate;          /* DC 漂移估计值（一阶高通状态） */
  int32_t smooth_value;         /* 平滑后的 AC 信号值 */
  uint32_t noise_level;         /* 自适应噪声基线 */
  uint32_t sample_count;        /* 上电以来总样本计数（用于稳定期判断） */
  uint32_t last_r_peak_ms;      /* 上一个有效 R 峰时间戳 */

  uint8_t initialized;          /* 1 = DC 估计已初始化 */
  uint8_t in_candidate;         /* 1 = 当前处于 QRS 候选态 */

  /* 候选态追踪 */
  uint32_t candidate_peak_abs;  /* 候选波群内的最大幅值 */
  uint32_t candidate_peak_ms;   /* 候选波群峰值时刻 */
  uint32_t candidate_start_ms;  /* 候选波群起始时刻 */
} AppEcgState_t;

static AppEcgState_t ecg_state;

/* === 内部辅助函数 ========================================================== */
static uint32_t app_ecg_abs_i32(int32_t value);
static int16_t app_ecg_clamp_i16(int32_t value);
static void app_ecg_reset_detector(void);
static AppEcgUpdate_t app_ecg_process_qrs(AppState_t *app,
                                          int32_t filtered_value,
                                          uint32_t timestamp_ms);

/* ========================================================================== */
/*  ECG 模块重置                                                               */
/* ========================================================================== */
void app_ecg_reset(AppState_t *app)
{
  app_ecg_reset_detector();

  if (app == NULL)
  {
    return;
  }

  app->ecg_raw = 0U;
  app->ecg_filtered = 0;
  app->ecg_lead_off = 0U;
  app->ecg_valid = 0U;
  app->ecg_hr = 0U;
  app->ecg_rr_ms = 0U;
  app->ecg_r_peak_ms = 0UL;
  app->ptt_valid = 0U;
  app->ptt_ms = 0U;
}

/* ========================================================================== */
/*  软件触发 ADC1 单次转换，阻塞等待完成                                        */
/*  返回值：1 = 转换成功，0 = 超时或参数错误                                    */
/* ========================================================================== */
uint8_t app_ecg_read_adc_raw(uint16_t *raw_value)
{
  HAL_StatusTypeDef status;

  if (raw_value == NULL)
  {
    return 0U;
  }

  /* 启动 ADC1 */
  status = HAL_ADC_Start(&hadc1);
  if (status != HAL_OK)
  {
    return 0U;
  }

  /* 阻塞轮询等待转换完成（单次模式，转换后自动停止） */
  status = HAL_ADC_PollForConversion(&hadc1, APP_ECG_ADC_TIMEOUT_MS);
  if (status == HAL_OK)
  {
    *raw_value = (uint16_t)HAL_ADC_GetValue(&hadc1);
  }

  (void)HAL_ADC_Stop(&hadc1);
  return (status == HAL_OK) ? 1U : 0U;
}

/* ========================================================================== */
/*  读取 AD8232 导联脱落引脚                                                   */
/*  LO+ / LO- 任一为高 → 对应电极脱落                                          */
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
/*  ECG 主处理：每拍调用一次                                                    */
/*                                                                             */
/*  处理顺序：                                                                  */
/*    1. 导联检查 → 脱落则重置                                                  */
/*    2. ADC 采样                                                               */
/*    3. DC 漂移消除 + 滑动平均                                                 */
/*    4. QRS 检测与心率更新                                                     */
/*    5. R 峰超时检查                                                           */
/* ========================================================================== */
AppEcgUpdate_t app_ecg_update(AppState_t *app, uint32_t timestamp_ms)
{
  AppEcgUpdate_t update;
  uint16_t raw_value;
  uint8_t lead_off;
  int32_t raw_i32;
  int32_t ac_value;
  int32_t delta;

  (void)memset(&update, 0, sizeof(update));

  if (app == NULL)
  {
    return update;
  }

  /* 1. 导联脱落检查 */
  lead_off = app_ecg_read_lead_off();
  app->ecg_lead_off = lead_off;

  /* 2. ADC 采样 — 失败则标记无效 */
  if (app_ecg_read_adc_raw(&raw_value) == 0U)
  {
    app->ecg_valid = 0U;
    app->ptt_valid = 0U;
    return update;
  }

  app->ecg_raw = raw_value;

  /* 导联脱落 → 重置状态，等待恢复 */
  if (lead_off != 0U)
  {
    app->ecg_filtered = 0;
    app->ecg_valid = 0U;
    app->ptt_valid = 0U;
    app_ecg_reset_detector();
    return update;
  }

  raw_i32 = (int32_t)raw_value;

  /* 3. 首次初始化：DC 估计 = 第一个样本值 */
  if (ecg_state.initialized == 0U)
  {
    ecg_state.dc_estimate = raw_i32;
    ecg_state.smooth_value = 0;
    ecg_state.noise_level = APP_ECG_QRS_MIN_THRESHOLD / APP_ECG_QRS_NOISE_GAIN;
    ecg_state.sample_count = 0U;
    ecg_state.initialized = 1U;
  }

  /* DC 漂移消除 — 一阶高通 IIR
   * DC_est[n] = DC_est[n-1] + (raw - DC_est[n-1]) / 2^DC_SHIFT
   * 截止频率 ≈ fs / (2π × 2^DC_SHIFT) ≈ 500/(2π×64) ≈ 1.2 Hz */
  delta = raw_i32 - ecg_state.dc_estimate;
  ecg_state.dc_estimate += (delta / (int32_t)(1UL << APP_ECG_DC_SHIFT));

  /* AC 分量 = 原始值 - DC 估计 */
  ac_value = raw_i32 - ecg_state.dc_estimate;

  /* 滑动平均平滑 — 一阶低通 FIR
   * smooth[n] = smooth[n-1] + (ac[n] - smooth[n-1]) / SMOOTH_DIV
   * 截止频率 ≈ fs / (2π × SMOOTH_DIV) ≈ 500/(2π×4) ≈ 20 Hz */
  ecg_state.smooth_value += ((ac_value - ecg_state.smooth_value) / APP_ECG_SMOOTH_DIV);
  app->ecg_filtered = app_ecg_clamp_i16(ecg_state.smooth_value);

  if (ecg_state.sample_count < 0xFFFFFFFFUL)
  {
    ecg_state.sample_count++;
  }

  /* 4. QRS 检测 */
  update = app_ecg_process_qrs(app, ecg_state.smooth_value, timestamp_ms);

  /* 5. R 峰超时检测：超过 STALE_MS 无新峰 → 信号丢失 */
  if ((ecg_state.last_r_peak_ms != 0UL) &&
      ((timestamp_ms - ecg_state.last_r_peak_ms) > APP_ECG_STALE_MS))
  {
    app->ecg_valid = 0U;
    app->ptt_valid = 0U;
  }

  return update;
}

/* ========================================================================== */
/*  绝对值                                                                     */
/* ========================================================================== */
static uint32_t app_ecg_abs_i32(int32_t value)
{
  if (value < 0)
  {
    return (uint32_t)(-value);
  }

  return (uint32_t)value;
}

/* ========================================================================== */
/*  限幅到 int16 范围 [-32768, 32767]                                          */
/* ========================================================================== */
static int16_t app_ecg_clamp_i16(int32_t value)
{
  if (value > 32767L)
  {
    return 32767;
  }

  if (value < -32768L)
  {
    return -32768;
  }

  return (int16_t)value;
}

/* ========================================================================== */
/*  重置 QRS 检测器内部状态                                                     */
/* ========================================================================== */
static void app_ecg_reset_detector(void)
{
  (void)memset(&ecg_state, 0, sizeof(ecg_state));
}

/* ========================================================================== */
/*  QRS 检测状态机（简化 Pan-Tompkins）                                         */
/*                                                                             */
/*  空闲态：                                                                   */
/*    - 持续跟踪噪声基线（不对称升降速率：快升慢降）                              */
/*    - 当信号幅值 > 噪声×增益 且满足不应期 → 进入候选态                         */
/*                                                                             */
/*  候选态：                                                                   */
/*    - 追踪波群内最大峰值及其时间戳                                            */
/*    - 当幅值跌破 阈值/END_DIV 或超过 MAX_WIDTH → 波群结束                    */
/*    - 若候选峰值满足阈值要求 → 确认 R 峰，计算 RR 与心率                     */
/*                                                                             */
/*  心率平滑：                                                                  */
/*    - 首个有效 R 峰直接采纳                                                   */
/*    - 后续采用 EMA (α=0.25): HR = (3×旧HR + 1×新HR) / 4                     */
/* ========================================================================== */
static AppEcgUpdate_t app_ecg_process_qrs(AppState_t *app,
                                          int32_t filtered_value,
                                          uint32_t timestamp_ms)
{
  AppEcgUpdate_t update;
  uint32_t magnitude;
  uint32_t threshold;
  uint32_t noise_delta;
  uint32_t rr_ms;
  uint32_t hr_bpm;

  (void)memset(&update, 0, sizeof(update));

  /* 上电稳定期内不检测，让 DC 估计与平滑器收敛 */
  if ((app == NULL) || (ecg_state.sample_count < APP_ECG_SETTLE_SAMPLES))
  {
    return update;
  }

  magnitude = app_ecg_abs_i32(filtered_value);

  /* 动态阈值 = 噪声基线 × 增益，钳位在 [MIN, MAX] */
  threshold = ecg_state.noise_level * APP_ECG_QRS_NOISE_GAIN;
  if (threshold < APP_ECG_QRS_MIN_THRESHOLD)
  {
    threshold = APP_ECG_QRS_MIN_THRESHOLD;
  }
  else if (threshold > APP_ECG_QRS_MAX_THRESHOLD)
  {
    threshold = APP_ECG_QRS_MAX_THRESHOLD;
  }

  /* ===== 空闲态 ===== */
  if (ecg_state.in_candidate == 0U)
  {
    /* 噪声基线自适应更新
     * - 信号 > 噪声 → 噪声缓慢上升 (1/32 每拍)
     * - 信号 < 噪声 → 噪声较快下降 (1/16 每拍) */
    if (magnitude > ecg_state.noise_level)
    {
      noise_delta = magnitude - ecg_state.noise_level;
      ecg_state.noise_level += ((noise_delta / 32U) != 0U) ? (noise_delta / 32U) : 1U;
    }
    else
    {
      noise_delta = ecg_state.noise_level - magnitude;
      ecg_state.noise_level -= (noise_delta / 16U);
    }

    /* 触发条件：幅值超过动态阈值 且 满足生理不应期 (MIN_RR) */
    if ((magnitude >= threshold) &&
        ((ecg_state.last_r_peak_ms == 0UL) ||
         ((timestamp_ms - ecg_state.last_r_peak_ms) >= APP_ECG_MIN_RR_MS)))
    {
      /* 进入候选态 */
      ecg_state.in_candidate = 1U;
      ecg_state.candidate_peak_abs = magnitude;
      ecg_state.candidate_peak_ms = timestamp_ms;
      ecg_state.candidate_start_ms = timestamp_ms;
    }

    return update;
  }

  /* ===== 候选态 ===== */

  /* 追踪波群最大峰值 */
  if (magnitude > ecg_state.candidate_peak_abs)
  {
    ecg_state.candidate_peak_abs = magnitude;
    ecg_state.candidate_peak_ms = timestamp_ms;
  }

  /* 波群尚未结束的条件：
   * - 幅值仍在 峰值/2 以上，且
   * - 宽度未超过 MAX_WIDTH_MS */
  if ((magnitude > (threshold / APP_ECG_QRS_END_DIV)) &&
      ((timestamp_ms - ecg_state.candidate_start_ms) < APP_ECG_QRS_MAX_WIDTH_MS))
  {
    return update;
  }

  /* 波群结束，回空闲态 */
  ecg_state.in_candidate = 0U;

  /* 候选峰值必须满足阈值要求 */
  if (ecg_state.candidate_peak_abs < threshold)
  {
    return update;
  }

  /* === 确认 R 峰 === */
  update.r_peak_detected = 1U;
  update.r_peak_ms = ecg_state.candidate_peak_ms;
  app->ecg_r_peak_ms = ecg_state.candidate_peak_ms;

  if (ecg_state.last_r_peak_ms != 0UL)
  {
    rr_ms = ecg_state.candidate_peak_ms - ecg_state.last_r_peak_ms;

    /* RR 间期必须在生理范围内 */
    if ((rr_ms >= APP_ECG_MIN_RR_MS) && (rr_ms <= APP_ECG_MAX_RR_MS))
    {
      /* 瞬时心率 = 60000 / RR_ms（四舍五入），上限 255 bpm */
      hr_bpm = (60000UL + (rr_ms / 2UL)) / rr_ms;
      if (hr_bpm > 255UL)
      {
        hr_bpm = 255UL;
      }

      app->ecg_rr_ms = (uint16_t)rr_ms;

      /* 心率 EMA 平滑 (α = 0.25)
       * 首个有效值直接采纳，后续与旧值加权平均 */
      if ((app->ecg_valid != 0U) && (app->ecg_hr != 0U))
      {
        app->ecg_hr = (uint8_t)((((uint16_t)app->ecg_hr * 3U) + (uint16_t)hr_bpm + 2U) / 4U);
      }
      else
      {
        app->ecg_hr = (uint8_t)hr_bpm;
      }

      app->ecg_valid = 1U;
      update.rr_ms = (uint16_t)rr_ms;
      update.hr_bpm = app->ecg_hr;
    }
    else
    {
      /* RR 间期异常 → 标记无效，PTT 也随之无效 */
      app->ecg_valid = 0U;
      app->ptt_valid = 0U;
    }
  }

  ecg_state.last_r_peak_ms = ecg_state.candidate_peak_ms;
  return update;
}
