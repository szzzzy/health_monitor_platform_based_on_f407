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
  *   5a.[DSP] 50 Hz 陷波 + 10-20 Hz 带通 biquad 级联 (M4F FPU 浮点)
  *   5b. 差分能量 + 120ms 移动窗口积分 MWI (全整数，O(1)/样本)
  *   6. 双估计自适应阈值 (signal_peak/noise_peak) QRS 状态机
  *   7. 有效 R 峰 → 至少 2 峰才更新 HR/RR/PTT
  *
  * F407 Cortex-M4F 相比 F103 Cortex-M3 在 ECG 实时处理上的优势：
  *   - FPU 单周期单精度乘加：Phase 1 浮点 biquad 级联 (10 FMAC/样本)
  *     仅占总 CPU 的 <0.002%，F103 软件浮点需 60+ 周期/乘加无法保证实时
  *   - 168MHz 主频 + DSP 指令 (SIMD)：整数 MWI 一次迭代约 12 周期，
  *     Phase 1+2 单样本总开销 <30 周期@250Hz，远低于 F103 (Cortex-M3 72MHz)
  *   - 余量充裕：可同时维持 MAX30102 100Hz I2C DMA、OLED SPI、
  *     SDIO FatFs 日志、FreeRTOS 调度，而 F103 各模块间互抢严重
  *   - CMSIS-DSP 加速路径：手工 biquad/MWI 接口兼容 CMSIS-DSP API，
  *     可一行替换为 arm_biquad_cascade_df1_f32 + arm_mean_f32 充分利用 SIMD
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

#if (APP_ECG_DEBUG_PRINTF != 0U)
#include <stdio.h>
#include "usart.h"
#endif

/* === ECG 采样率 ============================================================ */
#define APP_ECG_SAMPLE_RATE_HZ      250U
#define APP_ECG_SAMPLE_PERIOD_MS    4U    /* 1000 / 250 = 4 ms */

/* === DC 漂移消除 — 一阶高通 IIR ============================================ */
/* 截止频率 ≈ fs / (2π·2^DC_SHIFT) ≈ 250 / (2π·32) ≈ 1.24 Hz */
#define APP_ECG_DC_SHIFT            5U

/* === 滑动平均平滑 — 一阶低通 ================================================ */
/* 截止频率 ≈ fs / (2π·SMOOTH_DIV) ≈ 250 / (2π·3) ≈ 13.3 Hz */
#define APP_ECG_SMOOTH_DIV          3

/* === QRS 检测器参数 ======================================================== */
#define APP_ECG_SETTLE_SAMPLES      125U /* 500 ms 启动稳定期，×250 Hz */
#define APP_ECG_MIN_RR_MS           300U /* 生理最小 RR (200 bpm 上限) */
#define APP_ECG_MAX_RR_MS           2000U/* 生理最大 RR (30 bpm 下限) */
#define APP_ECG_STALE_MS            3000U/* 无新 R 峰超时 */
#define APP_ECG_HARD_STALE_MS       8000U/* 长时间无 R 峰才清除保留数值 */
#define APP_ECG_NO_R_SOFT_SQ_CAP    60U  /* 短暂无 R 峰时 SQ 只降级，不归零 */
#define APP_ECG_LOW_AMP_SQ_CAP      35U  /* 低幅但仍有信号时的软上限 */
#define APP_ECG_QRS_MIN_THRESHOLD   45U  /* QRS 检测绝对最小阈值 (LSB) */
#define APP_ECG_QRS_MAX_THRESHOLD   1000U/* QRS 检测绝对最大阈值 (LSB) */
#define APP_ECG_QRS_NOISE_GAIN      3U   /* 动态阈值 = 噪声基线 × 增益 */
#define APP_ECG_QRS_END_DIV         2U   /* 波群结束：幅值跌至候选峰值/2 */
#define APP_ECG_QRS_MAX_WIDTH_MS    160U /* QRS 波群最大宽度 */
#define APP_ECG_RR_HISTORY_SIZE     5U
#define APP_ECG_SEARCH_HISTORY_SIZE 8U
#define APP_ECG_DYNAMIC_REF_MIN_MS  300U
#define APP_ECG_DYNAMIC_REF_MAX_MS  420U
#define APP_ECG_DYNAMIC_REF_PERCENT 35U
#define APP_ECG_SEARCHBACK_DELAY_PERCENT 160U
#define APP_ECG_SEARCHBACK_THRESHOLD_PERCENT 55U
#define APP_ECG_T_WAVE_WINDOW_MS    420U
#define APP_ECG_T_WAVE_PEAK_PERCENT 60U

/* === ADC 饱和检测阈值 ====================================================== */
#define APP_ECG_ADC_SAT_THRESHOLD   4090U
#define APP_ECG_QUALITY_MIN_SAMPLES 25U
#define APP_ECG_RAW_FLATLINE_SPAN   12U
#define APP_ECG_QUALITY_WINDOW_SAMPLES 250U

/* === 显示用滤波参数（仅 OLED 使用，不参与 QRS 检测）======================== */
/* DC 消除：fc≈0.16 Hz，响应很慢，保持基线稳定，不扭曲 ST 段。 */
#define APP_ECG_VISUAL_DC_SHIFT     8U
/* 低通：SHIFT=3→fc≈5 Hz (原 2→10 Hz)，更有效抑制基线高频噪声。
 * M4F 整数移位零开销，R 峰仍保留足够锐度。 */
#define APP_ECG_VISUAL_LP_SHIFT     3U
/* 增益从 3→4 倍，补偿低通衰减和平均平滑。R 峰仍不超 ±1200 钳位。 */
#define APP_ECG_VISUAL_GAIN         4
/* 软钳位：±1200，留余量防过冲。 */
#define APP_ECG_VISUAL_CLAMP        1200
/* 降采样：每 3 样本一组，组内算术平均后输出 1 个，250/3≈83 px/s。
 * 平均替代直接抽样，抑制随机噪声约 √3≈1.7×。 */
#define APP_ECG_VISUAL_DECIM        3U
/* 启用 50 Hz 陷波，利用 Phase 1 EcgBiquad_t，独立于 QRS 检测链路。 */
#define APP_ECG_VISUAL_ENABLE_NOTCH 1U

/* 保持 ECG 波形运行同时观察原始 AD8232 导联脱落引脚。
 * 在验证 PE5/PE6 接线后设置为 1。 */
#define APP_ECG_ENABLE_LEAD_OFF_GATE 0U

/* MWI 窗口大小 — 在 DSP 块之前定义，供前向声明使用 */
#define APP_ECG_MWI_WINDOW  30U   /* 120 ms @ 250 Hz */

/* 差分能量 + 移动窗口积分类型定义 (实现在 DSP 块之后) */
typedef struct {
  int32_t prev;
  int32_t buf[APP_ECG_MWI_WINDOW];
  int32_t sum;
  uint8_t idx;
  uint8_t count;
} EcgDerivMwi_t;

/* MWI 实例 + 前向声明 */
static EcgDerivMwi_t ecg_dmwi;
static void ecg_dmwi_reset(EcgDerivMwi_t *d);
static int32_t ecg_dmwi_step(EcgDerivMwi_t *d, int32_t filtered);

/* =========================================================================
 * DSP 预处理：二阶 IIR Biquad 级联（50 Hz 陷波 + 10-20 Hz 带通）
 *
 * 使用单精度浮点 Direct Form I，利用 Cortex-M4F 硬件 FPU（单周期 MAC）。
 * 系数预计算为编译时常量，运行时仅 5 次乘加 + 2 次赋值/样本。
 *
 * STM32F103 (Cortex-M3, 无 FPU) 需软件浮点模拟——单样本 IIR 约 60+ CPU
 * 周期，在 250 Hz 下无法保证 MAX30102 + OLED 并行实时性。
 * STM32F407 (Cortex-M4F, 硬件 FPU) 单样本约 14 CPU 周期@168MHz，
 * 250 Hz × 14 ≈ 3500 cycles/s，占总算力 <0.002%，余量充裕。
 *
 * 接口兼容 CMSIS-DSP arm_biquad_casd_df1_inst_f32：
 *   typedef struct { float *pCoeffs; float *pState; ... } arm_biquad_casd_df1_inst_f32;
 * 当前使用内联 EcgBiquad_t 避免 malloc 和外部依赖；未来可替换为
 * arm_biquad_cascade_df1_f32() 一次调用处理整个样本块。
 * ========================================================================= */
#if (APP_ECG_DSP_PREPROCESS != 0U)

/* 50 Hz 陷波器系数 (fs=250 Hz, Q=30, 归一化 a0=1)
 * ω0 = 2π·50/250 = 1.256637, α = sin(ω0)/(2Q) = 0.015851 */
static const EcgBiquad_t ecg_notch_coeff_50hz = {
  0.984394f, -0.608386f, 0.984394f,
  -0.608386f, 0.968788f,
  0.0f, 0.0f, 0.0f, 0.0f
};

/* 10-20 Hz 带通滤波器系数 (fs=250 Hz, Q≈1.414)
 * ω0 = 0.355362, α = sin(ω0)/(2Q) = 0.123061 */
static const EcgBiquad_t ecg_bp_coeff_10_20hz = {
  0.109571f, 0.0f, -0.109571f,
  -1.670043f, 0.780914f,
  0.0f, 0.0f, 0.0f, 0.0f
};

/* 运行时 biquad 状态实例 */
static EcgBiquad_t ecg_notch;         /* QRS 检测链路 50 Hz 陷波 */
static EcgBiquad_t ecg_bp;            /* QRS 检测链路 10-20 Hz 带通 */
#if (APP_ECG_DSP_PREPROCESS != 0U) && (APP_ECG_VISUAL_ENABLE_NOTCH != 0U)
static EcgBiquad_t ecg_visual_notch;  /* OLED visual 链路 50 Hz 陷波 (独立) */
#endif

/* biquad 单步迭代：y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2
 * - a1/a2 存储为归一化后的原始符号系数
 * - 差分方程中为 -a1*y1 即减去 a1*y1 */
static __inline float ecg_biquad_step(EcgBiquad_t *f, float x)
{
  float y;
  y = (f->b0 * x) + (f->b1 * f->x1) + (f->b2 * f->x2)
      - (f->a1 * f->y1) - (f->a2 * f->y2);
  f->x2 = f->x1;
  f->x1 = x;
  f->y2 = f->y1;
  f->y1 = y;
  return y;
}

/* 重置 biquad 状态延迟线 */
static void ecg_biquad_reset(EcgBiquad_t *f)
{
  if (f == NULL) return;
  f->x1 = 0.0f;
  f->x2 = 0.0f;
  f->y1 = 0.0f;
  f->y2 = 0.0f;
}

/* 用预计算系数初始化 biquad 实例 */
static void ecg_biquad_init(EcgBiquad_t *f, const EcgBiquad_t *coeff)
{
  if ((f == NULL) || (coeff == NULL)) return;
  f->b0 = coeff->b0;
  f->b1 = coeff->b1;
  f->b2 = coeff->b2;
  f->a1 = coeff->a1;
  f->a2 = coeff->a2;
  ecg_biquad_reset(f);
}

/* 重置全部 DSP 预处理滤波器 (MWI 由 app_ecg_reset_detector 统一重置) */
static void ecg_dsp_preprocess_reset(void)
{
  ecg_biquad_reset(&ecg_notch);
  ecg_biquad_reset(&ecg_bp);
}

#endif /* APP_ECG_DSP_PREPROCESS */

/* =========================================================================
 * MWI 函数实现（类型和声明在 DSP 块之前）
 *
 * 差分能量 + 移动窗口积分 —— 标准 Pan-Tompkins 步骤 2-4。
 * 全整数运算，O(1)/样本，适合 250 Hz 实时逐样本处理。
 * Cortex-M4F 整数乘加约 12 周期/样本，F103 亦可运行。
 * ========================================================================= */

/* 重置差分 + MWI 全部状态 */
static void ecg_dmwi_reset(EcgDerivMwi_t *d)
{
  (void)memset(d, 0, sizeof(*d));
}

/* 单步推进：差分 → 平方能量 → 120ms 滑动窗平均。
 * 返回 MWI 输出包络值 (≥0)。首次调用 prev 未播种时返回 0。 */
static int32_t ecg_dmwi_step(EcgDerivMwi_t *d, int32_t filtered)
{
  int32_t deriv, energy;

  if (d->count > 0U) {
    deriv = filtered - d->prev;
  } else {
    deriv = 0;
  }
  d->prev = filtered;

  /* 平方能量 >>2 缩放，在 int32 范围内安全 */
  energy = (deriv * deriv) >> 2;

  if (d->count < APP_ECG_MWI_WINDOW) {
    d->buf[d->count] = energy;
    d->sum += energy;
    d->count++;
    return d->sum / (int32_t)d->count;
  }

  d->sum -= d->buf[d->idx];
  d->buf[d->idx] = energy;
  d->sum += energy;
  d->idx = (uint8_t)((d->idx + 1U) % APP_ECG_MWI_WINDOW);
  return d->sum / (int32_t)APP_ECG_MWI_WINDOW;
}

/* === 内部状态结构 ========================================================== */
typedef struct
{
  int32_t dc_estimate;
  int32_t smooth_value;
  uint32_t noise_level;
  uint32_t signal_peak;    /* 已确认 R 峰能量 EMA (α=1/8)，供双估计阈值 */
  uint32_t sample_count;
  uint32_t last_r_peak_ms;

  uint8_t initialized;
  uint8_t in_candidate;
  uint8_t peak_count;     /* 本次检出周期内的有效 R 峰计数 */

  uint32_t candidate_peak_abs;
  uint32_t candidate_peak_ms;
  uint32_t candidate_start_ms;
  uint16_t rr_history[APP_ECG_RR_HISTORY_SIZE];
  uint8_t  rr_count;
  uint8_t  rr_write_index;
  uint32_t search_peak_ms[APP_ECG_SEARCH_HISTORY_SIZE];
  uint32_t search_peak_abs[APP_ECG_SEARCH_HISTORY_SIZE];
  uint8_t  search_count;
  uint8_t  search_write_index;
} AppEcgState_t;

static AppEcgState_t ecg_state;

/* DMA 溢出近期标志，由 process 批次设置，quality 函数读取后清除 */
static uint8_t ecg_dma_overflow_recent = 0U;
static uint8_t ecg_adc_sat_recent = 0U;
static uint8_t ecg_quality_fall_decim = 0U;

/* === OLED 显示滤波 ========================================================= */
/* 检测链路使用 ecg_state.smooth_value（不变）。
 * 显示链路额外做 5 点滑动平均 + 慢速振幅跟踪 + 软限幅，
 * 目的是让 OLED 波形平滑稳定，不被单个 R 峰拉满屏。 */
typedef struct
{
  int32_t buf[5];
  uint8_t idx;
  uint8_t count;
  int32_t amplitude;
  uint8_t decim;   /* 降采样计数器：每 2 个样本推 1 次，减慢滚动 */
} EcgDispFilter_t;

static EcgDispFilter_t ecg_disp;
static uint8_t ecg_debug_disp_decim; /* 调试显示模式独立降采样计数器 */

/* === OLED 可视化显示滤波器：独立于 QRS 检测链路 =========================== */
/* 输出只送入 app_display_add_ecg_sample()，不会回写到 ecg_state 或
 * app->ecg_filtered。导联脱落、DMA 溢出等情况会重置该滤波器。 */
typedef struct
{
  int32_t dc;        /* 慢速 DC 跟踪，用于消除基线漂移 */
  int32_t lp;        /* 低通状态，用于降低高频噪声 */
  uint8_t decim;     /* 独立降采样计数器 */
} EcgVisualFilter_t;

static EcgVisualFilter_t ecg_visual;

/* 3 点滑动平均缓冲（VISUAL 降采样用，替代直接抽样） */
static int32_t ecg_visual_avg_buf[3];
static uint8_t ecg_visual_avg_idx;

/* === ECG 调试统计 ========================================================== */
typedef struct
{
  uint16_t raw_min;
  uint16_t raw_max;
  int16_t  filt_min;
  int16_t  filt_max;
  uint16_t sample_cnt;   /* 当前统计窗口内已累计样本数 */
} EcgDebugStats_t;

static EcgDebugStats_t ecg_dbg_stats;

#if (APP_ECG_DEBUG_PRINTF != 0U)
static void app_ecg_debug_print_stats(const EcgDebugStats_t *s,
                                      const AppState_t *app)
{
  char line[96];
  int len;

  if ((s == NULL) || (app == NULL)) return;

  len = snprintf(line, sizeof(line),
                 "ECGDBG raw_min=%u raw_max=%u raw_diff=%u"
                 " filt_min=%d filt_max=%d lead=0x%02X hr=%u rr=%u\r\n",
                 (unsigned int)s->raw_min,
                 (unsigned int)s->raw_max,
                 (unsigned int)(s->raw_max - s->raw_min),
                 (int)s->filt_min,
                 (int)s->filt_max,
                 (unsigned int)app_ecg_read_lead_off_raw(),
                 (unsigned int)app->ecg_hr,
                 (unsigned int)app->ecg_rr_ms);

  if ((len > 0) && ((size_t)len < sizeof(line)))
  {
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)len, 100U);
  }
}
#endif /* APP_ECG_DEBUG_PRINTF */

/* ---- ECG 调试快照读取函数（供 OLED D8 页面使用） ---- */
void app_ecg_get_debug_snapshot(const AppState_t *app,
                                AppEcgDebugSnapshot_t *out)
{
  if (out == NULL) return;

  (void)memset(out, 0, sizeof(*out));

  out->raw_min  = ecg_dbg_stats.raw_min;
  out->raw_max  = ecg_dbg_stats.raw_max;
  out->filt_min = ecg_dbg_stats.filt_min;
  out->filt_max = ecg_dbg_stats.filt_max;
  out->lead_raw = app_ecg_read_lead_off_raw();

  if (app != NULL)
  {
    out->ecg_valid               = app->ecg_valid;
    out->ecg_hr                  = app->ecg_hr;
    out->ecg_rr_ms              = app->ecg_rr_ms;
    out->sample_count           = app->ecg_sample_count;
    out->dma_overflow_count     = app->ecg_dma_overflow_count;
    out->adc_sat_count          = app->ecg_adc_sat_count;
    out->lead_off_count         = app->ecg_lead_off_count;
    out->no_r_peak_timeout_count = app->ecg_no_r_peak_timeout_count;
    out->signal_quality            = app->ecg_signal_quality;
    out->invalid_reason            = app->ecg_invalid_reason;
    out->raw_span                  = app->ecg_raw_span;
    out->filtered_span             = app->ecg_filtered_span;
    out->noise_level               = app->ecg_noise_level;
    out->qrs_threshold             = app->ecg_qrs_threshold;
    out->peak_snr_x100             = app->ecg_peak_snr_x100;
    out->dma_available_high_watermark = app->ecg_dma_available_high_watermark;

    /* PPG 字段来自现有 MAX30102/PPG BPM 流水线。
     * ppg_bpm > 0 作为辅助有效性保护，避免 bpm_valid 已置位但数值
     * 从未被真实测量填充。 */
    out->ppg_bpm   = app->bpm_value;
    out->ppg_valid = app->bpm_valid;

    if ((out->ecg_valid != 0U) && (out->ppg_valid != 0U)
        && (out->ecg_hr > 0U) && (out->ppg_bpm > 0U))
    {
      out->hr_diff = (int16_t)out->ecg_hr - (int16_t)out->ppg_bpm;
    }
    else
    {
      out->hr_diff = 0;
    }
  }
}

/* === 前向声明 ============================================================== */
static void     app_ecg_reset_detector(void);
static void     app_ecg_reset_display_filter(void);
static void     app_ecg_reset_visual_filter(void);
static void     app_ecg_reset_debug_stats(void);
static void     app_ecg_debug_update_raw(uint16_t raw_value,
                                         const AppState_t *app);
static void     app_ecg_debug_update_filtered(int16_t filtered_value);
static uint32_t app_ecg_abs_i32(int32_t v);
static int16_t  app_ecg_clamp_i16(int32_t v);
static uint8_t  app_ecg_smooth_quality(uint8_t current, uint8_t target);
static void     app_ecg_latch_quality_drop(AppState_t *app,
                                            uint8_t prev_reason,
                                            uint8_t prev_sq);
static uint32_t app_ecg_current_threshold(void);
static uint16_t app_ecg_rr_median(void);
static uint32_t app_ecg_dynamic_refractory_ms(void);
static void     app_ecg_add_rr_history(uint16_t rr_ms);
static void     app_ecg_store_search_candidate(uint32_t peak_ms,
                                               uint32_t peak_abs);
static uint8_t  app_ecg_is_t_wave_like(uint32_t peak_ms,
                                       uint32_t peak_abs);
static uint8_t  app_ecg_try_searchback(AppState_t *app,
                                       uint32_t timestamp_ms,
                                       uint32_t threshold,
                                       AppEcgUpdate_t *update);
static AppEcgUpdate_t app_ecg_accept_r_peak(AppState_t *app,
                                            uint32_t peak_ms,
                                            uint32_t peak_abs);
static AppEcgUpdate_t app_ecg_process_sample(AppState_t *app,
                                             int32_t filtered,
                                             uint32_t timestamp_ms);

/**
 ******************************************************************************
 * @brief  重置 ECG 模块：检测器、AppState 字段、PTT、显示波形。
 * @param  app AppState 指针（可为 NULL）。
 * @note   清除 ecg_raw、ecg_filtered、导联脱落标志、有效标志、HR、RR、R 峰
 *         时间戳、PTT 字段，并重置 QRS 检测器状态机。
 ******************************************************************************
 */
void app_ecg_reset(AppState_t *app)
{
  app_ecg_reset_detector();
  app_ecg_reset_display_filter();
  app_ecg_reset_debug_stats();
  if (app == NULL) return;

  app->ecg_raw       = 0U;
  app->ecg_filtered  = 0;
  app->ecg_lead_off  = 0U;
  app->ecg_valid     = 0U;
  app->ecg_hr        = 0U;
  app->ecg_rr_ms     = 0U;
  app->ecg_r_peak_ms = 0UL;
  app->ecg_signal_quality = 0U;
  app->ecg_invalid_reason = ECG_INVALID_OK;
  app->ecg_raw_span = 0U;
  app->ecg_filtered_span = 0U;
  app->ecg_noise_level = 0UL;
  app->ecg_qrs_threshold = 0UL;
  app->ecg_peak_snr_x100 = 0U;
  app->ecg_dma_available_high_watermark = 0U;
  app->ecg_last_drop_reason = ECG_INVALID_OK;
  app->ecg_last_drop_sq = 0U;
  app->ecg_last_drop_raw_span = 0U;
  app->ecg_last_drop_filtered_span = 0U;
  app->ecg_last_drop_snr_x100 = 0U;
  app->ecg_last_drop_dma_hwm = 0U;
  app->ecg_last_drop_ms = 0UL;
  ecg_dma_overflow_recent = 0U;
  ecg_adc_sat_recent = 0U;
  ecg_quality_fall_decim = 0U;
  app->ptt_valid     = 0U;
  app->ptt_ms        = 0U;
  app_ptt_reset(app);
  app_display_reset_ecg_waveform();
}

/**
 ******************************************************************************
 * @brief  读取 AD8232 导联脱落引脚（LO- = PE5，LO+ = PE6）。
 * @return APP_ECG_LEAD_OFF_MINUS (0x01 = 红色电极) 和/或
 *         APP_ECG_LEAD_OFF_PLUS (0x02 = 绿色电极) 的位掩码。置位表示
 *         该电极已断开或接触不良。
 * @note   仅红色 (LO-) 和绿色 (LO+) 电极可检测。
 *         黄色 RL/RLD 参考电极不经过 AD8232 LO 引脚，
 *         因此移除它不会改变返回值。
 *         这是预期的硬件行为，非软件错误。
 ******************************************************************************
 */
uint8_t app_ecg_read_lead_off(void)
{
  uint8_t lead_off = app_ecg_read_lead_off_raw();

#if (APP_ECG_ENABLE_LEAD_OFF_GATE != 0U)
  return lead_off;
#else
  (void)lead_off;
  return 0U;
#endif
}

uint8_t app_ecg_read_lead_off_raw(void)
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

/**
 ******************************************************************************
 * @brief  处理来自 DMA 环形缓冲的所有可用 ECG 样本。
 * @param  app AppState 指针。
 * @return 若该批次至少检测到一个 R 峰则返回 1，否则返回 0。
 * @note   每个样本获得独立的 4 ms 插值时间戳。
 *         DMA 溢出导致检测器重置并清除 ecg_valid/ptt_valid。
 *         流水线：导联脱落检查 -> ADC 饱和 -> DC 消除 ->
 *         低通平滑 -> QRS 状态机。即使无新样本到达也检查
 *         R 峰超时（APP_ECG_STALE_MS）。
 ******************************************************************************
 */
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

  /* DMA 可用量高水位 */
  if (avail > app->ecg_dma_available_high_watermark) {
    app->ecg_dma_available_high_watermark = avail;
  }

  /* DMA 覆盖检测：DMA 写入跑赢了软件消费，环形缓冲数据已丢失 */
  if (app_ecg_adc_had_overflow() != 0U)
  {
    dma_overflow_occurred = 1U;
    app->ecg_dma_overflow_count++;
    app_ecg_adc_clear_overflow();

    /* 丢弃已被覆盖的旧样本已在 adc.c read_sample 中处理，
     * 这里仅重置检测器状态和 ECG/PTT 输出 */
    app_ecg_reset_detector();
    app_ecg_reset_display_filter();
    app_ecg_reset_debug_stats();
    app->ecg_valid = 0U;
    app->ecg_hr = 0U;
    app->ecg_rr_ms = 0U;
    app->ptt_valid = 0U;
    app->ptt_ms = 0U;
    app_ptt_reset(app);
    app_display_reset_ecg_waveform();
    app->ecg_dma_available_high_watermark = 0U;
    ecg_dma_overflow_recent = 1U;

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
        app_ecg_reset_display_filter();
        app_ecg_reset_debug_stats();
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

      /* 2a. 调试统计 — 原始值 min/max 在饱和检测之前，以观察 4095 附近饱和样本 */
      app_ecg_debug_update_raw(raw_value, app);

      /* 2b. ADC 饱和检测 — 饱和样本不可用于 QRS 检测 */
      if (raw_value >= APP_ECG_ADC_SAT_THRESHOLD)
      {
        app->ecg_adc_sat_count++;
        ecg_adc_sat_recent = 1U;
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

#if (APP_ECG_DSP_PREPROCESS != 0U)
        /* 首次初始化 DSP 预处理 biquad 系数（仅执行一次） */
        {
          static uint8_t ecg_dsp_once = 0U;
          if (ecg_dsp_once == 0U) {
            ecg_dsp_once = 1U;
            ecg_biquad_init(&ecg_notch, &ecg_notch_coeff_50hz);
            ecg_biquad_init(&ecg_bp, &ecg_bp_coeff_10_20hz);
#if (APP_ECG_VISUAL_ENABLE_NOTCH != 0U)
            ecg_biquad_init(&ecg_visual_notch, &ecg_notch_coeff_50hz);
#endif
          }
        }
#endif
      }

      /* 4. DC 漂移消除 — 一阶高通 IIR */
      delta = raw_i32 - ecg_state.dc_estimate;
      ecg_state.dc_estimate += (delta / (int32_t)(1UL << APP_ECG_DC_SHIFT));
      ac_value = raw_i32 - ecg_state.dc_estimate;

      /* 5. 检测用低通平滑 (fc≈13.3 Hz, 不变) */
      ecg_state.smooth_value += ((ac_value - ecg_state.smooth_value) / APP_ECG_SMOOTH_DIV);
      app->ecg_filtered = app_ecg_clamp_i16(ecg_state.smooth_value);

      /* 5a. 调试统计 — 滤波值 min/max 在滤波计算完成后更新 */
      app_ecg_debug_update_filtered(app->ecg_filtered);

      /* 5b. OLED 显示 */
#if (APP_ECG_DEBUG_DISPLAY_RAW != 0U)
      {
        /* 直接显示原始 ADC：原始值 - 2048，3:1 降采样 */
        ecg_debug_disp_decim++;
        if ((ecg_debug_disp_decim % 3U) == 0U)
        {
          app_display_add_ecg_sample(raw_i32 - 2048);
        }
      }
#elif (APP_ECG_DEBUG_DISPLAY_FILTERED != 0U)
      {
        /* 直接显示滤波值：ecg_filtered，3:1 降采样 */
        ecg_debug_disp_decim++;
        if ((ecg_debug_disp_decim % 3U) == 0U)
        {
          app_display_add_ecg_sample((int32_t)app->ecg_filtered);
        }
      }
#elif (APP_ECG_DEBUG_DISPLAY_VISUAL != 0U)
      {
        /* 显示优化滤波链（仅 OLED 使用，独立于 QRS 检测）：
         *   raw → 慢 DC 消除 (fc≈0.16Hz) → 低通 (fc≈5Hz)
         *   → 50Hz 陷波 (浮点 biquad, M4F FPU) → 增益 (×4) → 钳位 (±1200)
         *   → 3 点组内平均 → OLED 波形缓冲
         *
         * 注意：本链路完全不回写 app->ecg_filtered 或 QRS 检测器状态。 */
        int32_t ac, disp;

        /* A. 慢速 DC 消除，稳定基线 */
        ecg_visual.dc += (raw_i32 - ecg_visual.dc) >> APP_ECG_VISUAL_DC_SHIFT;
        ac = raw_i32 - ecg_visual.dc;

        /* B. 低通：fc≈5 Hz (LP_SHIFT=3)，比旧值 10Hz 更有效抑制基线噪声 */
        ecg_visual.lp += (ac - ecg_visual.lp) >> APP_ECG_VISUAL_LP_SHIFT;

        /* C. 50 Hz 陷波（仅 OLED visual，需 DSP biquad 框架支持） */
#if (APP_ECG_DSP_PREPROCESS != 0U) && (APP_ECG_VISUAL_ENABLE_NOTCH != 0U)
        disp = (int32_t)ecg_biquad_step(&ecg_visual_notch,
                                         (float)ecg_visual.lp);
#else
        disp = ecg_visual.lp;
#endif

        /* D. 固定增益 (×4) */
        disp *= APP_ECG_VISUAL_GAIN;

        /* E. 软钳位 */
        if (disp > APP_ECG_VISUAL_CLAMP)       { disp =  APP_ECG_VISUAL_CLAMP; }
        else if (disp < -APP_ECG_VISUAL_CLAMP) { disp = -APP_ECG_VISUAL_CLAMP; }

        /* F. 3 点组内算术平均后推送（替代直接抽样）：
         *    累计 3 个样本 → 平均 → 输出 1 个
         *    抑制随机噪声约 √3≈1.7×，输出率不变 (250/3≈83 px/s) */
        ecg_visual_avg_buf[ecg_visual_avg_idx] = disp;
        ecg_visual_avg_idx = (uint8_t)((ecg_visual_avg_idx + 1U) % APP_ECG_VISUAL_DECIM);
        ecg_visual.decim++;
        if (ecg_visual.decim >= APP_ECG_VISUAL_DECIM)
        {
          int32_t avg;
          avg = (ecg_visual_avg_buf[0] + ecg_visual_avg_buf[1]
                 + ecg_visual_avg_buf[2]) / (int32_t)APP_ECG_VISUAL_DECIM;

          /* SQ is diagnostic only. Keep visual samples flowing so transient
           * quality dips do not collapse the OLED waveform into a flat line. */
          app_display_add_ecg_sample(avg);
          ecg_visual.decim = 0U;
        }
      }
#else
      /* 原始显示链路：5 点滑动平均 + 慢速振幅跟踪
       * + 软限幅 + 3:1 降采样。作为回退路径保留。 */
      {
        int32_t disp_val, abs_val, limit;
        uint8_t  k;

        ecg_disp.buf[ecg_disp.idx] = (int32_t)app->ecg_filtered;
        ecg_disp.idx = (uint8_t)((ecg_disp.idx + 1U) % 5U);
        if (ecg_disp.count < 5U) { ecg_disp.count++; }

        disp_val = 0;
        for (k = 0U; k < ecg_disp.count; k++) { disp_val += ecg_disp.buf[k]; }
        disp_val /= (int32_t)ecg_disp.count;

        abs_val = (disp_val >= 0) ? disp_val : -disp_val;
        if (ecg_disp.amplitude == 0) { ecg_disp.amplitude = abs_val; }
        /* 慢速 EMA：系数约 1/64，振幅不会跟随单个尖峰 */
        ecg_disp.amplitude += (abs_val - ecg_disp.amplitude) / 64;
        if (ecg_disp.amplitude < 50) { ecg_disp.amplitude = 50; }

        /* 软限幅：6 倍振幅，保持 R 峰可见并抑制尖峰 */
        limit = ecg_disp.amplitude * 6;
        if (disp_val > limit)      { disp_val = limit; }
        else if (disp_val < -limit) { disp_val = -limit; }

        /* 3:1 降采样：128px 约 1.5 s */
        ecg_disp.decim++;
        if ((ecg_disp.decim % 3U) == 0U)
        {
          app_display_add_ecg_sample(disp_val);
        }
      }
#endif

      if (ecg_state.sample_count < 0xFFFFFFFFUL)
      {
        ecg_state.sample_count++;
      }

      /* 6. QRS 检测 — 差分能量 + MWI + 自适应阈值 */
      {
        int32_t ecg_qrs_in, ecg_mwi_out;
#if (APP_ECG_DSP_PREPROCESS != 0U)
        /* DSP 预处理级联：50 Hz 陷波 → 10-20 Hz 带通 (Cortex-M4F 硬件 FPU) */
        float ecg_dsp;
        ecg_dsp  = ecg_biquad_step(&ecg_notch, (float)ecg_state.smooth_value);
        ecg_dsp  = ecg_biquad_step(&ecg_bp, ecg_dsp);
        ecg_qrs_in = (int32_t)ecg_dsp;
#else
        ecg_qrs_in = ecg_state.smooth_value;
#endif
        /* 差分能量 → 120ms MWI：平方压制低幅噪声，滑动窗整合多峰 QRS
         * 为单峰包络。全整数运算，O(1)/样本，F103 也可运行。 */
        ecg_mwi_out = ecg_dmwi_step(&ecg_dmwi, ecg_qrs_in);
        AppEcgUpdate_t update = app_ecg_process_sample(app, ecg_mwi_out, sample_ts);
        if (update.r_peak_detected != 0U)
        {
          r_peak_found = 1U;
          app_display_add_ecg_r_peak_marker();
        }
      }
    }
  }

  /* 7. R 峰超时检查 (即使无新样本也不断激活) */
  if ((ecg_state.last_r_peak_ms != 0UL) &&
      ((now_ms - ecg_state.last_r_peak_ms) > APP_ECG_STALE_MS))
  {
    app->ecg_valid = 0U;
    app->ptt_valid = 0U;
    app->ptt_ms = 0U;
    app_ptt_reset(app);
    if ((now_ms - ecg_state.last_r_peak_ms) > APP_ECG_HARD_STALE_MS)
    {
      app->ecg_hr = 0U;
      app->ecg_rr_ms = 0U;
    }
    app->ecg_no_r_peak_timeout_count++;
  }

  /* 每批次完成后更新 ECG 质量快照 */
  app_ecg_update_quality(app);

  return r_peak_found;
}

/* ---- 简化 Pan-Tompkins QRS 状态机（单样本迭代） ---- */
static AppEcgUpdate_t app_ecg_process_sample(AppState_t *app,
                                             int32_t filtered_value,
                                             uint32_t timestamp_ms)
{
  AppEcgUpdate_t update;
  uint32_t magnitude, threshold, noise_delta;
  uint32_t refractory_ms;

  (void)memset(&update, 0, sizeof(update));

  if (app == NULL) return update;

  /* 稳定期：让 DC 估计与平滑器收敛 */
  if (ecg_state.sample_count < APP_ECG_SETTLE_SAMPLES)
  {
    return update;
  }

  magnitude = app_ecg_abs_i32(filtered_value);

  /* ---- 双估计自适应阈值 (signal peak / noise peak) ----
   * 相比单纯 noise×gain，能适应信号幅值变化：
   *   信号明显时 threshold = noise + (signal-noise)/4  (两峰之间)
   *   信号微弱时 threshold = noise × GAIN               (回退到纯噪声增益)
   * 均钳位在 [MIN, MAX]，防静默或饱和。 */
  threshold = app_ecg_current_threshold();

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

    if (app_ecg_try_searchback(app, timestamp_ms, threshold, &update) != 0U)
    {
      return update;
    }

    refractory_ms = app_ecg_dynamic_refractory_ms();

    /* 触发：幅值超阈值 + 动态不应期满足 */
    if ((magnitude >= threshold) &&
        ((ecg_state.last_r_peak_ms == 0UL) ||
         ((timestamp_ms - ecg_state.last_r_peak_ms) >= refractory_ms)))
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
    if ((ecg_state.candidate_peak_abs * 100U) >=
        (threshold * APP_ECG_SEARCHBACK_THRESHOLD_PERCENT))
    {
      app_ecg_store_search_candidate(ecg_state.candidate_peak_ms,
                                     ecg_state.candidate_peak_abs);
    }
    return update;
  }

  if (app_ecg_is_t_wave_like(ecg_state.candidate_peak_ms,
                             ecg_state.candidate_peak_abs) != 0U)
  {
    return update;
  }

  return app_ecg_accept_r_peak(app,
                               ecg_state.candidate_peak_ms,
                               ecg_state.candidate_peak_abs);
}

static uint32_t app_ecg_current_threshold(void)
{
  uint32_t threshold;

  if ((ecg_state.signal_peak > (ecg_state.noise_level * 2U)) &&
      (ecg_state.signal_peak > APP_ECG_QRS_MIN_THRESHOLD))
  {
    threshold = ecg_state.noise_level +
                ((ecg_state.signal_peak - ecg_state.noise_level) / 4U);
  }
  else
  {
    threshold = ecg_state.noise_level * APP_ECG_QRS_NOISE_GAIN;
  }

  if (threshold < APP_ECG_QRS_MIN_THRESHOLD)
  {
    threshold = APP_ECG_QRS_MIN_THRESHOLD;
  }
  else if (threshold > APP_ECG_QRS_MAX_THRESHOLD)
  {
    threshold = APP_ECG_QRS_MAX_THRESHOLD;
  }

  return threshold;
}

static uint16_t app_ecg_rr_median(void)
{
  uint16_t sorted[APP_ECG_RR_HISTORY_SIZE];
  uint16_t tmp;
  uint8_t i;
  uint8_t j;
  uint8_t n;

  n = ecg_state.rr_count;
  if (n == 0U)
  {
    return 0U;
  }

  for (i = 0U; i < n; i++)
  {
    sorted[i] = ecg_state.rr_history[i];
  }

  for (i = 1U; i < n; i++)
  {
    tmp = sorted[i];
    j = i;
    while ((j > 0U) && (sorted[j - 1U] > tmp))
    {
      sorted[j] = sorted[j - 1U];
      j--;
    }
    sorted[j] = tmp;
  }

  return sorted[n / 2U];
}

static uint32_t app_ecg_dynamic_refractory_ms(void)
{
  uint16_t median_rr = app_ecg_rr_median();
  uint32_t refractory_ms;

  if (median_rr == 0U)
  {
    return APP_ECG_MIN_RR_MS;
  }

  refractory_ms = ((uint32_t)median_rr * APP_ECG_DYNAMIC_REF_PERCENT) / 100U;
  if (refractory_ms < APP_ECG_DYNAMIC_REF_MIN_MS)
  {
    refractory_ms = APP_ECG_DYNAMIC_REF_MIN_MS;
  }
  else if (refractory_ms > APP_ECG_DYNAMIC_REF_MAX_MS)
  {
    refractory_ms = APP_ECG_DYNAMIC_REF_MAX_MS;
  }

  return refractory_ms;
}

static void app_ecg_add_rr_history(uint16_t rr_ms)
{
  ecg_state.rr_history[ecg_state.rr_write_index] = rr_ms;
  ecg_state.rr_write_index = (uint8_t)((ecg_state.rr_write_index + 1U) %
                                       APP_ECG_RR_HISTORY_SIZE);
  if (ecg_state.rr_count < APP_ECG_RR_HISTORY_SIZE)
  {
    ecg_state.rr_count++;
  }
}

static void app_ecg_store_search_candidate(uint32_t peak_ms,
                                           uint32_t peak_abs)
{
  if (peak_ms == 0UL)
  {
    return;
  }

  ecg_state.search_peak_ms[ecg_state.search_write_index] = peak_ms;
  ecg_state.search_peak_abs[ecg_state.search_write_index] = peak_abs;
  ecg_state.search_write_index =
      (uint8_t)((ecg_state.search_write_index + 1U) %
                APP_ECG_SEARCH_HISTORY_SIZE);
  if (ecg_state.search_count < APP_ECG_SEARCH_HISTORY_SIZE)
  {
    ecg_state.search_count++;
  }
}

static uint8_t app_ecg_is_t_wave_like(uint32_t peak_ms,
                                      uint32_t peak_abs)
{
  uint32_t rr_since_last;

  if ((ecg_state.last_r_peak_ms == 0UL) || (peak_ms <= ecg_state.last_r_peak_ms) ||
      (ecg_state.signal_peak == 0UL))
  {
    return 0U;
  }

  rr_since_last = peak_ms - ecg_state.last_r_peak_ms;
  if ((rr_since_last <= APP_ECG_T_WAVE_WINDOW_MS) &&
      ((peak_abs * 100U) < (ecg_state.signal_peak * APP_ECG_T_WAVE_PEAK_PERCENT)))
  {
    return 1U;
  }

  return 0U;
}

static uint8_t app_ecg_try_searchback(AppState_t *app,
                                      uint32_t timestamp_ms,
                                      uint32_t threshold,
                                      AppEcgUpdate_t *update)
{
  uint16_t median_rr;
  uint32_t search_delay;
  uint32_t lower_threshold;
  uint32_t refractory_ms;
  uint32_t best_peak_abs = 0UL;
  uint32_t best_peak_ms = 0UL;
  uint8_t i;

  if ((app == NULL) || (update == NULL) ||
      (ecg_state.last_r_peak_ms == 0UL) ||
      (ecg_state.rr_count < 3U) ||
      (ecg_state.search_count == 0U) ||
      (timestamp_ms <= ecg_state.last_r_peak_ms))
  {
    return 0U;
  }

  median_rr = app_ecg_rr_median();
  if (median_rr == 0U)
  {
    return 0U;
  }

  search_delay = ((uint32_t)median_rr * APP_ECG_SEARCHBACK_DELAY_PERCENT) / 100U;
  if ((timestamp_ms - ecg_state.last_r_peak_ms) < search_delay)
  {
    return 0U;
  }

  lower_threshold = (threshold * APP_ECG_SEARCHBACK_THRESHOLD_PERCENT) / 100U;
  if (lower_threshold < (APP_ECG_QRS_MIN_THRESHOLD / 2U))
  {
    lower_threshold = APP_ECG_QRS_MIN_THRESHOLD / 2U;
  }

  refractory_ms = app_ecg_dynamic_refractory_ms();
  for (i = 0U; i < ecg_state.search_count; i++)
  {
    uint32_t candidate_ms = ecg_state.search_peak_ms[i];
    uint32_t candidate_abs = ecg_state.search_peak_abs[i];

    if ((candidate_ms > (ecg_state.last_r_peak_ms + refractory_ms)) &&
        ((candidate_ms - ecg_state.last_r_peak_ms) >= APP_ECG_MIN_RR_MS) &&
        (candidate_ms < timestamp_ms) &&
        ((candidate_ms - ecg_state.last_r_peak_ms) <= APP_ECG_MAX_RR_MS) &&
        (candidate_abs >= lower_threshold) &&
        (candidate_abs > best_peak_abs) &&
        (app_ecg_is_t_wave_like(candidate_ms, candidate_abs) == 0U))
    {
      best_peak_ms = candidate_ms;
      best_peak_abs = candidate_abs;
    }
  }

  if (best_peak_ms == 0UL)
  {
    return 0U;
  }

  *update = app_ecg_accept_r_peak(app, best_peak_ms, best_peak_abs);
  ecg_state.search_count = 0U;
  ecg_state.search_write_index = 0U;
  return update->r_peak_detected;
}

static AppEcgUpdate_t app_ecg_accept_r_peak(AppState_t *app,
                                            uint32_t peak_ms,
                                            uint32_t peak_abs)
{
  AppEcgUpdate_t update;
  uint32_t rr_ms;
  uint32_t hr_bpm;

  (void)memset(&update, 0, sizeof(update));
  if ((app == NULL) || (peak_ms == 0UL))
  {
    return update;
  }

  update.r_peak_detected = 1U;
  update.r_peak_ms = peak_ms;
  app->ecg_r_peak_ms = peak_ms;

  if (ecg_state.signal_peak == 0UL)
  {
    ecg_state.signal_peak = peak_abs;
  }
  else
  {
    if (peak_abs >= ecg_state.signal_peak)
    {
      ecg_state.signal_peak += ((peak_abs - ecg_state.signal_peak) >> 3);
    }
    else
    {
      ecg_state.signal_peak -= ((ecg_state.signal_peak - peak_abs) >> 3);
    }
  }

  if (ecg_state.last_r_peak_ms != 0UL)
  {
    rr_ms = peak_ms - ecg_state.last_r_peak_ms;
    if ((rr_ms >= APP_ECG_MIN_RR_MS) && (rr_ms <= APP_ECG_MAX_RR_MS))
    {
      hr_bpm = (60000UL + (rr_ms / 2UL)) / rr_ms;
      if (hr_bpm > 255UL)
      {
        hr_bpm = 255UL;
      }

      app->ecg_rr_ms = (uint16_t)rr_ms;
      app_ecg_add_rr_history((uint16_t)rr_ms);

      if ((app->ecg_valid != 0U) && (app->ecg_hr != 0U))
      {
        app->ecg_hr = (uint8_t)((((uint16_t)app->ecg_hr * 3U) +
                                  (uint16_t)hr_bpm + 2U) / 4U);
      }
      else
      {
        app->ecg_hr = (uint8_t)hr_bpm;
      }

      app->ecg_valid = 1U;
      update.rr_ms = (uint16_t)rr_ms;
      update.hr_bpm = app->ecg_hr;
      app_ptt_add_ecg_peak(peak_ms);
    }
    else
    {
      app->ecg_valid = 0U;
      app->ecg_hr = 0U;
      app->ecg_rr_ms = 0U;
      app->ptt_valid = 0U;
      app->ptt_ms = 0U;
      app_ptt_reset(app);
    }
  }

  ecg_state.last_r_peak_ms = peak_ms;
  return update;
}

/* ---- 清零整个 ECG 检测器状态 ---- */
static void app_ecg_reset_detector(void)
{
  (void)memset(&ecg_state, 0, sizeof(ecg_state));
  ecg_dma_overflow_recent = 0U;
  ecg_adc_sat_recent = 0U;
  ecg_dmwi_reset(&ecg_dmwi);
#if (APP_ECG_DSP_PREPROCESS != 0U)
  ecg_dsp_preprocess_reset();
#endif
}

/* ---- 清零 ECG 显示滤波器内部缓冲 ---- */
static void app_ecg_reset_display_filter(void)
{
  (void)memset(&ecg_disp, 0, sizeof(ecg_disp));
  ecg_debug_disp_decim = 0U;
  app_ecg_reset_visual_filter();
}

/* ---- 清零 ECG 可视化显示滤波器 ---- */
static void app_ecg_reset_visual_filter(void)
{
  (void)memset(&ecg_visual, 0, sizeof(ecg_visual));
  (void)memset(ecg_visual_avg_buf, 0, sizeof(ecg_visual_avg_buf));
  ecg_visual_avg_idx = 0U;
#if (APP_ECG_DSP_PREPROCESS != 0U) && (APP_ECG_VISUAL_ENABLE_NOTCH != 0U)
  ecg_biquad_reset(&ecg_visual_notch);
#endif
}

/* ---- 清零 ECG 调试统计窗口 ---- */
static void app_ecg_reset_debug_stats(void)
{
  (void)memset(&ecg_dbg_stats, 0, sizeof(ecg_dbg_stats));
}

/* ---- 更新原始 ADC 调试统计（在饱和检测之前调用，含饱和样本） ---- */
static void app_ecg_debug_update_raw(uint16_t raw_value,
                                     const AppState_t *app)
{
  if (ecg_dbg_stats.sample_cnt == 0U)
  {
    ecg_dbg_stats.raw_min = raw_value;
    ecg_dbg_stats.raw_max = raw_value;
  }
  else
  {
    if (raw_value < ecg_dbg_stats.raw_min)
      ecg_dbg_stats.raw_min = raw_value;
    if (raw_value > ecg_dbg_stats.raw_max)
      ecg_dbg_stats.raw_max = raw_value;
  }
  ecg_dbg_stats.sample_cnt++;

#if (APP_ECG_DEBUG_PRINTF != 0U)
  if (ecg_dbg_stats.sample_cnt >= 250U)
  {
    app_ecg_debug_print_stats(&ecg_dbg_stats, app);
    (void)memset(&ecg_dbg_stats, 0, sizeof(ecg_dbg_stats));
  }
#endif
}

/* ---- 更新滤波后 ECG 调试统计（仅在滤波计算完成后调用） ---- */
static void app_ecg_debug_update_filtered(int16_t filtered_value)
{
  /* 复位后 filt_min == filt_max == 0 表示首个样本 */
  if (ecg_dbg_stats.filt_min == 0 && ecg_dbg_stats.filt_max == 0)
  {
    ecg_dbg_stats.filt_min = filtered_value;
    ecg_dbg_stats.filt_max = filtered_value;
  }
  else
  {
    if (filtered_value < ecg_dbg_stats.filt_min)
      ecg_dbg_stats.filt_min = filtered_value;
    if (filtered_value > ecg_dbg_stats.filt_max)
      ecg_dbg_stats.filt_max = filtered_value;
  }
}

/* ---- 32 位有符号绝对值 ---- */
static uint32_t app_ecg_abs_i32(int32_t v)
{
  return (v < 0) ? (uint32_t)(-v) : (uint32_t)v;
}

/* ---- 将 int32 钳位到 int16 范围 ---- */
static int16_t app_ecg_clamp_i16(int32_t v)
{
  if (v > 32767L)  return 32767;
  if (v < -32768L) return -32768;
  return (int16_t)v;
}

static uint8_t app_ecg_smooth_quality(uint8_t current, uint8_t target)
{
  uint16_t delta;

  if (target >= current)
  {
    ecg_quality_fall_decim = 0U;
    delta = (uint16_t)target - (uint16_t)current;
    if (delta == 0U) { return current; }
    delta = (delta + 7U) / 8U;
    if (delta == 0U) { delta = 1U; }
    return (uint8_t)((uint16_t)current + delta);
  }

  delta = (uint16_t)current - (uint16_t)target;
  if (delta == 0U) { return current; }
  ecg_quality_fall_decim++;
  if (ecg_quality_fall_decim < 10U) { return current; }
  ecg_quality_fall_decim = 0U;
  return (uint8_t)(current - 1U);
}

/* ---- ECG 质量评分：0-100，基于信号范围和 SNR ---- */
static void app_ecg_latch_quality_drop(AppState_t *app,
                                       uint8_t prev_reason,
                                       uint8_t prev_sq)
{
  uint8_t reason;

  if (app == NULL) { return; }

  reason = app->ecg_invalid_reason;
  if (reason == ECG_INVALID_OK) { return; }

  if ((prev_reason != ECG_INVALID_OK) &&
      (prev_reason == reason) &&
      !((prev_sq != 0U) && (app->ecg_signal_quality == 0U)))
  {
    return;
  }

  app->ecg_last_drop_reason = reason;
  app->ecg_last_drop_sq = app->ecg_signal_quality;
  app->ecg_last_drop_raw_span = app->ecg_raw_span;
  app->ecg_last_drop_filtered_span = app->ecg_filtered_span;
  app->ecg_last_drop_snr_x100 = app->ecg_peak_snr_x100;
  app->ecg_last_drop_dma_hwm = app->ecg_dma_available_high_watermark;
  app->ecg_last_drop_ms = HAL_GetTick();
}

static uint8_t app_ecg_compute_quality_score(uint16_t filt_span,
                                             uint16_t peak_snr_x100,
                                             uint8_t has_recent_r_peak)
{
  uint32_t score = 0UL;

  if (filt_span == 0U || !has_recent_r_peak) return 0U;

  /* filtered span 贡献 0–40 分 */
  if (filt_span >= 200U)       score += 40UL;
  else if (filt_span >= 100U)  score += 30UL;
  else if (filt_span >= 50U)   score += 20UL;
  else if (filt_span >= 25U)   score += 10UL;

  /* SNR×100 贡献 0–60 分 */
  if (peak_snr_x100 >= 500U)       score += 60UL;
  else if (peak_snr_x100 >= 300U)  score += 50UL;
  else if (peak_snr_x100 >= 200U)  score += 40UL;
  else if (peak_snr_x100 >= 150U)  score += 30UL;
  else if (peak_snr_x100 >= 100U)  score += 20UL;
  else if (peak_snr_x100 >= 50U)  score += 10UL;

  return (score > 100UL) ? 100U : (uint8_t)score;
}

/* ---- 更新 AppState ECG 质量字段 ---- */
void app_ecg_update_quality(AppState_t *app)
{
  uint32_t threshold;
  uint32_t now_ms;
  uint32_t no_r_age_ms;
  uint16_t raw_span;
  uint16_t filt_span;
  uint32_t noise;
  uint8_t  lead_off;
  uint8_t  has_recent;
  uint8_t  sat_reason;
  uint8_t  instant_quality;
  uint8_t  instant_reason;
  uint8_t  prev_reason;
  uint8_t  prev_sq;

  if (app == NULL) return;

  prev_reason = app->ecg_invalid_reason;
  prev_sq = app->ecg_signal_quality;
  lead_off = app->ecg_lead_off;
  sat_reason = ecg_adc_sat_recent;
  ecg_adc_sat_recent = 0U;

  /* 原始值和滤波后跨度 */
  if (ecg_dbg_stats.sample_cnt >= APP_ECG_QUALITY_MIN_SAMPLES) {
    raw_span = (uint16_t)(ecg_dbg_stats.raw_max - ecg_dbg_stats.raw_min);
    filt_span = (uint16_t)(app_ecg_abs_i32(ecg_dbg_stats.filt_max - ecg_dbg_stats.filt_min));
    app->ecg_raw_span = raw_span;
    app->ecg_filtered_span = filt_span;
    if (ecg_dbg_stats.sample_cnt >= APP_ECG_QUALITY_WINDOW_SAMPLES) {
      app_ecg_reset_debug_stats();
    }
  } else {
    raw_span = app->ecg_raw_span;
    filt_span = app->ecg_filtered_span;
  }

  /* QRS 检测器状态 */
  noise = ecg_state.noise_level;
  app->ecg_noise_level = noise;

  threshold = app_ecg_current_threshold();
  app->ecg_qrs_threshold = threshold;

  /* 最新 R 峰 SNR */
  now_ms = HAL_GetTick();
  no_r_age_ms = (ecg_state.last_r_peak_ms != 0UL) ?
                (now_ms - ecg_state.last_r_peak_ms) : 0UL;
  has_recent = ((ecg_state.last_r_peak_ms != 0UL) &&
                (no_r_age_ms <= APP_ECG_STALE_MS)) ? 1U : 0U;

  if ((noise > 0UL) && (ecg_state.in_candidate == 0U)) {
    uint32_t snr = (ecg_state.candidate_peak_abs > 0UL)
                   ? (ecg_state.candidate_peak_abs * 100UL / noise)
                   : ((uint32_t)threshold * 100UL / noise);
    app->ecg_peak_snr_x100 = (snr > 65535UL) ? 65535U : (uint16_t)snr;
  } else {
    /* 候选态中暂不更新 SNR，保留上一值 */
  }

  /* 原因码和评分决策 */
  if (lead_off != 0U) {
    app->ecg_signal_quality = 0U;
    app->ecg_invalid_reason = ECG_INVALID_LEAD_OFF;
    app_ecg_latch_quality_drop(app, prev_reason, prev_sq);
    return;
  }

  /* 检查近期 DMA 溢出（最近批次） */
  if (ecg_dma_overflow_recent != 0U) {
    ecg_dma_overflow_recent = 0U;
    app->ecg_signal_quality = 0U;
    app->ecg_invalid_reason = ECG_INVALID_DMA_OVERFLOW;
    app_ecg_latch_quality_drop(app, prev_reason, prev_sq);
    return;
  }

  if (raw_span <= APP_ECG_RAW_FLATLINE_SPAN) {
    app->ecg_signal_quality = app_ecg_smooth_quality(app->ecg_signal_quality, 0U);
    app->ecg_invalid_reason = ECG_INVALID_RAW_FLATLINE;
    app_ecg_latch_quality_drop(app, prev_reason, prev_sq);
    return;
  }

  if (!has_recent) {
    instant_quality = app_ecg_compute_quality_score(
        filt_span, app->ecg_peak_snr_x100, 1U);
    if (instant_quality > APP_ECG_NO_R_SOFT_SQ_CAP) {
      instant_quality = APP_ECG_NO_R_SOFT_SQ_CAP;
    }
    app->ecg_invalid_reason = ECG_INVALID_NO_R_PEAK;
    if ((ecg_state.last_r_peak_ms == 0UL) ||
        (no_r_age_ms > APP_ECG_HARD_STALE_MS))
    {
      app->ecg_signal_quality = 0U;
    }
    else
    {
      app->ecg_signal_quality = app_ecg_smooth_quality(app->ecg_signal_quality, instant_quality);
    }
    app_ecg_latch_quality_drop(app, prev_reason, prev_sq);
    return;
  }

  /* filtered span < 25 LSB → 信号幅度极低 */
  if (filt_span < 25U) {
    app->ecg_signal_quality = app_ecg_smooth_quality(app->ecg_signal_quality, APP_ECG_LOW_AMP_SQ_CAP);
    app->ecg_invalid_reason = ECG_INVALID_LOW_AMPLITUDE;
    app_ecg_latch_quality_drop(app, prev_reason, prev_sq);
    return;
  }

  /* noise 非常高且 SNR 低 → 信号被噪声淹没 */
  if ((noise > 500UL) && (app->ecg_peak_snr_x100 < 80U)) {
    app->ecg_invalid_reason = ECG_INVALID_NOISY;
    instant_quality = (uint8_t)((app->ecg_peak_snr_x100 * 25U) / 80U);
    if (instant_quality > 40U) instant_quality = 40U;
    app->ecg_signal_quality = app_ecg_smooth_quality(app->ecg_signal_quality, instant_quality);
    app_ecg_latch_quality_drop(app, prev_reason, prev_sq);
    return;
  }

  /* 检查 ADC 饱和 -- 不作为致命错误但降低评分 */
  {
    instant_reason = sat_reason ? ECG_INVALID_ADC_SAT : ECG_INVALID_OK;
    instant_quality = app_ecg_compute_quality_score(
        filt_span, app->ecg_peak_snr_x100, has_recent);

    /* ADC 饱和惩罚：减半 */
    if (sat_reason) {
      instant_quality /= 2U;
    }
    app->ecg_invalid_reason = instant_reason;
    app->ecg_signal_quality = app_ecg_smooth_quality(app->ecg_signal_quality, instant_quality);
    app_ecg_latch_quality_drop(app, prev_reason, prev_sq);
  }
}
