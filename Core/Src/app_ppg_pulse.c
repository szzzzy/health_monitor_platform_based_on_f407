/**
  ******************************************************************************
  * @file    app_ppg_pulse.c
  * @brief   PPG 脉搏波峰检测与 IBI 提取（状态机型）
  *
  * 在带通滤波后的 PPG 信号上做状态机驱动的逐点峰/谷检测，
  * 提取逐拍间隔 (IBI) 与峰谷幅度，推送至 HRV 和 RR 模块。
  *
  * 检测算法（状态机）：
  *   1. IDLE — 跟踪初始信号摆动，积累足够幅值后进入 RISING
  *   2. RISING — 更新局部最大值；信号回落达到迟滞值 → 确认峰
  *   3. FALLING — 更新局部最小值；信号回升达到迟滞值 → 确认谷
  *   4. 每次确认峰时用峰谷幅度更新 beat_amp_ema
  *   5. 同极性峰间隔 (IBI) 必须在 [300ms, 2000ms] 范围内
  *   6. 迟滞值 = max(2, beat_amp_ema / 4)，初始用噪声门限
  *
  * 相比旧版 3 点相邻突出度：
  *   - 不受 PPG 峰顶圆滑导致的相邻点差过小问题影响
  *   - beat_amplitude 是真正的峰谷幅度，适合 RR/PI 模块
  *   - 自适应迟滞值随信号强度自动调节
  ******************************************************************************
  */

#include "app_ppg_pulse.h"

#include <string.h>

#include "app_display.h"
#include "app_hrv.h"
#include "app_rr.h"

/* 动态阈值参数 */
#define APP_BEAT_NOISE_FLOOR_DIV    5U    /* 噪声门限 = AC_RMS / 5 */
#define APP_BEAT_NOISE_FLOOR_MIN    2U    /* 绝对最小噪声门限 */
#define APP_BEAT_NOISE_FLOOR_CAP    20U   /* 噪声门限上限，避免强信号时门槛过高 */
#define APP_BEAT_HYSTERESIS_DIV     5U    /* 迟滞 = beat_amp_ema / 5 */
#define APP_BEAT_HYSTERESIS_MIN     2U    /* 绝对最小迟滞 */
#define APP_BEAT_EMA_NEW_WEIGHT     1U    /* EMA 新值权重 (1/8) */
#define APP_BEAT_EMA_OLD_WEIGHT     7U    /* EMA 旧值权重 (7/8) */
#define APP_BEAT_EMA_SHIFT          3U    /* 除数 = 2^3 = 8，用于手动 EMA */
#define APP_BEAT_SIGNAL_QUALITY_MIN 25U
#define APP_BEAT_IBI_HISTORY_SIZE   5U
#define APP_BEAT_IBI_JUMP_PERCENT   40U
#define APP_BEAT_MIN_NOISE_MULT     2U
#define APP_BEAT_AMP_MIN_PERCENT    35U
#define APP_BEAT_AMP_MAX_PERCENT    280U

/* 状态机状态 */
#define BEAT_STATE_IDLE     0U
#define BEAT_STATE_RISING   1U
#define BEAT_STATE_FALLING  2U

/* 流式搏动检测器内部状态 */
static struct
{
  int32_t  local_max;           /* 当前上升段局部最大值 */
  uint32_t local_max_sample;    /* local_max 的全局样本编号 */
  int32_t  local_min;           /* 当前下降段局部最小值 */
  uint32_t local_min_sample;    /* local_min 的全局样本编号 */
  int32_t  prev_trough;         /* 上一个已确认谷值（用于峰谷幅度） */
  uint32_t prev_peak_sample;    /* 上一个已确认峰的全局样本编号 */
  uint32_t beat_amp_ema;        /* 峰谷幅度的 EMA */
  uint8_t  beat_amp_ema_valid;  /* EMA 已初始化 */
  uint8_t  state;               /* 当前状态机状态 */
  uint8_t  sample_count;        /* 已接收样本计数（初始化用） */
  int32_t  sample_buf[2];       /* 初始化窗口样本缓冲 */
  uint32_t hysteresis;          /* 当前迟滞值（从 EMA 或噪声门限推导） */
  uint16_t ibi_history[APP_BEAT_IBI_HISTORY_SIZE];
  uint8_t  ibi_count;
  uint8_t  ibi_write_index;
} beat;

static uint32_t app_ppg_abs_diff_u32(uint32_t lhs, uint32_t rhs);
static uint16_t app_ppg_pulse_median_ibi(void);
static void app_ppg_pulse_add_ibi_history(uint16_t ibi_ms);
static uint8_t app_ppg_pulse_quality_ok(const AppState_t *app,
                                        uint16_t ibi_ms,
                                        uint32_t beat_amplitude,
                                        uint32_t noise_floor);
static void app_ppg_pulse_update_amp_ema(uint32_t beat_amplitude);

/**
 ******************************************************************************
 * @brief  重置 PPG 脉搏状态机（清除所有内部状态）。
 * @note   手指离开或测量重置时调用。清零整个搏动检测器结构体，
 *         包括 IDLE 状态和 EMA 值。
 ******************************************************************************
 */
void app_ppg_pulse_reset(void)
{
  (void)memset(&beat, 0, sizeof(beat));
}

/**
 ******************************************************************************
 * @brief  通过脉搏检测状态机处理一个滤波后的 PPG 样本。
 * @param  app             AppState 指针（内部检查 finger_present）。
 * @param  filtered_sample 带通滤波后的 PPG AC 样本值。
 * @param  total_samples   自启动以来的传感器样本总数。
 * @param  pulse_info      检测到的搏动输出结构体（无时清零）。
 * @return 检测到有效脉搏返回 1，否则返回 0。
 * @note   状态机：IDLE -> RISING -> FALLING。迟滞自适应来自
 *         beat_amp_ema。IBI 验证范围 [300, 2000] ms。
 ******************************************************************************
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

  uint32_t noise_floor;
  uint32_t interval_samples;
  uint32_t beat_amplitude;
  uint8_t  pulse_detected = 0U;

  if (pulse_info != NULL)
  {
    (void)memset(pulse_info, 0, sizeof(*pulse_info));
  }

  if ((app == NULL) || (pulse_info == NULL) || (app->finger_present == 0U))
  {
    app_ppg_pulse_reset();
    return 0U;
  }

  /* 初始化 2 点缓冲（用于确认初始运动方向） */
  if (beat.sample_count < 2U)
  {
    beat.sample_buf[beat.sample_count] = filtered_sample;
    beat.sample_count++;
    if (beat.sample_count >= 2U)
    {
      /* 用前两个样本初始化局部最小值/最大值 */
      beat.local_min = (beat.sample_buf[0] < beat.sample_buf[1]) ?
                        beat.sample_buf[0] : beat.sample_buf[1];
      beat.local_max = (beat.sample_buf[0] > beat.sample_buf[1]) ?
                        beat.sample_buf[0] : beat.sample_buf[1];
      beat.local_min_sample = total_samples - 1U;
      beat.local_max_sample = total_samples - 1U;
      beat.state = BEAT_STATE_IDLE;
      beat.prev_trough = beat.local_min;
    }
    return 0U;
  }

  /* 噪声门限：基于 IR 通道带通 AC RMS */
  noise_floor = app->signal_ir_ac_rms / APP_BEAT_NOISE_FLOOR_DIV;
  if (noise_floor < APP_BEAT_NOISE_FLOOR_MIN)
  {
    noise_floor = APP_BEAT_NOISE_FLOOR_MIN;
  }
  if (noise_floor > APP_BEAT_NOISE_FLOOR_CAP)
  {
    noise_floor = APP_BEAT_NOISE_FLOOR_CAP;
  }

  /* 迟滞：从 EMA 推导，未初始化时用噪声门限 */
  if (beat.beat_amp_ema_valid != 0U)
  {
    beat.hysteresis = beat.beat_amp_ema / APP_BEAT_HYSTERESIS_DIV;
    if (beat.hysteresis < APP_BEAT_HYSTERESIS_MIN)
    {
      beat.hysteresis = APP_BEAT_HYSTERESIS_MIN;
    }
  }
  else
  {
    beat.hysteresis = noise_floor;
  }

  /*
   * 状态机处理
   */
  if (beat.state == BEAT_STATE_IDLE)
  {
    /* IDLE：跟踪信号摆动，积累足够幅值后切换到 RISING */
    if (filtered_sample < beat.local_min)
    {
      beat.local_min = filtered_sample;
      beat.local_min_sample = total_samples;
    }
    if (filtered_sample > beat.local_max)
    {
      beat.local_max = filtered_sample;
      beat.local_max_sample = total_samples;
    }

    if ((beat.local_max - beat.local_min) >= (int32_t)noise_floor)
    {
      /* 信号摆动足够大，进入跟踪 */
      beat.state = BEAT_STATE_RISING;
      beat.prev_trough = beat.local_min;
      beat.local_max = filtered_sample;
      beat.local_max_sample = total_samples;
    }
  }
  else if (beat.state == BEAT_STATE_RISING)
  {
    /* RISING：跟踪上升沿，检测峰 */
    if (filtered_sample > beat.local_max)
    {
      beat.local_max = filtered_sample;
      beat.local_max_sample = total_samples;
    }
    else if ((beat.local_max - filtered_sample) >= (int32_t)beat.hysteresis)
    {
      /* 峰确认：信号从局部最大值回落达到迟滞值 */
      beat_amplitude = (uint32_t)(beat.local_max - beat.prev_trough);

      /* 仅当幅度超过噪声门限才接受 */
      if (beat_amplitude >= (noise_floor * APP_BEAT_MIN_NOISE_MULT))
      {
        /* 计算 IBI（从上一个峰到当前峰） */
        if (beat.prev_peak_sample != 0U)
        {
          interval_samples = beat.local_max_sample - beat.prev_peak_sample;
          if ((interval_samples >= min_interval_samples) &&
              (interval_samples <= max_interval_samples))
          {
            uint16_t ibi_ms;

            ibi_ms = (uint16_t)((interval_samples * 1000U +
                                 (MAX30102_ALGO_SAMPLE_RATE_HZ / 2U)) /
                                MAX30102_ALGO_SAMPLE_RATE_HZ);

            if (app_ppg_pulse_quality_ok(app, ibi_ms, beat_amplitude, noise_floor) != 0U)
            {
              pulse_info->beat_valid = 1U;
              pulse_info->interval_samples = (uint16_t)interval_samples;
              pulse_info->latest_ibi_ms = ibi_ms;
              pulse_info->latest_peak_sample = beat.local_max_sample;
              pulse_info->beat_amplitude = beat_amplitude;
              pulse_detected = 1U;
              app_ppg_pulse_add_ibi_history(ibi_ms);
              app_ppg_pulse_update_amp_ema(beat_amplitude);
              beat.prev_peak_sample = beat.local_max_sample;
            }
          }
          else if (interval_samples > max_interval_samples)
          {
            if ((app->signal_quality >= APP_BEAT_SIGNAL_QUALITY_MIN) &&
                (app->motion_artifact == 0U))
            {
              beat.prev_peak_sample = beat.local_max_sample;
              app_ppg_pulse_update_amp_ema(beat_amplitude);
            }
          }
        }

        else if ((app->signal_quality >= APP_BEAT_SIGNAL_QUALITY_MIN) &&
                 (app->motion_artifact == 0U))
        {
          beat.prev_peak_sample = beat.local_max_sample;
          app_ppg_pulse_update_amp_ema(beat_amplitude);
        }
      }

      /* 切换到下降跟踪 */
      beat.local_min = filtered_sample;
      beat.local_min_sample = total_samples;
      beat.state = BEAT_STATE_FALLING;
    }
  }
  else /* BEAT_STATE_FALLING */
  {
    /* FALLING：跟踪下降沿，检测谷 */
    if (filtered_sample < beat.local_min)
    {
      beat.local_min = filtered_sample;
      beat.local_min_sample = total_samples;
    }
    else if ((filtered_sample - beat.local_min) >= (int32_t)beat.hysteresis)
    {
      /* 谷确认：信号从局部最小值回升达到迟滞值 */
      beat.prev_trough = beat.local_min;

      /* 切换到上升跟踪 */
      beat.local_max = filtered_sample;
      beat.local_max_sample = total_samples;
      beat.state = BEAT_STATE_RISING;
    }
  }

  return pulse_detected;
}

/**
 ******************************************************************************
 * @brief  后处理检测到的脉搏：推送 IBI 到 HRV、更新 PI、通知 RR。
 * @param  app         AppState 指针。
 * @param  pulse_info  检测到的脉搏信息（须 beat_valid=1）。
 * @param  total_samples 自启动以来的传感器样本总数（用于 HRV 峰值窗口）。
 * @note   流水线：HRV 峰值去重 -> IBI 添加 -> 基于搏动的 PI EMA ->
 *         OLED 标记 -> RR 幅值推送（依赖信号质量）。被拒绝的
 *         重复峰值或 IBI 静默返回。
 ******************************************************************************
 */
uint8_t app_ppg_pulse_process_metrics(AppState_t *app,
                                      const MAX30102_PulseInfo_t *pulse_info,
                                      uint32_t total_samples)
{
  if ((app == NULL) || (pulse_info == NULL) || (pulse_info->beat_valid == 0U))
  {
    return 0U;
  }

  /* HRV 峰值可见性窗口：防止同一峰被重复标记 */
  if (app_hrv_mark_peak_seen(pulse_info->latest_peak_sample) == 0U)
  {
    return 0U;
  }

  /* 记录 IBI 到 HRV 缓冲区 */
  if (app_hrv_add_ibi(app, pulse_info->latest_ibi_ms) == 0U)
  {
    return 0U;
  }

  /* 更新基于搏动的 PI EMA（每个已接受搏动更新一次） */
  if (pulse_info->beat_amplitude != 0U)
  {
    if (app->ir_pi_ac_ema_valid != 0U)
    {
      app->ir_pi_ac_ema = ((app->ir_pi_ac_ema * APP_BEAT_EMA_OLD_WEIGHT) +
                           ((uint32_t)pulse_info->beat_amplitude * APP_BEAT_EMA_NEW_WEIGHT) +
                           (1U << (APP_BEAT_EMA_SHIFT - 1U))) >> APP_BEAT_EMA_SHIFT;
    }
    else
    {
      app->ir_pi_ac_ema = pulse_info->beat_amplitude;
      app->ir_pi_ac_ema_valid = 1U;
    }
  }

  app->last_beat_sample = total_samples;
  (void)total_samples;

  /* 触发 OLED 波形脉搏标记点 */
  app_display_add_ir_pulse_marker();

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

  return 1U;
}

static uint32_t app_ppg_abs_diff_u32(uint32_t lhs, uint32_t rhs)
{
  return (lhs >= rhs) ? (lhs - rhs) : (rhs - lhs);
}

static uint16_t app_ppg_pulse_median_ibi(void)
{
  uint16_t sorted[APP_BEAT_IBI_HISTORY_SIZE];
  uint16_t tmp;
  uint8_t i;
  uint8_t j;
  uint8_t n;
  uint8_t idx;

  n = (beat.ibi_count < APP_BEAT_IBI_HISTORY_SIZE) ? beat.ibi_count :
                                                    APP_BEAT_IBI_HISTORY_SIZE;
  if (n == 0U)
  {
    return 0U;
  }

  for (i = 0U; i < n; i++)
  {
    idx = (uint8_t)((beat.ibi_write_index + APP_BEAT_IBI_HISTORY_SIZE - 1U - i) %
                    APP_BEAT_IBI_HISTORY_SIZE);
    sorted[i] = beat.ibi_history[idx];
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

static void app_ppg_pulse_add_ibi_history(uint16_t ibi_ms)
{
  beat.ibi_history[beat.ibi_write_index] = ibi_ms;
  beat.ibi_write_index = (uint8_t)((beat.ibi_write_index + 1U) %
                                   APP_BEAT_IBI_HISTORY_SIZE);
  if (beat.ibi_count < APP_BEAT_IBI_HISTORY_SIZE)
  {
    beat.ibi_count++;
  }
}

static uint8_t app_ppg_pulse_quality_ok(const AppState_t *app,
                                        uint16_t ibi_ms,
                                        uint32_t beat_amplitude,
                                        uint32_t noise_floor)
{
  uint16_t median_ibi;

  if (app == NULL)
  {
    return 0U;
  }

  if ((app->signal_quality < APP_BEAT_SIGNAL_QUALITY_MIN) ||
      (app->motion_artifact != 0U))
  {
    return 0U;
  }

  if ((ibi_ms < APP_HRV_IBI_MIN_MS) || (ibi_ms > APP_HRV_IBI_MAX_MS))
  {
    return 0U;
  }

  if (beat_amplitude < (noise_floor * APP_BEAT_MIN_NOISE_MULT))
  {
    return 0U;
  }

  if (beat.beat_amp_ema_valid != 0U)
  {
    if ((beat_amplitude * 100U) <
        (beat.beat_amp_ema * APP_BEAT_AMP_MIN_PERCENT))
    {
      return 0U;
    }

    if ((beat_amplitude * 100U) >
        (beat.beat_amp_ema * APP_BEAT_AMP_MAX_PERCENT))
    {
      return 0U;
    }
  }

  if (beat.ibi_count >= 3U)
  {
    median_ibi = app_ppg_pulse_median_ibi();
    if ((median_ibi != 0U) &&
        (app_ppg_abs_diff_u32(ibi_ms, median_ibi) >
         (((uint32_t)median_ibi * APP_BEAT_IBI_JUMP_PERCENT) / 100U)))
    {
      return 0U;
    }
  }

  return 1U;
}

static void app_ppg_pulse_update_amp_ema(uint32_t beat_amplitude)
{
  if (beat.beat_amp_ema_valid != 0U)
  {
    beat.beat_amp_ema = ((beat.beat_amp_ema * APP_BEAT_EMA_OLD_WEIGHT) +
                         (beat_amplitude * APP_BEAT_EMA_NEW_WEIGHT) +
                         (1U << (APP_BEAT_EMA_SHIFT - 1U))) >> APP_BEAT_EMA_SHIFT;
  }
  else
  {
    beat.beat_amp_ema = beat_amplitude;
    beat.beat_amp_ema_valid = 1U;
  }
}
