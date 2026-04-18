#include "app_measurement.h"

#include <string.h>

#include "app_display.h"
#include "i2c.h"
#include "max30102.h"

/*
 * 手指检测采用“基线 + 双阈值 + 连续命中确认”：
 * - ON 阈值更高，避免环境抖动误判为有手指
 * - OFF 阈值更低，保证移开手指后能及时释放状态
 */
#define MAX30102_FINGER_ON_DELTA          6000UL
#define MAX30102_FINGER_OFF_DELTA         3000UL
#define MAX30102_FINGER_ON_NOISE_GAIN     4UL
#define MAX30102_FINGER_OFF_NOISE_GAIN    2UL
#define MAX30102_FINGER_ON_DELTA_MAX      18000UL
#define MAX30102_FINGER_OFF_DELTA_MAX     9000UL
#define MAX30102_SIGNAL_SPAN_MIN          800UL
#define MAX30102_SIGNAL_SPAN_MAX          12000UL
#define MAX30102_SIGNAL_SPAN_NOISE_GAIN   3UL
#define MAX30102_REACQUIRE_NOISE_IR       3000UL
#define MAX30102_FINGER_ON_CONFIRM_COUNT  3U
#define MAX30102_FINGER_OFF_CONFIRM_COUNT 3U
#define APP_SENSOR_RECOVERY_ERROR_COUNT   8U

/* BPM 输出平滑参数，用于抑制跳变与尖峰。 */
#define APP_BPM_CONFIRM_DELTA             8U
#define APP_BPM_SWITCH_DELTA              14U
#define APP_BPM_SPIKE_DELTA               24U
#define APP_BPM_START_CONFIRM_COUNT       3U
#define APP_BPM_SWITCH_CONFIRM_COUNT      4U
#define APP_BPM_SPIKE_CONFIRM_COUNT       8U
#define APP_BPM_INVALID_HOLD_TICKS        60U
#define APP_BPM_EVALUATE_INTERVAL_SAMPLES 5U
#define APP_BPM_MAX_STEP_PER_UPDATE       3U
#define APP_SIGNAL_QUALITY_MIN_FOR_SPO2   30U
#define APP_SIGNAL_QUALITY_MIN_FOR_BPM    40U

static uint8_t fifo_buf[6];
static MAX30102_Baseline_t baseline_data;
static MAX30102_SpO2_t spo2_state;
static struct
{
  uint32_t red_value;
  uint32_t ir_value;
  uint8_t initialized;
} sample_debug_state;
static uint8_t bpm_update_decimator;
static struct
{
  uint32_t ir_high;
  uint32_t ir_low;
  uint32_t red_high;
  uint32_t red_low;
} signal_envelope;

static uint32_t app_abs_diff_u32(uint32_t lhs, uint32_t rhs);
static uint8_t app_abs_diff_u8(uint8_t lhs, uint8_t rhs);
static uint8_t app_limit_bpm_step(uint8_t current_bpm, uint8_t target_bpm, uint8_t max_step);
static uint8_t app_median3_u8(uint8_t a, uint8_t b, uint8_t c);
static uint8_t app_filter_raw_bpm(AppState_t *app, uint8_t raw_bpm_value);
static uint32_t app_slow_follow_u32(uint32_t current, uint32_t target, uint8_t shift);
static void app_reset_signal_envelope(void);
static void app_update_signal_activity(AppState_t *app);
static uint32_t app_get_signal_span_threshold(const AppState_t *app);
static uint8_t app_is_raw_signal_present(const AppState_t *app);
static void app_track_background_ir(AppState_t *app);
static void app_reset_measurement_outputs(AppState_t *app);
static void app_update_bpm_output(AppState_t *app, uint8_t raw_bpm_valid, uint8_t raw_bpm_value);

/* 初始化测量模块相关的自适应阈值。 */
void app_measurement_init_state(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  app->adaptive_finger_on_delta = MAX30102_FINGER_ON_DELTA;
  app->adaptive_finger_off_delta = MAX30102_FINGER_OFF_DELTA;
}

/* 复位运行期算法状态，重新开始一次完整测量流程。 */
void app_measurement_reset_runtime(void)
{
  max30102_baseline_reset(&baseline_data);
  max30102_spo2_reset(&spo2_state);
  (void)memset(&sample_debug_state, 0, sizeof(sample_debug_state));
  bpm_update_decimator = 0U;
  app_reset_signal_envelope();
  app_display_reset_waveforms();
}

/* 基线采集阶段：读样本并累积背景 IR。 */
uint8_t app_measurement_collect_baseline_sample(AppState_t *app)
{
  if (app == NULL)
  {
    return 0U;
  }

  if (max30102_read_fifo(fifo_buf, 6U) != HAL_OK)
  {
    return 0U;
  }

  max30102_parse_spo2_sample(fifo_buf, &app->red_value, &app->ir_value);
  max30102_baseline_add_ir(&baseline_data, app->ir_value);
  return 1U;
}

/* 判断是否已经采集到足够多的基线样本。 */
uint8_t app_measurement_baseline_ready(void)
{
  return max30102_baseline_is_ready(&baseline_data, APP_MEASUREMENT_BASELINE_SAMPLES);
}

/* 返回 0~100 的基线采集进度，供状态页显示。 */
uint16_t app_measurement_get_baseline_progress_percent(void)
{
  if (baseline_data.sample_count >= APP_MEASUREMENT_BASELINE_SAMPLES)
  {
    return 100U;
  }

  return (uint16_t)((baseline_data.sample_count * 100U) / APP_MEASUREMENT_BASELINE_SAMPLES);
}

/* 读取基线平均值。 */
uint32_t app_measurement_get_baseline_average(void)
{
  return max30102_baseline_get_average_ir(&baseline_data);
}

/* 读取基线期间的波动范围。 */
uint32_t app_measurement_get_baseline_range(void)
{
  return max30102_baseline_get_range_ir(&baseline_data);
}

/* 获取后台跟踪后的当前基线。 */
uint32_t app_measurement_get_tracked_baseline(void)
{
  return max30102_baseline_get_tracked_ir(&baseline_data);
}

/* 用采集得到的背景值初始化跟踪器，进入正常运行阶段。 */
void app_measurement_seed_baseline_tracking(uint32_t baseline_ir, uint32_t noise_ir)
{
  max30102_baseline_seed_tracking(&baseline_data, baseline_ir, noise_ir);
}

/* 判断背景是否稳定，便于提示现场光照/接线问题。 */
uint8_t app_measurement_baseline_is_stable(void)
{
  return max30102_baseline_is_stable(&baseline_data, APP_MEASUREMENT_BASELINE_STABLE_RANGE);
}

/* 正常运行阶段读取一个 RED/IR 样本，并同步刷新基线快照。 */
AppMeasurementReadStatus_t app_measurement_read_sensor_sample(AppState_t *app)
{
  HAL_StatusTypeDef read_status;
  const MAX30102_FifoDebug_t *fifo_debug;

  if (app == NULL)
  {
    return APP_MEASUREMENT_READ_ERROR;
  }

  read_status = max30102_read_fifo(fifo_buf, 6U);
  fifo_debug = max30102_get_fifo_debug();
  if (fifo_debug != NULL)
  {
    app->sensor_fifo_overflow_count = fifo_debug->overflow_count;
    app->sensor_fifo_write_ptr = fifo_debug->write_ptr;
    app->sensor_fifo_read_ptr = fifo_debug->read_ptr;
    app->sensor_fifo_available_samples = fifo_debug->available_samples;
  }

  if (read_status == HAL_OK)
  {
    max30102_parse_spo2_sample(fifo_buf, &app->red_value, &app->ir_value);
    if (sample_debug_state.initialized != 0U)
    {
      if ((sample_debug_state.red_value != app->red_value) ||
          (sample_debug_state.ir_value != app->ir_value))
      {
        app->sensor_sample_change_count++;
      }
      else
      {
        app->sensor_sample_same_count++;
      }
    }
    else
    {
      app->sensor_sample_change_count++;
      sample_debug_state.initialized = 1U;
    }

    sample_debug_state.red_value = app->red_value;
    sample_debug_state.ir_value = app->ir_value;
    app->baseline_ir = max30102_baseline_get_tracked_ir(&baseline_data);
    app_update_signal_activity(app);
    app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_OK;
    app->sensor_read_ok_count++;
    app->sensor_error_streak = 0U;
    app->sensor_last_i2c_error = HAL_I2C_ERROR_NONE;
    app->sensor_last_sample_tick = HAL_GetTick();
    return APP_MEASUREMENT_READ_OK;
  }

  if (read_status == HAL_BUSY)
  {
    app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_WAIT;
    app->sensor_read_busy_count++;
    app->sensor_error_streak = 0U;
    app->sensor_last_i2c_error = HAL_I2C_ERROR_NONE;
    return APP_MEASUREMENT_READ_WAIT;
  }

  app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_ERROR;
  app->sensor_read_error_count++;
  app->sensor_last_i2c_error = HAL_I2C_GetError(&hi2c1);
  if (app->sensor_error_streak < 0xFFU)
  {
    app->sensor_error_streak++;
  }

  return APP_MEASUREMENT_READ_ERROR;
}

/* 根据背景噪声动态放大/收敛手指检测阈值，减小误判。 */
void app_measurement_update_adaptive_thresholds(AppState_t *app)
{
  uint32_t baseline_noise;
  uint32_t on_delta;
  uint32_t off_delta;

  if (app == NULL)
  {
    return;
  }

  baseline_noise = max30102_baseline_get_noise_ir(&baseline_data);
  on_delta = baseline_noise * MAX30102_FINGER_ON_NOISE_GAIN;
  off_delta = baseline_noise * MAX30102_FINGER_OFF_NOISE_GAIN;
  app->adaptive_finger_on_delta = MAX30102_FINGER_ON_DELTA;
  app->adaptive_finger_off_delta = MAX30102_FINGER_OFF_DELTA;

  if (on_delta > app->adaptive_finger_on_delta)
  {
    app->adaptive_finger_on_delta = on_delta;
  }

  if (off_delta > app->adaptive_finger_off_delta)
  {
    app->adaptive_finger_off_delta = off_delta;
  }

  if (app->adaptive_finger_on_delta > MAX30102_FINGER_ON_DELTA_MAX)
  {
    app->adaptive_finger_on_delta = MAX30102_FINGER_ON_DELTA_MAX;
  }

  if (app->adaptive_finger_off_delta > MAX30102_FINGER_OFF_DELTA_MAX)
  {
    app->adaptive_finger_off_delta = MAX30102_FINGER_OFF_DELTA_MAX;
  }
}

/*
 * 更新手指在位状态。
 * 只有在“确认为无手指”时，才允许基线继续缓慢跟踪环境变化。
 */
void app_measurement_update_finger_state(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  app->raw_signal_present = app_is_raw_signal_present(app);

  if (app->finger_present == 0U)
  {
    app->finger_off_confirm_count = 0U;

    if (app->raw_signal_present == 0U)
    {
      app->finger_on_confirm_count = 0U;
      app_track_background_ir(app);
      return;
    }

    if (app->finger_on_confirm_count < 0xFFU)
    {
      app->finger_on_confirm_count++;
    }

    if (app->finger_on_confirm_count >= MAX30102_FINGER_ON_CONFIRM_COUNT)
    {
      app->finger_present = 1U;
      app->finger_on_confirm_count = 0U;
      app->report_due = 1U;
      app->display_refresh_requested = 1U;
    }

    return;
  }

  app->finger_on_confirm_count = 0U;

  if (app->raw_signal_present != 0U)
  {
    app->finger_off_confirm_count = 0U;
    return;
  }

  if (app->finger_off_confirm_count < 0xFFU)
  {
    app->finger_off_confirm_count++;
  }

  if (app->finger_off_confirm_count >= MAX30102_FINGER_OFF_CONFIRM_COUNT)
  {
    app->finger_present = 0U;
    app->finger_off_confirm_count = 0U;
    app->finger_on_confirm_count = 0U;
    app_track_background_ir(app);
    max30102_baseline_seed_tracking(&baseline_data,
                                    app->baseline_ir,
                                    MAX30102_REACQUIRE_NOISE_IR);
    app->baseline_ir = max30102_baseline_get_tracked_ir(&baseline_data);
    app_reset_measurement_outputs(app);
    app_reset_signal_envelope();
    app->report_due = 1U;
    app->display_refresh_requested = 1U;
  }
}

/* 手指在位时推进波形、SpO2 与 BPM 算法窗口。 */
void app_measurement_process(AppState_t *app)
{
  MAX30102_SignalMetrics_t signal_metrics;
  uint8_t raw_bpm_valid = 0U;
  uint8_t raw_bpm_value = 0U;
  uint8_t raw_spo2_valid = 0U;
  uint8_t signal_quality = 0U;

  if (app == NULL)
  {
    return;
  }

  /*
   * Waveform and algorithm windows now follow the confirmed finger state:
   * - no finger: keep the screen quiet instead of drawing ambient noise
   * - finger confirmed: feed waveform + BPM/SpO2 windows together
   */
  if (app->finger_present == 0U)
  {
    return;
  }

  app_display_add_ir_sample(app->ir_value);
  app_display_add_red_sample(app->red_value);
  max30102_spo2_add_sample(&spo2_state, app->red_value, app->ir_value);

  if (max30102_get_signal_metrics(&spo2_state, &signal_metrics) != 0U)
  {
    app->signal_ir_ac_rms = signal_metrics.ir_ac_rms;
    app->signal_red_ac_rms = signal_metrics.red_ac_rms;
    if (max30102_calculate_signal_quality(&spo2_state, &signal_metrics, &signal_quality) != 0U)
    {
      app->signal_quality = signal_quality;
    }
    else
    {
      app->signal_quality = 0U;
    }

    app->signal_ir_pi_x1000 = (signal_metrics.ir_pi_x1000 > 0xFFU) ? 0xFFU :
                              (uint8_t)signal_metrics.ir_pi_x1000;
    app->signal_red_pi_x1000 = (signal_metrics.red_pi_x1000 > 0xFFU) ? 0xFFU :
                               (uint8_t)signal_metrics.red_pi_x1000;
  }
  else
  {
    app->signal_ir_ac_rms = 0U;
    app->signal_red_ac_rms = 0U;
    app->signal_quality = 0U;
    app->signal_ir_pi_x1000 = 0U;
    app->signal_red_pi_x1000 = 0U;
  }

  raw_spo2_valid = max30102_calculate_spo2(&spo2_state, &app->spo2_value);
  if ((raw_spo2_valid != 0U) && (app->signal_quality < APP_SIGNAL_QUALITY_MIN_FOR_SPO2))
  {
    raw_spo2_valid = 0U;
  }

  app->spo2_valid = raw_spo2_valid;

  bpm_update_decimator++;
  if (bpm_update_decimator < APP_BPM_EVALUATE_INTERVAL_SAMPLES)
  {
    return;
  }

  bpm_update_decimator = 0U;
  raw_bpm_valid = max30102_calculate_bpm(&spo2_state, &raw_bpm_value);
  if ((raw_bpm_valid != 0U) && (app->signal_quality < APP_SIGNAL_QUALITY_MIN_FOR_BPM))
  {
    raw_bpm_valid = 0U;
  }

  app_update_bpm_output(app, raw_bpm_valid, raw_bpm_value);
}

/* 把 50 Hz 采样节拍降频成约 10 Hz 的显示/上报节拍。 */
void app_measurement_update_periodic_flags(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  app->refresh_div++;
  if (app->refresh_div < 10U)
  {
    return;
  }

  app->refresh_div = 0U;
  app->report_due = 1U;
  app->display_refresh_requested = 1U;
}

void app_measurement_recover_sensor(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  if (app->sensor_error_streak < APP_SENSOR_RECOVERY_ERROR_COUNT)
  {
    return;
  }

  if (max30102_init() != HAL_OK)
  {
    app->sensor_error_streak = APP_SENSOR_RECOVERY_ERROR_COUNT;
    return;
  }

  app->sensor_error_streak = 0U;
  app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_WAIT;
  app->sensor_recover_count++;
  app_reset_signal_envelope();
  app_reset_measurement_outputs(app);
  max30102_baseline_seed_tracking(&baseline_data,
                                  app->baseline_ir,
                                  MAX30102_REACQUIRE_NOISE_IR);
  app->baseline_ir = max30102_baseline_get_tracked_ir(&baseline_data);
  app->display_refresh_requested = 1U;
  app->report_due = 1U;
}

/* 计算两个 8 位无符号数的绝对差。 */
static uint32_t app_abs_diff_u32(uint32_t lhs, uint32_t rhs)
{
  if (lhs >= rhs)
  {
    return lhs - rhs;
  }

  return rhs - lhs;
}

static uint32_t app_slow_follow_u32(uint32_t current, uint32_t target, uint8_t shift)
{
  uint32_t delta;
  uint32_t step;

  if (current == target)
  {
    return current;
  }

  if (current < target)
  {
    delta = target - current;
    step = delta >> shift;
    if (step == 0U)
    {
      step = 1U;
    }

    return current + step;
  }

  delta = current - target;
  step = delta >> shift;
  if (step == 0U)
  {
    step = 1U;
  }

  return current - step;
}

static uint8_t app_abs_diff_u8(uint8_t lhs, uint8_t rhs)
{
  if (lhs >= rhs)
  {
    return (uint8_t)(lhs - rhs);
  }

  return (uint8_t)(rhs - lhs);
}

static uint8_t app_limit_bpm_step(uint8_t current_bpm, uint8_t target_bpm, uint8_t max_step)
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

/* 3 点中值滤波，用于抑制原始 BPM 的偶发抖动。 */
static uint8_t app_median3_u8(uint8_t a, uint8_t b, uint8_t c)
{
  uint8_t temp;

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

/* 对原始 BPM 先做极轻量的历史平滑，再交给输出稳定器。 */
static uint8_t app_filter_raw_bpm(AppState_t *app, uint8_t raw_bpm_value)
{
  uint8_t first_index;
  uint8_t second_index;

  if (app == NULL)
  {
    return raw_bpm_value;
  }

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

  first_index = app->raw_bpm_history_index;
  second_index = (uint8_t)((app->raw_bpm_history_index + 1U) % 3U);

  return app_median3_u8(app->raw_bpm_history[first_index],
                        app->raw_bpm_history[second_index],
                        app->raw_bpm_history[(uint8_t)((app->raw_bpm_history_index + 2U) % 3U)]);
}

/* 仅在确认为“无手指”时，允许背景 IR 慢速跟踪环境变化。 */
static void app_reset_signal_envelope(void)
{
  (void)memset(&signal_envelope, 0, sizeof(signal_envelope));
}

static void app_update_signal_activity(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  if ((signal_envelope.ir_high == 0U) || (app->ir_value >= signal_envelope.ir_high))
  {
    signal_envelope.ir_high = app->ir_value;
  }
  else
  {
    signal_envelope.ir_high = app_slow_follow_u32(signal_envelope.ir_high, app->ir_value, 4U);
  }

  if ((signal_envelope.ir_low == 0U) || (app->ir_value <= signal_envelope.ir_low))
  {
    signal_envelope.ir_low = app->ir_value;
  }
  else
  {
    signal_envelope.ir_low = app_slow_follow_u32(signal_envelope.ir_low, app->ir_value, 4U);
  }

  if ((signal_envelope.red_high == 0U) || (app->red_value >= signal_envelope.red_high))
  {
    signal_envelope.red_high = app->red_value;
  }
  else
  {
    signal_envelope.red_high = app_slow_follow_u32(signal_envelope.red_high, app->red_value, 4U);
  }

  if ((signal_envelope.red_low == 0U) || (app->red_value <= signal_envelope.red_low))
  {
    signal_envelope.red_low = app->red_value;
  }
  else
  {
    signal_envelope.red_low = app_slow_follow_u32(signal_envelope.red_low, app->red_value, 4U);
  }

  app->ir_signal_delta = app_abs_diff_u32(app->ir_value, app->baseline_ir);
  app->ir_signal_span = (signal_envelope.ir_high >= signal_envelope.ir_low) ?
                        (signal_envelope.ir_high - signal_envelope.ir_low) : 0U;
  app->red_signal_span = (signal_envelope.red_high >= signal_envelope.red_low) ?
                         (signal_envelope.red_high - signal_envelope.red_low) : 0U;
}

static uint32_t app_get_signal_span_threshold(const AppState_t *app)
{
  uint32_t threshold = MAX30102_SIGNAL_SPAN_MIN;
  uint32_t noise_threshold = max30102_baseline_get_noise_ir(&baseline_data) *
                             MAX30102_SIGNAL_SPAN_NOISE_GAIN;

  if (noise_threshold > threshold)
  {
    threshold = noise_threshold;
  }

  if ((app != NULL) && (app->baseline_range_ir != 0U))
  {
    uint32_t startup_range_threshold = app->baseline_range_ir / 4U;
    if (startup_range_threshold > threshold)
    {
      threshold = startup_range_threshold;
    }
  }

  if (threshold > MAX30102_SIGNAL_SPAN_MAX)
  {
    threshold = MAX30102_SIGNAL_SPAN_MAX;
  }

  return threshold;
}

static uint8_t app_is_raw_signal_present(const AppState_t *app)
{
  uint32_t signal_span_threshold;
  uint32_t red_signal_span_threshold;

  if (app == NULL)
  {
    return 0U;
  }

  signal_span_threshold = app_get_signal_span_threshold(app);
  red_signal_span_threshold = (signal_span_threshold * 3U) / 4U;
  if (red_signal_span_threshold == 0U)
  {
    red_signal_span_threshold = 1U;
  }

  if (app->ir_signal_delta >= app->adaptive_finger_on_delta)
  {
    return 1U;
  }

  if (app->ir_signal_span >= signal_span_threshold)
  {
    return 1U;
  }

  if (app->red_signal_span >= red_signal_span_threshold)
  {
    return 1U;
  }

  return 0U;
}

static void app_track_background_ir(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  max30102_baseline_track_background(&baseline_data, app->ir_value);
  app->baseline_ir = max30102_baseline_get_tracked_ir(&baseline_data);
}

/* 手指移开后立即清空测量窗口，避免旧数据残留到下一轮。 */
static void app_reset_measurement_outputs(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  app_display_reset_waveforms();
  max30102_spo2_reset(&spo2_state);
  app->bpm_valid = 0U;
  app->bpm_value = 0U;
  (void)memset(app->raw_bpm_history, 0, sizeof(app->raw_bpm_history));
  app->raw_bpm_history_count = 0U;
  app->raw_bpm_history_index = 0U;
  app->bpm_candidate_value = 0U;
  app->bpm_candidate_count = 0U;
  app->bpm_invalid_hold_count = 0U;
  app->spo2_valid = 0U;
  app->spo2_value = 0U;
  app->raw_signal_present = 0U;
  app->ir_signal_delta = 0U;
  app->ir_signal_span = 0U;
  app->red_signal_span = 0U;
  app->signal_ir_ac_rms = 0U;
  app->signal_red_ac_rms = 0U;
  app->signal_quality = 0U;
  app->signal_ir_pi_x1000 = 0U;
  app->signal_red_pi_x1000 = 0U;
}

/*
 * BPM 输出稳定器：
 * - 起始阶段要求候选值连续命中
 * - 已有稳定值后，只有连续接近的新值才会平滑切换
 * - 大尖峰需要更高确认次数
 */
static void app_update_bpm_output(AppState_t *app, uint8_t raw_bpm_valid, uint8_t raw_bpm_value)
{
  uint8_t diff;
  uint8_t filtered_bpm;
  uint8_t required_confirm_count;
  uint16_t blended_value;

  if (app == NULL)
  {
    return;
  }

  if (raw_bpm_valid != 0U)
  {
    filtered_bpm = app_filter_raw_bpm(app, raw_bpm_value);
    app->bpm_invalid_hold_count = 0U;

    if (app->bpm_valid == 0U)
    {
      if ((app->bpm_candidate_count == 0U) ||
          (app_abs_diff_u8(filtered_bpm, app->bpm_candidate_value) > APP_BPM_CONFIRM_DELTA))
      {
        app->bpm_candidate_value = filtered_bpm;
        app->bpm_candidate_count = 1U;
        return;
      }

      if (app->bpm_candidate_count < 0xFFU)
      {
        app->bpm_candidate_count++;
      }

      blended_value = ((uint16_t)app->bpm_candidate_value + (uint16_t)filtered_bpm + 1U) / 2U;
      app->bpm_candidate_value = (uint8_t)blended_value;

      if (app->bpm_candidate_count >= APP_BPM_START_CONFIRM_COUNT)
      {
        app->bpm_valid = 1U;
        app->bpm_value = app->bpm_candidate_value;
        app->bpm_candidate_count = 0U;
      }

      return;
    }

    diff = app_abs_diff_u8(filtered_bpm, app->bpm_value);
    if (diff <= APP_BPM_CONFIRM_DELTA)
    {
      blended_value = (((uint16_t)app->bpm_value * 7U) + (uint16_t)filtered_bpm + 4U) / 8U;
      app->bpm_value = app_limit_bpm_step(app->bpm_value,
                                          (uint8_t)blended_value,
                                          APP_BPM_MAX_STEP_PER_UPDATE);
      app->bpm_candidate_value = app->bpm_value;
      app->bpm_candidate_count = 0U;
      return;
    }

    required_confirm_count = APP_BPM_SWITCH_CONFIRM_COUNT;
    if (diff > APP_BPM_SPIKE_DELTA)
    {
      required_confirm_count = APP_BPM_SPIKE_CONFIRM_COUNT;
    }

    if ((app->bpm_candidate_count == 0U) ||
        (app_abs_diff_u8(filtered_bpm, app->bpm_candidate_value) > APP_BPM_SWITCH_DELTA))
    {
      app->bpm_candidate_value = filtered_bpm;
      app->bpm_candidate_count = 1U;
      return;
    }

    if (app->bpm_candidate_count < 0xFFU)
    {
      app->bpm_candidate_count++;
    }

    blended_value = ((uint16_t)app->bpm_candidate_value + (uint16_t)filtered_bpm + 1U) / 2U;
    app->bpm_candidate_value = (uint8_t)blended_value;

    if (app->bpm_candidate_count >= required_confirm_count)
    {
      if (diff > APP_BPM_SPIKE_DELTA)
      {
        blended_value = (((uint16_t)app->bpm_value * 7U) + (uint16_t)app->bpm_candidate_value + 4U) / 8U;
      }
      else
      {
        blended_value = (((uint16_t)app->bpm_value * 5U) +
                         ((uint16_t)app->bpm_candidate_value * 3U) + 4U) / 8U;
      }

      app->bpm_value = app_limit_bpm_step(app->bpm_value,
                                          (uint8_t)blended_value,
                                          APP_BPM_MAX_STEP_PER_UPDATE);
      app->bpm_candidate_value = app->bpm_value;
      app->bpm_candidate_count = 0U;
    }

    return;
  }

  if (app->bpm_valid != 0U)
  {
    if (app->bpm_invalid_hold_count < APP_BPM_INVALID_HOLD_TICKS)
    {
      app->bpm_invalid_hold_count++;
      return;
    }
  }

  app->bpm_valid = 0U;
  app->bpm_value = 0U;
  bpm_update_decimator = 0U;
  (void)memset(app->raw_bpm_history, 0, sizeof(app->raw_bpm_history));
  app->raw_bpm_history_count = 0U;
  app->raw_bpm_history_index = 0U;
  app->bpm_candidate_value = 0U;
  app->bpm_candidate_count = 0U;
  app->bpm_invalid_hold_count = 0U;
}
