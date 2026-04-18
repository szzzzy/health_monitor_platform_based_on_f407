#ifndef __MAX30102_H__
#define __MAX30102_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* MAX30102 7 位地址为 0x57，HAL 使用左移后的 8 位地址 */
#define MAX30102_I2C_ADDR                       (0x57U << 1)
#define MAX30102_I2C_TIMEOUT_MS                 100U
#define MAX30102_PART_ID_VALUE                  0x15U

/* 常用寄存器 */
#define MAX30102_REG_INTR_STATUS_1              0x00U
#define MAX30102_REG_INTR_STATUS_2              0x01U
#define MAX30102_REG_INTR_ENABLE_1              0x02U
#define MAX30102_REG_INTR_ENABLE_2              0x03U
#define MAX30102_REG_FIFO_WR_PTR                0x04U
#define MAX30102_REG_OVF_COUNTER                0x05U
#define MAX30102_REG_FIFO_RD_PTR                0x06U
#define MAX30102_REG_FIFO_DATA                  0x07U
#define MAX30102_REG_FIFO_CONFIG                0x08U
#define MAX30102_REG_MODE_CONFIG                0x09U
#define MAX30102_REG_SPO2_CONFIG                0x0AU
#define MAX30102_REG_LED1_PA                    0x0CU
#define MAX30102_REG_LED2_PA                    0x0DU
#define MAX30102_REG_MULTI_LED_CTRL1            0x11U
#define MAX30102_REG_MULTI_LED_CTRL2            0x12U
#define MAX30102_REG_TEMP_INTR                  0x1FU
#define MAX30102_REG_TEMP_FRAC                  0x20U
#define MAX30102_REG_TEMP_CONFIG                0x21U
#define MAX30102_REG_REV_ID                     0xFEU
#define MAX30102_REG_PART_ID                    0xFFU

/* MODE_CONFIG 常用位 */
#define MAX30102_MODE_SHUTDOWN                  0x80U
#define MAX30102_MODE_RESET                     0x40U
#define MAX30102_MODE_HEART_RATE                0x02U
#define MAX30102_MODE_SPO2                      0x03U
#define MAX30102_MODE_MULTI_LED                 0x07U

/* SpO2 模式下单个样本包含 RED + IR，共 6 字节 */
#define MAX30102_FIFO_BYTES_PER_LED             3U
#define MAX30102_FIFO_BYTES_PER_SAMPLE_SPO2     6U

/* 默认配置先用于跑通流程，后续再按手册调整 */
#define MAX30102_DEFAULT_FIFO_CONFIG            0x0FU
#define MAX30102_DEFAULT_MODE_CONFIG            MAX30102_MODE_SPO2
/*
 * SPO2_CONFIG:
 * - ADC range: 4096 nA
 * - Sample rate: 50 sps
 * - LED pulse width: 411 us / 18-bit
 *
 * The firmware drains every pending FIFO sample during each 20 ms service pass,
 * so keep the sensor at 50 Hz to match the algorithm's fixed sample-rate math.
 */
#define MAX30102_DEFAULT_SPO2_CONFIG            0x23U

/* Slightly raise RED/IR drive current to improve SNR on weak reflective signals. */
#define MAX30102_DEFAULT_LED1_PA                0x30U
#define MAX30102_DEFAULT_LED2_PA                0x30U

/* 用于统计背景 IR 基线 */
typedef struct
{
  uint32_t ir_sum;
  uint32_t ir_min;
  uint32_t ir_max;
  uint32_t tracked_ir;
  uint32_t noise_ir;
  uint16_t sample_count;
} MAX30102_Baseline_t;

/*
 * 基础版 SpO2 估算使用固定长度滑动窗口：
 * - WINDOW_SIZE 表示最多保留多少个最近样本
 * - MIN_VALID_SAMPLES 表示至少累计多少个样本后才允许开始计算
 */
#define MAX30102_SPO2_WINDOW_SIZE             200U
#define MAX30102_SPO2_MIN_VALID_SAMPLES       30U
#define MAX30102_BPM_MIN_VALID_SAMPLES        40U

/*
 * 算法默认按 50 Hz 采样设计。
 * 这个值应与 main.c 中的采样周期保持一致，否则 BPM 结果会按比例偏差。
 */
#define MAX30102_ALGO_SAMPLE_RATE_HZ          50U

/* 用于保存最近一段 RED / IR 样本，给 SpO2 算法计算 AC/DC 比值。 */
typedef struct
{
  uint32_t red_samples[MAX30102_SPO2_WINDOW_SIZE];
  uint32_t ir_samples[MAX30102_SPO2_WINDOW_SIZE];
  int32_t red_filtered_samples[MAX30102_SPO2_WINDOW_SIZE];
  int32_t ir_filtered_samples[MAX30102_SPO2_WINDOW_SIZE];
  uint16_t write_index;
  uint16_t sample_count;
  uint32_t red_dc_estimate;
  uint32_t ir_dc_estimate;
  int32_t red_filter_state;
  int32_t ir_filter_state;
  uint64_t red_sum;
  uint64_t ir_sum;
  uint64_t red_square_sum;
  uint64_t ir_square_sum;
  uint64_t red_filtered_square_sum;
  uint64_t ir_filtered_square_sum;
} MAX30102_SpO2_t;

typedef struct
{
  uint32_t red_dc;
  uint32_t ir_dc;
  uint32_t red_ac_rms;
  uint32_t ir_ac_rms;
  uint16_t red_pi_x1000;
  uint16_t ir_pi_x1000;
} MAX30102_SignalMetrics_t;

typedef struct
{
  uint8_t overflow_count;
  uint8_t write_ptr;
  uint8_t read_ptr;
  uint8_t available_samples;
} MAX30102_FifoDebug_t;

HAL_StatusTypeDef max30102_init(void);
HAL_StatusTypeDef max30102_write_reg(uint8_t reg_addr, uint8_t data);
HAL_StatusTypeDef max30102_read_reg(uint8_t reg_addr, uint8_t *data);
HAL_StatusTypeDef max30102_read_fifo(uint8_t *fifo_data, uint16_t data_len);
const MAX30102_FifoDebug_t *max30102_get_fifo_debug(void);

/* 把 6 字节 FIFO 数据解析成 RED / IR 原始值 */
void max30102_parse_spo2_sample(const uint8_t *fifo_data,
                                uint32_t *red,
                                uint32_t *ir);

void max30102_baseline_reset(MAX30102_Baseline_t *baseline);
void max30102_baseline_add_ir(MAX30102_Baseline_t *baseline, uint32_t ir_value);
uint8_t max30102_baseline_is_ready(const MAX30102_Baseline_t *baseline, uint16_t target_samples);
uint32_t max30102_baseline_get_average_ir(const MAX30102_Baseline_t *baseline);
uint32_t max30102_baseline_get_range_ir(const MAX30102_Baseline_t *baseline);
uint8_t max30102_baseline_is_stable(const MAX30102_Baseline_t *baseline, uint32_t stable_range);
void max30102_baseline_seed_tracking(MAX30102_Baseline_t *baseline,
                                     uint32_t baseline_ir,
                                     uint32_t noise_ir);
void max30102_baseline_track_background(MAX30102_Baseline_t *baseline, uint32_t ir_value);
uint32_t max30102_baseline_get_tracked_ir(const MAX30102_Baseline_t *baseline);
uint32_t max30102_baseline_get_noise_ir(const MAX30102_Baseline_t *baseline);
uint8_t max30102_detect_finger(uint32_t ir_value,
                               uint32_t baseline_ir,
                               uint8_t current_state,
                               uint32_t finger_on_delta,
                               uint32_t finger_off_delta);

/* 重置 SpO2 滑动窗口。通常在“无手指”或重新开始测量时调用。 */
void max30102_spo2_reset(MAX30102_SpO2_t *spo2_state);

/* 向窗口追加一个新的 RED / IR 样本。 */
void max30102_spo2_add_sample(MAX30102_SpO2_t *spo2_state, uint32_t red_value, uint32_t ir_value);

/* 根据当前窗口内样本估算 SpO2，返回 1 表示结果有效，0 表示暂时无有效结果。 */
uint8_t max30102_calculate_spo2(const MAX30102_SpO2_t *spo2_state, uint8_t *spo2_value);

/* 根据当前窗口内 IR 波形估算 BPM，返回 1 表示结果有效，0 表示暂时无有效结果。 */
uint8_t max30102_calculate_bpm(const MAX30102_SpO2_t *spo2_state, uint8_t *bpm_value);
uint8_t max30102_get_signal_metrics(const MAX30102_SpO2_t *spo2_state, MAX30102_SignalMetrics_t *metrics);
uint8_t max30102_calculate_signal_quality(const MAX30102_SpO2_t *spo2_state,
                                          const MAX30102_SignalMetrics_t *metrics,
                                          uint8_t *signal_quality);

#ifdef __cplusplus
}
#endif

#endif /* __MAX30102_H__ */

