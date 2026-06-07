#ifndef __APP_MEASUREMENT_H__
#define __APP_MEASUREMENT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"

/* 上电后先采集 5 秒“无手指”背景 IR，用于建立环境基线。 */
#define APP_MEASUREMENT_BASELINE_TIME_MS      5000U
#define APP_MEASUREMENT_BASELINE_SAMPLES      (APP_MEASUREMENT_BASELINE_TIME_MS / APP_SAMPLE_PERIOD_MS)
/* 基线采集完成后，用这个范围判断背景是否足够稳定。 */
#define APP_MEASUREMENT_BASELINE_STABLE_RANGE 12000UL

typedef enum
{
  APP_MEASUREMENT_READ_WAIT = 0U,
  APP_MEASUREMENT_READ_OK,
  APP_MEASUREMENT_READ_ERROR
} AppMeasurementReadStatus_t;

/*
 * 测量模块职责边界：
 * - app_measurement: 流程编排、传感器读取、门控、输出状态协调。
 * - app_hrv:         IBI 接受、32 拍 ring buffer、SDNN/RMSSD/SD1/SD2、rhythm_irregular。
 * - app_rr:          RIAV amplitude buffer、RR 估计、SQ>=25 门控。
 * - app_spo2_filter: SpO2 EMA、invalid/低SQ/motion 时不推进平滑。
 * - app_motion:      PPG-only motion score、AC RMS baseline、迟滞状态。
 */

/* 初始化测量模块涉及的阈值等应用状态。 */
void app_measurement_init_state(AppState_t *app);
/* 清空基线、算法窗口、波形缓存与所有高级指标，回到”待测量”状态。
 * 区别于测量输出重置（手指离开）：runtime reset 会清 HRV/RR 历史。 */
void app_measurement_reset_runtime(void);
/* 在基线采集阶段读一个样本并更新背景统计。 */
uint8_t app_measurement_collect_baseline_sample(AppState_t *app);
/* 查询基线采集是否已经达到目标样本数。 */
uint8_t app_measurement_baseline_ready(void);
/* 返回基线采集进度百分比，用于状态页显示。 */
uint16_t app_measurement_get_baseline_progress_percent(void);
/* 获取基线平均 IR。 */
uint32_t app_measurement_get_baseline_average(void);
/* 获取基线采样期间的 IR 波动范围。 */
uint32_t app_measurement_get_baseline_range(void);
/* 获取当前被后台跟踪后的 IR 基线。 */
uint32_t app_measurement_get_tracked_baseline(void);
/* 用采集到的基线与噪声初值，给后台跟踪器做种子初始化。 */
void app_measurement_seed_baseline_tracking(uint32_t baseline_ir, uint32_t noise_ir);
/* 判断当前基线是否处于“足够稳定”的范围。 */
uint8_t app_measurement_baseline_is_stable(void);
/* 读取一个 RED/IR 样本，并同步刷新基线快照。 */
AppMeasurementReadStatus_t app_measurement_read_sensor_sample(AppState_t *app);
/* 根据背景噪声动态调整手指检测阈值。 */
void app_measurement_update_adaptive_thresholds(AppState_t *app);
/* 根据当前样本更新“有手指/无手指”状态。 */
void app_measurement_update_finger_state(AppState_t *app);
/* 在手指就位时推进 BPM/SpO2 与波形处理。 */
void app_measurement_process(AppState_t *app);
/* 统一生成较低频率的“上报/刷新显示”节拍。 */
void app_measurement_update_periodic_flags(AppState_t *app);
void app_measurement_recover_sensor(AppState_t *app);

/* 传感器无新样本看门狗超时 (ms)，供 main.c 安全窗口引用 */
#define APP_SENSOR_STALE_WARN_MS          1000U

/* 传感器健康状态：供 OLED / UART 上报显示当前 MAX30102 链路状态 */
typedef enum
{
  SENSOR_HEALTH_OK = 0U,
  SENSOR_HEALTH_STALE,
  SENSOR_HEALTH_RECOVERING,
  SENSOR_HEALTH_I2C_ERR,
  SENSOR_HEALTH_INIT_FAIL,
  SENSOR_HEALTH_FIFO_CLEAR_FAIL
} AppSensorHealth_t;

/* MAX30102 无新样本看门狗 — 当 sensor_last_sample_tick 长时间未更新时，
 * 强制执行 I2C 总线恢复 + max30102_init()。
 * 在主循环每轮和 baseline 采集循环中均需调用。 */
void app_measurement_service_sensor_watchdog(AppState_t *app);

/* 批量 FIFO drain：一次 I2C burst 读所有可用样本并送入测量管道。
 * 返回处理的样本数 (0 = 无数据/overflow/I2C 错误)。 */
uint8_t app_measurement_drain_fifo_batch(AppState_t *app);

/* OLED 刷新门控：返回 1 时应跳过 OLED 刷新（不清 display_refresh_requested）。 */
uint8_t app_measurement_should_skip_display(const AppState_t *app);

#ifdef __cplusplus
}
#endif

#endif /* __APP_MEASUREMENT_H__ */
