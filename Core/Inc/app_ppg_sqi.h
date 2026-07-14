/**
  ******************************************************************************
  * @file    app_ppg_sqi.h
  * @brief   PPG SQI 实验门控 / Experimental PPG signal-quality gate
  *
  * 基于本项目已有的 AC RMS、PI、RED/IR 平衡、运动状态、接触稳定状态和
  * beat-to-beat 稳定性，生成一个保守的 PPG 质量侧评分。模块不替换现有
  * PPG 峰值检测器，只用于降低置信度、清除 valid 标志和输出调试计数。
  *
  * This module builds a conservative side SQI score from existing low-cost PPG
  * features. It does not replace the production pulse detector; it only gates
  * confidence/valid flags and exposes rejection counters for log validation.
  ******************************************************************************
  */
#ifndef __APP_PPG_SQI_H__
#define __APP_PPG_SQI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"
#include "max30102_spo2.h"

/* 1 = 启用 SQI 侧评分逻辑；0 = 编译为空实现。 */
#ifndef APP_PPG_SQI_EXPERIMENTAL
#define APP_PPG_SQI_EXPERIMENTAL 1U
#endif

/* 1 = SQI 会参与 HR/SpO2/PTT valid 门控；0 = 仅统计调试字段。 */
#ifndef APP_PPG_SQI_GATE_OUTPUTS
#define APP_PPG_SQI_GATE_OUTPUTS 1U
#endif

/* SQI 位标志：低灌注、运动扰动、RED/IR 失衡、接触过渡、逐拍不稳定。 */
#define APP_PPG_SQI_FLAG_LOW_PERFUSION   0x01U
#define APP_PPG_SQI_FLAG_MOTION          0x02U
#define APP_PPG_SQI_FLAG_BALANCE         0x04U
#define APP_PPG_SQI_FLAG_TRANSITION      0x08U
#define APP_PPG_SQI_FLAG_BEAT_UNSTABLE   0x10U

/* 重置 SQI 内部自适应基线和 AppState 调试计数。 */
void app_ppg_sqi_reset(AppState_t *app);

/* 每个 PPG 质量窗口调用一次，更新 SQI score/flags 和自适应局部基线。 */
void app_ppg_sqi_update_window(AppState_t *app,
                               const MAX30102_SignalMetrics_t *metrics,
                               uint8_t raw_quality);

/* 将 SQI 侧评分向下钳制到 app->signal_quality，永不提高原始质量分。 */
void app_ppg_sqi_apply_quality_gate(AppState_t *app);

/* 已接受 beat 后调用，用于维护 IBI/幅度稳定性历史。 */
void app_ppg_sqi_note_accepted_beat(AppState_t *app,
                                    uint16_t ibi_ms,
                                    uint32_t beat_amplitude);

/* PPG 峰值检测器拒绝候选时调用，累计 IBI/幅度拒绝原因。 */
void app_ppg_sqi_note_ibi_reject(AppState_t *app);
void app_ppg_sqi_note_amp_reject(AppState_t *app);

/* 各输出路径使用的保守门控查询。 */
uint8_t app_ppg_sqi_allows_hr(const AppState_t *app);
uint8_t app_ppg_sqi_allows_spo2(const AppState_t *app);
uint8_t app_ppg_sqi_allows_ptt(const AppState_t *app);

#ifdef __cplusplus
}
#endif

#endif /* __APP_PPG_SQI_H__ */
