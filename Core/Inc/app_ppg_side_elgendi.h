/**
  ******************************************************************************
  * @file    app_ppg_side_elgendi.h
  * @brief   Elgendi 风格 PPG 侧路检测器，用于 A/B 诊断。
  *
  * 该模块只做 side-channel 诊断：不替换生产用流式 PPG 脉搏检测器，
  * 也不直接写 BPM/SpO2/PTT 输出。它把同一条滤波后 IR 波形送入
  * 双移动平均检测器，再统计与主检测器峰值的匹配、漏检和误检。
  ******************************************************************************
  */
#ifndef __APP_PPG_SIDE_ELGENDI_H__
#define __APP_PPG_SIDE_ELGENDI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"

#ifndef APP_PPG_SIDE_ELGENDI
/* 编译期开关：0 = 完全编译为空实现，1 = 开启侧路 A/B 统计。 */
#define APP_PPG_SIDE_ELGENDI 1U
#endif

/**
 *******************************************************************************
 * @brief  重置侧路检测器内部状态和 AppState 诊断计数。
 * @param  app AppState 指针（可为 NULL；NULL 时只清内部状态）。
 * @note   手指离开、接触稳定窗口结束、FIFO 溢出或测量重置时调用。
 *******************************************************************************
 */
void app_ppg_side_elgendi_reset(AppState_t *app);

/**
 *******************************************************************************
 * @brief  推进 Elgendi 风格侧路检测器一个 PPG 样本。
 * @param  app             AppState 指针。
 * @param  filtered_sample 带通滤波后的 IR AC 样本。
 * @param  total_samples   MAX30102 自启动以来的样本计数。
 * @note   只更新 ppg_side_* 诊断字段；手指未就位或接触稳定期会复位。
 *******************************************************************************
 */
void app_ppg_side_elgendi_update(AppState_t *app,
                                 int32_t filtered_sample,
                                 uint32_t current_sample);

/**
 *******************************************************************************
 * @brief  通知侧路检测器：主 PPG 检测器已接受一个峰。
 * @param  app         AppState 指针。
 * @param  peak_sample 主检测器接受峰的样本编号。
 * @param  ibi_ms      主检测器输出的 IBI（当前仅保留接口语义）。
 * @note   用于和侧路峰做 +/-150 ms 匹配统计。
 *******************************************************************************
 */
void app_ppg_side_elgendi_note_current_peak(AppState_t *app,
                                            uint32_t peak_sample,
                                            uint16_t ibi_ms);

#ifdef __cplusplus
}
#endif

#endif /* __APP_PPG_SIDE_ELGENDI_H__ */
