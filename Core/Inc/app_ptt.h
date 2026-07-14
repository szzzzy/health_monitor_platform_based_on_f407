/**
  ******************************************************************************
  * @file    app_ptt.h
  * @brief   PTT (脉搏传导时间) — ECG R 峰 → PPG 脉搏波峰时间差
  *
  * 本模块提供 ECG 与 PPG 两条采集链之间的工程时间差指标，不作血压、血管
  * 状态或其他临床结论。候选需同时满足双侧信号质量、时间新鲜度、
  * 80–350 ms 可信范围及短历史突跳门控。
  *
  * 统一时间基准：ECG R 峰时间戳 (HAL_GetTick ms) 与 PPG 波峰时间戳
  * (通过样本序号反推到 ms) 使用同一 HAL Tick 时钟。
  ******************************************************************************
  */
#ifndef __APP_PTT_H__
#define __APP_PTT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"

/** @brief 清空 ECG/PTT 环形历史及 AppState 中的 PTT 输出。 */
void app_ptt_reset(AppState_t *app);
/** @brief 把一个非零 ECG R 峰绝对时间戳写入 4 点匹配历史。 */
void app_ptt_add_ecg_peak(uint32_t r_peak_ms);
/**
 * @brief 由 PPG 峰样本序号回推时间戳，匹配前驱 ECG R 峰并更新 PTT。
 * @note  任一门控失败时 ptt_valid 置 0，但上次 ptt_ms 保留供带无效标记显示。
 */
void app_ptt_update_from_ppg_peak(AppState_t *app,
                                  uint32_t ppg_peak_sample,
                                  uint32_t ppg_total_samples);

#ifdef __cplusplus
}
#endif

#endif /* __APP_PTT_H__ */
