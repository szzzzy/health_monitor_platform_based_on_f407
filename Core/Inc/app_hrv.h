/*
 * HRV 模块——时域与 Poincare 短窗口指标。
 *
 * 历史管理策略：
 * - app_hrv_reset():     清零 IBI ring buffer 与所有输出（手指离开/测量重置）。
 * - app_hrv_invalidate_outputs(): 只标 valid=0，保留数值（信号短暂中断）。
 * - Motion artifact 期间进 invalidate，不清 buffer。
 *
 * IBI 接受规则：
 * - 绝对范围：300–2000 ms。
 * - 相对跳变：相对已有 >=4 个 IBI 均值的偏差 ≤50%。
 *
 * 输出门控：
 * - hrv_valid=1 要求 >=3 个已接受的 IBI；>=4 个后再启用跳变过滤。
 * - SD1 = RMSSD / sqrt(2)   (Poincare 短轴)。
 * - SD2 = sqrt(2*SDNN^2 - SD1^2)  (Poincare 长轴)。
 * - SD1/SD2 x100 为短/长期变异性比值。
 * - rhythm_irregular: 短窗口提示（RMSSD>=120ms 且 SD1/SD2>=0.70），非诊断分类。
 *
 * - Frequency HRV is a short-window LF/HF estimate from 32 beats.
 *   It is not diagnostic and not standard 5-minute frequency HRV.
 * - VLF is not estimated because the 32-beat window is too short.
 */
#ifndef __APP_HRV_H__
#define __APP_HRV_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"

#define APP_HRV_IBI_MIN_MS 300U
#define APP_HRV_IBI_MAX_MS 2000U

/* 清零所有 IBI 历史和输出值——手指离开/测量重置时调用。 */
void app_hrv_reset(AppState_t *app);
/* 只标 valid=0 保留旧值——信号短暂中断/motion 时调用。不清 ring buffer。 */
void app_hrv_invalidate_outputs(AppState_t *app);
/* 去重检查：同采样编号不重复触发。返回 1 表示新峰。 */
uint8_t app_hrv_mark_peak_seen(uint32_t peak_sample);
/* 陈旧性检查：上次峰距今是否超过 stale_samples。返回 1 表示陈旧。 */
uint8_t app_hrv_is_peak_stale(uint32_t current_sample, uint32_t stale_samples);
/* IBI 接受 + 压入 ring buffer + 触发输出更新。返回 1 表示被接受。 */
uint8_t app_hrv_add_ibi(AppState_t *app, uint16_t ibi_ms);

#ifdef __cplusplus
}
#endif

#endif /* __APP_HRV_H__ */
