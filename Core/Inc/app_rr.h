/*
 * RR 模块——呼吸率估计（RIAV：呼吸诱导幅度变化）。
 *
 * 特点（比 HRV 更慢、更严格）：
 * - SQ 门控 >=25（与 BPM 一致，避免弱但稳定的 PPG 被过早拒绝）。
 * - 要求 >=8 个心搏、>=6 秒时间窗口。
 * - 要求脉搏幅度有明显调制（modulation_span >= amplitude_mean/25 且 >=4）。
 * - 输出范围 8–30 次/分钟。
 *
 * 历史管理：
 * - app_rr_reset(): 清零振幅缓冲与输出（手指离开/测量重置）。
 * - SQ 不足/运动时只将有效标志置 0，不清缓冲。
 */
#ifndef __APP_RR_H__
#define __APP_RR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"

#define APP_RR_SIGNAL_QUALITY_MIN 25U

/* 清零振幅历史与输出值——手指离开/测量重置时调用。 */
void app_rr_reset(AppState_t *app);
/* 将一次心搏的峰值位置与幅度压入 RIAV 环形缓冲。 */
void app_rr_add_beat(uint32_t peak_sample, uint32_t amplitude);
/* 运行 RIAV 包络峰检测并更新 rr_bpm。满足所有门控条件才置有效标志。 */
void app_rr_update_output(AppState_t *app);

#ifdef __cplusplus
}
#endif

#endif /* __APP_RR_H__ */
