#ifndef __APP_DISPLAY_H__
#define __APP_DISPLAY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"

/* 初始化显示相关状态，例如默认页面与页面切换按键。 */
void app_display_init_state(AppState_t *app);
/* 轮询显示相关按键：上下翻页和亮度循环。 */
void app_display_handle_buttons(AppState_t *app);
/* 清空波形缓冲区，常用于“无手指”或重新开始测量时。 */
void app_display_reset_waveforms(void);
/* 向 IR 波形缓冲区压入一个带通滤波后的 PPG 样本。 */
void app_display_add_ir_sample(int32_t filtered_value);
/* 向 RED 波形缓冲区压入一个带通滤波后的 PPG 样本。 */
void app_display_add_red_sample(int32_t filtered_value);
/* 在脉搏波形上标记最近检测到的 IR 脉冲峰值。 */
void app_display_add_ir_pulse_marker(void);
/* 设置 ECG 波形独立最小 AGC scale，避免低质量信号时噪声被放大。
 * 上电默认 200，PPG 保持 32。运行时可根据 ecg_signal_quality 动态调高。 */
void app_display_set_ecg_min_scale(uint32_t min_scale);
/* 仅清除 ECG 波形缓冲区，PPG 接触变化不得影响 ECG 波形。 */
void app_display_reset_ecg_waveform(void);
/* 将一个滤波后的 ECG 样本压入 ECG 波形缓冲区。 */
void app_display_add_ecg_sample(int32_t filtered_value);
/* 在 ECG 波形上标记最近检测到的 ECG R 峰。 */
void app_display_add_ecg_r_peak_marker(void);
/* 根据当前页面绘制测量结果页。 */
void app_display_measurement_page(const AppState_t *app);
/* 绘制状态页，常用于开机、自检和基线采集阶段。 */
void app_display_status_page(const AppState_t *app, const char *status_line_1, const char *status_line_2);

#ifdef __cplusplus
}
#endif

#endif /* __APP_DISPLAY_H__ */

