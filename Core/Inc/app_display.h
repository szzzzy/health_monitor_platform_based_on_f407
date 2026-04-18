#ifndef __APP_DISPLAY_H__
#define __APP_DISPLAY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"

/* 初始化显示相关状态，例如默认页面与页面切换按键。 */
void app_display_init_state(AppState_t *app);
/* Poll display-related buttons: page up/down and brightness cycle. */
void app_display_handle_buttons(AppState_t *app);
/* 清空波形缓冲区，常用于“无手指”或重新开始测量时。 */
void app_display_reset_waveforms(void);
/* 向 IR 波形缓冲区压入一个新样本。 */
void app_display_add_ir_sample(uint32_t raw_value);
/* 向 RED 波形缓冲区压入一个新样本。 */
void app_display_add_red_sample(uint32_t raw_value);
/* 根据当前页面绘制测量结果页。 */
void app_display_measurement_page(const AppState_t *app);
/* 绘制状态页，常用于开机、自检和基线采集阶段。 */
void app_display_status_page(const AppState_t *app, const char *status_line_1, const char *status_line_2);

#ifdef __cplusplus
}
#endif

#endif /* __APP_DISPLAY_H__ */

