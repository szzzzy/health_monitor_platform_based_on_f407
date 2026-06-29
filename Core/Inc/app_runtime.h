#ifndef __APP_RUNTIME_H__
#define __APP_RUNTIME_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"

/*
 * 运行时公共函数，供 RTOS 任务和 boot 阶段共用。
 * 避免 main.c 和 freertos.c 各自维护等价静态副本。
 */
void    app_runtime_send_report_if_due(AppState_t *app);
void    app_runtime_refresh_display_if_needed(AppState_t *app);
void    app_runtime_update_sd_log_status(AppState_t *app);
void    app_runtime_service_eeprom_stats(const AppState_t *app);
uint8_t app_runtime_sd_service_safe(const AppState_t *app);

#ifdef __cplusplus
}
#endif

#endif /* __APP_RUNTIME_H__ */
