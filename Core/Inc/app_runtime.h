/**
  ******************************************************************************
  * @file    app_runtime.h
  * @brief   应用运行时公共服务接口。
  *
  * 这些函数供 main.c / freertos.c 调用，用于把“到点就做”的运行时服务
  * 从任务循环里拆出来：USART 上报、OLED 刷新、SD 状态快照、EEPROM 统计。
  ******************************************************************************
  */

#ifndef __APP_RUNTIME_H__
#define __APP_RUNTIME_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"

/** @brief 若 report_due 置位，发送当前测量 M 帧并清除节拍标志。 */
void    app_runtime_send_report_if_due(AppState_t *app);

/** @brief 若 display_refresh_requested 置位，基于 AppState 快照刷新 OLED。 */
void    app_runtime_refresh_display_if_needed(AppState_t *app);

/** @brief 把 SD 日志模块内部状态同步到 AppState，供显示和串口上报。 */
void    app_runtime_update_sd_log_status(AppState_t *app);

/** @brief 周期性把运行时小时数和错误/恢复计数同步到 EEPROM。 */
void    app_runtime_service_eeprom_stats(const AppState_t *app);

/** @brief 判断当前是否处于可服务普通 SD 后台写入的严格安全窗口。 */
uint8_t app_runtime_sd_service_safe(const AppState_t *app);

/** @brief 判断停止测量后的 SD 延迟排空是否可推进（不要求传感器样本新鲜）。 */
uint8_t app_runtime_sd_flush_safe(const AppState_t *app);

#ifdef __cplusplus
}
#endif

#endif /* __APP_RUNTIME_H__ */
