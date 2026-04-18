#ifndef __APP_PROTOCOL_H__
#define __APP_PROTOCOL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"

/* 初始化协议模块内部的串口接收缓冲状态。 */
void app_protocol_init(void);
/* 读取 RTC 当前快照，写回应用状态。 */
void app_protocol_update_rtc_snapshot(AppState_t *app);
/* 轮询串口命令，目前主要处理 TIME/SETTIME。 */
void app_protocol_poll_uart_commands(AppState_t *app);
/* 组包并通过串口上报当前测量数据。 */
void app_protocol_send_sensor_report(AppState_t *app);

#ifdef __cplusplus
}
#endif

#endif /* __APP_PROTOCOL_H__ */
