/**
  ******************************************************************************
  * @file    app_rtos.h
  * @brief   RTOS 任务间共享状态绑定、MAXtask 通知和 I2C 互斥接口。
  *
  * 该层只封装 FreeRTOS 同步原语，避免算法/显示/日志模块直接依赖任务句柄。
  ******************************************************************************
  */

#ifndef __APP_RTOS_H__
#define __APP_RTOS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_state.h"

/** @brief 在调度器启动前绑定全局 AppState 指针，供各任务共享访问。 */
void app_rtos_bind_state(AppState_t *state);
/** @brief 从 TIM6 ISR 唤醒 MAXtask，使其尽快消费 ECG DMA 并读取 PPG FIFO。 */
void app_rtos_notify_max_from_isr(void);
/** @brief 标记 RTOS 基础设施已就绪，允许 ISR 通知任务。 */
void app_rtos_mark_ready(void);
/** @brief 获取已绑定的 AppState 指针。 */
AppState_t *app_rtos_get_state(void);
/**
 * @brief  获取共享 I2C1 总线互斥锁。
 * @param  timeout_ms 最大等待时间，单位：ms；0 表示不等待。
 * @return 获得互斥量返回 1，超时返回 0；互斥量创建前的单线程启动阶段返回 1。
 */
uint8_t app_rtos_i2c_acquire(uint32_t timeout_ms);
/** @brief 释放共享 I2C1 总线互斥锁；互斥量尚未创建时为空操作。 */
void app_rtos_i2c_release(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_RTOS_H__ */
