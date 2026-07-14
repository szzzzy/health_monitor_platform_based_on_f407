/**
  ******************************************************************************
  * @file    app_sched_diag.h
  * @brief   调度节拍诊断的轻量读取接口。
  ******************************************************************************
  */

#ifndef __APP_SCHED_DIAG_H__
#define __APP_SCHED_DIAG_H__

#include <stdint.h>

/* 极轻量读取函数：返回 TIM6 100 Hz 周期回调的累计次数。
 * 无 HAL 调用、无 RTOS 原语、无锁，仅执行一次 uint32_t 读取。 */
uint32_t APP_TIM6_GetIsrCount(void);

#endif /* __APP_SCHED_DIAG_H__ */
