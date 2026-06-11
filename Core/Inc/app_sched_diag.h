#ifndef __APP_SCHED_DIAG_H__
#define __APP_SCHED_DIAG_H__

#include <stdint.h>

/* 极轻量 getter — 返回 main.c 中 static volatile tim6_isr_count。
 * 无 HAL、无 RTOS、无锁、无计算。仅 uint32_t 读取。 */
uint32_t APP_TIM6_GetIsrCount(void);

#endif /* __APP_SCHED_DIAG_H__ */
