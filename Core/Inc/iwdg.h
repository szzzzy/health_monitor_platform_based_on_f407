/**
  ******************************************************************************
  * @file    iwdg.h
  * @brief   独立看门狗 (Independent Watchdog) 接口
  *
  * 工作原理：
  *   独立看门狗使用 STM32F407 内部独立的 LSI 振荡器（约 32 kHz）作为时钟源。
  *   LSI 与系统主时钟完全独立，因此即使 HSE/PLL 停振，看门狗仍然运行。
  *   硬件一旦启用 IWDG，只有上电复位才能关闭它——软件无法停止。
  *
  * 超时时间计算：
  *   当前配置：预分频 = 64，重装载值 = 3999
  *   T_out = (64 × 3999) / 32000 Hz ≈ 7.998 秒 ≈ 8 秒
  *   如果在 8 秒内没有调用 APP_Watchdog_Refresh() 喂狗，
  *   硬件将产生复位信号，MCU 重新启动。
  *
  * 喂狗策略：
  *   主循环所有路径（包括传感器异常循环、基线采集循环）都必须在超时前喂狗。
  *   Error_Handler() 中不喂狗——故意让看门狗复位系统，从致命错误中自动恢复。
  ******************************************************************************
  */

#ifndef __IWDG_H__
#define __IWDG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* HAL 生成的看门狗句柄，供外部引用 */
extern IWDG_HandleTypeDef hiwdg;

/* 初始化独立看门狗：启用 LSI，配置预分频和重装载值。
   调用后看门狗立即开始递减，必须在超时前开始喂狗。 */
void MX_IWDG_Init(void);

/* 喂狗：将计数器重置为初始重装载值。
   应在每个主循环迭代中调用一次，确保系统活着时不会复位。 */
void APP_Watchdog_Refresh(void);

#ifdef __cplusplus
}
#endif

#endif /* __IWDG_H__ */
