/**
  ******************************************************************************
  * @file    max30102.h
  * @brief   MAX30102 兼容总入口 — 统一 include 所有子模块
  *
  * 外部文件只需 include "max30102.h" 即可获得完整 MAX30102 API。
  * 内部实现已拆分为：
  *   - max30102_driver.h   硬件驱动层（寄存器、I2C、FIFO、初始化）
  *   - max30102_baseline.h 背景基线统计与手指检测
  *   - max30102_spo2.h     SpO2 窗口管理、信号质量、血氧计算
  *   - max30102_bpm.h      BPM 峰值检测 + 自相关心率估算
  *   - max30102_algo_utils.h 共享数学工具函数
  ******************************************************************************
  */

#ifndef __MAX30102_H__
#define __MAX30102_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "max30102_driver.h"
#include "max30102_baseline.h"
#include "max30102_spo2.h"
#include "max30102_bpm.h"
#include "max30102_algo_utils.h"

#ifdef __cplusplus
}
#endif

#endif /* __MAX30102_H__ */
