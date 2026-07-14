/**
  ******************************************************************************
  * @file    max30102_driver.h
  * @brief   MAX30102 硬件驱动层 — 寄存器、I2C、FIFO、初始化
  ******************************************************************************
  */

#ifndef __MAX30102_DRIVER_H__
#define __MAX30102_DRIVER_H__

#include "main.h"
#include <stdint.h>

/* MAX30102 7 位地址为 0x57，HAL 使用左移后的 8 位地址 */
#define MAX30102_I2C_ADDR                       (0x57U << 1)
#define MAX30102_I2C_TIMEOUT_MS                 100U /* 配置寄存器访问超时，单位：ms */
#define MAX30102_FIFO_READ_TIMEOUT_MS            10U /* 实时 FIFO 突发读取超时，单位：ms */
#define MAX30102_PART_ID_VALUE                  0x15U

/* 常用寄存器 */
#define MAX30102_REG_INTR_STATUS_1              0x00U
#define MAX30102_REG_INTR_STATUS_2              0x01U
#define MAX30102_REG_INTR_ENABLE_1              0x02U
#define MAX30102_REG_INTR_ENABLE_2              0x03U
#define MAX30102_REG_FIFO_WR_PTR                0x04U
#define MAX30102_REG_OVF_COUNTER                0x05U
#define MAX30102_REG_FIFO_RD_PTR                0x06U
#define MAX30102_REG_FIFO_DATA                  0x07U
#define MAX30102_REG_FIFO_CONFIG                0x08U
#define MAX30102_REG_MODE_CONFIG                0x09U
#define MAX30102_REG_SPO2_CONFIG                0x0AU
#define MAX30102_REG_LED1_PA                    0x0CU
#define MAX30102_REG_LED2_PA                    0x0DU
#define MAX30102_REG_MULTI_LED_CTRL1            0x11U
#define MAX30102_REG_MULTI_LED_CTRL2            0x12U
#define MAX30102_REG_TEMP_INTR                  0x1FU
#define MAX30102_REG_TEMP_FRAC                  0x20U
#define MAX30102_REG_TEMP_CONFIG                0x21U
#define MAX30102_REG_REV_ID                     0xFEU
#define MAX30102_REG_PART_ID                    0xFFU

/* MODE_CONFIG 常用位 */
#define MAX30102_MODE_SHUTDOWN                  0x80U
#define MAX30102_MODE_RESET                     0x40U
#define MAX30102_MODE_HEART_RATE                0x02U
#define MAX30102_MODE_SPO2                      0x03U
#define MAX30102_MODE_MULTI_LED                 0x07U

/* SpO2 模式下单个样本包含 RED + IR，共 6 字节 */
#define MAX30102_FIFO_BYTES_PER_LED             3U
#define MAX30102_FIFO_BYTES_PER_SAMPLE_SPO2     6U

/* 默认配置 */
#define MAX30102_DEFAULT_FIFO_CONFIG            0x0FU
#define MAX30102_DEFAULT_MODE_CONFIG            MAX30102_MODE_SPO2
/*
 * SPO2_CONFIG：
 * - ADC 量程：16384 nA
 * - 采样率：100 sps
 * - LED 脉宽：411 us / 18-bit
 *
 * 固件在每个 10 ms 服务周期中读取所有待处理的 FIFO 样本，
 * 因此传感器保持 100 Hz 以匹配算法固定的采样率计算。
 * 使用最大 ADC 量程，使手指接触不会将 RED/IR 锁定在 0x03FFFF。
 */
#define MAX30102_DEFAULT_SPO2_CONFIG            0x67U

/* LED 电流：数值 * 0.2 mA。在 ADC 量程为 16384 nA 时，25.6 mA
   应将手指接触的 DC 值恢复至有用的中量程范围附近，
   而不会将 RED/IR 锁定在 0x03FFFF。 */
#define MAX30102_DEFAULT_LED1_PA                0x80U
#define MAX30102_DEFAULT_LED2_PA                0x80U

#define MAX30102_INTR_STATUS_PPG_RDY            0x40U
#define MAX30102_FIFO_DEPTH                     32U
#define MAX30102_FIFO_PTR_MASK                  0x1FU
#define MAX30102_INT_POLL_FALLBACK_TICKS        10U

#ifndef MAX30102_USE_INT_PIN
#define MAX30102_USE_INT_PIN                    0U
#endif

#if (MAX30102_USE_INT_PIN != 0U)
#define MAX30102_DEFAULT_INTR_ENABLE_1          MAX30102_INTR_STATUS_PPG_RDY
#else
#define MAX30102_DEFAULT_INTR_ENABLE_1          0x00U
#endif

typedef struct
{
  uint8_t overflow_count;    /* 芯片 FIFO 溢出计数器快照 */
  uint8_t write_ptr;         /* 5 位 FIFO 写指针 */
  uint8_t read_ptr;          /* 5 位 FIFO 读指针 */
  uint8_t available_samples; /* 按环形指针差计算的待读样本数 */
} MAX30102_FifoDebug_t;

typedef enum
{
  MAX30102_BATCH_OK = 0U,        /* 成功读取至少一个样本 */
  MAX30102_BATCH_EMPTY,          /* FIFO 当前无可用样本 */
  MAX30102_BATCH_I2C_ERROR,      /* 指针或数据寄存器访问失败 */
  MAX30102_BATCH_OVERFLOW,       /* 检测到硬件 FIFO 溢出 */
  MAX30102_BATCH_FIFO_CLEAR_FAIL /* 溢出后的 FIFO 清理失败 */
} MAX30102_BatchStatus_t;

/** @brief 校验 PART_ID，复位芯片并写入 FIFO、SpO2、LED 与中断默认配置。 */
HAL_StatusTypeDef max30102_init(void);
/** @brief 写/读一个 8 位寄存器；调用方负责 I2C1 互斥。 */
HAL_StatusTypeDef max30102_write_reg(uint8_t reg_addr, uint8_t data);
HAL_StatusTypeDef max30102_read_reg(uint8_t reg_addr, uint8_t *data);
/** @brief 从 FIFO_DATA 连续读取原始字节。 */
HAL_StatusTypeDef max30102_read_fifo(uint8_t *fifo_data, uint16_t data_len);
/**
 * @brief  批量读取当前 FIFO 中的 RED/IR 样本，并同步溢出与指针诊断。
 * @return 实际解包的样本数，不超过 max_count；失败原因写入 batch_status。
 */
uint8_t max30102_read_fifo_batch(uint32_t *red, uint32_t *ir,
                                  uint8_t max_count, uint8_t *p_ovf,
                                  MAX30102_BatchStatus_t *batch_status);
/** @brief 返回最近一次 FIFO 指针/溢出快照的只读地址。 */
const MAX30102_FifoDebug_t *max30102_get_fifo_debug(void);
/** @brief 从 TIM6/EXTI ISR 置数据就绪标志；不执行 I2C 访问。 */
void max30102_mark_data_ready_from_isr(void);
/** @brief 根据轮询/INT 编译配置判断本次 MAXtask 周期是否应读 FIFO。 */
uint8_t max30102_should_service_fifo(void);

/** @brief 把 6 字节大端 FIFO 数据解析为两个 18 位 RED/IR 原始值。 */
void max30102_parse_spo2_sample(const uint8_t *fifo_data,
                                uint32_t *red,
                                uint32_t *ir);

#endif /* __MAX30102_DRIVER_H__ */
