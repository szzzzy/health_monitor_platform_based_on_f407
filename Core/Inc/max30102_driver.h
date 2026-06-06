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
#define MAX30102_I2C_TIMEOUT_MS                 100U
#define MAX30102_FIFO_READ_TIMEOUT_MS            10U
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
 * SPO2_CONFIG:
 * - ADC range: 16384 nA
 * - Sample rate: 100 sps
 * - LED pulse width: 411 us / 18-bit
 *
 * The firmware drains every pending FIFO sample during each 10 ms service pass,
 * so keep the sensor at 100 Hz to match the algorithm's fixed sample-rate math.
 * Use the largest ADC range so finger contact does not pin RED/IR at 0x03FFFF.
 */
#define MAX30102_DEFAULT_SPO2_CONFIG            0x67U

/* LED current: value * 0.2 mA. With the ADC range at 16384 nA, 25.6 mA
   should bring finger-contact DC values back near the useful mid-scale range
   without pinning RED/IR at 0x03FFFF. */
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
  uint8_t overflow_count;
  uint8_t write_ptr;
  uint8_t read_ptr;
  uint8_t available_samples;
} MAX30102_FifoDebug_t;

HAL_StatusTypeDef max30102_init(void);
HAL_StatusTypeDef max30102_write_reg(uint8_t reg_addr, uint8_t data);
HAL_StatusTypeDef max30102_read_reg(uint8_t reg_addr, uint8_t *data);
HAL_StatusTypeDef max30102_read_fifo(uint8_t *fifo_data, uint16_t data_len);
const MAX30102_FifoDebug_t *max30102_get_fifo_debug(void);
void max30102_mark_data_ready_from_isr(void);
uint8_t max30102_should_service_fifo(void);

/* 把 6 字节 FIFO 数据解析成 RED / IR 原始值 */
void max30102_parse_spo2_sample(const uint8_t *fifo_data,
                                uint32_t *red,
                                uint32_t *ir);

#endif /* __MAX30102_DRIVER_H__ */
