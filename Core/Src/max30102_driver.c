/**
  ******************************************************************************
  * @file    max30102_driver.c
  * @brief   MAX30102 硬件驱动 — I2C 寄存器访问、FIFO 管理、初始化、中断
  ******************************************************************************
  */

#include "max30102_driver.h"
#include "i2c.h"
#include <stdint.h>

static MAX30102_FifoDebug_t max30102_fifo_debug;

/*
 * MAX30102 FIFO 数据就绪检测。
 * 当前项目默认关闭 MAX30102 INT，主循环按 TIM6 100 Hz 纯轮询 FIFO。
 *
 * 设计思路：
 *   max30102_data_ready_flag — 仅在启用 INT 时由 EXTI ISR 置 1，主循环读取后清零。
 *     上电初始值为 1，确保 TIM6 第一次节拍就会读一次 FIFO。
 *   max30102_int_seen — 记录是否至少触发过一次 EXTI 中断。
 *     - 若 INT 引脚已连接，首次 EXTI 后置 1，切换到"INT 驱动模式"。
 *     - 若 INT 引脚未连接，始终为 0，系统退化到"纯 TIM6 轮询模式"。
 *
 *   max30102_should_service_fifo() 决策逻辑：
 *     1. 默认纯轮询：每个 TIM6 节拍都读 FIFO
 *     2. 若 data_ready_flag == 1 → 立即读（INT 刚触发）
 *     3. 若 INT 曾被触发过但当前 data_ready_flag == 0：
 *        - 等待 INT（最多 10 个 TIM6 节拍 = 100 ms）
 *        - 超时后强制读一次（防止 INT 丢失导致死等）
 *     4. 若 INT 从未触发 → 每个 TIM6 节拍都读（纯轮询模式）
 */
static volatile uint8_t max30102_data_ready_flag = 1U;
static volatile uint8_t max30102_int_seen = 0U;
static uint8_t max30102_poll_fallback_ticks = 0U;

/* ---- 数据就绪 ---- */

/* 由 EXTI 中断回调调用，标记 MAX30102 FIFO 有新数据可读。 */
void max30102_mark_data_ready_from_isr(void)
{
  max30102_data_ready_flag = 1U;
  max30102_int_seen = 1U;
}

/*
 * 主循环门控：决定当前节拍是否需要读 FIFO。
 *
 *   默认 MAX30102_USE_INT_PIN == 0：每拍返回 1，按 TIM6 纯轮询 FIFO。
 *   若启用 INT 且 INT 已触发过（int_seen != 0）：
 *     - data_ready_flag 已置位 → 立即读，同时清零 flag 和 fallback 计数
 *     - data_ready_flag 未置位 → 等待 INT（最多 INT_POLL_FALLBACK_TICKS 拍），
 *       超时后强制读一次，防止 INT 偶然丢失导致系统死等
 *   若启用 INT 但 INT 从未触发（int_seen == 0）：
 *     - 每拍都返回 1，纯轮询模式
 */
uint8_t max30102_should_service_fifo(void)
{
  if (max30102_data_ready_flag != 0U)
  {
    max30102_data_ready_flag = 0U;
    max30102_poll_fallback_ticks = 0U;
    return 1U;
  }

#if (MAX30102_USE_INT_PIN != 0U)
  if (max30102_int_seen != 0U)
  {
    if (max30102_poll_fallback_ticks < MAX30102_INT_POLL_FALLBACK_TICKS)
    {
      max30102_poll_fallback_ticks++;
      return 0U;
    }

    max30102_poll_fallback_ticks = 0U;
  }
#endif

  return 1U;
}

/* ---- 内部辅助 ---- */

/*
 * 触发 MAX30102 软件复位。
 * 复位后内部状态机会回到默认状态，因此这里额外等待一小段时间，
 * 避免紧接着访问寄存器时芯片尚未准备好。
 */
static HAL_StatusTypeDef max30102_reset(void)
{
  HAL_StatusTypeDef status;

  status = max30102_write_reg(MAX30102_REG_MODE_CONFIG, MAX30102_MODE_RESET);
  if (status != HAL_OK)
  {
    return status;
  }

  HAL_Delay(10U);
  max30102_data_ready_flag = 1U;
  max30102_int_seen = 0U;
  max30102_poll_fallback_ticks = 0U;

  return HAL_OK;
}

/*
 * 清空 FIFO 写指针、读指针与溢出计数器。
 * 这样做的目的是保证后续第一次读 FIFO 时拿到的是"当前配置下的新样本"，
 * 而不是上电或上次运行遗留下来的旧数据。
 */
static HAL_StatusTypeDef max30102_clear_fifo(void)
{
  HAL_StatusTypeDef status;

  status = max30102_write_reg(MAX30102_REG_FIFO_WR_PTR, 0x00U);
  if (status != HAL_OK)
  {
    return status;
  }

  status = max30102_write_reg(MAX30102_REG_OVF_COUNTER, 0x00U);
  if (status != HAL_OK)
  {
    return status;
  }

  status = max30102_write_reg(MAX30102_REG_FIFO_RD_PTR, 0x00U);
  if (status != HAL_OK)
  {
    return status;
  }

  return HAL_OK;
}

/*
 * 清除 MAX30102 中断状态。
 * 读取 INTR_STATUS_1 和 INTR_STATUS_2 寄存器即可自动清除中断标志位。
 * INT 引脚模式的必需要操作——否则 INT 引脚会一直保持低电平，无法产生新中断。
 */
static HAL_StatusTypeDef max30102_clear_interrupt_status(void)
{
  HAL_StatusTypeDef status;
  uint8_t status_value;

  status = max30102_read_reg(MAX30102_REG_INTR_STATUS_1, &status_value);
  if (status != HAL_OK)
  {
    return status;
  }

  return max30102_read_reg(MAX30102_REG_INTR_STATUS_2, &status_value);
}

static HAL_StatusTypeDef max30102_get_fifo_sample_count(uint8_t *sample_count)
{
  HAL_StatusTypeDef status;
  uint8_t overflow_count;
  uint8_t write_ptr;
  uint8_t read_ptr;

  if (sample_count == NULL)
  {
    return HAL_ERROR;
  }

  status = max30102_read_reg(MAX30102_REG_OVF_COUNTER, &overflow_count);
  if (status != HAL_OK)
  {
    return status;
  }

  status = max30102_read_reg(MAX30102_REG_FIFO_WR_PTR, &write_ptr);
  if (status != HAL_OK)
  {
    return status;
  }

  status = max30102_read_reg(MAX30102_REG_FIFO_RD_PTR, &read_ptr);
  if (status != HAL_OK)
  {
    return status;
  }

  overflow_count &= MAX30102_FIFO_PTR_MASK;
  write_ptr &= MAX30102_FIFO_PTR_MASK;
  read_ptr &= MAX30102_FIFO_PTR_MASK;
  max30102_fifo_debug.overflow_count = overflow_count;
  max30102_fifo_debug.write_ptr = write_ptr;
  max30102_fifo_debug.read_ptr = read_ptr;

  if (overflow_count != 0U)
  {
    status = max30102_clear_fifo();
    max30102_fifo_debug.available_samples = 0U;
    *sample_count = 0U;
    if (status != HAL_OK)
    {
      return status;
    }

#if (MAX30102_USE_INT_PIN != 0U)
    (void)max30102_clear_interrupt_status();
#endif
    max30102_data_ready_flag = 1U;
    max30102_poll_fallback_ticks = 0U;
    return HAL_BUSY;
  }

  *sample_count = (uint8_t)((write_ptr - read_ptr) & (MAX30102_FIFO_DEPTH - 1U));
  max30102_fifo_debug.available_samples = *sample_count;
  return HAL_OK;
}

/* ---- 公共 API ---- */

HAL_StatusTypeDef max30102_write_reg(uint8_t reg_addr, uint8_t data)
{
  return HAL_I2C_Mem_Write(&hi2c1,
                           MAX30102_I2C_ADDR,
                           reg_addr,
                           I2C_MEMADD_SIZE_8BIT,
                           &data,
                           1U,
                           MAX30102_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef max30102_read_reg(uint8_t reg_addr, uint8_t *data)
{
  if (data == NULL)
  {
    return HAL_ERROR;
  }

  return HAL_I2C_Mem_Read(&hi2c1,
                          MAX30102_I2C_ADDR,
                          reg_addr,
                          I2C_MEMADD_SIZE_8BIT,
                          data,
                          1U,
                          MAX30102_I2C_TIMEOUT_MS);
}

/*
 * 初始化 MAX30102 当前所需的最小工作配置。
 * 现在目标是稳定读出 RED/IR 原始值，因此只打开 SpO2 模式下的基础寄存器配置，
 * 暂时不引入中断、温度通道或多 LED 时序配置。
 */
HAL_StatusTypeDef max30102_init(void)
{
  HAL_StatusTypeDef status;
  uint8_t part_id;

  HAL_Delay(10U);

  /* 先读器件 ID，确认总线上响应的确实是 MAX30102。 */
  status = max30102_read_reg(MAX30102_REG_PART_ID, &part_id);
  if (status != HAL_OK)
  {
    return status;
  }

  if (part_id != MAX30102_PART_ID_VALUE)
  {
    return HAL_ERROR;
  }

  status = max30102_reset();
  if (status != HAL_OK)
  {
    return status;
  }

  /* 根据 MAX30102_USE_INT_PIN 配置使能或不使能 PPG 数据就绪中断。 */
  status = max30102_write_reg(MAX30102_REG_INTR_ENABLE_1, MAX30102_DEFAULT_INTR_ENABLE_1);
  if (status != HAL_OK)
  {
    return status;
  }

  status = max30102_write_reg(MAX30102_REG_INTR_ENABLE_2, 0x00U);
  if (status != HAL_OK)
  {
    return status;
  }

  (void)max30102_clear_interrupt_status();

  status = max30102_clear_fifo();
  if (status != HAL_OK)
  {
    return status;
  }

  /* 配置 FIFO 行为，例如平均、回卷和采样满阈值。 */
  status = max30102_write_reg(MAX30102_REG_FIFO_CONFIG, MAX30102_DEFAULT_FIFO_CONFIG);
  if (status != HAL_OK)
  {
    return status;
  }

  /* 选择 SpO2 模式，使 FIFO 中按 RED + IR 的顺序输出样本。 */
  status = max30102_write_reg(MAX30102_REG_MODE_CONFIG, MAX30102_DEFAULT_MODE_CONFIG);
  if (status != HAL_OK)
  {
    return status;
  }

  /* 配置 ADC 量程、采样率、脉宽等参数。 */
  status = max30102_write_reg(MAX30102_REG_SPO2_CONFIG, MAX30102_DEFAULT_SPO2_CONFIG);
  if (status != HAL_OK)
  {
    return status;
  }

  /* 设置 RED LED 电流。 */
  status = max30102_write_reg(MAX30102_REG_LED1_PA, MAX30102_DEFAULT_LED1_PA);
  if (status != HAL_OK)
  {
    return status;
  }

  /* 设置 IR LED 电流。 */
  status = max30102_write_reg(MAX30102_REG_LED2_PA, MAX30102_DEFAULT_LED2_PA);
  if (status != HAL_OK)
  {
    return status;
  }

  max30102_data_ready_flag = 1U;
  max30102_int_seen = 0U;
  max30102_poll_fallback_ticks = 0U;

  return HAL_OK;
}

/*
 * 从 FIFO 读取 6 字节原始样本（RED[3] + IR[3]）。
 *
 * 使用短超时阻塞 I2C 读（10ms），避免 DMA 中断链在 I2C 恢复/竞争期
 * 引入的复杂状态。ECG ADC DMA、UART RX DMA、SDIO 保持独立。
 */
HAL_StatusTypeDef max30102_read_fifo(uint8_t *fifo_data, uint16_t data_len)
{
  HAL_StatusTypeDef status;
  uint8_t available_samples;
  uint16_t required_samples;

  if ((fifo_data == NULL) || (data_len == 0U))
  {
    return HAL_ERROR;
  }

  if ((data_len % MAX30102_FIFO_BYTES_PER_SAMPLE_SPO2) != 0U)
  {
    return HAL_ERROR;
  }

  required_samples = (uint16_t)(data_len / MAX30102_FIFO_BYTES_PER_SAMPLE_SPO2);
  if (required_samples == 0U)
  {
    return HAL_ERROR;
  }

  status = max30102_get_fifo_sample_count(&available_samples);
  if (status != HAL_OK)
  {
    return status;
  }

  if (available_samples < required_samples)
  {
    return HAL_BUSY;
  }

  /*
   * 阻塞 I2C 读，10ms 超时。6 字节 @ 400kHz ≈ 0.2ms，
   * 10ms 有充足余量处理 I2C 重试和时钟拉伸。
   * SDIO / UART DMA / ECG DMA 不受影响。
   */
  status = HAL_I2C_Mem_Read(&hi2c1,
                            MAX30102_I2C_ADDR,
                            MAX30102_REG_FIFO_DATA,
                            I2C_MEMADD_SIZE_8BIT,
                            fifo_data,
                            data_len,
                            MAX30102_FIFO_READ_TIMEOUT_MS);

#if (MAX30102_USE_INT_PIN != 0U)
  (void)max30102_clear_interrupt_status();
#endif

  return status;
}

const MAX30102_FifoDebug_t *max30102_get_fifo_debug(void)
{
  return &max30102_fifo_debug;
}

/*
 * 把 FIFO 中 6 字节样本解析为 18 位 RED / IR 原始值。
 * MAX30102 每个通道占 3 字节，但真正有效位数是低 18 位，因此最后要掩码。
 */
void max30102_parse_spo2_sample(const uint8_t *fifo_data, uint32_t *red, uint32_t *ir)
{
  if ((fifo_data == NULL) || (red == NULL) || (ir == NULL))
  {
    return;
  }

  *red = ((uint32_t)fifo_data[0] << 16) |
         ((uint32_t)fifo_data[1] << 8) |
         ((uint32_t)fifo_data[2]);
  *red &= 0x03FFFFU;

  *ir = ((uint32_t)fifo_data[3] << 16) |
        ((uint32_t)fifo_data[4] << 8) |
        ((uint32_t)fifo_data[5]);
  *ir &= 0x03FFFFU;
}
