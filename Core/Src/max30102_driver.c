/**
  ******************************************************************************
  * @file    max30102_driver.c
  * @brief   MAX30102 硬件驱动 — I2C 寄存器访问、FIFO 管理、初始化、中断
  ******************************************************************************
  */

#include "max30102_driver.h"
#include "eeprom_store.h"
#include "i2c.h"
#include <stdint.h>

static MAX30102_FifoDebug_t max30102_fifo_debug;

/*
 * MAX30102 FIFO 数据就绪检测。
 * 当前项目默认关闭 MAX30102 INT，由 TIM6 每 10 ms 唤醒 MAXtask 轮询 FIFO。
 *
 * 设计思路：
 *   max30102_data_ready_flag — TIM6 回调或启用后的 EXTI 回调置 1，MAXtask 读取后清零。
 *     上电初始值为 1，确保首次任务服务会读取一次 FIFO。
 *   max30102_int_seen — 记录是否调用过数据就绪标记入口；仅在编译启用 INT
 *     分支时参与“等待 INT/超时回退”决策。
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

static uint8_t max30102_get_red_led_pa(void)
{
  if (g_eeprom.max30102_led_red_pa == 0U)
  {
    return MAX30102_DEFAULT_LED1_PA;
  }

  return g_eeprom.max30102_led_red_pa;
}

static uint8_t max30102_get_ir_led_pa(void)
{
  if (g_eeprom.max30102_led_ir_pa == 0U)
  {
    return MAX30102_DEFAULT_LED2_PA;
  }

  return g_eeprom.max30102_led_ir_pa;
}

/* ---- 数据就绪 ---- */

/**
 * @brief  从 TIM6 或 EXTI 中断回调标记 FIFO 数据就绪。
 * @note   仅写原子宽度标志，不访问 I2C。是否启用 INT 等待模式仍由
 *         MAX30102_USE_INT_PIN 编译开关决定。
 */
void max30102_mark_data_ready_from_isr(void)
{
  max30102_data_ready_flag = 1U;
  max30102_int_seen = 1U;
}

/**
 * @brief  门控函数，决定本次节拍是否服务 FIFO。
 * @return 1 表示本次周期应读取 FIFO，0 表示等待 INT。
 * @note   默认为纯轮询（每节拍返回 1）。启用 INT 后：
 *         当 data_ready_flag 置位时立即返回 1；
 *         等待 INT 时返回 0（最多到回退节拍计数）；
 *         超时后强制读取，防止死锁。
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

/**
 * @brief  触发 MAX30102 软件复位。
 * @return HAL_OK 表示成功，否则返回 HAL_ERROR。
 * @note   复位后添加 10 ms 延时以使芯片稳定。
 *         复位内部 data-ready 和 INT-seen 标志。
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

/**
 * @brief  清除 FIFO 写指针、读指针和溢出计数器。
 * @return HAL_OK 表示成功，否则返回 HAL_ERROR。
 * @note   确保后续 FIFO 读取返回新样本而非
 *         之前会话的陈旧数据。
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

/**
 * @brief  通过读取状态寄存器清除 MAX30102 中断状态。
 * @return HAL_OK 表示成功，否则返回 HAL_ERROR。
 * @note   INT 引脚模式下需要；读取 INTR_STATUS_1 和 INTR_STATUS_2
 *         会自动清除中断标志，否则 INT 引脚保持低电平。
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

/**
 * @brief  获取 FIFO 中可用样本数。
 * @param  sample_count 可用样本数的输出指针。
 * @return HAL_OK 表示成功，HAL_ERROR 表示参数为 NULL，HAL_BUSY 表示溢出。
 * @note   发生溢出时，清除 FIFO 并复位 data-ready 标志。
 */
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

/**
 * @brief  通过 I2C 向 MAX30102 寄存器写入单个字节。
 * @param  reg_addr 寄存器地址。
 * @param  data     要写入的值。
 * @return HAL_OK 表示成功，HAL_ERROR 表示 I2C 通信失败。
 */
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

/**
 * @brief  通过 I2C 从 MAX30102 寄存器读取单个字节。
 * @param  reg_addr 寄存器地址。
 * @param  data     读取值的输出指针。
 * @return HAL_OK 表示成功，HAL_ERROR 表示参数为 NULL 或 I2C 通信失败。
 */
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

/**
 * @brief  以最小工作配置初始化 MAX30102。
 * @return HAL_OK 表示成功，HAL_ERROR 表示器件 ID 不匹配或 I2C 通信失败。
 * @note   验证器件 ID、复位芯片、配置中断使能、FIFO
 *         行为、SpO2 模式、ADC 量程、采样率和 LED 电流。
 *         工作在 SpO2 模式（FIFO 中包含 RED + IR 样本）。
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
  status = max30102_write_reg(MAX30102_REG_LED1_PA, max30102_get_red_led_pa());
  if (status != HAL_OK)
  {
    return status;
  }

  /* 设置 IR LED 电流。 */
  status = max30102_write_reg(MAX30102_REG_LED2_PA, max30102_get_ir_led_pa());
  if (status != HAL_OK)
  {
    return status;
  }

  max30102_data_ready_flag = 1U;
  max30102_int_seen = 0U;
  max30102_poll_fallback_ticks = 0U;

  return HAL_OK;
}

/**
 * @brief  从 MAX30102 FIFO 读取原始 SpO2 样本。
 * @param  fifo_data 原始 FIFO 字节的输出缓冲区。
 * @param  data_len  要读取的字节数（必须为 6 的倍数）。
 * @return HAL_OK 表示成功，HAL_BUSY 表示样本不足，
 *         HAL_ERROR 表示参数错误。
 * @note   使用阻塞 I2C，超时时间短（10 ms）。每个样本为
 *         6 字节（RED[3] + IR[3]）。
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

/**
 * @brief  获取 FIFO 调试结构体的指针。
 * @return 指向内部 MAX30102_FifoDebug_t 结构体的指针。
 * @note   提供 FIFO 指针、溢出和样本计数诊断信息的访问，
 *         用于调试和显示。
 */
const MAX30102_FifoDebug_t *max30102_get_fifo_debug(void)
{
  return &max30102_fifo_debug;
}

/**
 * @brief  将 6 字节 FIFO 样本解析为 18 位 RED 和 IR 原始值。
 * @param  fifo_data 指向 6 字节原始 FIFO 数据的指针。
 * @param  red       RED 通道值的输出指针。
 * @param  ir        IR 通道值的输出指针。
 * @note   每个通道占用 3 字节；仅低 18 位有效。
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

/* ---- 批量 FIFO 排空 ---- */

static uint8_t fifo_batch_raw[MAX30102_FIFO_DEPTH * MAX30102_FIFO_BYTES_PER_SAMPLE_SPO2];

/**
 * @brief  从 FIFO 突发读取多个 SpO2 样本并解析。
 * @param  red       RED 通道值的输出数组。
 * @param  ir        IR 通道值的输出数组。
 * @param  max_count 要读取的最大样本数。
 * @param  p_ovf     溢出计数的输出指针（无溢出时为 0）。
 * @return 实际读取的样本数（出错或溢出时返回 0）。
 * @note   为提升效率，使用单次 I2C 突发事务。发生溢出时
 *         清除 FIFO 并返回 0，同时设置溢出计数。
 */
uint8_t max30102_read_fifo_batch(uint32_t *red, uint32_t *ir,
                                  uint8_t max_count, uint8_t *p_ovf,
                                  MAX30102_BatchStatus_t *batch_status)
{
  HAL_StatusTypeDef status;
  uint8_t wr_ptr, rd_ptr, ovf;
  uint8_t available, to_read;
  uint8_t i;

  if ((red == NULL) || (ir == NULL) || (p_ovf == NULL) ||
      (batch_status == NULL) || (max_count == 0U))
  {
    return 0U;
  }

  *p_ovf = 0U;
  *batch_status = MAX30102_BATCH_EMPTY;

  /* 步骤 1：读取 FIFO 指针 */
  status = max30102_read_reg(MAX30102_REG_OVF_COUNTER, &ovf);
  if (status != HAL_OK) { *batch_status = MAX30102_BATCH_I2C_ERROR; return 0U; }

  status = max30102_read_reg(MAX30102_REG_FIFO_WR_PTR, &wr_ptr);
  if (status != HAL_OK) { *batch_status = MAX30102_BATCH_I2C_ERROR; return 0U; }

  status = max30102_read_reg(MAX30102_REG_FIFO_RD_PTR, &rd_ptr);
  if (status != HAL_OK) { *batch_status = MAX30102_BATCH_I2C_ERROR; return 0U; }

  ovf &= MAX30102_FIFO_PTR_MASK;
  wr_ptr &= MAX30102_FIFO_PTR_MASK;
  rd_ptr &= MAX30102_FIFO_PTR_MASK;

  max30102_fifo_debug.overflow_count = ovf;
  max30102_fifo_debug.write_ptr = wr_ptr;
  max30102_fifo_debug.read_ptr = rd_ptr;

  /* 步骤 2：处理溢出 — 清除 FIFO，设置 *p_ovf 后返回 0 */
  if (ovf > 0U)
  {
    *p_ovf = ovf;
    status = max30102_clear_fifo();
    max30102_fifo_debug.available_samples = 0U;
    if (status != HAL_OK)
    {
      *batch_status = MAX30102_BATCH_FIFO_CLEAR_FAIL;
      return 0U;
    }
#if (MAX30102_USE_INT_PIN != 0U)
    (void)max30102_clear_interrupt_status();
#endif
    max30102_data_ready_flag = 1U;
    max30102_poll_fallback_ticks = 0U;
    *batch_status = MAX30102_BATCH_OVERFLOW;
    return 0U;
  }

  /* 步骤 3：计算可用样本数 */
  available = (uint8_t)((wr_ptr - rd_ptr) & (MAX30102_FIFO_DEPTH - 1U));
  max30102_fifo_debug.available_samples = available;

  if (available == 0U) { return 0U; }

  to_read = (available < max_count) ? available : max_count;

  /* 步骤 4：突发读取 FIFO 数据 — 所有样本一次 I2C 事务完成 */
  status = HAL_I2C_Mem_Read(&hi2c1,
                             MAX30102_I2C_ADDR,
                             MAX30102_REG_FIFO_DATA,
                             I2C_MEMADD_SIZE_8BIT,
                             fifo_batch_raw,
                             (uint16_t)(to_read * MAX30102_FIFO_BYTES_PER_SAMPLE_SPO2),
                             MAX30102_FIFO_READ_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    max30102_fifo_debug.available_samples = available;
    *batch_status = MAX30102_BATCH_I2C_ERROR;
    return 0U;
  }

#if (MAX30102_USE_INT_PIN != 0U)
  (void)max30102_clear_interrupt_status();
#endif

  /* 步骤 5：解析每个样本 */
  for (i = 0U; i < to_read; i++)
  {
    max30102_parse_spo2_sample(
        &fifo_batch_raw[i * MAX30102_FIFO_BYTES_PER_SAMPLE_SPO2],
        &red[i], &ir[i]);
  }

  /* 步骤 6：更新可用数以反映已消耗的样本 */
  max30102_fifo_debug.available_samples =
      (uint8_t)((wr_ptr - (rd_ptr + to_read)) & (MAX30102_FIFO_DEPTH - 1U));

  *batch_status = MAX30102_BATCH_OK;
  return to_read;
}
