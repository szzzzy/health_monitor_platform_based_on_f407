#include "max30102.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "arm_math.h"
#include "i2c.h"

/*
 * arm_bitreversal_32 的纯 C 替代实现。
 *
 * CMSIS-DSP 原版使用汇编（GAS 语法），ARMCLANG v6 / armasm 无法汇编。
 * 本函数提供等效功能：对交错的复数缓冲区进行 in-place 位逆序重排。
 *
 * 关键细节：CMSIS-DSP 的位逆序表 pBitRevTable 存储的是"字节偏移"而非
 * "复数样本索引"。因此需要用 uint8_t* 基址 + 字节偏移来定位待交换的
 * 复数对（每个复数 = 2 个 float32 = 8 字节）。
 */
void arm_bitreversal_32(uint32_t *pSrc, const uint16_t bitRevLength, const uint16_t *pBitRevTable)
{
  uint16_t i;
  uint8_t *base;
  uint32_t *a;
  uint32_t *b;
  uint32_t tmp_re;
  uint32_t tmp_im;

  if ((pSrc == NULL) || (pBitRevTable == NULL))
  {
    return;
  }

  base = (uint8_t *)pSrc;
  for (i = 0U; (uint16_t)(i + 1U) < bitRevLength; i = (uint16_t)(i + 2U))
  {
    a = (uint32_t *)(base + pBitRevTable[i]);
    b = (uint32_t *)(base + pBitRevTable[i + 1U]);

    tmp_re = a[0];
    tmp_im = a[1];
    a[0] = b[0];
    a[1] = b[1];
    b[0] = tmp_re;
    b[1] = tmp_im;
  }
}

/* =========================================================================
 * BPM / SpO2 算法参数
 *
 * 所有参数按 100 Hz 采样率设计。若修改 MAX30102_DEFAULT_SPO2_CONFIG
 * 中的采样率，以下阈值需要同步缩放。
 * ========================================================================= */
#define MAX30102_BPM_MIN_RESULT            35U
#define MAX30102_BPM_MAX_RESULT            220U
#define MAX30102_BPM_MIN_AMPLITUDE         24U
#define MAX30102_BPM_MIN_AC_RMS            4U
#define MAX30102_BPM_THRESHOLD_NUM         4U
#define MAX30102_BPM_THRESHOLD_DEN         100U
#define MAX30102_BPM_MIN_THRESHOLD         4U
#define MAX30102_BPM_PROMINENCE_NUM        3U
#define MAX30102_BPM_PROMINENCE_DEN        100U
#define MAX30102_BPM_MIN_PROMINENCE        4U
#define MAX30102_BPM_MAX_INTERVAL_JITTER   85U
#define MAX30102_BPM_MIN_EDGE_DELTA        2U
#define MAX30102_BPM_MAX_PEAK_COUNT        24U
#define MAX30102_BPM_RANGE_CAP_RMS_FACTOR  4U
/* 心搏幅度估算用的半窗口（±10 样本 = ±100ms @100Hz），在峰值附近搜索局部最小值计算 peak-local_min。 */
#define MAX30102_BPM_PEAK_AMP_HALF_WINDOW  10U
#define MAX30102_SPO2_MIN_DC               1U
#define MAX30102_SPO2_MIN_AC_RMS           40U
/* 通用比值缩放因子 ×1000，用于 PI=(AC/DC)*1000 和 SpO2 R 比值计算。PI 存储值 = 标准 PI% × 10。 */
#define MAX30102_SPO2_RATIO_SCALE          1000U
#define MAX30102_FILTER_DC_SHIFT           4U
#define MAX30102_INTR_STATUS_PPG_RDY       0x40U
#define MAX30102_SIGNAL_QUALITY_IR_PI_GOOD 4U
#define MAX30102_SIGNAL_QUALITY_RED_PI_GOOD 3U
#define MAX30102_SIGNAL_QUALITY_IR_RMS_GOOD 24U
#define MAX30102_SIGNAL_QUALITY_RED_RMS_GOOD 18U
#define MAX30102_SIGNAL_QUALITY_WINDOW_GOOD 80U
#define MAX30102_FIFO_DEPTH                32U
#define MAX30102_FIFO_PTR_MASK             0x1FU
#define MAX30102_INT_POLL_FALLBACK_TICKS   10U

#ifndef MAX30102_USE_INT_PIN
#define MAX30102_USE_INT_PIN               0U
#endif

#if (MAX30102_USE_INT_PIN != 0U)
#define MAX30102_DEFAULT_INTR_ENABLE_1     MAX30102_INTR_STATUS_PPG_RDY
#else
#define MAX30102_DEFAULT_INTR_ENABLE_1     0x00U
#endif

static MAX30102_FifoDebug_t max30102_fifo_debug;

/* --------------------------------------------------------------------------
 * I2C DMA 传输状态 — HAL 回调中设置，max30102_read_fifo 中轮询等待。
 * -------------------------------------------------------------------------- */
static volatile uint8_t i2c_dma_busy = 0U;
static volatile HAL_StatusTypeDef i2c_dma_result;

/* --------------------------------------------------------------------------
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
 * -------------------------------------------------------------------------- */
static volatile uint8_t max30102_data_ready_flag = 1U;
static volatile uint8_t max30102_int_seen = 0U;
static uint8_t max30102_poll_fallback_ticks = 0U;

static HAL_StatusTypeDef max30102_clear_interrupt_status(void);

/* HAL I2C 接收完成回调 — DMA 传输结束后由 I2C 中断链调用。 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    i2c_dma_result = HAL_OK;
    i2c_dma_busy = 0U;
  }
}

/* HAL I2C 错误回调 — DMA 或总线错误时由 I2C 中断链调用。 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    i2c_dma_result = HAL_ERROR;
    i2c_dma_busy = 0U;
  }
}

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

/* --------------------------------------------------------------------------
 * CMSIS-DSP 4th-order Butterworth bandpass (2 biquad stages)
 *
 * Designed for fs = 100 Hz, passband ≈ 0.5–5 Hz (30–300 BPM).
 * Coefficients are in CMSIS order: {b0, b1, b2, -a1, -a2}.
 * -------------------------------------------------------------------------- */
#define BIQUAD_STAGES 2U

static const float32_t biquad_coeffs[5U * BIQUAD_STAGES] = {
  /* Stage 1 – HPF, fc ≈ 0.5 Hz */
   0.97803048f, -1.95606096f, 0.97803048f,  1.95557824f, -0.95654368f,
  /* Stage 2 – LPF, fc ≈ 5 Hz */
   0.02008337f,  0.04016673f, 0.02008337f,  1.56101808f, -0.64135154f
};

static arm_biquad_casd_df1_inst_f32 biquad_red;
static arm_biquad_casd_df1_inst_f32 biquad_ir;
static float32_t biquad_state_red[4U * BIQUAD_STAGES];
static float32_t biquad_state_ir[4U * BIQUAD_STAGES];
static uint8_t biquad_initialized = 0U;

static void biquad_ensure_init(void)
{
  if (biquad_initialized != 0U)
  {
    return;
  }

  arm_biquad_cascade_df1_init_f32(&biquad_red, BIQUAD_STAGES,
                                  (float32_t *)biquad_coeffs, biquad_state_red);
  arm_biquad_cascade_df1_init_f32(&biquad_ir, BIQUAD_STAGES,
                                  (float32_t *)biquad_coeffs, biquad_state_ir);
  biquad_initialized = 1U;
}

static void biquad_reset(void)
{
  (void)memset(biquad_state_red, 0, sizeof(biquad_state_red));
  (void)memset(biquad_state_ir,  0, sizeof(biquad_state_ir));
  biquad_initialized = 0U;
  biquad_ensure_init();
}

/*
 * 用整数形式做一个慢速一阶低通：
 * - shift 越大，跟随越慢
 * - 至少移动 1 个计数，避免小误差长期卡住不更新
 */
static uint32_t max30102_slow_follow_u32(uint32_t current, uint32_t target, uint8_t shift)
{
  uint32_t delta;
  uint32_t step;

  if (current == target)
  {
    return current;
  }

  if (current < target)
  {
    delta = target - current;
    step = delta >> shift;
    if (step == 0U)
    {
      step = 1U;
    }

    return current + step;
  }

  delta = current - target;
  step = delta >> shift;
  if (step == 0U)
  {
    step = 1U;
  }

  return current - step;
}

static uint32_t max30102_isqrt_u64(uint64_t value)
{
  uint64_t bit = 1ULL << 62;
  uint64_t result = 0ULL;

  while (bit > value)
  {
    bit >>= 2;
  }

  while (bit != 0ULL)
  {
    if (value >= (result + bit))
    {
      value -= (result + bit);
      result = (result >> 1) + bit;
    }
    else
    {
      result >>= 1;
    }

    bit >>= 2;
  }

  return (uint32_t)result;
}

/*
 * STM32F407 has an M4F core, so using `sqrtf` here is cheaper than walking a
 * full software integer square root for the medium-sized energy terms used by
 * the algorithm windows. Keep the integer fallback for portability.
 */
static uint32_t max30102_fast_sqrt_u64(uint64_t value)
{
  if (value == 0ULL)
  {
    return 0U;
  }

#if defined(__FPU_PRESENT) && defined(__FPU_USED) && (__FPU_PRESENT == 1U) && (__FPU_USED == 1U)
  return (uint32_t)(sqrtf((float)value) + 0.5f);
#else
  return max30102_isqrt_u64(value);
#endif
}

static uint8_t max30102_scale_score_u32(uint32_t value, uint32_t target, uint8_t full_score)
{
  uint32_t scaled_score;

  if (full_score == 0U)
  {
    return 0U;
  }

  if (target == 0U)
  {
    return full_score;
  }

  scaled_score = ((value * full_score) + (target / 2U)) / target;
  if (scaled_score > full_score)
  {
    scaled_score = full_score;
  }

  return (uint8_t)scaled_score;
}

/*
 * 把“逻辑上的第 i 个样本”映射到环形缓冲区中的真实下标。
 * 当窗口未填满时，样本按 0..sample_count-1 顺序存放；
 * 当窗口填满后，最旧样本从 write_index 开始。
 */
static uint64_t max30102_square_u32(uint32_t value)
{
  return (uint64_t)value * (uint64_t)value;
}

static uint64_t max30102_square_i32(int32_t value)
{
  return (uint64_t)((int64_t)value * (int64_t)value);
}

static uint32_t max30102_calculate_window_rms(uint64_t square_sum, uint16_t sample_count)
{
  if (sample_count == 0U)
  {
    return 0U;
  }

  return max30102_fast_sqrt_u64(square_sum / sample_count);
}

/*
 * Compute the centered RMS from the exact integer variance numerator
 * `n * sum(x^2) - sum(x)^2`, then let the FPU do the final square root. This
 * avoids the large-DC cancellation problem of direct float variance.
 */
static uint32_t max30102_calculate_centered_rms(uint64_t sum, uint64_t square_sum, uint16_t sample_count)
{
  uint64_t variance_n2;
  uint64_t sum_square;

  if (sample_count == 0U)
  {
    return 0U;
  }

  variance_n2 = square_sum * (uint64_t)sample_count;
  sum_square = sum * sum;
  if (variance_n2 <= sum_square)
  {
    return 0U;
  }

  variance_n2 -= sum_square;

#if defined(__FPU_PRESENT) && defined(__FPU_USED) && (__FPU_PRESENT == 1U) && (__FPU_USED == 1U)
  return (uint32_t)((sqrtf((float)variance_n2) + ((float)sample_count * 0.5f)) /
                    (float)sample_count);
#else
  return (max30102_isqrt_u64(variance_n2) + (sample_count / 2U)) / sample_count;
#endif
}

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
 * 这样做的目的是保证后续第一次读 FIFO 时拿到的是“当前配置下的新样本”，
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
 * 向 MAX30102 指定寄存器写入 1 字节数据。
 * 外层统一通过 HAL I2C 内存访问接口操作，避免上层业务直接散落 I2C 细节。
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

/*
 * 从 MAX30102 指定寄存器读取 1 字节数据。
 * 如果 data 为空指针，直接返回错误，防止调用层把结果写到非法地址。
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

  /* TODO: 多 LED 模式下继续配置 MULTI_LED_CTRL1 / MULTI_LED_CTRL2。 */

  return HAL_OK;
}

/*
 * 从 FIFO 连续读取原始样本字节流。
 * SpO2 模式下一个样本包含 RED 3 字节 + IR 3 字节，因此常见读取长度是 6 字节。
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

  /* 仅当 FIFO 中已有整数个完整样本时才读取。 */
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

  /* 启动 DMA 传输。I2C 传输完成后，HAL 回调会清除 i2c_dma_busy。 */
  i2c_dma_result = HAL_BUSY;
  i2c_dma_busy = 1U;
  status = HAL_I2C_Mem_Read_DMA(&hi2c1,
                                MAX30102_I2C_ADDR,
                                MAX30102_REG_FIFO_DATA,
                                I2C_MEMADD_SIZE_8BIT,
                                fifo_data,
                                data_len);
  if (status != HAL_OK)
  {
    i2c_dma_busy = 0U;
    return status;
  }

  /* 短轮询等待 DMA 完成：6 字节 @ 400 kHz I2C ≈ 135 µs 传输时间。 */
  {
    uint32_t timeout = HAL_GetTick() + 5U;

    while (i2c_dma_busy != 0U)
    {
      if (HAL_GetTick() >= timeout)
      {
        i2c_dma_busy = 0U;
        return HAL_TIMEOUT;
      }
    }
  }

  status = i2c_dma_result;

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

/*
 * 重置背景基线统计结构体。
 * 初始化后：
 * - ir_sum 用于求平均
 * - ir_min / ir_max 用于评估背景稳定性
 * - sample_count 用于判断是否采够目标样本数
 */
void max30102_baseline_reset(MAX30102_Baseline_t *baseline)
{
  if (baseline == NULL)
  {
    return;
  }

  baseline->ir_sum = 0U;
  baseline->ir_min = 0xFFFFFFFFUL;
  baseline->ir_max = 0U;
  baseline->tracked_ir = 0U;
  baseline->noise_ir = 0U;
  baseline->sample_count = 0U;
}

/*
 * 把一个新的 IR 样本纳入背景统计。
 * 该函数只用于“无手指阶段”的背景采样，不用于运行时的实时滤波。
 */
void max30102_baseline_add_ir(MAX30102_Baseline_t *baseline, uint32_t ir_value)
{
  if (baseline == NULL)
  {
    return;
  }

  baseline->ir_sum += ir_value;

  if (ir_value < baseline->ir_min)
  {
    baseline->ir_min = ir_value;
  }

  if (ir_value > baseline->ir_max)
  {
    baseline->ir_max = ir_value;
  }

  baseline->sample_count++;
}

/* 判断背景采样是否已经采够目标数量。 */
uint8_t max30102_baseline_is_ready(const MAX30102_Baseline_t *baseline, uint16_t target_samples)
{
  if (baseline == NULL)
  {
    return 0U;
  }

  return (baseline->sample_count >= target_samples) ? 1U : 0U;
}

/* 计算背景平均 IR。若尚无样本，则返回 0。 */
uint32_t max30102_baseline_get_average_ir(const MAX30102_Baseline_t *baseline)
{
  if ((baseline == NULL) || (baseline->sample_count == 0U))
  {
    return 0U;
  }

  return (baseline->ir_sum / baseline->sample_count);
}

/* 计算背景阶段的波动范围。若尚无样本，则返回 0。 */
uint32_t max30102_baseline_get_range_ir(const MAX30102_Baseline_t *baseline)
{
  if ((baseline == NULL) || (baseline->sample_count == 0U))
  {
    return 0U;
  }

  return (baseline->ir_max - baseline->ir_min);
}

/*
 * 判断背景采样阶段是否足够稳定。
 * 这里使用“最大值 - 最小值”的简单波动范围判断，优点是实现轻量、直观。
 */
uint8_t max30102_baseline_is_stable(const MAX30102_Baseline_t *baseline, uint32_t stable_range)
{
  if ((baseline == NULL) || (baseline->sample_count == 0U))
  {
    return 0U;
  }

  return ((baseline->ir_max - baseline->ir_min) <= stable_range) ? 1U : 0U;
}

/*
 * 用开机背景采样的结果为运行时基线跟踪器设定初值。
 * tracked_ir 表示当前认为的“无手指背景 IR”，noise_ir 表示背景波动量级。
 */
void max30102_baseline_seed_tracking(MAX30102_Baseline_t *baseline,
                                     uint32_t baseline_ir,
                                     uint32_t noise_ir)
{
  if (baseline == NULL)
  {
    return;
  }

  baseline->tracked_ir = baseline_ir;
  baseline->noise_ir = (noise_ir != 0U) ? noise_ir : 1U;
}

/*
 * 在“当前确认无手指”的阶段，持续跟踪背景 IR。
 * 这样即使环境光、供电或模块温漂导致背景缓慢变化，基线也能慢慢跟上，
 * 不会一直抱着开机那一次采样结果不放。
 */
void max30102_baseline_track_background(MAX30102_Baseline_t *baseline, uint32_t ir_value)
{
  uint32_t deviation;

  if (baseline == NULL)
  {
    return;
  }

  if (baseline->tracked_ir == 0U)
  {
    baseline->tracked_ir = ir_value;
    baseline->noise_ir = 1U;
    return;
  }

  if (ir_value >= baseline->tracked_ir)
  {
    deviation = ir_value - baseline->tracked_ir;
  }
  else
  {
    deviation = baseline->tracked_ir - ir_value;
  }

  /*
   * 背景基线本身跟随得更慢，避免把短时噪声直接吞进基线；
   * 背景波动量级跟随得稍快，用于后续自适应阈值。
   */
  baseline->tracked_ir = max30102_slow_follow_u32(baseline->tracked_ir, ir_value, 4U);
  baseline->noise_ir = max30102_slow_follow_u32(baseline->noise_ir, deviation, 3U);

  if (baseline->noise_ir == 0U)
  {
    baseline->noise_ir = 1U;
  }
}

/* 获取运行时动态背景基线。 */
uint32_t max30102_baseline_get_tracked_ir(const MAX30102_Baseline_t *baseline)
{
  if (baseline == NULL)
  {
    return 0U;
  }

  if (baseline->tracked_ir != 0U)
  {
    return baseline->tracked_ir;
  }

  return max30102_baseline_get_average_ir(baseline);
}

/* 获取运行时背景噪声估计值。 */
uint32_t max30102_baseline_get_noise_ir(const MAX30102_Baseline_t *baseline)
{
  if (baseline == NULL)
  {
    return 0U;
  }

  return baseline->noise_ir;
}

/*
 * 用双阈值滞回做手指状态切换，避免临界点附近因为噪声产生反复跳变。
 * current_state 表示调用前的状态：
 * - 0 表示当前认为“无手指”
 * - 1 表示当前认为“有手指”
 */
uint8_t max30102_detect_finger(uint32_t ir_value,
                               uint32_t baseline_ir,
                               uint8_t current_state,
                               uint32_t finger_on_delta,
                               uint32_t finger_off_delta)
{
  if (current_state == 0U)
  {
    if (ir_value > (baseline_ir + finger_on_delta))
    {
      return 1U;
    }
  }
  else
  {
    if (ir_value < (baseline_ir + finger_off_delta))
    {
      return 0U;
    }
  }

  return current_state;
}

/*
 * 重置测量算法窗口。
 * 当前采用固定长度滑动窗口缓存最近一段 RED/IR 样本，
 * 供 BPM 峰值检测与 SpO2 AC/DC 比值计算共同使用。
 */
void max30102_spo2_reset(MAX30102_SpO2_t *spo2_state)
{
  uint16_t i;

  if (spo2_state == NULL)
  {
    return;
  }

  for (i = 0U; i < MAX30102_SPO2_WINDOW_SIZE; i++)
  {
    spo2_state->red_samples[i] = 0U;
    spo2_state->ir_samples[i] = 0U;
    spo2_state->red_filtered_samples[i] = 0;
    spo2_state->ir_filtered_samples[i] = 0;
  }

  spo2_state->write_index = 0U;
  spo2_state->sample_count = 0U;
  spo2_state->red_dc_estimate = 0U;
  spo2_state->ir_dc_estimate = 0U;
  spo2_state->red_filter_state = 0;
  spo2_state->ir_filter_state = 0;
  spo2_state->red_sum = 0ULL;
  spo2_state->ir_sum = 0ULL;
  spo2_state->red_square_sum = 0ULL;
  spo2_state->ir_square_sum = 0ULL;
  spo2_state->red_filtered_square_sum = 0ULL;
  spo2_state->ir_filtered_square_sum = 0ULL;
  spo2_state->total_samples = 0U;
  biquad_reset();
}

/* 向测量算法窗口中压入一个新的 RED/IR 样本。 */
void max30102_spo2_add_sample(MAX30102_SpO2_t *spo2_state, uint32_t red_value, uint32_t ir_value)
{
  int32_t old_red_filtered;
  int32_t old_ir_filtered;
  uint16_t sample_index;

  if (spo2_state == NULL)
  {
    return;
  }

  sample_index = spo2_state->write_index;
  if (spo2_state->sample_count >= MAX30102_SPO2_WINDOW_SIZE)
  {
    spo2_state->red_sum -= spo2_state->red_samples[sample_index];
    spo2_state->ir_sum -= spo2_state->ir_samples[sample_index];
    spo2_state->red_square_sum -= max30102_square_u32(spo2_state->red_samples[sample_index]);
    spo2_state->ir_square_sum -= max30102_square_u32(spo2_state->ir_samples[sample_index]);

    old_red_filtered = spo2_state->red_filtered_samples[sample_index];
    old_ir_filtered = spo2_state->ir_filtered_samples[sample_index];
    spo2_state->red_filtered_square_sum -= max30102_square_i32(old_red_filtered);
    spo2_state->ir_filtered_square_sum -= max30102_square_i32(old_ir_filtered);
  }

  /* 维护慢速 DC 估计（用于信号质量 PI 计算和外部诊断）。 */
  if (spo2_state->red_dc_estimate == 0U)
  {
    spo2_state->red_dc_estimate = red_value;
  }
  else
  {
    spo2_state->red_dc_estimate = max30102_slow_follow_u32(spo2_state->red_dc_estimate,
                                                           red_value,
                                                           MAX30102_FILTER_DC_SHIFT);
  }

  if (spo2_state->ir_dc_estimate == 0U)
  {
    spo2_state->ir_dc_estimate = ir_value;
  }
  else
  {
    spo2_state->ir_dc_estimate = max30102_slow_follow_u32(spo2_state->ir_dc_estimate,
                                                          ir_value,
                                                          MAX30102_FILTER_DC_SHIFT);
  }

  /* CMSIS-DSP 4th-order Butterworth bandpass isolates the pulse waveform. */
  {
    float32_t in, out;

    biquad_ensure_init();

    in = (float32_t)((int32_t)red_value - (int32_t)spo2_state->red_dc_estimate);
    arm_biquad_cascade_df1_f32(&biquad_red, &in, &out, 1U);
    spo2_state->red_filter_state = (int32_t)(out + (out >= 0.0f ? 0.5f : -0.5f));

    in = (float32_t)((int32_t)ir_value - (int32_t)spo2_state->ir_dc_estimate);
    arm_biquad_cascade_df1_f32(&biquad_ir, &in, &out, 1U);
    spo2_state->ir_filter_state = (int32_t)(out + (out >= 0.0f ? 0.5f : -0.5f));
  }

  spo2_state->red_samples[sample_index] = red_value;
  spo2_state->ir_samples[sample_index] = ir_value;
  spo2_state->red_filtered_samples[sample_index] = spo2_state->red_filter_state;
  spo2_state->ir_filtered_samples[sample_index] = spo2_state->ir_filter_state;
  spo2_state->red_sum += red_value;
  spo2_state->ir_sum += ir_value;
  spo2_state->red_square_sum += max30102_square_u32(red_value);
  spo2_state->ir_square_sum += max30102_square_u32(ir_value);
  spo2_state->red_filtered_square_sum += max30102_square_i32(spo2_state->red_filter_state);
  spo2_state->ir_filtered_square_sum += max30102_square_i32(spo2_state->ir_filter_state);

  spo2_state->write_index++;
  if (spo2_state->write_index >= MAX30102_SPO2_WINDOW_SIZE)
  {
    spo2_state->write_index = 0U;
  }

  if (spo2_state->sample_count < MAX30102_SPO2_WINDOW_SIZE)
  {
    spo2_state->sample_count++;
  }

  /* total_samples 永不重置的全局累加器，给流式脉冲检测提供绝对采样编号参考。 */
  if (spo2_state->total_samples < 0xFFFFFFFFUL)
  {
    spo2_state->total_samples++;
  }
}

uint8_t max30102_spo2_get_latest_filtered(const MAX30102_SpO2_t *spo2_state,
                                          int32_t *red_filtered,
                                          int32_t *ir_filtered)
{
  uint16_t sample_index;

  if ((spo2_state == NULL) || (red_filtered == NULL) || (ir_filtered == NULL))
  {
    return 0U;
  }

  if (spo2_state->sample_count == 0U)
  {
    return 0U;
  }

  sample_index = (spo2_state->write_index == 0U) ?
                 (uint16_t)(MAX30102_SPO2_WINDOW_SIZE - 1U) :
                 (uint16_t)(spo2_state->write_index - 1U);
  *red_filtered = spo2_state->red_filtered_samples[sample_index];
  *ir_filtered = spo2_state->ir_filtered_samples[sample_index];
  return 1U;
}

uint8_t max30102_get_signal_metrics(const MAX30102_SpO2_t *spo2_state, MAX30102_SignalMetrics_t *metrics)
{
  uint16_t sample_count;
  uint64_t denominator;
  uint32_t red_pi_rms;
  uint32_t ir_pi_rms;

  if ((spo2_state == NULL) || (metrics == NULL))
  {
    return 0U;
  }

  if (spo2_state->sample_count < MAX30102_SPO2_MIN_VALID_SAMPLES)
  {
    return 0U;
  }

  sample_count = spo2_state->sample_count;
  metrics->red_dc = (uint32_t)(spo2_state->red_sum / sample_count);
  metrics->ir_dc = (uint32_t)(spo2_state->ir_sum / sample_count);
  metrics->red_ac_rms = max30102_calculate_window_rms(spo2_state->red_filtered_square_sum, sample_count);
  metrics->ir_ac_rms = max30102_calculate_window_rms(spo2_state->ir_filtered_square_sum, sample_count);

  /*
   * PI uses max(filtered RMS, raw centered RMS) as the AC source.
   * Filtered RMS decays after the DC tracker settles on a steady finger,
   * but the raw centered RMS retains the true pulse amplitude.
   * Filtered RMS is kept unchanged for pulse detection and SQ scoring.
   */
  red_pi_rms = max30102_calculate_centered_rms(spo2_state->red_sum,
                                               spo2_state->red_square_sum,
                                               sample_count);
  ir_pi_rms = max30102_calculate_centered_rms(spo2_state->ir_sum,
                                              spo2_state->ir_square_sum,
                                              sample_count);
  if (red_pi_rms < metrics->red_ac_rms)
  {
    red_pi_rms = metrics->red_ac_rms;
  }
  if (ir_pi_rms < metrics->ir_ac_rms)
  {
    ir_pi_rms = metrics->ir_ac_rms;
  }

  metrics->red_pi_x1000 = 0U;
  metrics->ir_pi_x1000 = 0U;

  denominator = metrics->red_dc;
  if (denominator != 0ULL)
  {
    uint64_t red_pi = (((uint64_t)red_pi_rms * MAX30102_SPO2_RATIO_SCALE) +
                       (denominator / 2ULL)) / denominator;
    if (red_pi > 0xFFFFULL)
    {
      red_pi = 0xFFFFULL;
    }

    metrics->red_pi_x1000 = (uint16_t)red_pi;
  }

  denominator = metrics->ir_dc;
  if (denominator != 0ULL)
  {
    uint64_t ir_pi = (((uint64_t)ir_pi_rms * MAX30102_SPO2_RATIO_SCALE) +
                      (denominator / 2ULL)) / denominator;
    if (ir_pi > 0xFFFFULL)
    {
      ir_pi = 0xFFFFULL;
    }

    metrics->ir_pi_x1000 = (uint16_t)ir_pi;
  }

  return 1U;
}

uint8_t max30102_calculate_signal_quality(const MAX30102_SpO2_t *spo2_state,
                                          const MAX30102_SignalMetrics_t *metrics,
                                          uint8_t *signal_quality)
{
  uint32_t quality_score;
  uint32_t red_rms_for_quality;
  uint32_t ir_rms_for_quality;
  uint32_t red_centered_rms;
  uint32_t ir_centered_rms;
  uint32_t max_ac_rms;
  uint32_t min_ac_rms;
  uint16_t max_pi;
  uint16_t min_pi;

  if ((spo2_state == NULL) || (metrics == NULL) || (signal_quality == NULL))
  {
    return 0U;
  }

  /*
   * filtered RMS is best for pulse detection, but during steady finger contact
   * it can become very small after the bandpass/DC trackers settle. For SQ only,
   * keep a conservative fallback from raw centered RMS so a still finger does
   * not look worse than the initial contact transient.
   */
  red_centered_rms = max30102_calculate_centered_rms(spo2_state->red_sum,
                                                     spo2_state->red_square_sum,
                                                     spo2_state->sample_count);
  ir_centered_rms = max30102_calculate_centered_rms(spo2_state->ir_sum,
                                                    spo2_state->ir_square_sum,
                                                    spo2_state->sample_count);
  red_rms_for_quality = (red_centered_rms > metrics->red_ac_rms) ?
                        red_centered_rms : metrics->red_ac_rms;
  ir_rms_for_quality = (ir_centered_rms > metrics->ir_ac_rms) ?
                       ir_centered_rms : metrics->ir_ac_rms;

  quality_score = 0U;
  quality_score += max30102_scale_score_u32(metrics->ir_pi_x1000, MAX30102_SIGNAL_QUALITY_IR_PI_GOOD, 30U);
  quality_score += max30102_scale_score_u32(metrics->red_pi_x1000, MAX30102_SIGNAL_QUALITY_RED_PI_GOOD, 20U);
  quality_score += max30102_scale_score_u32(ir_rms_for_quality, MAX30102_SIGNAL_QUALITY_IR_RMS_GOOD, 20U);
  quality_score += max30102_scale_score_u32(red_rms_for_quality, MAX30102_SIGNAL_QUALITY_RED_RMS_GOOD, 10U);
  quality_score += max30102_scale_score_u32(spo2_state->sample_count, MAX30102_SIGNAL_QUALITY_WINDOW_GOOD, 5U);

  max_pi = metrics->red_pi_x1000;
  min_pi = metrics->ir_pi_x1000;
  if (max_pi < min_pi)
  {
    max_pi = metrics->ir_pi_x1000;
    min_pi = metrics->red_pi_x1000;
  }

  if (max_pi != 0U)
  {
    quality_score += ((uint32_t)min_pi * 15U + (max_pi / 2U)) / max_pi;
  }
  else
  {
    max_ac_rms = red_rms_for_quality;
    min_ac_rms = ir_rms_for_quality;
    if (max_ac_rms < min_ac_rms)
    {
      max_ac_rms = ir_rms_for_quality;
      min_ac_rms = red_rms_for_quality;
    }

    if (max_ac_rms != 0U)
    {
      quality_score += ((uint32_t)min_ac_rms * 15U + (max_ac_rms / 2U)) / max_ac_rms;
    }
  }

  if (quality_score > 100U)
  {
    quality_score = 100U;
  }

  *signal_quality = (uint8_t)quality_score;
  return 1U;
}

/*
 * 根据窗口内 RED/IR 的 AC/DC 比值估算 SpO2。
 * 这里先求直流分量 DC，再求交流分量的 RMS，最后计算比值 R。
 * 与简单 max-min 相比，RMS 对偶发毛刺更不敏感，结果通常更稳。
 */
uint8_t max30102_calculate_spo2(const MAX30102_SpO2_t *spo2_state, uint8_t *spo2_value)
{
  uint16_t sample_count;
  uint32_t red_dc;
  uint32_t ir_dc;
  uint32_t red_ac_rms;
  uint32_t ir_ac_rms;
  uint32_t ratio_milli;
  int32_t spo2_milli;
  int64_t ratio_square_term;
  uint64_t denominator;

  if ((spo2_state == NULL) || (spo2_value == NULL))
  {
    return 0U;
  }

  if (spo2_state->sample_count < MAX30102_SPO2_MIN_VALID_SAMPLES)
  {
    return 0U;
  }

  sample_count = spo2_state->sample_count;
  red_dc = (uint32_t)(spo2_state->red_sum / sample_count);
  ir_dc = (uint32_t)(spo2_state->ir_sum / sample_count);
  red_ac_rms = max30102_calculate_centered_rms(spo2_state->red_sum,
                                               spo2_state->red_square_sum,
                                               sample_count);
  ir_ac_rms = max30102_calculate_centered_rms(spo2_state->ir_sum,
                                              spo2_state->ir_square_sum,
                                              sample_count);

  /*
   * 过滤明显无效的情况：
   * - DC 太小：通常表示未正确接触或读数异常
   * - AC 太小：波形起伏不足，用来算比值会非常不稳定
   */
  if ((red_dc <= MAX30102_SPO2_MIN_DC) ||
      (ir_dc <= MAX30102_SPO2_MIN_DC) ||
      (red_ac_rms < MAX30102_SPO2_MIN_AC_RMS) ||
      (ir_ac_rms < MAX30102_SPO2_MIN_AC_RMS))
  {
    return 0U;
  }

  /*
   * R = (ACred / DCred) / (ACir / DCir)
   * 这里用 RMS 作为 AC 幅值，比简单 max-min 更抗偶发毛刺。
   * 二次拟合系数来自 MAX3010x 系列常见开源示例，适合做演示版估算。
   */
  denominator = (uint64_t)ir_ac_rms * (uint64_t)red_dc;
  if (denominator == 0ULL)
  {
    return 0U;
  }

  ratio_milli = (uint32_t)((((uint64_t)red_ac_rms * (uint64_t)ir_dc * MAX30102_SPO2_RATIO_SCALE) +
                            (denominator / 2ULL)) / denominator);
  ratio_square_term = ((int64_t)ratio_milli * (int64_t)ratio_milli + 500LL) / 1000LL;
  spo2_milli = (int32_t)(94845LL + ((30354LL * (int64_t)ratio_milli + 500LL) / 1000LL) -
                         ((45060LL * ratio_square_term + 500LL) / 1000LL));

  if ((spo2_milli < 70000L) || (spo2_milli > 100000L))
  {
    return 0U;
  }

  *spo2_value = (uint8_t)((spo2_milli + 500L) / 1000L);
  return 1U;
}

/*
 * 根据 IR 波形检测峰值，并用相邻峰值的间隔估算心率。
 * 核心思路：
 * 1. 先计算窗口内的直流分量和波形幅度；
 * 2. 再用简单平滑后的波形寻找局部峰值；
 * 3. 统计相邻峰之间的平均样本间隔，换算成 BPM。
 */
uint8_t max30102_calculate_bpm(const MAX30102_SpO2_t *spo2_state, uint8_t *bpm_value)
{
  return max30102_calculate_bpm_with_pulse(spo2_state, bpm_value, NULL);
}

/*
 * BPM + 脉冲信息联合计算：
 * 与 max30102_calculate_bpm 使用相同的峰值检测逻辑，但额外通过 pulse_info
 * 向上层流式脉冲检测器 (app_stream_pulse_update) 提供交叉校验数据，
 * 解决原始窗口峰值算法在某些波形形态下漏检 IBI 的问题。
 */
uint8_t max30102_calculate_bpm_with_pulse(const MAX30102_SpO2_t *spo2_state,
                                          uint8_t *bpm_value,
                                          MAX30102_PulseInfo_t *pulse_info)
{
  uint16_t i;
  uint16_t sample_index;
  uint16_t sample_count;
  uint16_t start_index;
  uint16_t peak_positions[MAX30102_BPM_MAX_PEAK_COUNT];
  uint16_t peak_count;
  int32_t ir_min;
  int32_t ir_second_min;
  int32_t ir_max;
  int32_t ir_second_max;
  uint32_t amplitude;
  uint32_t threshold_amplitude;
  uint32_t amplitude_cap;
  uint32_t ir_ac_rms;
  int32_t threshold;
  int32_t prominence_threshold;
  int32_t edge_threshold;
  int32_t prev2;
  int32_t prev1;
  int32_t current;
  int32_t next1;
  int32_t next2;
  int32_t smoothed_prev1;
  int32_t smoothed_current;
  int32_t smoothed_next;
  int32_t left_min;
  int32_t right_min;
  int32_t prominence;
  int32_t left_edge;
  int32_t right_edge;
  uint16_t min_peak_distance;
  uint16_t max_interval_samples;
  uint32_t interval_sum;
  uint16_t interval_count;
  uint16_t min_interval;
  uint16_t max_interval;
  uint32_t average_interval;
  uint32_t allowed_jitter;
  uint32_t bpm_estimate;
  uint32_t peak_span_samples;
  uint16_t interval_samples;
  uint16_t latest_interval_samples;
  uint16_t latest_peak_position;
  uint32_t oldest_sample_number;
  int32_t filtered_value;
  uint8_t fallback_used = 0U;

  if (pulse_info != NULL)
  {
    (void)memset(pulse_info, 0, sizeof(*pulse_info));
  }

  if ((spo2_state == NULL) || (bpm_value == NULL))
  {
    return 0U;
  }

  if (spo2_state->sample_count < MAX30102_BPM_MIN_VALID_SAMPLES)
  {
    return 0U;
  }

  sample_count = spo2_state->sample_count;
  ir_min = INT32_MAX;
  ir_second_min = INT32_MAX;
  ir_max = INT32_MIN;
  ir_second_max = INT32_MIN;
  start_index = (sample_count < MAX30102_SPO2_WINDOW_SIZE) ? 0U : spo2_state->write_index;
  sample_index = start_index;

  for (i = 0U; i < sample_count; i++)
  {
    filtered_value = spo2_state->ir_filtered_samples[sample_index];

    if (filtered_value < ir_min)
    {
      ir_second_min = ir_min;
      ir_min = filtered_value;
    }
    else if (filtered_value < ir_second_min)
    {
      ir_second_min = filtered_value;
    }

    if (filtered_value > ir_max)
    {
      ir_second_max = ir_max;
      ir_max = filtered_value;
    }
    else if (filtered_value > ir_second_max)
    {
      ir_second_max = filtered_value;
    }

    sample_index++;
    if (sample_index >= MAX30102_SPO2_WINDOW_SIZE)
    {
      sample_index = 0U;
    }
  }

  amplitude = (uint32_t)((int64_t)ir_max - (int64_t)ir_min);

  /*
   * 若波形起伏太小，通常说明手指未放稳、压得太轻，
   * 或此时还没有足够明显的脉搏波。
   */
  /* 允许较小的脉搏波摆动进入 BPM 检测，避免弱信号被完全过滤。 */
  if (amplitude < MAX30102_BPM_MIN_AMPLITUDE)
  {
    return 0U;
  }

  ir_ac_rms = max30102_calculate_window_rms(spo2_state->ir_filtered_square_sum, sample_count);
  if (ir_ac_rms < MAX30102_BPM_MIN_AC_RMS)
  {
    return 0U;
  }

  if (ir_second_min == INT32_MAX)
  {
    ir_second_min = ir_min;
  }

  if (ir_second_max == INT32_MIN)
  {
    ir_second_max = ir_max;
  }

  /* 去掉一个极端值后计算幅度，并用 RMS 做上限约束，减小尖峰干扰。 */
  threshold_amplitude = (uint32_t)((int64_t)ir_second_max - (int64_t)ir_second_min);
  amplitude_cap = ir_ac_rms * MAX30102_BPM_RANGE_CAP_RMS_FACTOR;
  if (threshold_amplitude > amplitude_cap)
  {
    threshold_amplitude = amplitude_cap;
  }

  if (threshold_amplitude < MAX30102_BPM_MIN_AMPLITUDE)
  {
    threshold_amplitude = MAX30102_BPM_MIN_AMPLITUDE;
  }

  threshold = (int32_t)((threshold_amplitude * MAX30102_BPM_THRESHOLD_NUM) /
                        MAX30102_BPM_THRESHOLD_DEN);
  if (threshold < (int32_t)MAX30102_BPM_MIN_THRESHOLD)
  {
    threshold = MAX30102_BPM_MIN_THRESHOLD;
  }

  prominence_threshold = (int32_t)((threshold_amplitude * MAX30102_BPM_PROMINENCE_NUM) /
                                   MAX30102_BPM_PROMINENCE_DEN);
  if (prominence_threshold < (int32_t)MAX30102_BPM_MIN_PROMINENCE)
  {
    prominence_threshold = MAX30102_BPM_MIN_PROMINENCE;
  }

  if (prominence_threshold < threshold)
  {
    prominence_threshold = threshold;
  }

  edge_threshold = (int32_t)(ir_ac_rms / 5U);
  if (edge_threshold < (int32_t)MAX30102_BPM_MIN_EDGE_DELTA)
  {
    edge_threshold = MAX30102_BPM_MIN_EDGE_DELTA;
  }

  /*
   * 约束两个峰之间的最小样本间隔，避免一次脉搏中的噪声被重复算成多个峰。
   * 这里按最大心率 220 BPM 反推得到最小峰距。
   */
  /* 峰距和最终 BPM 验证使用相同的上下限（35~220 BPM）。 */
  min_peak_distance = (uint16_t)((MAX30102_ALGO_SAMPLE_RATE_HZ * 60U) / MAX30102_BPM_MAX_RESULT);
  if (min_peak_distance < 10U)
  {
    min_peak_distance = 10U;
  }

  max_interval_samples = (uint16_t)((MAX30102_ALGO_SAMPLE_RATE_HZ * 60U) / MAX30102_BPM_MIN_RESULT);

  peak_count = 0U;

  if (sample_count < 5U)
  {
    return 0U;
  }

  sample_index = start_index;
  prev2 = spo2_state->ir_filtered_samples[sample_index];
  sample_index++;
  if (sample_index >= MAX30102_SPO2_WINDOW_SIZE)
  {
    sample_index = 0U;
  }

  prev1 = spo2_state->ir_filtered_samples[sample_index];
  sample_index++;
  if (sample_index >= MAX30102_SPO2_WINDOW_SIZE)
  {
    sample_index = 0U;
  }

  current = spo2_state->ir_filtered_samples[sample_index];
  sample_index++;
  if (sample_index >= MAX30102_SPO2_WINDOW_SIZE)
  {
    sample_index = 0U;
  }

  next1 = spo2_state->ir_filtered_samples[sample_index];
  sample_index++;
  if (sample_index >= MAX30102_SPO2_WINDOW_SIZE)
  {
    sample_index = 0U;
  }

  next2 = spo2_state->ir_filtered_samples[sample_index];
  sample_index++;
  if (sample_index >= MAX30102_SPO2_WINDOW_SIZE)
  {
    sample_index = 0U;
  }

  for (i = 2U; i + 2U < sample_count; i++)
  {
    smoothed_prev1 = (prev2 + prev1 + current) / 3;
    smoothed_current = (prev1 + current + next1) / 3;
    smoothed_next = (current + next1 + next2) / 3;
    left_min = prev2;
    if (prev1 < left_min)
    {
      left_min = prev1;
    }

    right_min = next1;
    if (next2 < right_min)
    {
      right_min = next2;
    }

    prominence = smoothed_current - ((left_min > right_min) ? left_min : right_min);
    left_edge = smoothed_current - smoothed_prev1;
    right_edge = smoothed_current - smoothed_next;

    if ((smoothed_current > threshold) &&
        (smoothed_current >= smoothed_prev1) &&
        (smoothed_current >= smoothed_next) &&
        (prominence >= prominence_threshold) &&
        ((left_edge >= edge_threshold) || (right_edge >= edge_threshold)))
    {
      if ((peak_count == 0U) ||
          ((i - peak_positions[peak_count - 1U]) >= min_peak_distance))
      {
        if (peak_count < MAX30102_BPM_MAX_PEAK_COUNT)
        {
          peak_positions[peak_count] = i;
          peak_count++;
        }
      }
    }

    if ((i + 3U) < sample_count)
    {
      prev2 = prev1;
      prev1 = current;
      current = next1;
      next1 = next2;
      next2 = spo2_state->ir_filtered_samples[sample_index];
      sample_index++;
      if (sample_index >= MAX30102_SPO2_WINDOW_SIZE)
      {
        sample_index = 0U;
      }
    }
  }

  if (peak_count < 2U)
  {
    return 0U;
  }

  interval_sum = 0U;
  interval_count = 0U;
  min_interval = 0xFFFFU;
  max_interval = 0U;

  for (i = 1U; i < peak_count; i++)
  {
    interval_samples = (uint16_t)(peak_positions[i] - peak_positions[i - 1U]);

    /*
     * 仅保留落在合理心率区间内的峰距，避免偶发异常峰把平均值拉偏。
     * 100 Hz 下，峰距范围由 MAX30102_ALGO_SAMPLE_RATE_HZ 动态换算。
     */
    if ((interval_samples >= min_peak_distance) && (interval_samples <= max_interval_samples))
    {
      interval_sum += interval_samples;
      interval_count++;

      if (interval_samples < min_interval)
      {
        min_interval = interval_samples;
      }

      if (interval_samples > max_interval)
      {
        max_interval = interval_samples;
      }
    }
  }

  if (interval_count == 0U)
  {
    if (peak_count < 2U)
    {
      return 0U;
    }

    peak_span_samples = (uint32_t)(peak_positions[peak_count - 1U] - peak_positions[0]);
    if (peak_span_samples == 0U)
    {
      return 0U;
    }

    interval_sum = peak_span_samples;
    interval_count = (uint16_t)(peak_count - 1U);
    fallback_used = 1U;
  }

  if (interval_count >= 3U)
  {
    average_interval = (interval_sum + (interval_count / 2U)) / interval_count;
    allowed_jitter = (average_interval * MAX30102_BPM_MAX_INTERVAL_JITTER) / 100U;
    if (allowed_jitter < 3U)
    {
      allowed_jitter = 3U;
    }

    if ((uint32_t)(max_interval - min_interval) > allowed_jitter)
    {
      peak_span_samples = (uint32_t)(peak_positions[peak_count - 1U] - peak_positions[0]);
      if (peak_span_samples > 0U)
      {
        interval_sum = peak_span_samples;
        interval_count = (uint16_t)(peak_count - 1U);
        fallback_used = 1U;
      }
      else
      {
        return 0U;
      }
    }
  }

  if ((fallback_used == 0U) &&
      (interval_count >= 3U) &&
      (interval_sum > ((uint32_t)min_interval + (uint32_t)max_interval)))
  {
    interval_sum -= (uint32_t)min_interval + (uint32_t)max_interval;
    interval_count -= 2U;
  }

  bpm_estimate = (uint32_t)(((uint32_t)60U * MAX30102_ALGO_SAMPLE_RATE_HZ * interval_count +
                             (interval_sum / 2U)) / interval_sum);

  if ((bpm_estimate < MAX30102_BPM_MIN_RESULT) ||
      (bpm_estimate > MAX30102_BPM_MAX_RESULT))
  {
    return 0U;
  }

  /* 从窗口峰值中提取最近一次有效心搏信息，供上层 app_stream_pulse_update
   * 做 IBI/HRV 输入——与流式检测互补，避免单一检测路径漏掉心搏。 */
  if (pulse_info != NULL)
  {
    latest_interval_samples = 0U;
    latest_peak_position = 0U;

    for (i = (uint16_t)(peak_count - 1U); i > 0U; i--)
    {
      interval_samples = (uint16_t)(peak_positions[i] - peak_positions[i - 1U]);
      if ((interval_samples >= min_peak_distance) && (interval_samples <= max_interval_samples))
      {
        latest_interval_samples = interval_samples;
        latest_peak_position = peak_positions[i];
        break;
      }
    }

    /* 在峰值 ±10 样本窗口内搜索局部最小值，估算心搏幅度 peak-local_min，
     * 该幅度值供 RIAV 呼吸率算法使用（呼吸调制体现在心搏幅度的周期性变化上）。 */
    if ((latest_interval_samples != 0U) && (spo2_state->total_samples >= sample_count))
    {
      uint16_t peak_window_start;
      uint16_t peak_window_end;
      uint16_t peak_sample_index;
      uint16_t peak_offset;
      int32_t peak_value;
      int32_t local_min;
      int32_t local_value;

      peak_window_start = (latest_peak_position > MAX30102_BPM_PEAK_AMP_HALF_WINDOW) ?
                          (uint16_t)(latest_peak_position - MAX30102_BPM_PEAK_AMP_HALF_WINDOW) : 0U;
      peak_window_end = (uint16_t)(latest_peak_position + MAX30102_BPM_PEAK_AMP_HALF_WINDOW);
      if (peak_window_end >= sample_count)
      {
        peak_window_end = (uint16_t)(sample_count - 1U);
      }

      peak_sample_index = (uint16_t)((start_index + latest_peak_position) % MAX30102_SPO2_WINDOW_SIZE);
      peak_value = spo2_state->ir_filtered_samples[peak_sample_index];
      local_min = peak_value;

      for (peak_offset = peak_window_start; peak_offset <= peak_window_end; peak_offset++)
      {
        peak_sample_index = (uint16_t)((start_index + peak_offset) % MAX30102_SPO2_WINDOW_SIZE);
        local_value = spo2_state->ir_filtered_samples[peak_sample_index];
        if (local_value < local_min)
        {
          local_min = local_value;
        }
      }

      oldest_sample_number = spo2_state->total_samples - sample_count;
      pulse_info->beat_valid = 1U;
      pulse_info->interval_samples = latest_interval_samples;
      pulse_info->latest_ibi_ms = (uint16_t)(((uint32_t)latest_interval_samples * 1000U +
                                              (MAX30102_ALGO_SAMPLE_RATE_HZ / 2U)) /
                                             MAX30102_ALGO_SAMPLE_RATE_HZ);
      pulse_info->latest_peak_sample = oldest_sample_number + latest_peak_position;
      pulse_info->beat_amplitude = (peak_value > local_min) ? (uint32_t)(peak_value - local_min) : 0U;
    }
  }

  *bpm_value = (uint8_t)bpm_estimate;
  return 1U;
}

/* --------------------------------------------------------------------------
 * 基于 FFT 的自相关心率估算（Wiener–Khinchin 定理）
 *
 * 原理：信号的自相关 = 功率谱的逆傅里叶变换
 *       自相关峰值位置 = 信号的基频周期
 *
 * 算法流程：
 *   1. 从环形缓冲区复制 256 个滤波后的 IR 样本
 *   2. 减均值 + Hann 窗 → 抑制频谱泄漏
 *   3. 正变换 RFFT → 功率谱 |X[k]|^2
 *   4. 逆变换 RFFT → 自相关序列
 *   5. 在 [LAG_MIN, LAG_MAX] 范围内搜索自相关峰值
 *   6. BPM = 采样率 × 60 / 滞后值
 *
 * 相比时域峰值检测，自相关法对弱灌注 / 噪声信号更鲁棒，
 * 但对突变心率（如心律不齐）响应较慢（需要 256 点 ≈ 2.56 秒窗口）。
 * -------------------------------------------------------------------------- */
#define FFT_SIZE  256U                    /* 256 点 RFFT，窗口 = 2.56 秒 @ 100 Hz */
#define LAG_MIN   27U                    /* 100 * 60 / 27 ~= 222 BPM */
#define LAG_MAX   171U                   /* 100 * 60 / 171 ~= 35 BPM */
#define AUTOCORR_MIN_PEAK_RATIO 0.08f

static float32_t fft_in[FFT_SIZE];
static float32_t fft_out[FFT_SIZE];
static arm_rfft_fast_instance_f32 rfft_inst;
static uint8_t rfft_initialized = 0U;

static void autocorr_ensure_init(void)
{
  if (rfft_initialized == 0U)
  {
    arm_rfft_fast_init_f32(&rfft_inst, FFT_SIZE);
    rfft_initialized = 1U;
  }
}

uint8_t max30102_autocorr_bpm(const MAX30102_SpO2_t *spo2_state, uint8_t *bpm)
{
  uint16_t i;
  uint16_t sample_count;
  uint16_t start_index;
  uint16_t sample_index;
  float32_t mean_val;
  uint32_t lag;
  float32_t max_val;
  uint32_t max_lag;
  uint32_t bpm_estimate;

  if ((spo2_state == NULL) || (bpm == NULL))
  {
    return 0U;
  }

  sample_count = spo2_state->sample_count;
  if (sample_count < FFT_SIZE)
  {
    return 0U;
  }

  autocorr_ensure_init();

  /* 从环形缓冲区复制滤波后的 IR 样本到 FFT 输入。 */
  start_index = spo2_state->write_index;
  sample_index = start_index;
  for (i = 0U; i < FFT_SIZE; i++)
  {
    fft_in[i] = (float32_t)spo2_state->ir_filtered_samples[sample_index];
    sample_index++;
    if (sample_index >= MAX30102_SPO2_WINDOW_SIZE)
    {
      sample_index = 0U;
    }
  }

  /* 减均值 + Hann 窗，抑制 DC 分量和频谱泄漏。 */
  arm_mean_f32(fft_in, FFT_SIZE, &mean_val);
  for (i = 0U; i < FFT_SIZE; i++)
  {
    fft_in[i] -= mean_val;
    fft_in[i] *= 0.5f * (1.0f - arm_cos_f32(2.0f * 3.14159265f * (float32_t)i
                                             / (float32_t)(FFT_SIZE - 1U)));
  }

  /* 正变换 RFFT：fft_in → fft_out（CMSIS-DSP packed complex 格式）。 */
  arm_rfft_fast_f32(&rfft_inst, fft_in, fft_out, 0U);

  /* 计算功率谱 |X[k]|^2。
   * CMSIS-DSP RFFT 输出布局：[Re0, Re(N/2), Re1, Im1, Re2, Im2, ...]
   * 其中 Re0 = DC 分量，Re(N/2) = Nyquist 分量。 */
  {
    float32_t re, im;

    re = fft_out[0];
    fft_out[0] = re * re;
    re = fft_out[1];
    fft_out[1] = re * re;

    for (i = 2U; i < FFT_SIZE; i += 2U)
    {
      re = fft_out[i];
      im = fft_out[i + 1U];
      fft_out[i]       = re * re + im * im;
      fft_out[i + 1U]  = 0.0f;
    }
  }

  /* 逆变换 RFFT 输出到 fft_in（此版本 CMSIS-DSP 的 RFFT 逆变换不支持真正的
   * in-place 操作，需要独立的输出缓冲区）。 */
  arm_rfft_fast_f32(&rfft_inst, fft_out, fft_in, 1U);

  /* 归一化：RFFT 逆变换不含缩放因子，需除以 FFT_SIZE。 */
  for (i = 0U; i < FFT_SIZE; i++)
  {
    fft_in[i] /= (float32_t)FFT_SIZE;
  }

  /* 在有效滞后范围内搜索自相关峰值。 */
  max_val = -1e30f;
  max_lag = LAG_MIN;

  for (lag = LAG_MIN; lag <= LAG_MAX; lag++)
  {
    if (fft_in[lag] > max_val)
    {
      max_val = fft_in[lag];
      max_lag = lag;
    }
  }

  /* 自相关峰度过弱则拒绝；真实 PPG 波形较弱时不要过早卡死 BPM。 */
  if (max_val < (fft_in[0] * AUTOCORR_MIN_PEAK_RATIO))
  {
    return 0U;
  }

  bpm_estimate = (MAX30102_ALGO_SAMPLE_RATE_HZ * 60U) / max_lag;

  if ((bpm_estimate < MAX30102_BPM_MIN_RESULT) ||
      (bpm_estimate > MAX30102_BPM_MAX_RESULT))
  {
    return 0U;
  }

  *bpm = (uint8_t)bpm_estimate;
  return 1U;
}
