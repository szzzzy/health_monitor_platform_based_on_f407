/**
  ******************************************************************************
  * @file    app_spo2_filter.c
  * @brief   SpO2 输出平滑滤波器
  *
  * 对原始 SpO2 值施加 EMA（指数移动平均）平滑，减少逐拍抖动。
  *
  * 关键设计：低 SQ / invalid / motion 时不推进 EMA 状态，
  * 防止无效数据污染平滑结果。旧值保留以供 UI 显示（带"?"标记），
  * 但 spo2_valid 清零。
  ******************************************************************************
  */

#include "app_spo2_filter.h"

#include <string.h>

/* EMA 平滑参数：新值权重 25%，旧值权重 75% */
#define APP_SPO2_EMA_OLD_WEIGHT 3U   /* 旧值乘数 */
#define APP_SPO2_EMA_ROUNDING   2U   /* 四舍五入项 */
#define APP_SPO2_EMA_DIVISOR    4U   /* 归一化除数 */

/* EMA 内部状态 */
static struct
{
  uint8_t initialized;  /* 1 = 已收到首帧有效值 */
  uint8_t value;        /* 当前平滑后的 SpO2 值 */
} spo2_smooth_state;

void app_spo2_filter_reset(AppState_t *app)
{
  (void)memset(&spo2_smooth_state, 0, sizeof(spo2_smooth_state));

  if (app == NULL)
  {
    return;
  }

  app->spo2_valid = 0U;
  app->spo2_value = 0U;
}

void app_spo2_filter_update_output(AppState_t *app,
                                   uint8_t raw_spo2_valid,
                                   uint8_t raw_spo2_value)
{
  uint16_t smoothed_value;

  if (app == NULL)
  {
    return;
  }

  /*
   * 关键语义：raw_spo2_valid==0 时不推进 EMA，不更新 app->spo2_value。
   * 旧值保留以供 UI 显示（带 "?" 标记），但 spo2_valid 清零。
   * 这防止低 SQ / invalid / motion 污染平滑状态。
   */
  if (raw_spo2_valid == 0U)
  {
    app->spo2_valid = 0U;
    return;
  }

  if (spo2_smooth_state.initialized == 0U)
  {
    /* 首帧有效值：直接初始化 EMA 状态，不做平滑。 */
    spo2_smooth_state.value = raw_spo2_value;
    spo2_smooth_state.initialized = 1U;
  }
  else
  {
    /* EMA 公式: next = (old * 3 + new + 2) / 4  （约 75% 旧值 + 25% 新值）。 */
    smoothed_value = (uint16_t)(((uint16_t)spo2_smooth_state.value * APP_SPO2_EMA_OLD_WEIGHT) +
                                (uint16_t)raw_spo2_value + APP_SPO2_EMA_ROUNDING);
    spo2_smooth_state.value = (uint8_t)(smoothed_value / APP_SPO2_EMA_DIVISOR);
  }

  app->spo2_value = spo2_smooth_state.value;
  app->spo2_valid = 1U;
}
