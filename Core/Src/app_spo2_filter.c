#include "app_spo2_filter.h"

#include <string.h>

#define APP_SPO2_EMA_OLD_WEIGHT 3U
#define APP_SPO2_EMA_ROUNDING   2U
#define APP_SPO2_EMA_DIVISOR    4U

static struct
{
  uint8_t initialized;
  uint8_t value;
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
