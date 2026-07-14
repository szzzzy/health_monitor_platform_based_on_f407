/**
  ******************************************************************************
  * @file    app_settings.h
  * @brief   OLED 设置/运行信息页面渲染接口。
  ******************************************************************************
  */

#ifndef __APP_SETTINGS_H__
#define __APP_SETTINGS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"

/** @brief 根据当前 settings_sub_page 渲染设置/运行信息页面。 */
void app_settings_render_page(const AppState_t *app);

#ifdef __cplusplus
}
#endif

#endif /* __APP_SETTINGS_H__ */
