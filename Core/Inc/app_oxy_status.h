#ifndef __APP_OXY_STATUS_H__
#define __APP_OXY_STATUS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"
#include "max30102.h"

void app_oxy_status_reset(AppState_t *app);
void app_oxy_status_clear_instant(AppState_t *app);
void app_oxy_status_update_from_metrics(AppState_t *app,
                                        const MAX30102_SignalMetrics_t *metrics,
                                        uint8_t raw_quality);
void app_oxy_status_update_quality(AppState_t *app, uint8_t raw_quality);

#ifdef __cplusplus
}
#endif

#endif /* __APP_OXY_STATUS_H__ */
