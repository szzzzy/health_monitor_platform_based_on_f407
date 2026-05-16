#ifndef __APP_BPM_FILTER_H__
#define __APP_BPM_FILTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"

void app_bpm_filter_reset(AppState_t *app);
uint8_t app_bpm_filter_update(AppState_t *app, uint8_t raw_bpm_valid, uint8_t raw_bpm_value);

#ifdef __cplusplus
}
#endif

#endif /* __APP_BPM_FILTER_H__ */
