#ifndef __APP_PPG_SIGNAL_H__
#define __APP_PPG_SIGNAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"
#include "max30102.h"

#define APP_PPG_SIGNAL_FINGER_ON_CONFIRM_COUNT  8U
#define APP_PPG_SIGNAL_FINGER_OFF_CONFIRM_COUNT 75U
#define APP_PPG_SIGNAL_REACQUIRE_NOISE_IR       3000UL

void app_ppg_signal_init_state(AppState_t *app);
void app_ppg_signal_reset_envelope(void);
void app_ppg_signal_update_activity(AppState_t *app);
void app_ppg_signal_update_adaptive_thresholds(AppState_t *app,
                                               const MAX30102_Baseline_t *baseline);
uint8_t app_ppg_signal_is_raw_present(const AppState_t *app);
void app_ppg_signal_track_background_ir(AppState_t *app, MAX30102_Baseline_t *baseline);

#ifdef __cplusplus
}
#endif

#endif /* __APP_PPG_SIGNAL_H__ */
