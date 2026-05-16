#ifndef __APP_PPG_PULSE_H__
#define __APP_PPG_PULSE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"
#include "max30102.h"

void app_ppg_pulse_reset(void);
uint8_t app_ppg_pulse_update(AppState_t *app,
                             int32_t filtered_sample,
                             uint32_t total_samples,
                             MAX30102_PulseInfo_t *pulse_info);
void app_ppg_pulse_process_metrics(AppState_t *app,
                                   const MAX30102_PulseInfo_t *pulse_info,
                                   uint32_t total_samples);

#ifdef __cplusplus
}
#endif

#endif /* __APP_PPG_PULSE_H__ */
