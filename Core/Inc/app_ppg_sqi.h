#ifndef __APP_PPG_SQI_H__
#define __APP_PPG_SQI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"
#include "max30102_spo2.h"

#ifndef APP_PPG_SQI_EXPERIMENTAL
#define APP_PPG_SQI_EXPERIMENTAL 1U
#endif

#ifndef APP_PPG_SQI_GATE_OUTPUTS
#define APP_PPG_SQI_GATE_OUTPUTS 1U
#endif

#define APP_PPG_SQI_FLAG_LOW_PERFUSION   0x01U
#define APP_PPG_SQI_FLAG_MOTION          0x02U
#define APP_PPG_SQI_FLAG_BALANCE         0x04U
#define APP_PPG_SQI_FLAG_TRANSITION      0x08U
#define APP_PPG_SQI_FLAG_BEAT_UNSTABLE   0x10U

void app_ppg_sqi_reset(AppState_t *app);
void app_ppg_sqi_update_window(AppState_t *app,
                               const MAX30102_SignalMetrics_t *metrics,
                               uint8_t raw_quality);
void app_ppg_sqi_apply_quality_gate(AppState_t *app);
void app_ppg_sqi_note_accepted_beat(AppState_t *app,
                                    uint16_t ibi_ms,
                                    uint32_t beat_amplitude);
void app_ppg_sqi_note_ibi_reject(AppState_t *app);
void app_ppg_sqi_note_amp_reject(AppState_t *app);
uint8_t app_ppg_sqi_allows_hr(const AppState_t *app);
uint8_t app_ppg_sqi_allows_spo2(const AppState_t *app);
uint8_t app_ppg_sqi_allows_ptt(const AppState_t *app);

#ifdef __cplusplus
}
#endif

#endif /* __APP_PPG_SQI_H__ */
