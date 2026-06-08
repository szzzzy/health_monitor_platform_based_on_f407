#ifndef __APP_RTOS_H__
#define __APP_RTOS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"

void app_rtos_bind_state(AppState_t *state);
void app_rtos_notify_max_from_isr(void);
void app_rtos_mark_ready(void);
AppState_t *app_rtos_get_state(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_RTOS_H__ */
