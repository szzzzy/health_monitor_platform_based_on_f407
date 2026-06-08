#ifndef __APP_DIAG_H__
#define __APP_DIAG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "app_state.h"

/* Crash sources */
#define DIAG_CRASH_NONE          0U
#define DIAG_CRASH_HARDFAULT     1U
#define DIAG_CRASH_MEMMANAGE     2U
#define DIAG_CRASH_BUSFAULT      3U
#define DIAG_CRASH_USAGEFAULT    4U
#define DIAG_CRASH_STACKOVF      5U
#define DIAG_CRASH_ASSERT        6U
#define DIAG_CRASH_MALLOCFAIL    7U
#define DIAG_CRASH_NMI           8U
#define DIAG_CRASH_ERROR_HANDLER 9U

/* BKP register layout (RTC backup domain, survives warm reset):
 * DR1 = crash magic 0xCA7A570F (set only on crash)
 * DR2 = crash_source | (task_id << 8) | (phase << 16)
 * DR3 = crash tick
 * DR4 = reboot_count (shared crash + liveness)
 * DR5 = liveness magic 0x11FE5AFE
 * DR6 = max_phase[7:0] | max_hb_lsb[15:8]
 * DR7 = ui_phase[7:0] | sd_phase[15:8]
 * DR8 = wdt_phase[7:0] | wdt_refresh_tick_lsb[15:8]
 * DR9 = max_hb_msb[15:0] | ui_hb_lsb[31:16]
 * DR10= min_stack_hwm (smallest free words across all tasks) */

#define APP_DIAG_BKP_MAGIC_CRASH    0xCA7A570FUL
#define APP_DIAG_BKP_MAGIC_LIVENESS 0x11FE5AFEUL

/* ---- Public API ---- */
void APP_Diag_Init(void);
void APP_Diag_CaptureCrash(uint8_t source, uint8_t task_id, uint8_t phase);
void APP_Diag_ClearCrash(void);
uint8_t APP_Diag_HasCrashRecord(void);
void APP_Diag_ReadCrashToAppState(AppState_t *app);

/* Periodic liveness snapshot (call from watchdogtask every ~1s) */
void APP_Diag_SaveLiveness(const AppState_t *app);

/* Stack HWM snapshot (call from Uitask) */
uint8_t APP_Diag_SampleStackHWM(uint16_t *hwm_max, uint16_t *hwm_ui,
                                 uint16_t *hwm_sd, uint16_t *hwm_wdt);
/* Save minimum HWM to BKP (call from Uitask after SampleStackHWM) */
void APP_Diag_SaveStackHWMBkp(const uint16_t *hwm_max, const uint16_t *hwm_ui,
                               const uint16_t *hwm_sd, const uint16_t *hwm_wdt);

#ifdef __cplusplus
}
#endif

#endif /* __APP_DIAG_H__ */
