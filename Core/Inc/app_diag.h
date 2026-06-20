#ifndef __APP_DIAG_H__
#define __APP_DIAG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "app_state.h"

/* 崩溃源 */
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

/* BKP 寄存器布局（RTC 备份域，热复位后保持）：
 * DR1 = 崩溃魔数 0xCA7A570F（仅在崩溃时设置）
 * DR2 = 崩溃源 | (任务 ID << 8) | (阶段码 << 16)
 * DR3 = 崩溃滴答（崩溃时滴答）
 * DR4 = reboot_count（共享崩溃+活跃计数）
 * DR5 = 活跃快照魔数 0x11FE5AFE
 * DR6 = max_phase[7:0] | max_hb_lsb[15:8]
 * DR7 = ui_phase[7:0] | sd_phase[15:8]
 * DR8 = wdt_phase[7:0] | wdt_refresh_tick_lsb[15:8]
 * DR9 = max_hb_msb[15:0] | ui_hb_lsb[31:16]
 * DR10= min_stack_hwm（所有任务中的最小空闲字） */

#define APP_DIAG_BKP_MAGIC_CRASH    0xCA7A570FUL
#define APP_DIAG_BKP_MAGIC_LIVENESS 0x11FE5AFEUL

/* ---- 公共 API ---- */
void APP_Diag_Init(void);
void APP_Diag_CaptureCrash(uint8_t source, uint8_t task_id, uint8_t phase);
void APP_Diag_ClearCrash(void);
uint8_t APP_Diag_HasCrashRecord(void);
void APP_Diag_ReadCrashToAppState(AppState_t *app);

/* 周期性活跃快照（每约 1 秒由看门狗任务调用） */
void APP_Diag_SaveLiveness(const AppState_t *app);

/* 栈高水位快照（由 UiTask 调用） */
uint8_t APP_Diag_SampleStackHWM(uint16_t *hwm_max, uint16_t *hwm_ui,
                                 uint16_t *hwm_sd, uint16_t *hwm_wdt);
/* 将最小高水位保存到 BKP（在 SampleStackHWM 后由 UiTask 调用） */
void APP_Diag_SaveStackHWMBkp(const uint16_t *hwm_max, const uint16_t *hwm_ui,
                               const uint16_t *hwm_sd, const uint16_t *hwm_wdt);

#ifdef __cplusplus
}
#endif

#endif /* __APP_DIAG_H__ */
