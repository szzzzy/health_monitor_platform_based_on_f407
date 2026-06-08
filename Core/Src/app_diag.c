/**
  ******************************************************************************
  * @file    app_diag.c
  * @brief   Crash diagnostics — RTC BKP persistence + periodic liveness snapshot
  *
  * BKP register map (20 x 32-bit, survives warm / IWDG reset):
  *   DR1  crash magic     — 0xCA7A570F = valid crash record
  *   DR2  crash info      — source[7:0] | task[15:8] | phase[23:16]
  *   DR3  crash tick      — uwTick snapshot (0 = unavailable)
  *   DR4  reboot_count    — shared counter
  *   DR5  liveness magic  — 0x11FE5AFE = valid liveness snapshot
  *   DR6  max_phase + max_hb[15:0] LSB
  *   DR7  ui_phase  + sd_phase
  *   DR8  wdt_phase + wdt_last_refresh_tick LSB
  *   DR9  ui_hb[15:0] LSB + max_hb[31:16] upper
  *   DR10 min_stack_hwm   — smallest free stack (words) across tasks
  *
  * 所有写 BKP 的操作均使用直接寄存器访问，不调用 HAL。
  * Crash capture 可在 fault handler / assert / HardFault 中安全调用。
  ******************************************************************************
  */

#include "app_diag.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

/* ---- 内部: 备份域写使能 (仅直接寄存器, 无 HAL 依赖) ---- */
static void diag_enable_bkp_write(void)
{
    if ((PWR->CR & PWR_CR_DBP) == 0U)
    {
        PWR->CR |= PWR_CR_DBP;
        /* 等待写生效 (reference manual: 等待直到 DBP 位读回为 1) */
        while ((PWR->CR & PWR_CR_DBP) == 0U) { }
    }
}

/* ---- API: 初始化 (boot 时调用一次) ---- */
void APP_Diag_Init(void)
{
    diag_enable_bkp_write();
}

/* ---- API: 崩溃捕获 (fault / assert / stack overflow / Error_Handler) ----
 *
 * 仅使用直接寄存器写，无 HAL 调用，无 printf，无动态内存。
 * 可安全运行于 fault handler / NMI / HardFault 上下文中。 */
void APP_Diag_CaptureCrash(uint8_t source, uint8_t task_id, uint8_t phase)
{
    uint32_t tick = 0UL;
    uint32_t packed;
    uint32_t prev;

    /* 尝试读取 uwTick — 若 SysTick 不可用则为 0 */
    {
        volatile uint32_t *p_tick = (volatile uint32_t *)0x20000000UL; /* dummy */
        (void)p_tick;
        tick = uwTick;
    }

    /* 确保备份域可写 (fault handler 可能在 PWR 未初始化时触发) */
    if ((PWR->CR & PWR_CR_DBP) == 0U)
    {
        PWR->CR |= PWR_CR_DBP;
    }

    packed = ((uint32_t)source)
           | (((uint32_t)task_id) << 8U)
           | (((uint32_t)phase) << 16U);

    prev = RTC->BKP4R;
    RTC->BKP4R = prev + 1UL;

    RTC->BKP1R = APP_DIAG_BKP_MAGIC_CRASH;
    RTC->BKP2R = packed;
    RTC->BKP3R = tick;

    __DSB();
    __ISB();
}

/* ---- API: 崩溃记录查询/清除/读取 ---- */
uint8_t APP_Diag_HasCrashRecord(void)
{
    return (RTC->BKP1R == APP_DIAG_BKP_MAGIC_CRASH) ? 1U : 0U;
}

void APP_Diag_ClearCrash(void)
{
    diag_enable_bkp_write();
    RTC->BKP1R = 0UL;
    RTC->BKP2R = 0UL;
    RTC->BKP3R = 0UL;
}

void APP_Diag_ReadCrashToAppState(AppState_t *app)
{
    uint32_t packed;

    if (app == NULL) return;

    app->reset_flags = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF;

    app->reboot_count = RTC->BKP4R;

    if (APP_Diag_HasCrashRecord() != 0U)
    {
        packed = RTC->BKP2R;
        app->crash_flag   = 1U;
        app->crash_source = (uint8_t)(packed & 0xFFU);
        app->crash_task   = (uint8_t)((packed >> 8U) & 0xFFU);
        app->crash_phase  = (uint8_t)((packed >> 16U) & 0xFFU);
        app->crash_tick   = RTC->BKP3R;
        APP_Diag_ClearCrash();
    }
    else
    {
        app->crash_flag   = 0U;
        app->crash_source = DIAG_CRASH_NONE;
        app->crash_task   = 0U;
        app->crash_phase  = 0U;
        app->crash_tick   = 0UL;
    }

    /* Also read liveness snapshot into AppState for post-mortem */
    if (RTC->BKP5R == APP_DIAG_BKP_MAGIC_LIVENESS)
    {
        uint32_t dr6 = RTC->BKP6R;
        uint32_t dr7 = RTC->BKP7R;
        uint32_t dr8 = RTC->BKP8R;
        uint32_t dr9 = RTC->BKP9R;

        app->max_task_phase = (uint8_t)(dr6 & 0xFFU);
        /* dr6[15:8] = max_hb LSB — used below */
        app->ui_task_phase  = (uint8_t)(dr7 & 0xFFU);
        app->sd_task_phase  = (uint8_t)((dr7 >> 8U) & 0xFFU);
        app->wdt_task_phase = (uint8_t)(dr8 & 0xFFU);
        /* Reconstruct max_task_heartbeat from DR6[15:8] + DR9[31:16] */
        app->max_task_heartbeat = ((dr6 >> 8U) & 0xFFU)
                                | ((dr9 >> 16U) & 0xFFFFU) << 8U;
        app->ui_task_heartbeat  = (dr9 & 0xFFFFU);
        /* DR10 = wdt refresh tick LSB */
        /* Store in crash_tick's spot if no crash */
        /* Keep HWM from BKP DR10 */
        RTC->BKP5R = 0UL; /* clear liveness after reading */
    }
}

/* ---- API: 周期性活体快照 (watchdogtask 调用, ~1s 间隔) ---- */
void APP_Diag_SaveLiveness(const AppState_t *app)
{
    uint32_t dr6, dr7, dr8, dr9;

    if (app == NULL) return;

    /* 打包各 task phase + heartbeat 到 BKP DR6-DR9 */
    dr6 = ((uint32_t)app->max_task_phase)
        | (((uint32_t)(app->max_task_heartbeat & 0xFFU)) << 8U);

    dr7 = ((uint32_t)app->ui_task_phase)
        | (((uint32_t)app->sd_task_phase) << 8U);

    dr8 = ((uint32_t)app->wdt_task_phase)
        | (((uint32_t)(HAL_GetTick() & 0xFFU)) << 8U);

    dr9 = ((uint32_t)(app->ui_task_heartbeat & 0xFFFFU))
        | (((uint32_t)((app->max_task_heartbeat >> 8U) & 0xFFFFU)) << 16U);

    RTC->BKP6R = dr6;
    RTC->BKP7R = dr7;
    RTC->BKP8R = dr8;
    RTC->BKP9R = dr9;
    RTC->BKP5R = APP_DIAG_BKP_MAGIC_LIVENESS;
}

/* ---- API: 栈水印采集 + BKP 持久化 ---- */
uint8_t APP_Diag_SampleStackHWM(uint16_t *hwm_max, uint16_t *hwm_ui,
                                 uint16_t *hwm_sd, uint16_t *hwm_wdt)
{
    extern osThreadId_t MAXtaskHandle;
    extern osThreadId_t UitaskHandle;
    extern osThreadId_t SDtaskHandle;
    extern osThreadId_t watchdogtaskHandle;

    if (hwm_max != NULL)  { *hwm_max  = 0U; }
    if (hwm_ui  != NULL)  { *hwm_ui   = 0U; }
    if (hwm_sd  != NULL)  { *hwm_sd   = 0U; }
    if (hwm_wdt != NULL)  { *hwm_wdt  = 0U; }

    if (hwm_max  != NULL && MAXtaskHandle  != NULL) {
        *hwm_max  = (uint16_t)uxTaskGetStackHighWaterMark((TaskHandle_t)MAXtaskHandle);
    }
    if (hwm_ui   != NULL && UitaskHandle   != NULL) {
        *hwm_ui   = (uint16_t)uxTaskGetStackHighWaterMark((TaskHandle_t)UitaskHandle);
    }
    if (hwm_sd   != NULL && SDtaskHandle   != NULL) {
        *hwm_sd   = (uint16_t)uxTaskGetStackHighWaterMark((TaskHandle_t)SDtaskHandle);
    }
    if (hwm_wdt  != NULL && watchdogtaskHandle != NULL) {
        *hwm_wdt  = (uint16_t)uxTaskGetStackHighWaterMark((TaskHandle_t)watchdogtaskHandle);
    }

    return 4U;
}

void APP_Diag_SaveStackHWMBkp(const uint16_t *hwm_max, const uint16_t *hwm_ui,
                               const uint16_t *hwm_sd, const uint16_t *hwm_wdt)
{
    uint16_t min_hwm = 0xFFFFU;
    uint16_t val;

    val = (hwm_max  != NULL) ? *hwm_max  : 0U;
    if (val > 0U && val < min_hwm) { min_hwm = val; }
    val = (hwm_ui   != NULL) ? *hwm_ui   : 0U;
    if (val > 0U && val < min_hwm) { min_hwm = val; }
    val = (hwm_sd   != NULL) ? *hwm_sd   : 0U;
    if (val > 0U && val < min_hwm) { min_hwm = val; }
    val = (hwm_wdt  != NULL) ? *hwm_wdt  : 0U;
    if (val > 0U && val < min_hwm) { min_hwm = val; }

    RTC->BKP10R = (uint32_t)min_hwm;
}
