/**
  ******************************************************************************
  * @file    app_diag.c
  * @brief   崩溃诊断 — RTC BKP 持久化 + 周期性存活快照
  *
  * BKP 寄存器映射 (20 个 32 位寄存器，热复位 / IWDG 复位后保持):
  *   DR1  崩溃魔法值     — 0xCA7A570F = 有效崩溃记录
  *   DR2  崩溃信息       — 崩溃源[7:0] | 任务[15:8] | 阶段码[23:16]
  *   DR3  崩溃滴答       — uwTick 快照 (0 = 不可用)
  *   DR4  reboot_count    — 共享计数器
  *   DR5  存活魔法值     — 0x11FE5AFE = 有效存活快照
  *   DR6  max_phase + max_hb[15:0] 低位
  *   DR7  ui_phase  + sd_phase
  *   DR8  wdt_phase + wdt_last_refresh_tick 低位
  *   DR9  ui_hb[15:0] 低位 + max_hb[31:16] 高位
  *   DR10 min_stack_hwm   — 所有任务中最小的空闲栈 (字)
  *
  * 存活快照策略:
  *   看门狗任务 (~1 秒间隔) 调用 APP_Diag_SaveLiveness(),
  *   将四个任务 (MAX, UI, SD, WDT) 的当前阶段码和心跳计数器
  *   通过直接寄存器写入 (无 HAL) 打包到 BKP 寄存器 DR5–DR9。
  *   如果随后发生崩溃 (HardFault, NMI 等), 最近已知的任务阶段和心跳
  *   保存于电池支持的 SRAM 中。
  *   下次启动时, APP_Diag_ReadCrashToAppState() 检查 DR5 中的
  *   存活魔法值, 如果存在, 将快照解包到 AppState 中,
  *   与任何崩溃记录一起。这允许事后重建故障前各任务的活动状态。
  *
  * 崩溃捕获策略:
  *   stm32f4xx_it.c 中的 fault_get_task_phase() 通过按优先级
  *   (MAX > UI > WDT > SD) 扫描 AppState 阶段字段来推断可疑任务。
  *   然后调用 APP_Diag_CaptureCrash(), 传入崩溃源、任务 ID 和阶段码,
  *   将打包的崩溃记录写入 BKP DR1–DR4, 并在 DR4 中递增重启计数器。
  *   所有写入使用直接寄存器访问, 因此函数可在 HardFault/NMI 上下文中安全调用。
  *
  * 所有写 BKP 的操作均使用直接寄存器访问，不调用 HAL。
  * 崩溃捕获可在故障处理器 / 断言 / HardFault 中安全调用。
  ******************************************************************************
  */

#include "app_diag.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

/* ---- 启用备份域写访问 (直接寄存器访问, 无 HAL) ---- */
static void diag_enable_bkp_write(void)
{
    if ((PWR->CR & PWR_CR_DBP) == 0U)
    {
        PWR->CR |= PWR_CR_DBP;
        /* 等待写生效 (参考手册: 等待直到 DBP 位读回为 1) */
        while ((PWR->CR & PWR_CR_DBP) == 0U) { }
    }
}

/**
 ******************************************************************************
 * @brief  初始化诊断子系统 (启用 BKP 写访问)。
 * @note   必须在启动时调用一次, 在任何 BKP 寄存器访问之前。
 *         启用备份域写保护 (PWR_CR_DBP)。
 ******************************************************************************
 */
void APP_Diag_Init(void)
{
    diag_enable_bkp_write();
}

/**
 ******************************************************************************
 * @brief  将崩溃记录捕获到 BKP 寄存器 (可在故障上下文中安全调用)。
 * @param  source  崩溃源标识符 (DIAG_CRASH_*)。
 * @param  task_id 可疑任务 ID (0 = 未知)。
 * @param  phase   崩溃时的任务阶段码。
 * @note   仅使用直接寄存器写入 — 无 HAL, 无 printf, 无 malloc。
 *         可在 HardFault、NMI、MemManage、BusFault、UsageFault 中安全调用。
 *         将崩溃源/任务/阶段码写入 BKP2R, uwTick 快照写入 BKP3R,
 *         递增 BKP4R 中的重启计数器, 并在 BKP1R 中设置崩溃魔法值。
 ******************************************************************************
 */
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

    /* 确保备份域可写 (故障处理器可能在 PWR 未初始化时触发) */
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

/**
 ******************************************************************************
 * @brief  检查 BKP DR1 中是否存在有效的崩溃记录。
 * @return 如果存在崩溃魔法值 (0xCA7A570F), 返回 1, 否则返回 0。
 ******************************************************************************
 */
uint8_t APP_Diag_HasCrashRecord(void)
{
    return (RTC->BKP1R == APP_DIAG_BKP_MAGIC_CRASH) ? 1U : 0U;
}

/**
 ******************************************************************************
 * @brief  清除 BKP 寄存器 DR1-DR3 中的崩溃记录。
 * @note   启用 BKP 写访问, 清零 DR1-DR3。在崩溃记录被应用读取并消费后调用。
 ******************************************************************************
 */
void APP_Diag_ClearCrash(void)
{
    diag_enable_bkp_write();
    RTC->BKP1R = 0UL;
    RTC->BKP2R = 0UL;
    RTC->BKP3R = 0UL;
}

/**
 ******************************************************************************
 * @brief  将崩溃记录和存活快照读取到 AppState。
 * @param  app 指向 AppState 的指针 (可为 NULL)。
 * @note   从 RCC->CSR 读取复位标志, 从 BKP4R 读取重启计数。
 *         如果存在崩溃魔法值, 将崩溃源/任务/阶段码/滴答从
 *         BKP1R-BKP3R 解包到 AppState 的崩溃字段中, 并清除记录。
 *         如果存在存活魔法值, 将 DR6-DR9 解包到任务阶段
 *         和心跳字段中, 然后清除存活魔法值。
 ******************************************************************************
 */
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

    /* 也将存活快照读取到 AppState 用于事后分析 */
    if (RTC->BKP5R == APP_DIAG_BKP_MAGIC_LIVENESS)
    {
        uint32_t dr6 = RTC->BKP6R;
        uint32_t dr7 = RTC->BKP7R;
        uint32_t dr8 = RTC->BKP8R;
        uint32_t dr9 = RTC->BKP9R;

        app->max_task_phase = (uint8_t)(dr6 & 0xFFU);
        /* dr6[15:8] = max_hb 低16位 — 下面用 */
        app->ui_task_phase  = (uint8_t)(dr7 & 0xFFU);
        app->sd_task_phase  = (uint8_t)((dr7 >> 8U) & 0xFFU);
        app->wdt_task_phase = (uint8_t)(dr8 & 0xFFU);
        /* 从 DR6[15:8] + DR9[31:16] 重构 max_task_heartbeat */
        app->max_task_heartbeat = ((dr6 >> 8U) & 0xFFU)
                                | ((dr9 >> 16U) & 0xFFFFU) << 8U;
        app->ui_task_heartbeat  = (dr9 & 0xFFFFU);
        /* DR10 = wdt 刷新 tick 低8位 */
        /* 若无崩溃则存入 crash_tick 位置 */
        /* 保留 BKP DR10 中的 HWM */
        RTC->BKP5R = 0UL; /* 读取后清除存活快照 */
    }
}

/**
 ******************************************************************************
 * @brief  将所有任务阶段的周期性存活快照保存到 BKP DR5-DR9。
 * @param  app 指向 AppState 的指针, 提供阶段码和心跳。
 * @note   由看门狗任务以 ~1 秒间隔调用。将 MAX/UI/SD/WDT
 *         的阶段码和心跳打包到 BKP 寄存器。在 DR5 中设置存活
 *         魔法值 (0x11FE5AFE), 以便下次启动时识别有效快照。
 *         崩溃时, 这些寄存器保留最后已知的任务状态用于事后分析。
 ******************************************************************************
 */
void APP_Diag_SaveLiveness(const AppState_t *app)
{
    uint32_t dr6, dr7, dr8, dr9;

    if (app == NULL) return;

    /* 打包各任务阶段 + 心跳到 BKP DR6-DR9 */
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

/**
 ******************************************************************************
 * @brief  采样所有四个任务的栈高水位标记。
 * @param  hwm_max  输出: MAX 任务空闲栈 (字)。可为 NULL。
 * @param  hwm_ui   输出: UI 任务空闲栈 (字)。可为 NULL。
 * @param  hwm_sd   输出: SD 任务空闲栈 (字)。可为 NULL。
 * @param  hwm_wdt  输出: WDT 任务空闲栈 (字)。可为 NULL。
 * @return 已采样的任务数 (始终为 4)。
 * @note   对每个任务句柄调用 uxTaskGetStackHighWaterMark。
 *         任何输出指针可为 NULL 以跳过该任务。
 ******************************************************************************
 */
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

/**
 ******************************************************************************
 * @brief  将最小栈高水位标记持久化到 BKP DR10。
 * @param  hwm_max  MAX 任务 HWM (可为 NULL)。
 * @param  hwm_ui   UI 任务 HWM (可为 NULL)。
 * @param  hwm_sd   SD 任务 HWM (可为 NULL)。
 * @param  hwm_wdt  WDT 任务 HWM (可为 NULL)。
 * @note   找出所有四个任务中最小的空闲栈, 并将其写入
 *         RTC->BKP10R, 以便在热复位后保持。
 ******************************************************************************
 */
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
