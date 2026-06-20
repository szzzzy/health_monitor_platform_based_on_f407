/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c.h"
#include "max30102.h"
#include "usart.h"
#include "app_sd_card.h"
#include "app_diag.h"
#include "app_rtos.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 故障处理函数从 AppState 推断“最可能正在运行的任务”及其阶段码。
 * 因为故障处理函数在异常上下文中执行，不能调用 FreeRTOS API。
 * 策略：阶段码非零的任务 = 最可能是肇事者（优先级高的优先）。 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ---- 从 AppState 推断最可能的故障任务与阶段 ----
 *
 * 崩溃记录策略：
 *   故障处理函数 (HardFault、NMI、MemManage、BusFault、UsageFault) 运行在
 *   异常上下文，大多数 FreeRTOS/HAL API 都不安全。为了在不调用 OS 函数的
 *   前提下保留诊断信息，系统依赖看门狗任务周期性写入 RTC 备份寄存器的
 *   “活体快照”（见 app_diag.c）。故障发生时，各处理函数调用
 *   fault_get_task_phase() 读取 AppState 中最近保存的任务阶段，再传给
 *   APP_Diag_CaptureCrash()；后者仅用直接寄存器写入把崩溃记录持久化到 BKP SRAM。
 *
 *   本函数按优先级顺序扫描 AppState 的阶段字段
 *   (MAX > UI > WDT > SD，与 FreeRTOS 任务优先级一致)。第一个阶段码
 *   不是 IDLE 的任务被视为最可能的故障来源。若所有任务看起来都空闲，
 *   返回任务 ID 0（未知）。
 *
 *   这是尽力而为的启发式判断：任务可能在最后一次阶段码更新后到故障发生前
 *   被抢占，从而产生漏判。BKP 寄存器里的活体快照会为离线分析提供额外上下文。 */
static void fault_get_task_phase(uint8_t *task_id, uint8_t *phase)
{
    AppState_t *s = app_rtos_get_state();

    if ((task_id == NULL) || (phase == NULL)) return;

    if (s != NULL)
    {
        if (s->max_task_phase != PHASE_MAX_IDLE)
        {
            *task_id = 1U;  *phase = s->max_task_phase; return;
        }
        if (s->ui_task_phase != PHASE_UI_IDLE)
        {
            *task_id = 3U;  *phase = s->ui_task_phase; return;
        }
        if (s->wdt_task_phase != PHASE_WDT_IDLE)
        {
            *task_id = 2U;  *phase = s->wdt_task_phase; return;
        }
        if (s->sd_task_phase != PHASE_SD_IDLE)
        {
            *task_id = 4U;  *phase = s->sd_task_phase; return;
        }
    }
    *task_id = 0U;
    *phase  = 0U;
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

/* USER CODE BEGIN EV */
extern volatile uint8_t tim6_tick_flag;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  APP_Diag_CaptureCrash(DIAG_CRASH_NMI, 0U, 0U);
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  {
    uint8_t tid, ph;
    fault_get_task_phase(&tid, &ph);
    APP_Diag_CaptureCrash(DIAG_CRASH_HARDFAULT, tid, ph);
  }
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  {
    uint8_t tid, ph;
    fault_get_task_phase(&tid, &ph);
    APP_Diag_CaptureCrash(DIAG_CRASH_MEMMANAGE, tid, ph);
  }
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  {
    uint8_t tid, ph;
    fault_get_task_phase(&tid, &ph);
    APP_Diag_CaptureCrash(DIAG_CRASH_BUSFAULT, tid, ph);
  }
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  {
    uint8_t tid, ph;
    fault_get_task_phase(&tid, &ph);
    APP_Diag_CaptureCrash(DIAG_CRASH_USAGEFAULT, tid, ph);
  }
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* ---- DMA1 Stream5：USART2 接收 ---- */
void DMA1_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(huart2.hdmarx);
}

/* ---- USART2：DMA 接收空闲线检测 ---- */
void USART2_IRQHandler(void)
{
  if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET)
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);
    usart_set_dma_idle_flag();
  }
  HAL_UART_IRQHandler(&huart2);
}

/* ---- DMA1 Stream0：I2C1 接收 ---- */
void DMA1_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hi2c1.hdmarx);
}

/* ---- DMA1 Stream6：I2C1 发送 ---- */
void DMA1_Stream6_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hi2c1.hdmatx);
}

/* ---- I2C1 事件中断 ---- */
void I2C1_EV_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&hi2c1);
}

/* ---- I2C1 错误中断 ---- */
void I2C1_ER_IRQHandler(void)
{
  HAL_I2C_ER_IRQHandler(&hi2c1);
}

/* MAX30102 PPG_RDY 中断（PE5 下降沿）
 * 当 PE5 未连接到 MAX30102 INT 引脚，此 ISR 永远不会触发。
 * 系统会通过 TIM6 节拍兜底轮询。 */
/* ---- HAL GPIO EXTI 回调：按引脚分发（MAX30102 数据就绪） ---- */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
#if (MAX30102_USE_INT_PIN != 0U)
  if (GPIO_Pin == MAX30102_INT_Pin)
  {
    max30102_mark_data_ready_from_isr();
  }
#else
  (void)GPIO_Pin;
#endif
}

/* ---- SDIO 全局中断 ---- */
void SDIO_IRQHandler(void)
{
  HAL_SD_IRQHandler(APP_SD_Card_GetHandle());
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
