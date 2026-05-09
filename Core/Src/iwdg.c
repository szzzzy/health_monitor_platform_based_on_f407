/**
  ******************************************************************************
  * @file    iwdg.c
  * @brief   独立看门狗初始化与喂狗实现
  *
  * 时钟源与超时时间：
  *   - LSI 典型值 32 kHz（实际范围 17~47 kHz，由芯片个体差异决定）
  *   - 预分频器设为 64（IWDG_PRESCALER_64），分频后 500 Hz
  *   - 重装载值 3999，因此最大超时 ≈ (64 × 3999) / 32000 ≈ 8 秒
  *   - 实际超时随 LSI 漂移而变化，设计上留有充足余量
  *
  * 对主循环的要求：
  *   - 任何单次循环迭代（含传感器读取、OLED 刷新、SD 卡写入等）
  *     总耗时必须远小于 8 秒，否则会触发意外复位
  *   - 奥莱德写和 SD 卡扇区操作是最大的不确定延迟来源，
  *     因此在这些操作之后立即喂狗，确保计数不被耗尽
  ******************************************************************************
  */

#include "iwdg.h"

IWDG_HandleTypeDef hiwdg;

/*
 * 初始化独立看门狗。
 *
 * IWDG_PRESCALER_64：LSI 的 1/64 = 500 Hz 计数器时钟
 * Reload = 3999：超时 = 3999 / 500 ≈ 8 秒
 *
 * 注意：一旦 HAL_IWDG_Init 返回成功，看门狗就开始运行，
 * 此后无法通过软件停止，只能喂狗延时。
 */
void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 3999U;

  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
 * 喂狗：将递减计数器重新加载为重装载值。
 *
 * 在系统正常运行的主循环、基线采集循环、传感器异常循环等所有
 * 长时间执行的路径中调用此函数，防止看门狗超时复位。
 *
 * HAL_IWDG_Refresh 本身是轻量操作（写两个键寄存器），
 * 不会阻塞超过几个 APB1 周期。
 */
void APP_Watchdog_Refresh(void)
{
  (void)HAL_IWDG_Refresh(&hiwdg);
}
