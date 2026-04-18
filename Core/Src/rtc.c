/**
  ******************************************************************************
  * @file    rtc.c
  * @brief   This file provides code for the configuration
  *          of the RTC instances.
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

/* Includes ------------------------------------------------------------------*/
#include "rtc.h"

/* USER CODE BEGIN 0 */
#define APP_RTC_BKP_INIT_MAGIC    0xA5A5U
#define APP_RTC_BKP_VALID_MAGIC   0x5A5AU
#define APP_RTC_DEFAULT_YEAR      2000U

static uint8_t app_rtc_is_leap_year(uint16_t year);
static uint8_t app_rtc_get_days_in_month(uint16_t year, uint8_t month);
static uint8_t app_rtc_calculate_weekday(uint16_t year, uint8_t month, uint8_t day);
static uint8_t app_rtc_is_datetime_valid(const APP_RTC_DateTime_t *date_time);
static HAL_StatusTypeDef app_rtc_apply_datetime(const APP_RTC_DateTime_t *date_time);
static void app_rtc_mark_initialized(void);
static uint8_t app_rtc_is_initialized(void);
static void app_rtc_mark_time_valid(uint8_t is_valid);

/* USER CODE END 0 */

RTC_HandleTypeDef hrtc;

/* RTC init function */
void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  HAL_PWR_EnableBkUpAccess();

  if (app_rtc_is_initialized() == 0U)
  {
    sTime.Hours = 0U;
    sTime.Minutes = 0U;
    sTime.Seconds = 0U;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
    {
      Error_Handler();
    }

    sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
    sDate.Month = RTC_MONTH_JANUARY;
    sDate.Date = 1U;
    sDate.Year = 0U;
    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
    {
      Error_Handler();
    }

    app_rtc_mark_initialized();
    app_rtc_mark_time_valid(0U);
  }

  /* USER CODE END Check_RTC_BKUP */
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{

  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */

  /* USER CODE END RTC_MspInit 0 */
  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* RTC clock enable */
    __HAL_RCC_RTC_ENABLE();
  /* USER CODE BEGIN RTC_MspInit 1 */

  /* USER CODE END RTC_MspInit 1 */
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
HAL_StatusTypeDef APP_RTC_GetDateTime(APP_RTC_DateTime_t *date_time)
{
  RTC_TimeTypeDef time = {0};
  RTC_DateTypeDef date = {0};

  if (date_time == NULL)
  {
    return HAL_ERROR;
  }

  if (HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK)
  {
    return HAL_ERROR;
  }

  date_time->year = (uint16_t)(APP_RTC_DEFAULT_YEAR + date.Year);
  date_time->month = date.Month;
  date_time->date = date.Date;
  date_time->weekday = date.WeekDay;
  date_time->hours = time.Hours;
  date_time->minutes = time.Minutes;
  date_time->seconds = time.Seconds;
  return HAL_OK;
}

HAL_StatusTypeDef APP_RTC_SetDateTime(const APP_RTC_DateTime_t *date_time)
{
  HAL_StatusTypeDef status;

  status = app_rtc_apply_datetime(date_time);
  if (status != HAL_OK)
  {
    return status;
  }

  app_rtc_mark_initialized();
  app_rtc_mark_time_valid(1U);
  return HAL_OK;
}

uint8_t APP_RTC_IsTimeValid(void)
{
  HAL_PWR_EnableBkUpAccess();
  return (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) == APP_RTC_BKP_VALID_MAGIC) ? 1U : 0U;
}

static uint8_t app_rtc_is_leap_year(uint16_t year)
{
  if ((year % 400U) == 0U)
  {
    return 1U;
  }

  if ((year % 100U) == 0U)
  {
    return 0U;
  }

  return ((year % 4U) == 0U) ? 1U : 0U;
}

static uint8_t app_rtc_get_days_in_month(uint16_t year, uint8_t month)
{
  switch (month)
  {
    case 1U:
    case 3U:
    case 5U:
    case 7U:
    case 8U:
    case 10U:
    case 12U:
      return 31U;

    case 4U:
    case 6U:
    case 9U:
    case 11U:
      return 30U;

    case 2U:
      return (app_rtc_is_leap_year(year) != 0U) ? 29U : 28U;

    default:
      return 0U;
  }
}

static uint8_t app_rtc_calculate_weekday(uint16_t year, uint8_t month, uint8_t day)
{
  static const uint8_t month_offset[] = {0U, 3U, 2U, 5U, 0U, 3U, 5U, 1U, 4U, 6U, 2U, 4U};
  uint16_t adjusted_year;
  uint8_t weekday_index;

  adjusted_year = year;
  if (month < 3U)
  {
    adjusted_year--;
  }

  weekday_index = (uint8_t)((adjusted_year + (adjusted_year / 4U) - (adjusted_year / 100U) +
                              (adjusted_year / 400U) + month_offset[month - 1U] + day) % 7U);

  switch (weekday_index)
  {
    case 0U:
      return RTC_WEEKDAY_SUNDAY;

    case 1U:
      return RTC_WEEKDAY_MONDAY;

    case 2U:
      return RTC_WEEKDAY_TUESDAY;

    case 3U:
      return RTC_WEEKDAY_WEDNESDAY;

    case 4U:
      return RTC_WEEKDAY_THURSDAY;

    case 5U:
      return RTC_WEEKDAY_FRIDAY;

    default:
      return RTC_WEEKDAY_SATURDAY;
  }
}

static uint8_t app_rtc_is_datetime_valid(const APP_RTC_DateTime_t *date_time)
{
  uint8_t days_in_month;

  if (date_time == NULL)
  {
    return 0U;
  }

  if ((date_time->year < APP_RTC_DEFAULT_YEAR) || (date_time->year > 2099U))
  {
    return 0U;
  }

  if ((date_time->month < 1U) || (date_time->month > 12U))
  {
    return 0U;
  }

  days_in_month = app_rtc_get_days_in_month(date_time->year, date_time->month);
  if ((date_time->date < 1U) || (date_time->date > days_in_month))
  {
    return 0U;
  }

  if ((date_time->hours > 23U) || (date_time->minutes > 59U) || (date_time->seconds > 59U))
  {
    return 0U;
  }

  return 1U;
}

static HAL_StatusTypeDef app_rtc_apply_datetime(const APP_RTC_DateTime_t *date_time)
{
  RTC_TimeTypeDef time = {0};
  RTC_DateTypeDef date = {0};

  if (app_rtc_is_datetime_valid(date_time) == 0U)
  {
    return HAL_ERROR;
  }

  time.Hours = date_time->hours;
  time.Minutes = date_time->minutes;
  time.Seconds = date_time->seconds;
  time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  time.StoreOperation = RTC_STOREOPERATION_RESET;

  date.Year = (uint8_t)(date_time->year - APP_RTC_DEFAULT_YEAR);
  date.Month = date_time->month;
  date.Date = date_time->date;
  date.WeekDay = app_rtc_calculate_weekday(date_time->year, date_time->month, date_time->date);

  if (HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static void app_rtc_mark_initialized(void)
{
  HAL_PWR_EnableBkUpAccess();
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, APP_RTC_BKP_INIT_MAGIC);
}

static uint8_t app_rtc_is_initialized(void)
{
  return (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) == APP_RTC_BKP_INIT_MAGIC) ? 1U : 0U;
}

static void app_rtc_mark_time_valid(uint8_t is_valid)
{
  HAL_PWR_EnableBkUpAccess();
  HAL_RTCEx_BKUPWrite(&hrtc,
                      RTC_BKP_DR1,
                      (is_valid != 0U) ? APP_RTC_BKP_VALID_MAGIC : 0U);
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
