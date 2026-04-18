#include "app_protocol.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "usart.h"

/* 串口上报与接收缓冲长度，足够覆盖当前文本协议。 */
#define JSON_PAYLOAD_SIZE     256U
#define UART_RX_LINE_SIZE     64U
#define STM32_UART_TIMEOUT_MS 100U

static char json_payload[JSON_PAYLOAD_SIZE];
static char uart_rx_line[UART_RX_LINE_SIZE];
static uint16_t uart_rx_line_len;

static uint16_t build_sensor_packet(const AppState_t *app, char *buffer, size_t buffer_size);
static bool send_uart_line(AppState_t *app, const char *payload, uint16_t payload_len);
static const char *app_skip_spaces(const char *text);
static uint8_t app_text_starts_with_keyword(const char *text, const char *keyword);
static const char *app_get_time_command_payload(const char *line);
static uint8_t app_parse_fixed_uint(const char *text, uint8_t digits, uint16_t *value);
static uint8_t app_parse_datetime_text(const char *text, APP_RTC_DateTime_t *date_time);
static void app_send_rtc_set_response(AppState_t *app, uint8_t success, const char *reason);
static bool app_process_uart_line(AppState_t *app, const char *line);

/* 初始化串口接收行缓冲。 */
void app_protocol_init(void)
{
  uart_rx_line_len = 0U;
}

/* 读取 RTC 当前快照，供显示与上报统一使用。 */
void app_protocol_update_rtc_snapshot(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  if (APP_RTC_GetDateTime(&app->rtc_datetime) == HAL_OK)
  {
    app->rtc_read_ok = 1U;
  }
  else
  {
    (void)memset(&app->rtc_datetime, 0, sizeof(app->rtc_datetime));
    app->rtc_read_ok = 0U;
  }

  app->rtc_time_valid = APP_RTC_IsTimeValid();
}

/* 轮询 USART2，按“单行命令”协议接收 TIME / SETTIME。 */
void app_protocol_poll_uart_commands(AppState_t *app)
{
  uint8_t rx_byte;

  if (app == NULL)
  {
    return;
  }

  while (HAL_UART_Receive(&huart2, &rx_byte, 1U, 0U) == HAL_OK)
  {
    if (rx_byte == '\r')
    {
      continue;
    }

    if (rx_byte == '\n')
    {
      if (uart_rx_line_len > 0U)
      {
        uart_rx_line[uart_rx_line_len] = '\0';
        app->uart_rx_message_valid = app_process_uart_line(app, uart_rx_line);
        uart_rx_line_len = 0U;
      }

      continue;
    }

    if (uart_rx_line_len < (UART_RX_LINE_SIZE - 1U))
    {
      uart_rx_line[uart_rx_line_len++] = (char)rx_byte;
    }
    else
    {
      app->uart_rx_message_valid = false;
      uart_rx_line_len = 0U;
    }
  }
}

/* 组装当前测量报文并通过串口发送。 */
void app_protocol_send_sensor_report(AppState_t *app)
{
  uint16_t payload_len;

  if (app == NULL)
  {
    return;
  }

  app_protocol_update_rtc_snapshot(app);
  payload_len = build_sensor_packet(app, json_payload, sizeof(json_payload));
  (void)send_uart_line(app, json_payload, payload_len);
}

/*
 * 紧凑测量报文格式：
 * M,rtc_valid,yyyymmdd,hhmmss,red,ir,baseline_ir,finger,bpm_valid,bpm,spo2_valid,spo2
 */
static uint16_t build_sensor_packet(const AppState_t *app, char *buffer, size_t buffer_size)
{
  if ((app == NULL) || (buffer == NULL) || (buffer_size == 0U))
  {
    return 0U;
  }

  if (app->rtc_read_ok != 0U)
  {
    (void)snprintf(buffer,
                   buffer_size,
                   "M,%u,%04u%02u%02u,%02u%02u%02u,%lu,%lu,%lu,%u,%u,%u,%u,%u",
                   (unsigned int)app->rtc_time_valid,
                   (unsigned int)app->rtc_datetime.year,
                   (unsigned int)app->rtc_datetime.month,
                   (unsigned int)app->rtc_datetime.date,
                   (unsigned int)app->rtc_datetime.hours,
                   (unsigned int)app->rtc_datetime.minutes,
                   (unsigned int)app->rtc_datetime.seconds,
                   (unsigned long)app->red_value,
                   (unsigned long)app->ir_value,
                   (unsigned long)app->baseline_ir,
                   (unsigned int)app->finger_present,
                   (unsigned int)app->bpm_valid,
                   (unsigned int)app->bpm_value,
                   (unsigned int)app->spo2_valid,
                   (unsigned int)app->spo2_value);
  }
  else
  {
    (void)snprintf(buffer,
                   buffer_size,
                   "M,%u,00000000,000000,%lu,%lu,%lu,%u,%u,%u,%u,%u",
                   (unsigned int)app->rtc_time_valid,
                   (unsigned long)app->red_value,
                   (unsigned long)app->ir_value,
                   (unsigned long)app->baseline_ir,
                   (unsigned int)app->finger_present,
                   (unsigned int)app->bpm_valid,
                   (unsigned int)app->bpm_value,
                   (unsigned int)app->spo2_valid,
                   (unsigned int)app->spo2_value);
  }

  return (uint16_t)strlen(buffer);
}

/* 通过 USART2 发送一行文本，并在末尾补 CRLF。 */
static bool send_uart_line(AppState_t *app, const char *payload, uint16_t payload_len)
{
  HAL_StatusTypeDef status;
  static const uint8_t line_end[] = "\r\n";

  if ((app == NULL) || (payload == NULL) || (payload_len == 0U))
  {
    if (app != NULL)
    {
      app->uart_tx_message_valid = false;
    }

    return false;
  }

  status = HAL_UART_Transmit(&huart2,
                             (uint8_t *)payload,
                             (uint16_t)payload_len,
                             STM32_UART_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    app->uart_tx_message_valid = false;
    return false;
  }

  status = HAL_UART_Transmit(&huart2,
                             (uint8_t *)line_end,
                             (uint16_t)(sizeof(line_end) - 1U),
                             STM32_UART_TIMEOUT_MS);
  app->uart_tx_message_valid = (status == HAL_OK);
  return app->uart_tx_message_valid;
}

/* 跳过命令字符串前导空格。 */
static const char *app_skip_spaces(const char *text)
{
  if (text == NULL)
  {
    return NULL;
  }

  while ((*text == ' ') || (*text == '\t'))
  {
    text++;
  }

  return text;
}

/* 不区分大小写地判断字符串是否以某个关键字开头。 */
static uint8_t app_text_starts_with_keyword(const char *text, const char *keyword)
{
  if ((text == NULL) || (keyword == NULL))
  {
    return 0U;
  }

  while (*keyword != '\0')
  {
    if (*text == '\0')
    {
      return 0U;
    }

    if (toupper((unsigned char)*text) != toupper((unsigned char)*keyword))
    {
      return 0U;
    }

    text++;
    keyword++;
  }

  return 1U;
}

/* 解析 TIME / SETTIME 命令后面的时间负载。 */
static const char *app_get_time_command_payload(const char *line)
{
  const char *cursor;
  uint8_t keyword_length;

  cursor = app_skip_spaces(line);
  if (cursor == NULL)
  {
    return NULL;
  }

  if (app_text_starts_with_keyword(cursor, "TIME") != 0U)
  {
    keyword_length = 4U;
  }
  else if (app_text_starts_with_keyword(cursor, "SETTIME") != 0U)
  {
    keyword_length = 7U;
  }
  else
  {
    return NULL;
  }

  if ((cursor[keyword_length] != '\0') &&
      (cursor[keyword_length] != ' ') &&
      (cursor[keyword_length] != '\t') &&
      (cursor[keyword_length] != '='))
  {
    return NULL;
  }

  cursor += keyword_length;
  cursor = app_skip_spaces(cursor);
  if ((cursor != NULL) && (*cursor == '='))
  {
    cursor++;
  }

  return app_skip_spaces(cursor);
}

/* 固定位数十进制解析器，用于解析 yyyy/mm/dd/hh/mm/ss。 */
static uint8_t app_parse_fixed_uint(const char *text, uint8_t digits, uint16_t *value)
{
  uint8_t i;
  uint16_t parsed_value = 0U;

  if ((text == NULL) || (value == NULL))
  {
    return 0U;
  }

  for (i = 0U; i < digits; i++)
  {
    if (isdigit((unsigned char)text[i]) == 0)
    {
      return 0U;
    }

    parsed_value = (uint16_t)((parsed_value * 10U) + (uint16_t)(text[i] - '0'));
  }

  *value = parsed_value;
  return 1U;
}

/* 解析形如 2026-04-14 12:34:56 的时间文本。 */
static uint8_t app_parse_datetime_text(const char *text, APP_RTC_DateTime_t *date_time)
{
  uint16_t year;
  uint16_t month;
  uint16_t day;
  uint16_t hours;
  uint16_t minutes;
  uint16_t seconds;
  const char *tail;

  if ((text == NULL) || (date_time == NULL))
  {
    return 0U;
  }

  if (strlen(text) < 19U)
  {
    return 0U;
  }

  if ((text[4] != '-') || (text[7] != '-') ||
      ((text[10] != ' ') && (text[10] != 'T')) ||
      (text[13] != ':') || (text[16] != ':'))
  {
    return 0U;
  }

  if ((app_parse_fixed_uint(text, 4U, &year) == 0U) ||
      (app_parse_fixed_uint(&text[5], 2U, &month) == 0U) ||
      (app_parse_fixed_uint(&text[8], 2U, &day) == 0U) ||
      (app_parse_fixed_uint(&text[11], 2U, &hours) == 0U) ||
      (app_parse_fixed_uint(&text[14], 2U, &minutes) == 0U) ||
      (app_parse_fixed_uint(&text[17], 2U, &seconds) == 0U))
  {
    return 0U;
  }

  tail = app_skip_spaces(&text[19]);
  if ((tail == NULL) || (*tail != '\0'))
  {
    return 0U;
  }

  date_time->year = year;
  date_time->month = (uint8_t)month;
  date_time->date = (uint8_t)day;
  date_time->weekday = 0U;
  date_time->hours = (uint8_t)hours;
  date_time->minutes = (uint8_t)minutes;
  date_time->seconds = (uint8_t)seconds;
  return 1U;
}

/*
 * RTC 设置回包格式：
 * T,set_ok,rtc_valid,yyyymmdd,hhmmss,reason
 */
static void app_send_rtc_set_response(AppState_t *app, uint8_t success, const char *reason)
{
  char response[160];
  uint16_t year = 0U;
  uint16_t month = 0U;
  uint16_t date = 0U;
  uint16_t hours = 0U;
  uint16_t minutes = 0U;
  uint16_t seconds = 0U;
  uint8_t rtc_valid = 0U;

  if (app != NULL)
  {
    rtc_valid = app->rtc_time_valid;

    if (app->rtc_read_ok != 0U)
    {
      year = app->rtc_datetime.year;
      month = app->rtc_datetime.month;
      date = app->rtc_datetime.date;
      hours = app->rtc_datetime.hours;
      minutes = app->rtc_datetime.minutes;
      seconds = app->rtc_datetime.seconds;
    }
  }

  (void)snprintf(response,
                 sizeof(response),
                 "T,%u,%u,%04u%02u%02u,%02u%02u%02u,%s",
                 (unsigned int)success,
                 (unsigned int)rtc_valid,
                 (unsigned int)year,
                 (unsigned int)month,
                 (unsigned int)date,
                 (unsigned int)hours,
                 (unsigned int)minutes,
                 (unsigned int)seconds,
                 (reason != NULL) ? reason : "ok");

  (void)send_uart_line(app, response, (uint16_t)strlen(response));
}

/* 处理单行串口命令，目前仅支持设置 RTC 时间。 */
static bool app_process_uart_line(AppState_t *app, const char *line)
{
  APP_RTC_DateTime_t new_time = {0};
  const char *payload;

  if ((app == NULL) || (line == NULL))
  {
    return false;
  }

  payload = app_get_time_command_payload(line);
  if (payload == NULL)
  {
    return false;
  }

  if (app_parse_datetime_text(payload, &new_time) == 0U)
  {
    app_send_rtc_set_response(app, 0U, "bad_format");
    return false;
  }

  if (APP_RTC_SetDateTime(&new_time) != HAL_OK)
  {
    app_send_rtc_set_response(app, 0U, "invalid_datetime");
    return false;
  }

  app_protocol_update_rtc_snapshot(app);
  app->display_refresh_requested = 1U;
  app->report_due = 1U;
  app_send_rtc_set_response(app, 1U, NULL);
  return true;
}
