/**
  ******************************************************************************
  * @file           : main_max30102_simple_test.c
  * @brief          : Minimal MAX30102 validation firmware.
  ******************************************************************************
  */

#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

#include "max30102.h"
#include "ssd1306.h"

#define MAX_TEST_UI_PERIOD_MS    200U
#define MAX_TEST_UART_PERIOD_MS  500U
#define MAX_TEST_LOOP_DELAY_MS   5U
#define MAX_TEST_UART_TIMEOUT_MS 100U

static void SystemClock_Config(void);
static void max_test_display(const char *line0,
                             const char *line1,
                             const char *line2,
                             const char *line3,
                             const char *line4,
                             const char *line5);
static void max_test_uart_send_line(const char *text);
static void max_test_uart_report(const char *status,
                                 uint8_t part_id,
                                 uint32_t red_value,
                                 uint32_t ir_value,
                                 uint32_t ok_count,
                                 uint32_t busy_count,
                                 uint32_t error_count);
static void max_test_error_loop(const char *status, const char *detail, uint8_t part_id);

int main(void)
{
  uint8_t part_id = 0U;
  uint8_t fifo_buf[6] = {0};
  uint32_t red_value = 0U;
  uint32_t ir_value = 0U;
  uint32_t ok_count = 0U;
  uint32_t busy_count = 0U;
  uint32_t error_count = 0U;
  uint32_t last_ui_tick = 0U;
  uint32_t last_uart_tick = 0U;
  uint32_t now;
  HAL_StatusTypeDef status;
  const char *read_status = "WAIT";
  char line1[24];
  char line2[24];
  char line3[24];
  char line4[24];
  char line5[24];

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();

  ssd1306_Init();
  max_test_display("MAX30102 TEST", "BOOT", "", "", "", "");
  max_test_uart_send_line("MAX30102 SIMPLE TEST BOOT");

  if (max30102_read_reg(MAX30102_REG_PART_ID, &part_id) != HAL_OK)
  {
    max_test_error_loop("ID READ ERR", "CHECK I2C", part_id);
  }

  if (part_id != MAX30102_PART_ID_VALUE)
  {
    max_test_error_loop("ID MISMATCH", "NOT MAX30102", part_id);
  }

  if (max30102_init() != HAL_OK)
  {
    max_test_error_loop("INIT FAIL", "CHECK SENSOR", part_id);
  }

  max_test_uart_report("init_ok", part_id, red_value, ir_value, ok_count, busy_count, error_count);

  while (1)
  {
    status = max30102_read_fifo(fifo_buf, sizeof(fifo_buf));
    if (status == HAL_OK)
    {
      max30102_parse_spo2_sample(fifo_buf, &red_value, &ir_value);
      ok_count++;
      read_status = "OK";
    }
    else if (status == HAL_BUSY)
    {
      busy_count++;
      read_status = "WAIT";
    }
    else
    {
      error_count++;
      read_status = "ERR";
    }

    now = HAL_GetTick();

    if ((now - last_ui_tick) >= MAX_TEST_UI_PERIOD_MS)
    {
      last_ui_tick = now;

      (void)snprintf(line1, sizeof(line1), "ID:%02X ST:%s", (unsigned int)part_id, read_status);
      (void)snprintf(line2, sizeof(line2), "RED:%lu", (unsigned long)red_value);
      (void)snprintf(line3, sizeof(line3), "IR :%lu", (unsigned long)ir_value);
      (void)snprintf(line4, sizeof(line4), "OK:%lu B:%lu", (unsigned long)ok_count, (unsigned long)busy_count);
      (void)snprintf(line5, sizeof(line5), "ERR:%lu", (unsigned long)error_count);
      max_test_display("MAX30102 TEST", line1, line2, line3, line4, line5);
    }

    if ((now - last_uart_tick) >= MAX_TEST_UART_PERIOD_MS)
    {
      last_uart_tick = now;
      max_test_uart_report(read_status, part_id, red_value, ir_value, ok_count, busy_count, error_count);
    }

    HAL_Delay(MAX_TEST_LOOP_DELAY_MS);
  }
}

static void max_test_display(const char *line0,
                             const char *line1,
                             const char *line2,
                             const char *line3,
                             const char *line4,
                             const char *line5)
{
  ssd1306_Clear(SSD1306_COLOR_BLACK);
  ssd1306_DrawString(0, 0, (line0 != NULL) ? line0 : "");
  ssd1306_DrawString(0, 8, (line1 != NULL) ? line1 : "");
  ssd1306_DrawString(0, 16, (line2 != NULL) ? line2 : "");
  ssd1306_DrawString(0, 24, (line3 != NULL) ? line3 : "");
  ssd1306_DrawString(0, 32, (line4 != NULL) ? line4 : "");
  ssd1306_DrawString(0, 40, (line5 != NULL) ? line5 : "");
  ssd1306_UpdateScreen();
}

static void max_test_uart_send_line(const char *text)
{
  static const uint8_t line_end[] = "\r\n";

  if (text == NULL)
  {
    return;
  }

  (void)HAL_UART_Transmit(&huart2,
                          (uint8_t *)text,
                          (uint16_t)strlen(text),
                          MAX_TEST_UART_TIMEOUT_MS);
  (void)HAL_UART_Transmit(&huart2,
                          (uint8_t *)line_end,
                          (uint16_t)(sizeof(line_end) - 1U),
                          MAX_TEST_UART_TIMEOUT_MS);
}

static void max_test_uart_report(const char *status,
                                 uint8_t part_id,
                                 uint32_t red_value,
                                 uint32_t ir_value,
                                 uint32_t ok_count,
                                 uint32_t busy_count,
                                 uint32_t error_count)
{
  char line[128];

  (void)snprintf(line,
                 sizeof(line),
                 "MAX,%s,ID=%02X,RED=%lu,IR=%lu,OK=%lu,BUSY=%lu,ERR=%lu",
                 (status != NULL) ? status : "NA",
                 (unsigned int)part_id,
                 (unsigned long)red_value,
                 (unsigned long)ir_value,
                 (unsigned long)ok_count,
                 (unsigned long)busy_count,
                 (unsigned long)error_count);
  max_test_uart_send_line(line);
}

static void max_test_error_loop(const char *status, const char *detail, uint8_t part_id)
{
  char line1[24];
  uint32_t last_uart_tick = 0U;
  uint32_t now;

  (void)snprintf(line1, sizeof(line1), "ID:%02X", (unsigned int)part_id);

  while (1)
  {
    max_test_display("MAX30102 TEST",
                     (status != NULL) ? status : "ERROR",
                     (detail != NULL) ? detail : "",
                     line1,
                     "CHECK POWER/I2C",
                     "");

    now = HAL_GetTick();
    if ((now - last_uart_tick) >= MAX_TEST_UART_PERIOD_MS)
    {
      last_uart_tick = now;
      max_test_uart_report((status != NULL) ? status : "error", part_id, 0U, 0U, 0U, 0U, 1U);
    }

    HAL_Delay(200U);
  }
}

static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
