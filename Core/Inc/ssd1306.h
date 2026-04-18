#ifndef __SSD1306_H__
#define __SSD1306_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define SSD1306_WIDTH  128U
#define SSD1306_HEIGHT 64U

typedef enum
{
  SSD1306_COLOR_BLACK = 0U,
  SSD1306_COLOR_WHITE = 1U
} SSD1306_COLOR;

/* 128x64 单色屏显存：128 * 64 / 8 = 1024 字节 */
extern uint8_t ssd1306_FrameBuffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8U];

void ssd1306_Init(void);
void ssd1306_SetContrast(uint8_t contrast);
void ssd1306_Clear(SSD1306_COLOR color);
void ssd1306_UpdateScreen(void);
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
void ssd1306_DrawString(uint8_t x, uint8_t y, const char *str);

#ifdef __cplusplus
}
#endif

#endif /* __SSD1306_H__ */

