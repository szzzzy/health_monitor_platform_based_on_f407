#ifndef __SSD1306_H__
#define __SSD1306_H__

/**
 * @file ssd1306.h
 * @brief 128×64 单色 OLED 帧缓冲与基础绘图接口。
 *
 * 绘图函数只修改 1024 字节 RAM 帧缓冲；UpdateScreen 才通过 I2C1 提交整帧。
 */

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

/** @brief 初始化控制器并清空显示；调用方负责共享 I2C1 的串行化。 */
void ssd1306_Init(void);
/** @brief 设置控制器对比度寄存器。 */
void ssd1306_SetContrast(uint8_t contrast);
/** @brief 用指定颜色填充本地帧缓冲，不立即访问总线。 */
void ssd1306_Clear(SSD1306_COLOR color);
/** @brief 把完整本地帧缓冲发送到 OLED。 */
void ssd1306_UpdateScreen(void);
/** @brief 修改一个像素；坐标越界时忽略。 */
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
/** @brief 从指定坐标绘制以 NUL 结尾的 ASCII 字符串。 */
void ssd1306_DrawString(uint8_t x, uint8_t y, const char *str);

#ifdef __cplusplus
}
#endif

#endif /* __SSD1306_H__ */

