#include "ssd1306.h"
#include "i2c.h"

#include <string.h>

/* SSD1306 I2C 7 位地址左移 1 位（HAL 库要求地址不含 R/W 位） */
#define SSD1306_I2C_ADDR      (0x3CU << 1)

/* I2C 控制字节：0x00 = 接下来是命令，0x40 = 接下来是数据（GRAM 写入） */
#define SSD1306_CONTROL_CMD   0x00U
#define SSD1306_CONTROL_DATA  0x40U

/* OLED I2C 传输超时时间 (ms) */
#define SSD1306_TIMEOUT_MS    100U

/* 128x64 显示屏共 8 页，每页 8 像素高 */
#define SSD1306_PAGE_COUNT    (SSD1306_HEIGHT / 8U)

/*
 * OLED 全屏帧缓冲。
 * 所有像素操作先写入此缓冲区，调用 UpdateScreen 后一次性刷新到 OLED。
 * 1 bit/像素 = 1/8 字节/像素，共 128×64/8 = 1024 字节。
 * 格式：按页组织（page 0 = y 0–7, page 1 = y 8–15, ...），每页 128 列。
 */
uint8_t ssd1306_FrameBuffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8U];

/*
 * 向 OLED 发送单字节命令。
 * 先发控制字节 0x00 表示后续为命令，再发命令本身。
 * 这里采用阻塞 I2C 发送，SSD1306 命令/数据包最长为 2 字节。
 */
static void ssd1306_WriteCommand(uint8_t cmd)
{
  uint8_t packet[2] = {SSD1306_CONTROL_CMD, cmd};
  (void)HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR, packet, sizeof(packet), SSD1306_TIMEOUT_MS);
}

/*
 * 向 OLED 发送一页（128 字节）GRAM 数据。
 * 包格式：0x40 控制字节 + 128 字节数据。
 * 发送前会检查长度，超过 128 字节直接返回。
 */
static void ssd1306_WriteData(const uint8_t *data, uint16_t size)
{
  uint8_t packet[SSD1306_WIDTH + 1U];

  if (size > SSD1306_WIDTH)
  {
    return;
  }

  packet[0] = SSD1306_CONTROL_DATA;
  memcpy(&packet[1], data, size);
  (void)HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR, packet, size + 1U, SSD1306_TIMEOUT_MS);
}

/*
 * 5x7 字模垂直翻转。
 * SSD1306 的 GDDRAM 按页（page）组织，每页 8 像素垂直排列，
 * 字模中的位 0 对应页内最低像素。此函数将位顺序反转，
 * 使字模与 OLED 物理像素对齐。
 */
static uint8_t ssd1306_FlipGlyph7(uint8_t x)
{
  uint8_t y = 0U;
  uint8_t i;

  for (i = 0U; i < 7U; i++)
  {
    if ((x & (1U << i)) != 0U)
    {
      y |= (uint8_t)(1U << (6U - i));
    }
  }

  return y;
}

/*
 * 获取字符的 5x7 像素字模。
 * 仅支持 ASCII 可打印字符（含部分符号）。
 * 小写字母先转大写再查表，字模高位在上。
 * 大写字母最后要垂直翻转 (ssd1306_FlipGlyph7)，使其底部对齐页边界。
 */
static void ssd1306_GetGlyph5x7(char c, uint8_t glyph[5])
{
  uint8_t i;

  for (i = 0U; i < 5U; i++)
  {
    glyph[i] = 0x00U;
  }

  if ((c >= 'a') && (c <= 'z'))
  {
    c = (char)(c - 'a' + 'A');
  }

  switch (c)
  {
    case '0': glyph[0] = 0x3E; glyph[1] = 0x45; glyph[2] = 0x49; glyph[3] = 0x51; glyph[4] = 0x3E; break;
    case '1': glyph[0] = 0x00; glyph[1] = 0x21; glyph[2] = 0x7F; glyph[3] = 0x01; glyph[4] = 0x00; break;
    case '2': glyph[0] = 0x21; glyph[1] = 0x43; glyph[2] = 0x45; glyph[3] = 0x49; glyph[4] = 0x31; break;
    case '3': glyph[0] = 0x42; glyph[1] = 0x41; glyph[2] = 0x51; glyph[3] = 0x69; glyph[4] = 0x46; break;
    case '4': glyph[0] = 0x0C; glyph[1] = 0x14; glyph[2] = 0x24; glyph[3] = 0x7F; glyph[4] = 0x04; break;
    case '5': glyph[0] = 0x72; glyph[1] = 0x51; glyph[2] = 0x51; glyph[3] = 0x51; glyph[4] = 0x4E; break;
    case '6': glyph[0] = 0x1E; glyph[1] = 0x29; glyph[2] = 0x49; glyph[3] = 0x49; glyph[4] = 0x06; break;
    case '7': glyph[0] = 0x40; glyph[1] = 0x47; glyph[2] = 0x48; glyph[3] = 0x50; glyph[4] = 0x60; break;
    case '8': glyph[0] = 0x36; glyph[1] = 0x49; glyph[2] = 0x49; glyph[3] = 0x49; glyph[4] = 0x36; break;
    case '9': glyph[0] = 0x30; glyph[1] = 0x49; glyph[2] = 0x49; glyph[3] = 0x4A; glyph[4] = 0x3C; break;
    case 'A': glyph[0] = 0x7E; glyph[1] = 0x11; glyph[2] = 0x11; glyph[3] = 0x11; glyph[4] = 0x7E; break;
    case 'B': glyph[0] = 0x7F; glyph[1] = 0x49; glyph[2] = 0x49; glyph[3] = 0x49; glyph[4] = 0x36; break;
    case 'C': glyph[0] = 0x3E; glyph[1] = 0x41; glyph[2] = 0x41; glyph[3] = 0x41; glyph[4] = 0x22; break;
    case 'D': glyph[0] = 0x7F; glyph[1] = 0x41; glyph[2] = 0x41; glyph[3] = 0x22; glyph[4] = 0x1C; break;
    case 'E': glyph[0] = 0x7F; glyph[1] = 0x49; glyph[2] = 0x49; glyph[3] = 0x49; glyph[4] = 0x41; break;
    case 'F': glyph[0] = 0x7F; glyph[1] = 0x09; glyph[2] = 0x09; glyph[3] = 0x09; glyph[4] = 0x01; break;
    case 'G': glyph[0] = 0x3E; glyph[1] = 0x41; glyph[2] = 0x49; glyph[3] = 0x49; glyph[4] = 0x7A; break;
    case 'H': glyph[0] = 0x7F; glyph[1] = 0x08; glyph[2] = 0x08; glyph[3] = 0x08; glyph[4] = 0x7F; break;
    case 'I': glyph[0] = 0x00; glyph[1] = 0x41; glyph[2] = 0x7F; glyph[3] = 0x41; glyph[4] = 0x00; break;
    case 'J': glyph[0] = 0x20; glyph[1] = 0x40; glyph[2] = 0x41; glyph[3] = 0x3F; glyph[4] = 0x01; break;
    case 'K': glyph[0] = 0x7F; glyph[1] = 0x08; glyph[2] = 0x14; glyph[3] = 0x22; glyph[4] = 0x41; break;
    case 'L': glyph[0] = 0x7F; glyph[1] = 0x40; glyph[2] = 0x40; glyph[3] = 0x40; glyph[4] = 0x40; break;
    case 'M': glyph[0] = 0x7F; glyph[1] = 0x02; glyph[2] = 0x0C; glyph[3] = 0x02; glyph[4] = 0x7F; break;
    case 'N': glyph[0] = 0x7F; glyph[1] = 0x04; glyph[2] = 0x08; glyph[3] = 0x10; glyph[4] = 0x7F; break;
    case 'O': glyph[0] = 0x3E; glyph[1] = 0x41; glyph[2] = 0x41; glyph[3] = 0x41; glyph[4] = 0x3E; break;
    case 'P': glyph[0] = 0x7F; glyph[1] = 0x09; glyph[2] = 0x09; glyph[3] = 0x09; glyph[4] = 0x06; break;
    case 'Q': glyph[0] = 0x3E; glyph[1] = 0x41; glyph[2] = 0x51; glyph[3] = 0x21; glyph[4] = 0x5E; break;
    case 'R': glyph[0] = 0x7F; glyph[1] = 0x09; glyph[2] = 0x19; glyph[3] = 0x29; glyph[4] = 0x46; break;
    case 'S': glyph[0] = 0x46; glyph[1] = 0x49; glyph[2] = 0x49; glyph[3] = 0x49; glyph[4] = 0x31; break;
    case 'T': glyph[0] = 0x01; glyph[1] = 0x01; glyph[2] = 0x7F; glyph[3] = 0x01; glyph[4] = 0x01; break;
    case 'U': glyph[0] = 0x3F; glyph[1] = 0x40; glyph[2] = 0x40; glyph[3] = 0x40; glyph[4] = 0x3F; break;
    case 'V': glyph[0] = 0x1F; glyph[1] = 0x20; glyph[2] = 0x40; glyph[3] = 0x20; glyph[4] = 0x1F; break;
    case 'W': glyph[0] = 0x7F; glyph[1] = 0x20; glyph[2] = 0x18; glyph[3] = 0x20; glyph[4] = 0x7F; break;
    case 'X': glyph[0] = 0x63; glyph[1] = 0x14; glyph[2] = 0x08; glyph[3] = 0x14; glyph[4] = 0x63; break;
    case 'Y': glyph[0] = 0x03; glyph[1] = 0x04; glyph[2] = 0x78; glyph[3] = 0x04; glyph[4] = 0x03; break;
    case 'Z': glyph[0] = 0x61; glyph[1] = 0x51; glyph[2] = 0x49; glyph[3] = 0x45; glyph[4] = 0x43; break;
    case '!': glyph[0] = 0x00; glyph[1] = 0x00; glyph[2] = 0x5F; glyph[3] = 0x00; glyph[4] = 0x00; break;
    case '.': glyph[0] = 0x00; glyph[1] = 0x60; glyph[2] = 0x60; glyph[3] = 0x00; glyph[4] = 0x00; break;
    case ',': glyph[0] = 0x00; glyph[1] = 0x02; glyph[2] = 0x1C; glyph[3] = 0x00; glyph[4] = 0x00; break;
    case ':': glyph[0] = 0x00; glyph[1] = 0x36; glyph[2] = 0x36; glyph[3] = 0x00; glyph[4] = 0x00; break;
    case ';': glyph[0] = 0x00; glyph[1] = 0x02; glyph[2] = 0x36; glyph[3] = 0x00; glyph[4] = 0x00; break;
    case '-': glyph[0] = 0x08; glyph[1] = 0x08; glyph[2] = 0x08; glyph[3] = 0x08; glyph[4] = 0x08; break;
    case '+': glyph[0] = 0x08; glyph[1] = 0x1C; glyph[2] = 0x08; glyph[3] = 0x1C; glyph[4] = 0x08; break;
    case '/': glyph[0] = 0x20; glyph[1] = 0x10; glyph[2] = 0x08; glyph[3] = 0x04; glyph[4] = 0x02; break;
    case '_': glyph[0] = 0x40; glyph[1] = 0x40; glyph[2] = 0x40; glyph[3] = 0x40; glyph[4] = 0x40; break;
    case '?': glyph[0] = 0x20; glyph[1] = 0x40; glyph[2] = 0x4D; glyph[3] = 0x50; glyph[4] = 0x20; break;
    default: break;
  }

  /* 大写字母需要垂直翻转，使字模顶部与页顶部对齐 */
  if ((c >= 'A') && (c <= 'Z'))
  {
    glyph[0] = ssd1306_FlipGlyph7(glyph[0]);
    glyph[1] = ssd1306_FlipGlyph7(glyph[1]);
    glyph[2] = ssd1306_FlipGlyph7(glyph[2]);
    glyph[3] = ssd1306_FlipGlyph7(glyph[3]);
    glyph[4] = ssd1306_FlipGlyph7(glyph[4]);
  }
}

/*
 * 初始化 OLED 控制器。
 * 上电后先等待 100ms 确保电源稳定，再依次发送 SSD1306 初始化命令序列。
 * 命令序列包括：关显示、设地址模式、设对比度、设扫描方向、开电荷泵、
 * 设预充电周期、设 COM 引脚配置、开显示。
 * 最后清屏并刷新，确保上电后不显示随机噪点。
 */
void ssd1306_Init(void)
{
  static const uint8_t init_cmds[] =
  {
    0xAE,
    0x20, 0x00,
    0xB0,
    0xC0,
    0x00,
    0x10,
    0x40,
    0x81, 0x7F,
    0xA1,
    0xA6,
    0xA8, 0x3F,
    0xA4,
    0xD3, 0x00,
    0xD5, 0x80,
    0xD9, 0xF1,
    0xDA, 0x12,
    0xDB, 0x40,
    0x8D, 0x14,
    0xAF
  };
  uint32_t i;

  /* 上电后等待 100ms，确保 OLED 内部电源稳定 */
  HAL_Delay(100U);

  for (i = 0U; i < (sizeof(init_cmds) / sizeof(init_cmds[0])); i++)
  {
    ssd1306_WriteCommand(init_cmds[i]);
  }

  ssd1306_Clear(SSD1306_COLOR_BLACK);
  ssd1306_UpdateScreen();
}

void ssd1306_SetContrast(uint8_t contrast)
{
  ssd1306_WriteCommand(0x81U);
  ssd1306_WriteCommand(contrast);
}

/* 清空帧缓冲：白 = 全 0xFF，黑 = 全 0x00 */
void ssd1306_Clear(SSD1306_COLOR color)
{
  memset(ssd1306_FrameBuffer,
         (color == SSD1306_COLOR_WHITE) ? 0xFF : 0x00,
         sizeof(ssd1306_FrameBuffer));
}

/*
 * 将帧缓冲刷新到 OLED。
 * 逐页发送：先设页地址 (0xB0+page)，再设列地址低/高字节 (0x00, 0x10)，
 * 最后发送该页 128 字节 GRAM 数据。
 * 全屏刷新 8 页 × 128 字节 = 1KB I2C 传输。
 */
void ssd1306_UpdateScreen(void)
{
  uint8_t page;

  for (page = 0U; page < SSD1306_PAGE_COUNT; page++)
  {
    ssd1306_WriteCommand((uint8_t)(0xB0U + page));
    ssd1306_WriteCommand(0x00U);
    ssd1306_WriteCommand(0x10U);
    ssd1306_WriteData(&ssd1306_FrameBuffer[SSD1306_WIDTH * page], SSD1306_WIDTH);
  }
}

/*
 * 在帧缓冲中绘制单个像素。
 * SSD1306 内存组织：每字节对应 1 列中的 8 个垂直像素 (page)。
 * 写入 = 置位对应 bit，擦除 = 清零对应 bit。
 * 坐标越界自动忽略。
 */
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color)
{
  uint16_t index;
  uint8_t mask;

  if ((x >= SSD1306_WIDTH) || (y >= SSD1306_HEIGHT))
  {
    return;
  }

  index = (uint16_t)x + ((uint16_t)(y / 8U) * SSD1306_WIDTH);
  mask = (uint8_t)(1U << (y % 8U));

  if (color == SSD1306_COLOR_WHITE)
  {
    ssd1306_FrameBuffer[index] |= mask;
  }
  else
  {
    ssd1306_FrameBuffer[index] &= (uint8_t)(~mask);
  }
}

/*
 * 在指定位置绘制 ASCII 字符串。
 * 使用 5×7 像素字模，字符间距 1 像素，总宽 6 像素/字符。
 * 超出右边界自动换行（x=0, y+8）；超出下边界停止绘制。
 * 每个字符先画前景像素，再用黑色画字符间距列。
 */
void ssd1306_DrawString(uint8_t x, uint8_t y, const char *str)
{
  uint8_t glyph[5];
  uint8_t col;
  uint8_t row;

  if (str == NULL)
  {
    return;
  }

  while (*str != '\0')
  {
    if ((x + 5U) >= SSD1306_WIDTH)
    {
      x = 0U;
      y = (uint8_t)(y + 8U);
    }

    if ((y + 7U) >= SSD1306_HEIGHT)
    {
      break;
    }

    ssd1306_GetGlyph5x7(*str, glyph);

    for (col = 0U; col < 5U; col++)
    {
      for (row = 0U; row < 7U; row++)
      {
        if ((glyph[col] & (1U << row)) != 0U)
        {
          ssd1306_DrawPixel((uint8_t)(x + col), (uint8_t)(y + row), SSD1306_COLOR_WHITE);
        }
        else
        {
          ssd1306_DrawPixel((uint8_t)(x + col), (uint8_t)(y + row), SSD1306_COLOR_BLACK);
        }
      }
    }

    /* 字符间距 1 列，用黑色分隔 */
    for (row = 0U; row < 7U; row++)
    {
      ssd1306_DrawPixel((uint8_t)(x + 5U), (uint8_t)(y + row), SSD1306_COLOR_BLACK);
    }

    x = (uint8_t)(x + 6U);
    str++;
  }
}

