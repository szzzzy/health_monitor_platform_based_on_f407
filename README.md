# BME

基于 `STM32F407ZGTx` 的 `MAX30102 + SSD1306 + RTC + USART2` 生理信号采集工程。

当前版本主要完成了心率、血氧、波形显示和串口上报这条链路，适合作为后续继续加入 `AD8232`、`DMA/ADC`、`CMSIS-DSP` 等功能的基础版本。

## 当前功能

- `MAX30102` 红光 / 红外采样
- `OLED(SSD1306)` 实时页面显示
- `BPM / SpO2` 基础计算
- 开机 `5s` 无手指基线采集
- 自适应手指检测与测量状态切换
- `RTC` 时间维护与串口校时
- `USART2` 文本协议上报，方便上位机直接解析
- 调试页显示 `FIFO`、读数状态、`Q(signal quality)`、`PI`

## 硬件与引脚

- MCU：`STM32F407ZGTx`
- 传感器：`MAX30102`
- 显示：`SSD1306 128x64 I2C OLED`
- 时钟：`HSE 8MHz`，`LSE 32.768kHz`
- `I2C1`
  - `PB8` -> `SCL`
  - `PB9` -> `SDA`
- `USART2`
  - `PA2` -> `TX`
  - `PA3` -> `RX`
- 按键
  - `PE2` -> 亮度切换
  - `PE3` -> 上一页
  - `PE4` -> 下一页

## 页面说明

- `BPM` 页：显示 `IR` 波形和心率
- `SpO2` 页：显示 `RED` 波形和血氧
- `DEBUG` 页：显示传感器原始值、FIFO、读写计数、错误恢复、`Q/PI`

## 串口协议

默认通过 `USART2` 按行发送文本，便于上位机直接用串口助手、Python 或现有界面程序解析。

周期上报格式：

```text
M,rtc_valid,yyyymmdd,hhmmss,red,ir,baseline_ir,finger,bpm_valid,bpm,spo2_valid,spo2
```

示例：

```text
M,1,20260416,203015,53210,64892,41200,1,1,76,1,98
```

校时命令：

```text
SETTIME 2026-04-16 20:30:15
TIME=2026-04-16 20:30:15
```

返回格式：

```text
T,success,rtc_valid,yyyymmdd,hhmmss,reason
```

说明：

- `TIME` 和 `SETTIME` 当前都用于设置 RTC
- 时间格式固定为 `yyyy-mm-dd hh:mm:ss`
- `success` 为 `1` 表示设置成功，`0` 表示失败
- 每条消息末尾带 `CRLF`

## 工程结构

- `Core/`：应用逻辑、驱动封装、状态管理
- `Drivers/`：HAL / CMSIS 相关依赖
- `MDK-ARM/BME.uvprojx`：主工程
- `MDK-ARM/MAX30102_Simple_Test.uvprojx`：传感器最小验证工程
- `BME.ioc`：CubeMX 工程文件

## 编译环境

- `STM32CubeMX 6.3.0`
- `STM32Cube FW_F4 V1.26.2`
- `Keil MDK-ARM 5.32`
- 编译器：`ARMCLANG`

## 上手流程

1. 用 `Keil` 打开 `MDK-ARM/BME.uvprojx`
2. 连接 `MAX30102`、`SSD1306`、串口和按键
3. 下载程序后上电
4. 按提示先保持 `5s` 不放手指，完成基线采集
5. 基线完成后进入测量页，再把手指放到 `MAX30102`
6. 上位机读取 `USART2` 输出即可显示或存档

## 当前状态

这个版本重点是把基础采集、显示、RTC、上位机通信和一轮信号质量指标先稳定下来，适合先存档到 GitHub，再继续往 `AD8232`、更强滤波、异常检测、`DMA/RTOS` 方向扩展。
