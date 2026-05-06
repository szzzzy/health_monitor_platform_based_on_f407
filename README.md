# BME

基于 `STM32F407ZGTx` 的 `MAX30102 + SSD1306 + RTC + USART2` 生理信号采集工程。  
当前版本在 50 Hz 精确采样节拍、CMSIS-DSP 带通滤波、自相关 BPM 和 I2C DMA 传输的基础上，完成了心率、血氧、波形显示和串口上报的主链路，适合作为后续继续扩展 `AD8232`、`DMA/ADC`、`RTOS` 等功能的基础版本。

This project is a physiological signal acquisition firmware based on `STM32F407ZGTx`, using `MAX30102 + SSD1306 + RTC + USART2`.  
The current version builds on a 50 Hz precise sampling tick, CMSIS-DSP bandpass filtering, autocorrelation BPM, and I2C DMA transfers. It covers the main pipeline of heart-rate estimation, SpO2 estimation, waveform display, and UART reporting, and is suitable as a base for future extensions such as `AD8232`, `DMA/ADC`, and `RTOS`.

## 当前功能 / Current Features

- `MAX30102` 红光 / 红外采样，LED 驱动电流 19 mA  
  `MAX30102` RED / IR sampling, LED drive current 19 mA
- `TIM6` 50 Hz 精确采样节拍 + `WFI` 低功耗等待  
  `TIM6` 50 Hz precise sampling tick with `WFI` power-saving idle
- `MAX30102 INT` 引脚中断驱动采样，TIM6 轮询兜底  
  `MAX30102 INT` pin interrupt-driven sampling, TIM6 polling fallback
- `I2C DMA` 传输 FIFO 数据，主循环不阻塞  
  `I2C DMA` FIFO data transfer, non-blocking main loop
- `CMSIS-DSP` 4 阶 Butterworth 带通滤波 (0.5–5 Hz)  
  `CMSIS-DSP` 4th-order Butterworth bandpass filter (0.5–5 Hz)
- 自相关 `BPM` + 时域 `SpO2` 双算法  
  Autocorrelation `BPM` + time-domain `SpO2` dual algorithm
- `OLED (SSD1306)` 实时页面显示  
  Real-time page rendering on `OLED (SSD1306)`
- 开机 `5s` 无手指基线采集  
  `5s` no-finger baseline acquisition after boot
- 自适应手指检测与测量状态切换  
  Adaptive finger detection and measurement-state switching
- `RTC` 时间维护与串口校时  
  `RTC` time keeping and UART-based time synchronization
- `USART2` 文本协议上报，便于上位机直接解析  
  `USART2` text protocol reporting for direct host-side parsing
- 调试页显示 `FIFO`、读数状态、`SQ (signal quality)`、`PI`  
  Debug page showing `FIFO`, read status, `SQ (signal quality)`, and `PI`

## 采样架构 / Sampling Architecture

系统采用 `TIM6` 产生 50 Hz 精确采样节拍（周期 20 ms），主循环在每个节拍内执行以下流程：

The system uses `TIM6` to generate a precise 50 Hz sampling tick (20 ms period). The main loop runs the following flow on each tick:

1. 轮询串口命令（USART2 DMA + IDLE 接收）  
   Poll UART commands (USART2 DMA + IDLE reception)
2. 处理按键（软件消抖）  
   Handle buttons (software debounce)
3. `max30102_should_service_fifo()` 门控判断是否需要读传感器  
   `max30102_should_service_fifo()` gate to decide whether to read the sensor
4. 推进 BPM / SpO2 算法 + 波形显示  
   Advance BPM / SpO2 algorithms + waveform display
5. 200 ms 周期上报 + OLED 刷新  
   200 ms periodic report + OLED refresh

节拍间通过 `__WFI()` 进入低功耗等待，TIM6 中断唤醒。  
Between ticks the CPU enters low-power wait via `__WFI()`, woken by the TIM6 interrupt.

`MAX30102 INT` 引脚（PA0）配合 EXTI 中断提供硬件数据就绪通知；若硬件未连接 INT 引脚，将 `MAX30102_USE_INT_PIN` 改为 `0U` 即可退化为纯 TIM6 轮询模式。

The `MAX30102 INT` pin (PA0) with EXTI interrupt provides hardware data-ready notification. If the INT pin is not connected on your hardware, set `MAX30102_USE_INT_PIN` to `0U` to fall back to pure TIM6 polling mode.

## 硬件与引脚 / Hardware and Pinout

| 项目 / Item | 内容 / Value |
| --- | --- |
| MCU | `STM32F407ZGTx` |
| 传感器 / Sensor | `MAX30102` |
| 显示 / Display | `SSD1306 128x64 I2C OLED` |
| 时钟 / Clock | `HSE 8MHz`, `LSE 32.768kHz` |

### `I2C1`

- `PB8` -> `SCL`
- `PB9` -> `SDA`
- `PA0` -> `MAX30102 INT`（可选，通过 `MAX30102_USE_INT_PIN` 控制 / optional, controlled by `MAX30102_USE_INT_PIN`）

### `USART2`

- `PA2` -> `TX`
- `PA3` -> `RX`

### 按键 / Buttons

- `PE2` -> 亮度切换 / Brightness switch
- `PE3` -> 上一页 / Previous page
- `PE4` -> 下一页 / Next page

## 页面说明 / UI Pages

- `BPM` 页：显示 `IR` 波形和心率  
  `BPM` page: shows the `IR` waveform and heart rate
- `SpO2` 页：显示 `RED` 波形和血氧  
  `SpO2` page: shows the `RED` waveform and oxygen saturation
- `DEBUG` 页：显示传感器原始值、`FIFO`、读写计数、错误恢复、`SQ/PI`  
  `DEBUG` page: shows raw sensor values, `FIFO`, read/write counters, recovery status, and `SQ/PI`

## 串口协议 / UART Protocol

默认通过 `USART2` 按行发送文本，便于上位机直接用串口助手、Python 脚本或现有界面程序解析。  
By default, the firmware sends line-based text messages through `USART2`, making it easy to parse from a serial terminal, Python script, or PC application.

周期上报格式 / Periodic report format:

```text
M,rtc_valid,yyyymmdd,hhmmss,red,ir,baseline_ir,finger,bpm_valid,bpm,spo2_valid,spo2
```

示例 / Example:

```text
M,1,20260416,203015,53210,64892,41200,1,1,76,1,98
```

校时命令 / Time-sync commands:

```text
SETTIME 2026-04-16 20:30:15
TIME=2026-04-16 20:30:15
```

返回格式 / Response format:

```text
T,success,rtc_valid,yyyymmdd,hhmmss,reason
```

说明 / Notes:

- `TIME` 和 `SETTIME` 当前都用于设置 `RTC`  
  Both `TIME` and `SETTIME` are currently accepted as RTC set commands.
- 时间格式固定为 `yyyy-mm-dd hh:mm:ss`  
  The accepted time format is `yyyy-mm-dd hh:mm:ss`.
- `success=1` 表示设置成功，`success=0` 表示失败  
  `success=1` means success, and `success=0` means failure.
- 每条消息末尾带 `CRLF`  
  Each message ends with `CRLF`.

## 工程结构 / Project Structure

- `Core/`：应用逻辑、驱动封装、状态管理  
  `Core/`: application logic, driver wrappers, and state management
- `Drivers/`：HAL / CMSIS 相关依赖  
  `Drivers/`: HAL / CMSIS dependencies
- `MDK-ARM/BME.uvprojx`：主工程  
  `MDK-ARM/BME.uvprojx`: main project
- `MDK-ARM/MAX30102_Simple_Test.uvprojx`：传感器最小验证工程  
  `MDK-ARM/MAX30102_Simple_Test.uvprojx`: minimal verification project for the sensor
- `BME.ioc`：CubeMX 工程文件  
  `BME.ioc`: CubeMX project file

## 编译环境 / Build Environment

- `STM32CubeMX 6.3.0`
- `STM32Cube FW_F4 V1.26.2`
- `Keil MDK-ARM 5.32`
- 编译器 / Compiler: `ARMCLANG`

## 上手流程 / Quick Start

1. 用 `Keil` 打开 `MDK-ARM/BME.uvprojx`  
   Open `MDK-ARM/BME.uvprojx` with `Keil`.
2. 连接 `MAX30102`、`SSD1306`、串口和按键  
   Connect `MAX30102`, `SSD1306`, UART, and buttons.
3. 下载程序后上电  
   Flash the firmware and power on the board.
4. 按提示先保持 `5s` 不放手指，完成基线采集  
   Keep the sensor untouched for `5s` to finish baseline acquisition.
5. 基线完成后进入测量页，再把手指放到 `MAX30102`  
   After baseline acquisition, enter the measurement page and place a finger on the `MAX30102`.
6. 上位机读取 `USART2` 输出即可显示或存档  
   Read `USART2` output on the host side for display or logging.

## 当前状态 / Current Status

这一版已经完成了 TIM6 50 Hz 精确节拍、INT 引脚中断驱动采样、I2C DMA 传输、CMSIS-DSP Butterworth 带通滤波和自相关 BPM 的集成。系统以 20 ms 为周期稳定运行，主循环在 `__WFI()` 低功耗等待中度过大部分时间。适合作为继续往 `AD8232` 心电、更强滤波、异常检测、`RTOS` 等方向扩展的基线。

This version integrates a TIM6 50 Hz precise tick, INT pin interrupt-driven sampling, I2C DMA transfers, CMSIS-DSP Butterworth bandpass filtering, and autocorrelation BPM. The system runs stably on a 20 ms cycle, with the main loop spending most of its time in `__WFI()` low-power wait. It serves as a solid baseline for extensions into `AD8232` ECG, stronger filtering, anomaly detection, or `RTOS`.
