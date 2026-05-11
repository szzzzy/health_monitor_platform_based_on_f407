# BME

基于 `STM32F407ZGTx` 的 `MAX30102 + SSD1306 + RTC + USART2` 生理信号采集工程。  
当前版本在 100 Hz 精确采样节拍、CMSIS-DSP 带通滤波、自相关 BPM、I2C DMA、独立看门狗和 SD 卡日志的基础上，完成了心率、血氧、波形显示、串口上报与 CSV 落盘的主链路，适合作为后续继续扩展 `AD8232`、`DMA/ADC`、`RTOS` 等功能的基础版本。

This project is a physiological signal acquisition firmware based on `STM32F407ZGTx`, using `MAX30102 + SSD1306 + RTC + USART2`.  
The current version builds on a 100 Hz precise sampling tick, CMSIS-DSP bandpass filtering, autocorrelation BPM, I2C DMA transfers, independent watchdog, and SD-card CSV logging. It covers the main pipeline of heart-rate estimation, SpO2 estimation, waveform display, UART reporting, and data logging, and is suitable as a base for future extensions such as `AD8232`, `DMA/ADC`, and `RTOS`.

## 当前功能 / Current Features

- `MAX30102` 红光 / 红外采样，LED 驱动电流约 25.6 mA  
  `MAX30102` RED / IR sampling, LED drive current about 25.6 mA
- `TIM6` 100 Hz 精确采样节拍 + `WFI` 低功耗等待  
  `TIM6` 100 Hz precise sampling tick with `WFI` power-saving idle
- `MAX30102 INT` 引脚可选，当前默认禁用并使用 TIM6 轮询  
  `MAX30102 INT` pin is optional; current default uses TIM6 polling
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
- `IWDG` 独立看门狗，长循环和主循环路径必须持续喂狗  
  `IWDG` independent watchdog; long loops and main-loop paths must refresh it
- `FatFs + SDIO` CSV 日志，通过可失败、可重试路径初始化 SD 卡  
  `FatFs + SDIO` CSV logging with a fail-tolerant, retryable SD-card initialization path
- 调试页显示 `FIFO`、读数状态、`SQ (signal quality)`、`PI`  
  Debug page showing `FIFO`, read status, `SQ (signal quality)`, and `PI`

## 采样架构 / Sampling Architecture

系统采用 `TIM6` 产生 100 Hz 精确采样节拍（周期 10 ms），主循环在每个节拍内执行以下流程：

The system uses `TIM6` to generate a precise 100 Hz sampling tick (10 ms period). The main loop runs the following flow on each tick:

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
6. 刷新独立看门狗  
   Refresh the independent watchdog

节拍间通过 `__WFI()` 进入低功耗等待，TIM6 中断唤醒。  
Between ticks the CPU enters low-power wait via `__WFI()`, woken by the TIM6 interrupt.

`MAX30102 INT` 引脚当前定义为 `PE5`，通过 `MAX30102_USE_INT_PIN` 控制是否启用。当前默认值为 `0U`，也就是纯 `TIM6` 轮询模式；若硬件确认连接了 INT 引脚，可改为 `1U` 后再启用 EXTI 数据就绪路径。

The `MAX30102 INT` pin is currently defined as `PE5` and controlled by `MAX30102_USE_INT_PIN`. The default is `0U`, meaning pure `TIM6` polling mode. Set it to `1U` only after the hardware INT line is confirmed.

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
- `PE5` -> `MAX30102 INT`（可选，当前默认禁用 / optional, disabled by default）

### `SDIO`

- `PC8` -> `SDIO_D0`
- `PC9` -> `SDIO_D1`
- `PC10` -> `SDIO_D2`
- `PC11` -> `SDIO_D3`
- `PC12` -> `SDIO_CK`
- `PD2` -> `SDIO_CMD`

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

## 关键集成约定 / Critical Integration Rules

这一节是为了避免“只改注释或只对齐局部文件，却把运行链路拆散”的问题。修改代码前请先按这些约定检查。

### `IWDG` 看门狗

- `Core/Inc/stm32f4xx_hal_conf.h` 必须启用 `HAL_IWDG_MODULE_ENABLED`，否则 `IWDG_HandleTypeDef`、`HAL_IWDG_Init()`、`HAL_IWDG_Refresh()` 不可见。
- `Core/Src/main.c` 必须包含 `iwdg.h`，并在 `MX_IWDG_Init()` 后立即调用一次 `APP_Watchdog_Refresh()`。
- `MX_IWDG_Init()` 会真正启动硬件看门狗；启动后软件无法关闭，只能周期性喂狗或等待复位。
- `APP_Watchdog_Refresh()` 是唯一应用层喂狗入口。所有可能超过数百毫秒的路径都必须喂狗，包括：
  - `MAX30102` 初始化失败后的错误显示循环；
  - 开机无手指基线采集循环；
  - FIFO 密集补采样循环；
  - 主循环每轮调度末尾。
- `Error_Handler()` 中不要喂狗，保留致命错误后由看门狗复位的恢复策略。

### SD 卡与 FatFs

- 不要在 `main()` 开机阶段调用 `MX_SDIO_SD_Init()` 去初始化物理 SD 卡。无卡时它会把启动流程变成 `Error_Handler()`。
- `Core/Src/sdio.c` 中的 `MX_SDIO_SD_Init()` 只保留为 CubeMX 兼容的安全占位，不应直接调用 `HAL_SD_Init()`。
- SD 卡真实初始化由 `APP_DataLog_StartSession()` → `APP_SdFile_StartSession()` → `APP_SD_Card_InitHardware()` / `APP_SD_Card_Init()` 负责。这条链路允许失败并由上层重试，不能改成硬失败。
- `APP_SD_Card_Init()` 的职责是：
  - 以 48 MHz SDIOCLK、`APP_SD_INIT_CLK_DIV=118` 进入约 400 kHz 低速初始化；
  - 调用 `HAL_SD_Init()` 完成 HAL 层 SDIO 初始化和卡识别；
  - 获取卡信息；
  - 设置 `APP_SD_FAST_CLK_DIV=2`，通过 `HAL_SD_ConfigWideBusOperation()` 切换到 4-bit 与约 12 MHz 传输时钟；
  - 如果 4-bit 切换失败，清 `hsd.ErrorCode` 后回退 1-bit；若 1-bit 也失败，则返回 `APP_SD_CARD_ERROR`。
- `APP_SD_Card_Deinit()` 必须能清理初始化中途失败后的 SDIO/GPIO/NVIC 状态，不要只在 `card_initialized == true` 时才调用 `HAL_SD_DeInit()`。
- `Middlewares/Third_Party/FatFs/src/option/syscall.c` 当前只是 `_FS_REENTRANT == 0` 的占位文件；如果以后启用 RTOS/FatFs 重入，必须真正实现互斥钩子。

### 时钟与采样率

- `BME.ioc` 中 `PLLQ=7`，`RCC.PLLQCLKFreq_Value=48000000`。`main.c` 里的 `RCC_OscInitStruct.PLL.PLLQ` 必须保持为 `7`，否则 SDIO 时钟和 USB/48 MHz 域都会偏离配置。
- `TIM6` 当前是 100 Hz 调度节拍，算法常量 `MAX30102_ALGO_SAMPLE_RATE_HZ` 也是 100 Hz。不要把 README、TIM6、算法常量改成互相不一致的值。
- 主循环通过 `tim6_tick_flag` 和 `__WFI()` 等待 10 ms tick；不要在主循环里加入长阻塞任务。OLED、SD、I2C 异常路径都要保持可恢复。

### `main.c` 内部函数职责

- `app_state_init(AppState_t *app)`：清零 `AppState_t`，初始化测量状态、显示状态和串口协议状态。
- `app_send_report_if_due(AppState_t *app)`：当 `report_due` 置位时发送 USART2 测量报文，调用 `APP_DataLog_WriteRecord()` 写 CSV，并通过 `app_update_sd_log_status()` 同步 SD 日志状态。
- `app_refresh_display_if_needed(AppState_t *app)`：当 `display_refresh_requested` 置位时更新 RTC 快照、刷新 OLED 测量页，并记录刷新计数和 tick。
- `app_schedule_periodic_refresh(AppState_t *app)`：每 200 ms 置位 `report_due` 和 `display_refresh_requested`。
- `app_update_sd_log_status(AppState_t *app, AppDataLogStatus_t status)`：把 SD 文件会话状态同步到 `AppState_t`，包括 `sd_log_active`、`sd_card_ready`、`sd_log_error` 和 `sd_total_written`。

### SD/FatFs 函数职责

- `APP_DataLog_Init()`：初始化数据日志模块和底层 SD 文件模块状态。
- `APP_DataLog_StartSession()`：启动一次 CSV 文件会话，必要时挂载 FatFs、创建/追加当天文件并写入表头。
- `APP_DataLog_WriteRecord(const AppState_t *app)`：把当前测量状态写成多行 CSV 字段。失败时立即返回错误码，由 `main.c` 同步到 `AppState_t`。
- `APP_DataLog_IsReady()`：返回文件会话是否活跃，用于 UI/状态同步。
- `APP_DataLog_GetTotalWritten()`：返回累计落盘/flush 统计，用于状态显示和诊断。
- `APP_SdFile_StartSession()`：挂载卷、选择文件名、打开文件并定位到末尾，是 SD 文件会话的核心入口。
- `APP_SdFile_Write()`：写入缓冲区；无会话时按重试间隔尝试重新启动 session。
- `APP_SdFile_Flush()`：把缓冲写入 FatFs 并同步介质，失败时关闭会话。
- `APP_SD_Card_InitHardware()`：清零并准备静态 `SD_HandleTypeDef`，不访问物理卡。
- `APP_SD_Card_Init()`：执行 SDIO/SD 卡低速初始化、卡识别、总线宽度和高速分频切换。
- `APP_SD_Card_Read()` / `APP_SD_Card_Write()`：执行阻塞读写，失败或超时时 deinit，并等待卡回到 `HAL_SD_CARD_TRANSFER` 后才返回成功。
- `APP_SD_Card_GetHandle()`：给 `SDIO_IRQHandler()` 提供当前应用层持有的 SD handle。

## 给维护 Agent 的提示词 / Prompt for Maintenance Agents

下面这段可以直接复制给另一个代码 agent，用来约束它在本工程里的修改方式：

```text
你正在维护 STM32F407ZGTx 的 BME 工程，目录为 D:\CUBEMX\template\BME。请先阅读 README.md 的“关键集成约定 / Critical Integration Rules”，再改代码。你的目标是保持功能链路一致，不要为了注释、格式或局部编译而破坏运行时初始化顺序。

必须遵守：
1. 工程使用 Keil ARMCLANG，主工程是 MDK-ARM/BME.uvprojx。修改后必须运行 uVision rebuild，并确认 BME\BME.axf 为 0 Error(s), 0 Warning(s)。
2. IWDG 已启用：Core/Inc/stm32f4xx_hal_conf.h 必须保留 HAL_IWDG_MODULE_ENABLED。main.c 必须 include "iwdg.h"。MX_IWDG_Init() 启动硬件看门狗后必须立即 APP_Watchdog_Refresh()。所有长循环必须喂狗：MAX30102 错误循环、基线采集循环、FIFO 密集采样循环、主循环末尾。不要在 Error_Handler() 里喂狗。
3. SD 卡不能在 main() 开机阶段硬初始化。不要调用 MX_SDIO_SD_Init() 来探测物理卡；无卡不能导致 Error_Handler()。SD 日志只能通过 APP_DataLog_StartSession() -> APP_SdFile_StartSession() -> APP_SD_Card_InitHardware()/APP_SD_Card_Init() 的可失败、可重试路径启动。
4. Core/Src/sdio.c 的 MX_SDIO_SD_Init() 是 CubeMX 安全占位，只设置 handle 默认字段，不应调用 HAL_SD_Init() 或 HAL_SD_ConfigWideBusOperation()。
5. SDIO 时钟域必须保持 48 MHz：main.c 的 RCC_OscInitStruct.PLL.PLLQ 必须是 7，并与 BME.ioc 的 RCC.PLLQ=7 对齐。不要改成 4。
6. APP_SD_Card_Init() 的职责是：用 APP_SD_INIT_CLK_DIV=118 在 48 MHz SDIOCLK 下约 400 kHz 低速初始化；HAL_SD_Init() 完成卡识别；HAL_SD_GetCardInfo() 读取卡信息；把 hsd.Init.ClockDiv 改成 APP_SD_FAST_CLK_DIV=2；用 HAL_SD_ConfigWideBusOperation() 切 4-bit 和高速时钟；4-bit 失败时清 hsd.ErrorCode 并回退 1-bit；仍失败则返回 APP_SD_CARD_ERROR。不要再额外调用 HAL_SD_InitCard()，也不要在切 4-bit 后再调用 HAL_SD_Init()。
7. APP_SD_Card_Deinit() 必须能清理初始化中途失败后的 SDIO 状态。不要只在 card_initialized 为 true 时才调用 HAL_SD_DeInit()。
8. main.c 的函数职责不能混淆：
   - app_state_init：清零 AppState_t，并初始化 measurement/display/protocol 状态。
   - app_schedule_periodic_refresh：每 200 ms 置位 report_due 和 display_refresh_requested。
   - app_send_report_if_due：发送串口测量报文，调用 APP_DataLog_WriteRecord 写 CSV，并调用 app_update_sd_log_status 同步 SD 状态。
   - app_refresh_display_if_needed：更新 RTC 快照、刷新 OLED 页面、记录刷新计数和 tick。
   - app_update_sd_log_status：同步 sd_log_active、sd_card_ready、sd_log_error、sd_total_written。
9. 采样节拍是 TIM6 100 Hz，MAX30102_ALGO_SAMPLE_RATE_HZ 也是 100 Hz。README、代码和 .ioc 不能写成 50 Hz 或 20 ms。
10. MAX30102 INT 当前定义为 PE5，MAX30102_USE_INT_PIN 默认为 0U，即 TIM6 轮询模式。不要写成 PA0，也不要默认启用 EXTI，除非硬件确认并同步修改 main.h/gpio/it。
11. FatFs 的 Middlewares/Third_Party/FatFs/src/option/syscall.c 当前只是 _FS_REENTRANT == 0 的占位文件；不要删除它，因为 Keil 工程引用它。若启用 RTOS/FatFs 重入，必须实现真正的互斥钩子。
12. 如果只改注释，仍要确认注释与当前代码一致，尤其是频率、引脚、初始化顺序、看门狗策略和 SD 错误策略。不要把 UTF-8 中文注释转成乱码，也不要生成 .utf8/.bak 临时文件留在工程目录。

修改建议流程：
1. 先用 rg 搜索相关符号和工程引用，不要凭记忆改。
2. 修改前确认 README.md 与代码是否一致。
3. 改完运行 uVision rebuild。
4. 报告改动时说明是否影响 IWDG、SD/FatFs、时钟、TIM6 采样、MAX30102 INT。
```

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

这一版已经完成了 TIM6 100 Hz 精确节拍、MAX30102 轮询采样、I2C DMA 传输、CMSIS-DSP Butterworth 带通滤波、自相关 BPM、IWDG 看门狗和 SD 卡 CSV 日志的集成。系统以 10 ms 为周期运行，主循环在 `__WFI()` 低功耗等待中度过大部分时间。适合作为继续往 `AD8232` 心电、更强滤波、异常检测、`RTOS` 等方向扩展的基线。

This version integrates a TIM6 100 Hz precise tick, MAX30102 polling-based sampling, I2C DMA transfers, CMSIS-DSP Butterworth bandpass filtering, autocorrelation BPM, IWDG watchdog, and SD-card CSV logging. The system runs on a 10 ms cycle, with the main loop spending most of its time in `__WFI()` low-power wait. It serves as a solid baseline for extensions into `AD8232` ECG, stronger filtering, anomaly detection, or `RTOS`.
