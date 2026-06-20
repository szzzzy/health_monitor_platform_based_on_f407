# BME

基于 `STM32F407ZGTx` 的生理信号采集工程，主链路为 `MAX30102 PPG + AD8232 ECG + SSD1306 OLED + RTC + USART2 + SD/FatFs`。当前源码已经不是裸机主循环模型，而是 FreeRTOS 任务驱动模型：高优先级 `MAXtask` 负责 PPG FIFO drain 与 ECG DMA 样本消费，`Uitask` 负责按键、OLED 与串口报告，`SDtask` 在安全窗口后台写 SD，`watchdogtask` 负责 IWDG 与活体快照。

This project is an STM32F407ZGTx physiological-signal acquisition firmware. The current implementation is FreeRTOS based: `MAXtask` handles MAX30102 and ECG sample consumption, `Uitask` handles UI/UART, `SDtask` performs background SD logging in safe windows, and `watchdogtask` refreshes IWDG and stores liveness diagnostics.

## 当前状态 / Current Status

- MCU / 工具链：`STM32F407ZGTx`, CubeMX `6.3.0`, STM32Cube FW_F4 `V1.26.2`, Keil MDK-ARM `V5.32`, ARMCLANG `V6.24`.
  MCU / toolchain: `STM32F407ZGTx`, CubeMX `6.3.0`, STM32Cube FW_F4 `V1.26.2`, Keil MDK-ARM `V5.32`, ARMCLANG `V6.24`.
- PPG：`TIM6` 100 Hz 调度节拍，`MAX30102_ALGO_SAMPLE_RATE_HZ = 100U`。
  PPG: `TIM6` 100 Hz scheduling tick, `MAX30102_ALGO_SAMPLE_RATE_HZ = 100U`.
- ECG：`TIM2` 250 Hz TRGO 触发 `ADC1` DMA circular buffer，任务侧消费样本并执行 QRS/HR/RR/PTT 更新。
  ECG: `TIM2` 250 Hz TRGO triggers `ADC1` DMA circular buffer; the task consumes samples and performs QRS/HR/RR/PTT updates.
- 调度：`StartTask02()` 首次运行时调用 `app_rtos_mark_ready()` 并启动 `HAL_TIM_Base_Start_IT(&htim6)`，避免调度器启动前 ISR 调用 FreeRTOS API。
  Scheduling: `StartTask02()` calls `app_rtos_mark_ready()` and `HAL_TIM_Base_Start_IT(&htim6)` on its first iteration, preventing ISR calls to FreeRTOS API before the scheduler starts.
- 显示：SSD1306 仍是阻塞式 I2C 全屏刷新，但 `Uitask` 只用 `try-lock` 获取 I2C mutex，并在 FIFO 积压或强信号窗口时跳过/延迟刷新。
  Display: SSD1306 still uses blocking I2C full-screen refresh, but `Uitask` acquires the I2C mutex with try-lock only, and skips/defers refresh under FIFO pressure or strong signal windows.
- 串口：`USART2` 使用 DMA circular RX + IDLE 接收命令；TX 仍通过 `HAL_UART_Transmit()` 阻塞发送，单段超时 `100 ms`。
  UART: `USART2` uses DMA circular RX + IDLE for command reception; TX still uses blocking `HAL_UART_Transmit()`, single-segment timeout `100 ms`.
- SD 日志：当前是二进制日志，不是 CSV。实时路径只 `APP_DataLog_PushSample()` 写 16B 样本到 RAM ring；物理 `f_write/f_sync/f_close` 只在 `SDtask` 安全窗口执行。
  SD logging: currently binary, not CSV. The real-time path only calls `APP_DataLog_PushSample()` to write 16B records into a RAM ring; physical `f_write/f_sync/f_close` only happens in the `SDtask` safe window.

## 系统架构 / Architecture

```text
MAX30102 FIFO
  -> TIM6 100 Hz ISR
  -> MAXtask notification
  -> app_ecg_process_samples()
  -> max30102_should_service_fifo()
  -> app_measurement_drain_fifo_batch()
  -> PPG/BPM/SpO2/HRV/RR/PTT/state update
  -> APP_DataLog_PushSample() RAM ring

AD8232 analog output
  -> TIM2 TRGO 250 Hz
  -> ADC1 DMA circular buffer
  -> MAXtask consumes DMA samples
  -> ECG filter/QRS/HR/RR/PTT/quality update

AppState + waveform buffers
  -> Uitask
  -> OLED pages + USART2 M/T frames

DataLog RAM ring
  -> SDtask safe-window check
  -> APP_DataLog_ServiceBudget() / APP_DataLog_ServiceDeferredStop()
  -> APP_SdFile_WriteBytes()
  -> FatFs
  -> APP_SD_Card_Write()
  -> HAL_SD_WriteBlocks()
```

### FreeRTOS Tasks / 任务列表

| Task / 任务 | Priority / 优先级 | Main responsibility / 主要职责 |
| --- | --- | --- |
| `MAXtask` | `osPriorityHigh` | 100 Hz wake-up, ECG DMA consumption, MAX30102 FIFO drain, sensor recovery/watchdog / 100 Hz 唤醒，ECG DMA 消费，MAX30102 FIFO 排空，传感器恢复与看门狗 |
| `watchdogtask` | `osPriorityAboveNormal` | `APP_Watchdog_Refresh()` every 50 ms, BKP liveness snapshots about every 1 s / 每 50 ms 喂狗，约每秒保存 BKP 活体快照 |
| `Uitask` | `osPriorityNormal` | UART command polling, button handling, OLED refresh, periodic UART report, stack HWM sampling / UART 命令轮询、按键处理、OLED 刷新、周期上报、栈高水位采样 |
| `SDtask` | `osPriorityLow` | SD safe-window gating, binary log write/flush, SD status sync / SD 安全窗口门控、二进制日志写入与同步 |
| `defaultTask` | `osPriorityNormal` | Idle placeholder / 空闲占位 |

## 当前功能 / Current Features

- `MAX30102` RED/IR sampling, default LED current about `25.6 mA`.
  `MAX30102` 红光/红外采样，默认 LED 驱动电流约 `25.6 mA`。
- `TIM6` 100 Hz PPG service tick and `TIM2` 250 Hz ECG ADC trigger.
  `TIM6` 100 Hz PPG 调度节拍 + `TIM2` 250 Hz ECG ADC 触发。
- `MAX30102 INT` is intentionally disabled: `MAX30102_USE_INT_PIN = 0U`.
  `MAX30102 INT` 引脚有意禁用：`MAX30102_USE_INT_PIN = 0U`。
- `PE5` is reserved for `AD8232 LO-`; do not reuse it as MAX30102 INT.
  `PE5` 保留给 `AD8232 LO-`，不可复用为 MAX30102 INT。
- PPG algorithms include BPM, SpO2 ratio/balance, PI, SQ, motion hint, IBI, HRV time-domain/frequency-domain short-window metrics, and RR estimate.
  PPG 算法：BPM、SpO2 比率/平衡、PI、SQ、运动提示、IBI、HRV 时域/频域短窗口指标、RR 估计。
- ECG path includes DC removal, lowpass, 50 Hz notch, 10-20 Hz bandpass, derivative energy, 120 ms MWI, adaptive QRS threshold, ECG HR/RR/PTT, lead-off detection, and ECG quality scoring.
  ECG 链路：DC 去漂、低通、50 Hz 陷波、10-20 Hz 带通、差分能量、120 ms MWI、自适应 QRS 阈值、ECG HR/RR/PTT、导联脱落检测、ECG 质量评分。
- OLED pages: `PULSE`, `OXY`, `VITALS`, `ECG`, `ECG Q`, and debug pages `D1-D10`.
  OLED 页面：`PULSE`、`OXY`、`VITALS`、`ECG`、`ECG Q`，以及调试页 `D1-D10`。
- UART protocol supports periodic `M` measurement frames and `T` RTC set acknowledgements.
  串口协议支持周期性 `M` 测量帧和 `T` RTC 设置应答帧。
- SD logging uses a 2048-sample RAM ring and 512B background write chunks.
  SD 日志使用 2048 样本 RAM 环形缓冲，512B 后台写入块。
- Crash/liveness diagnostics are stored through backup registers and reported through UART/OLED diagnostic pages.
  崩溃/活体诊断通过备份寄存器存储，并通过串口/OLED 诊断页上报。

## 硬件与引脚 / Hardware and Pinout

| Item | Value |
| --- | --- |
| MCU | `STM32F407ZGTx` |
| PPG sensor | `MAX30102` |
| ECG front-end | `AD8232` |
| Display | `SSD1306 128x64 I2C OLED` |
| Clock | `HSE 8 MHz`, `LSE 32.768 kHz`, `PLLQ = 7`, 48 MHz domain enabled |

### I2C1

- `PB8` -> `I2C1_SCL`
- `PB9` -> `I2C1_SDA`
- MAX30102 and SSD1306 share this bus. / MAX30102 和 SSD1306 共享此总线。
- I2C1 is configured as Fast mode; board-level external pull-ups are required. / I2C1 配置为 Fast 模式，板上需外接上拉电阻。

### MAX30102

- Uses I2C1 register/FIFO access. / 通过 I2C1 访问寄存器/FIFO。
- `MAX30102_USE_INT_PIN` must remain `0U` unless a dedicated EXTI pin is added in hardware and software. / `MAX30102_USE_INT_PIN` 必须保持 `0U`，除非硬件和软件同步增加专用 EXTI 引脚。
- Current firmware relies on `TIM6` polling/notification, not the MAX30102 INT pin. / 当前固件依赖 `TIM6` 轮询/通知，不使用 MAX30102 INT 引脚。

### AD8232 / ADC1

- `PA5` -> `ADC1_IN5`, AD8232 analog output. / AD8232 模拟输出。
- `PE5` -> `AD8232 LO-`, red electrode lead-off detection. / 红色电极脱落检测。
- `PE6` -> `AD8232 LO+`, green electrode lead-off detection. / 绿色电极脱落检测。
- `TIM2` triggers ADC1 DMA at 250 Hz. / `TIM2` 以 250 Hz 触发 ADC1 DMA。

### SDIO

- `PC8` -> `SDIO_D0`
- `PC9` -> `SDIO_D1`
- `PC10` -> `SDIO_D2`
- `PC11` -> `SDIO_D3`
- `PC12` -> `SDIO_CK`
- `PD2` -> `SDIO_CMD`

Current SDIO policy: initialize/card-identify at about 400 kHz, try 4-bit once at 400 kHz, and stay at 400 kHz. Any 4-bit read/write/wait error disables 4-bit for the current power cycle.
当前 SDIO 策略：约 400 kHz 低速初始化与卡识别，尝试一次 4-bit 总线（400 kHz），失败后保持 400 kHz。任何 4-bit 读写/等待错误会在本次上电周期禁用 4-bit。

### USART2

- `PA2` -> `USART2_TX`
- `PA3` -> `USART2_RX`
- Default protocol is line-oriented text ending with `CRLF`. / 默认协议为行文本，以 `CRLF` 结尾。

### Buttons / 按键

- `PE2` -> brightness switch / 亮度切换
- `PE3` -> previous page / 上一页
- `PE4` -> next page / 下一页

## SD 二进制日志 / SD Binary Log

README 旧版本写过 CSV 落盘；当前源码已经改为二进制日志。
Earlier README versions described CSV logging; the current source has been changed to binary logging.

### File Naming / 文件命名

`APP_SdFile_StartSession()` uses RTC date plus sequence: / 使用 RTC 日期加序号：

```text
YYYYMMDD_NN.BIN
```

If RTC date is not valid, the prefix falls back to `00000000`. / 若 RTC 日期无效，前缀回退为 `00000000`。

### File Format / 文件格式

Each file starts with a 32-byte header: / 每个文件以 32 字节头开始：

```c
typedef struct __attribute__((packed)) {
  uint8_t  magic[4];      /* "BMLG" */
  uint16_t version;       /* 1 */
  uint16_t sample_rate_hz;
  uint32_t start_tick;
  uint8_t  reserved[20];
} DataLogFileHeader_t;
```

Then repeated 16-byte raw samples: / 之后重复 16 字节原始样本：

```c
typedef struct __attribute__((packed)) {
  uint32_t tick;
  uint32_t red;
  uint32_t ir;
  int16_t  ecg;
  uint8_t  flags;    /* bit0: finger, bit1: contact stable, bit2: FIFO overflow */
  uint8_t  seq;
} DataLogRawSample_t;
```

Runtime constraints: / 运行时约束：

- `APP_DataLog_PushSample()` is the only real-time logging entry; it only copies one 16B record into RAM. / `APP_DataLog_PushSample()` 是唯一实时日志入口，仅将一条 16B 记录拷贝到 RAM。
- Ring size is `2048` samples, about 20 s at 100 Hz. / 环形缓冲区大小为 `2048` 样本，100 Hz 下约 20 秒。
- Write chunk is `32` samples = `512 B`. / 写入块为 `32` 样本 = `512 B`。
- Ring full means oldest samples are overwritten and `dropped` is incremented; sampling must not wait for SD. / 缓冲区满时最旧样本被覆盖，`dropped` 递增；采样不得等待 SD。
- `measurement_active != 0` blocks all physical SD I/O. / `measurement_active != 0` 时阻塞所有物理 SD I/O。
- `APP_DataLog_ServiceDeferredStop()` drains and syncs after the finger leaves / measurement stops. / `APP_DataLog_ServiceDeferredStop()` 在手指离开/测量停止后排空并同步。

## USART2 Protocol / 串口协议

### Measurement Frame / 测量帧

STM32 currently emits a `110`-field `M` frame: / STM32 当前输出 `110` 字段 `M` 帧：

```text
M,rtc_valid,yyyymmdd,hhmmss,red,ir,baseline_ir,finger,bpm_valid,bpm,spo2_valid,spo2,rr_valid,rr,ibi_valid,ibi,hrv_valid,mean_ibi,sdnn,rmssd,motion_artifact,motion_score,sd1,sd2,sd1_sd2_x100,rhythm_irregular,hrv_freq_valid,lf_power_x100,hf_power_x100,lf_hf_x100,signal_quality,raw_signal_present,signal_ir_pi_x1000,signal_red_pi_x1000,signal_ir_ac_rms,signal_red_ac_rms,spo2_ratio_valid,spo2_ratio_x1000,spo2_balance_status,baseline_range_ir,adaptive_finger_on_delta,adaptive_finger_off_delta,ir_signal_delta,ir_signal_span,red_signal_span,finger_on_confirm_count,finger_off_confirm_count,sensor_last_read_status,sensor_error_streak,sensor_fifo_write_ptr,sensor_fifo_read_ptr,sensor_fifo_overflow_count,sensor_fifo_available_samples,sensor_read_ok_count,sensor_read_busy_count,sensor_read_error_count,sensor_recover_count,sensor_last_sample_tick,sensor_sample_change_count,sensor_sample_same_count,sensor_last_i2c_error,rtc_read_ok,uart_rx_message_valid,uart_tx_message_valid,sd_log_active,sd_state,sd_error,sd_total_written,display_refresh_count,display_last_refresh_tick,debug_mode,current_page,ecg_valid,ecg_hr,ecg_rr_ms,ecg_lead_off,ecg_r_peak_ms,ecg_filtered,ptt_valid,ptt_ms,ecg_sample_count,ecg_adc_sat_count,ecg_dma_overflow_count,ecg_lead_off_count,ecg_no_r_peak_timeout_count,crash_flag,crash_source,crash_task,crash_phase,crash_tick,reboot_count,reset_flags,max_task_phase,ui_task_phase,sd_task_phase,wdt_task_phase,max_task_stack_hwm,ui_task_stack_hwm,sd_task_stack_hwm,wdt_task_stack_hwm,max_task_heartbeat,ui_task_heartbeat,ecg_signal_quality,ecg_invalid_reason,ecg_raw_span,ecg_filtered_span,ecg_noise_level,ecg_qrs_threshold,ecg_peak_snr_x100,ecg_dma_available_high_watermark
```

Important appended ECG quality fields: / 尾部追加的 ECG 质量字段说明：

| Col / 列 | Field / 字段 | Meaning / 含义 |
| --- | --- | --- |
| 102 | `ecg_signal_quality` | 0-100 composite ECG quality / 0-100 ECG 综合质量评分 |
| 103 | `ecg_invalid_reason` | `0=OK, 1=LEAD_OFF, 2=ADC_SAT, 3=DMA_OVERFLOW, 4=NO_R_PEAK, 5=LOW_AMPLITUDE, 6=NOISY, 7=RAW_FLATLINE` |
| 104 | `ecg_raw_span` | Raw ECG min/max span / ECG 原始值跨度 |
| 105 | `ecg_filtered_span` | Filtered ECG min/max span / 滤波后 ECG 值跨度 |
| 106 | `ecg_noise_level` | Adaptive QRS noise baseline / QRS 自适应噪声基线 |
| 107 | `ecg_qrs_threshold` | Dynamic QRS threshold / 动态 QRS 阈值 |
| 108 | `ecg_peak_snr_x100` | Latest R-peak SNR x100 / 最新 R 峰信噪比 ×100 |
| 109 | `ecg_dma_available_high_watermark` | ADC DMA backlog high watermark / ADC DMA 积压高水位 |

### RTC Commands / RTC 校时命令

Accepted commands: / 接受的命令：

```text
SETTIME 2026-04-16 20:30:15
TIME=2026-04-16 20:30:15
TIME 2026-04-16 20:30:15
```

Response: / 应答：

```text
T,success,rtc_valid,yyyymmdd,hhmmss,reason
```

`success=1` means the RTC was set successfully. Every response line ends with `CRLF`. / `success=1` 表示 RTC 设置成功。每条应答行以 `CRLF` 结尾。

## Metric Availability / 指标有效性说明

- `SQ`: PPG signal quality based on PI, AC RMS, RED/IR balance, and window length; smoothed before display. / 基于 PI、AC RMS、RED/IR 平衡和窗口长度的 PPG 信号质量，显示前经平滑。
- `PI`: stored as `(AC/DC) x1000`, equivalent to PI percent x10. For example `123` displays as `12.3%`. / 存储为 `(AC/DC) ×1000`，等效 PI%×10。例如 `123` 显示为 `12.3%`。
- `R/BAL`: SpO2 ratio/balance requires non-zero RED/IR DC and measurable AC RMS. / SpO2 比率/平衡需非零 RED/IR DC 和可测量的 AC RMS。
- `MotionArtifact`: PPG-only motion hint. During motion, HR/SpO2/RR valid flags are cleared while old values may remain visible for context. / 纯 PPG 运动提示。运动期间 HR/SpO2/RR 有效标志清零，旧值可能保留以供参考。
- `IBI/REG`: requires accepted pulse intervals in the 300-2000 ms physiological window. / 需在 300-2000 ms 生理窗口内接受的心搏间隔。
- `SDNN/RMSSD/SD1/SD2`: require enough accepted IBI samples; SD1/SD2 are short-window Poincare descriptors, not diagnostic labels. / 需足够的已接受 IBI 样本。SD1/SD2 为短窗口 Poincare 描述符，不是诊断标签。
- `LF/HF`: short-window HRV estimate from the 32-beat IBI buffer using 4 Hz resampling, mean removal, Hann windowing, CMSIS-DSP RFFT, LF `0.04-0.15 Hz`, HF `0.15-0.40 Hz`. It is for engineering trend observation only, not standard 5-minute HRV spectral analysis. / 基于 32 拍 IBI 缓冲的短窗口 HRV 估计，工程趋势观察用，不是标准 5 分钟 HRV 频谱分析。
- `RR`: requires sufficient SQ, accepted beats, window length, and pulse-amplitude modulation. / 需足够的 SQ、已接受搏动数、窗口长度和脉搏幅度调制。
- `ECG HR/RR`: requires at least two valid R-peaks in the 300-2000 ms range. / 需在 300-2000 ms 范围内至少两个有效 R 峰。
- `PTT`: computed from ECG R-peak to IR PPG pulse peak; it is not a blood-pressure estimate. / 由 ECG R 峰到 IR PPG 脉搏峰计算；不是血压估计值。

## 工程结构 / Project Structure

```text
Core/
  Inc/                      Application headers
  Src/                      STM32 application, drivers, RTOS tasks
Drivers/                    STM32 HAL, CMSIS, CMSIS-DSP
Middlewares/Third_Party/    FatFs and FreeRTOS middleware
MDK-ARM/BME.uvprojx         Main Keil project
BME.ioc                     STM32CubeMX project file
docs/                       STM32 architecture notes and maintenance prompts
```

Only `MDK-ARM/BME.uvprojx` exists as the Keil project in this workspace. Do not rely on older README references to a `MAX30102_Simple_Test.uvprojx` file.

## Build / 编译

用 Keil uVision 打开 `MDK-ARM/BME.uvprojx` 并重建目标 `BME`。
Open `MDK-ARM/BME.uvprojx` in Keil uVision and rebuild target `BME`.

本机 CLI 重建示例 / CLI rebuild example on this machine:

```powershell
cd D:\CUBEMX\template\BME\MDK-ARM
& 'D:\Keil_v5\UV4\UV4.exe' -b BME.uvprojx -t BME -o build.log
```

Expected result for a clean firmware build is `0 Error(s), 0 Warning(s)` in the generated log. / 干净固件构建的预期结果是生成日志中 `0 Error(s), 0 Warning(s)`。

## Quick Start / 快速上手

1. Connect MAX30102, AD8232, SSD1306, USART2, buttons, and optional SD card according to the pinout above. / 按上述引脚连接 MAX30102、AD8232、SSD1306、USART2、按键和（可选）SD 卡。
2. Build and flash `MDK-ARM/BME.uvprojx`. / 编译并烧录 `MDK-ARM/BME.uvprojx`。
3. Power on without a finger on MAX30102; the boot sequence collects the no-finger baseline. / 上电时不要将手指放在 MAX30102 上；启动流程先采集无手指基线。
4. After the baseline page reports ready, place a finger on MAX30102. / 基线页显示就绪后，将手指放在 MAX30102 上。
5. Read `USART2` directly for `M` measurement frames. / 直接读取 `USART2` 获取 `M` 测量帧。
6. For SD logging, remember that `.BIN` files are raw binary logs with `BMLG` headers, not human-readable CSV files. / SD 日志方面，注意 `.BIN` 文件是带 `BMLG` 头的原始二进制日志，不是可读的 CSV 文件。

## Critical Integration Rules / 关键集成约定

### TIM6 / RTOS Scheduling / RTOS 调度

- Keep `TIM6` at 100 Hz and `MAX30102_ALGO_SAMPLE_RATE_HZ` at `100U`. / 保持 `TIM6` 100 Hz，`MAX30102_ALGO_SAMPLE_RATE_HZ` 为 `100U`。
- Do not start TIM6 interrupts in `main()` or `MX_FREERTOS_Init()`. / 不要在 `main()` 或 `MX_FREERTOS_Init()` 中启动 TIM6 中断。
- `StartTask02()` must remain the place that calls `app_rtos_mark_ready()` and `HAL_TIM_Base_Start_IT(&htim6)` after the scheduler is running. / `StartTask02()` 必须是调用 `app_rtos_mark_ready()` 和 `HAL_TIM_Base_Start_IT(&htim6)` 的唯一位置，在调度器运行之后执行。
- `HAL_TIM_PeriodElapsedCallback()` for TIM6 must remain short: increment diagnostics, set the MAX30102 data-ready flag, and notify `MAXtask`. / TIM6 的 `HAL_TIM_PeriodElapsedCallback()` 必须保持简短：递增诊断计数器、置位 MAX30102 数据就绪标志、通知 `MAXtask`。

### I2C1 Sharing / I2C1 共享

- MAX30102 and SSD1306 share I2C1. / MAX30102 和 SSD1306 共享 I2C1。
- `MAXtask` may wait briefly for the I2C mutex (`APP_RTOS_MAX_I2C_TIMEOUT_MS = 8U`). / `MAXtask` 可短暂等待 I2C 互斥锁。
- `Uitask` must not block waiting for I2C; OLED refresh uses a zero-timeout try-lock and may skip frames. / `Uitask` 不得阻塞等待 I2C；OLED 刷新使用零超时 try-lock，可跳过帧。
- Do not add OLED, printf, SD, or long HAL delays to the MAX/ECG real-time path. / 不要在 MAX/ECG 实时路径中加入 OLED、printf、SD 或长 HAL 延迟。

### SD / FatFs / SD 卡与 FatFs

- `MX_SDIO_SD_Init()` is a CubeMX compatibility placeholder. It must not probe the physical SD card or call `HAL_SD_Init()`. / `MX_SDIO_SD_Init()` 是 CubeMX 兼容占位，不得探测物理 SD 卡或调用 `HAL_SD_Init()`。
- Real SD initialization is lazy: `APP_DataLog_ServiceBudget()` -> `APP_SdFile_StartSession()` -> `APP_SD_Card_InitHardware()` / `APP_SD_Card_Init()`. / 真实 SD 初始化为延迟启动。
- No physical SD I/O is allowed while measurement is active. / 测量活跃期间禁止物理 SD I/O。
- Current SDIO policy is all-400 kHz with one 4-bit attempt, not a 12 MHz fast-mode switch. / 当前 SDIO 策略为全程 400 kHz，仅尝试一次 4-bit。
- On write/sync failure, the file layer intentionally drops/invalidates the session instead of blocking on repeated `f_sync` attempts. / 写入/同步失败时文件层有意丢弃会话，而非阻塞重试。
- If logging changes, preserve the invariant that sampling never waits for SD. / 修改日志时保持"采样不等待 SD"这一不变式。

### IWDG / Diagnostics / IWDG 与诊断

- `MX_IWDG_Init()` starts the hardware watchdog during boot. / `MX_IWDG_Init()` 在启动期间启动硬件看门狗。
- `watchdogtask` refreshes IWDG; fault hooks and `Error_Handler()` intentionally do not refresh it. / `watchdogtask` 喂狗；故障钩子和 `Error_Handler()` 有意不喂狗。
- Diagnostic phase fields (`max_task_phase`, `ui_task_phase`, `sd_task_phase`, `wdt_task_phase`) are part of the crash/liveness story; keep them updated around blocking or risky operations. / 诊断阶段码字段是崩溃/活体链路的一部分，在阻塞或高风险操作前后更新。

### STM32 Protocol / STM32 协议

- If the STM32 `M` frame field list changes, update `Core/Src/app_protocol.c` and this README in the same change. / 若 STM32 `M` 帧字段列表变更，同步更新 `Core/Src/app_protocol.c` 和本 README。
- Keep old fields append-only when possible so host tools can continue parsing the stable prefix. / 尽可能以追加方式保留旧字段，使上位机可继续解析稳定的前缀字段。

## Known Gaps / Follow-up / 已知缺口与后续

- USART TX is still blocking; high-rate or long reports can affect UI/SD timing. / USART TX 仍为阻塞式；高频或长报告可能影响 UI/SD 时序。
- OLED refresh is still full-screen blocking I2C, mitigated by try-lock/skip but not yet page-sliced. / OLED 刷新仍为全屏阻塞 I2C，由 try-lock/skip 缓解但尚未分页。
- SD log files are binary; a PC-side decoder/export tool would make them easier to inspect. / SD 日志为二进制格式；需要 PC 端解码/导出工具以便检查。
- `docs/architecture_audit_codex.md` is useful historical context, but some findings are stale now that TIM6->MAXtask startup has been implemented. Verify against source before copying conclusions. / `docs/architecture_audit_codex.md` 有历史参考价值，但部分发现已过时（TIM6→MAXtask 启动已实现），引用前请对照源码。
