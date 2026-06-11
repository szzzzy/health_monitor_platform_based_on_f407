# STM32F407 生物医学信号采集工程架构审查

审查日期：2026-06-11  
审查范围：当前仓库源码、CubeMX/MDK 工程文件、README、ESP32 侧桥接工程和 PC GUI 目录结构。  
硬性边界：本报告只做架构审查，不修改医学算法公式、阈值、滤波器或核心判定逻辑。

## 核心结论速览

当前工程不是简单裸机循环，已经迁移到 FreeRTOS 任务驱动：MAX30102 采集、OLED/USART UI、SD 日志、看门狗被拆成不同任务。方向是对的，尤其是 SD 日志已经采用 RAM ring buffer 和后台分片写入，这比在采样路径中直接 `f_write` 安全得多。

但当前架构仍然脆弱，不适合直接宣称为稳定的实时医学采集架构。最高优先级问题是：源码里看到了 TIM6 100 Hz 通知 MAXtask 的设计，但没有看到启动 TIM6 中断或调用 `app_rtos_mark_ready()` 的证据。这样 MAX30102 FIFO drain 会退化为 `MAXtask` 的 20 ms 超时轮询，实时性和 README 中的 100 Hz 节拍承诺不一致。

另一个高风险点是 OLED 全屏刷新使用阻塞式 I2C，并且与 MAX30102 共用 I2C1。即使 OLED 在 UI task 中执行，它仍可能长时间持有 I2C mutex，导致高优先级 MAXtask 等待总线，从而增加 FIFO backlog 和 overflow 风险。串口报告也使用阻塞式 `HAL_UART_Transmit`，不是采样路径里的直接阻塞源，但会增加系统抖动和后台任务延迟。

SD/FatFs 日志的总体方向可接受：实时采样路径只 `APP_DataLog_PushSample()` 入 RAM ring，不直接操作文件系统；物理 SD I/O 在低优先级 `SDtask` 中执行，并有“测量活跃时禁止 SD I/O”的门控。风险在于 `f_mount`、目录扫描、`f_sync`、`f_close`、底层 `HAL_SD_WriteBlocks` 仍可能出现 200 ms 级阻塞，只是被隔离到了后台任务；如果调度闭环、优先级或安全窗口判断失效，仍可能间接影响采样质量。

## 一、系统架构总览

### 主要模块和职责

| 模块 | 主要文件 | 职责 | 审查备注 |
|---|---|---|---|
| 启动与全局编排 | `Core/Src/main.c` | HAL 初始化、时钟、外设初始化、OLED 启动页、MAX30102 初始化、基线采集、ECG ADC 启动、启动 FreeRTOS | `main()` 最终进入 `osKernelStart()`，旧的裸机循环辅助函数仍保留但运行时基本不再作为主调度器 |
| RTOS 调度 | `Core/Src/freertos.c`, `Core/Inc/app_rtos.h` | 创建 `MAXtask`、`Uitask`、`SDtask`、`watchdogtask`，管理 I2C mutex，TIM6 ISR 到 MAXtask 通知 | 设计上是任务驱动，但 `app_rtos_mark_ready()` 未见调用，TIM6 未见启动 |
| MAX30102 驱动 | `Core/Src/max30102_driver.c`, `Core/Inc/max30102_driver.h` | 配置 MAX30102、读取 FIFO、检测 overflow、I2C 总线恢复辅助 | 当前 `MAX30102_USE_INT_PIN` 为 0，使用定时轮询/通知而非 INT 引脚 |
| PPG/血氧/心率处理 | `Core/Src/app_measurement.c`, `app_ppg_signal.c`, `app_ppg_pulse.c`, `app_spo2_filter.c`, `app_bpm_filter.c`, `app_hrv.c`, `app_rr.c`, `app_motion.c`, `app_oxy_status.c`, `app_ptt.c`, `max30102_spo2.c`, `max30102_bpm.c` | PPG 样本解析、手指状态、BPM、SpO2、RR/HRV/PTT、运动/质量状态 | 本报告不建议修改算法公式、阈值、滤波逻辑 |
| AD8232 ECG | `Core/Src/adc.c`, `Core/Src/app_ecg.c`, `Core/Inc/app_ecg.h` | TIM2 触发 ADC1，DMA 环形缓冲，任务侧消费 ECG 样本，QRS/HR/PTT 相关状态 | ADC DMA 路径比轮询安全，但需要验证 DMA buffer 放置和消费水位 |
| OLED 显示 | `Core/Src/app_display.c`, `Core/Src/ssd1306.c` | 页面渲染、PPG/ECG 波形、状态页、OLED I2C 刷屏 | 全屏刷新为阻塞式 I2C，且与 MAX30102 共用 I2C1，是实时性风险 |
| SD 日志应用层 | `Core/Src/app_data_log.c`, `Core/Inc/app_data_log.h` | RAM ring buffer、数据丢弃计数、后台分片写入、延迟 stop/flush | 方向正确，实时路径只 push RAM，不做 FatFs |
| FatFs 文件层 | `Core/Src/app_sd_file.c` | session start/stop、文件名序号、`f_write`、`f_sync`、`f_close` | 所有文件系统阻塞风险集中于此，应保持后台低优先级 |
| SDIO/磁盘层 | `Core/Src/app_sd_card.c`, `Core/Src/sd_diskio.c`, `Core/Src/sdio.c` | SDIO 初始化、block read/write、FatFs diskio 适配 | 当前源码显示 SDIO 保持 400 kHz 初始化分频，README 中速度描述可能已过时 |
| USART 协议 | `Core/Src/usart.c`, `Core/Src/app_protocol.c` | USART2 DMA RX、IDLE 中断、命令解析、CSV/状态上报 | RX 设计较安全，TX 仍是阻塞式 `HAL_UART_Transmit` |
| RTC | `Core/Src/rtc.c` | 日志文件时间、显示时间、FAT 时间戳 | 与采样实时性弱相关 |
| 看门狗/诊断 | `Core/Src/iwdg.c`, `Core/Src/app_diag.c`, `Core/Src/freertos.c` | IWDG 刷新、崩溃上下文保存、生命周期状态 | 当前 watchdog task 能证明调度器存活，不一定能证明 MAX/ECG 采样健康 |
| ESP32 桥接 | `ESP32 WIFI+MQTT/` | UART/WiFi/MQTT 桥接 | 仓库包含该侧工程，但本次主要审查 STM32 端实时采样架构 |
| PC GUI | `GUI/` | 上位机监视/展示 | 不是 STM32 端实时风险来源 |

### 数据流

```text
MAX30102 FIFO
  -> TIM6/任务通知或 20 ms 超时轮询
  -> MAXtask
  -> app_measurement_drain_fifo_batch()
  -> max30102_read_fifo_batch()
  -> app_measurement_process_parsed_sample()
  -> PPG/SpO2/BPM/HRV/RR/PTT/质量状态
  -> AppState + OLED 波形缓冲
  -> APP_DataLog_PushSample() 写入 RAM ring

AD8232 模拟输出
  -> TIM2 TRGO 250 Hz
  -> ADC1 DMA circular buffer
  -> MAXtask 中 app_ecg_process_samples()
  -> ECG 滤波/QRS/HR/PTT 状态
  -> AppState + OLED ECG 波形缓冲
  -> 日志样本中的 ecg_raw/ecg_flags

AppState / waveform buffers
  -> Uitask
  -> OLED 页面刷新
  -> USART2 调试/状态报告

RAM log ring
  -> SDtask
  -> APP_DataLog_ServiceBudget() / ServiceDeferredStop()
  -> APP_SdFile_WriteBytes()
  -> FatFs
  -> sd_diskio
  -> APP_SD_Card_Write()
  -> HAL_SD_WriteBlocks()
```

### 当前驱动模型判断

当前工程偏 RTOS 任务驱动：

| 证据 | 文件/函数 |
|---|---|
| `main()` 初始化外设后调用 `osKernelInitialize()`、`MX_FREERTOS_Init()`、`osKernelStart()` | `Core/Src/main.c` |
| 创建 `MAXtask`、`watchdogtask`、`Uitask`、`SDtask` | `Core/Src/freertos.c:447` 到 `Core/Src/freertos.c:459` |
| MAX 采集由 `StartTask02()` 驱动 | `Core/Src/freertos.c:496` |
| OLED/USART/UI 由 `StartTask04()` 驱动 | `Core/Src/freertos.c:561` |
| SD 日志由 `StartTask05()` 驱动 | `Core/Src/freertos.c:676` |

但也存在“RTOS 迁移未闭环”的痕迹：

| 现象 | 说明 |
|---|---|
| `main.c` 中仍有旧的 `app_send_report_if_due()`、`app_refresh_display_if_needed()`、`app_sd_service_safe()` 辅助函数 | 看起来来自裸机主循环版本，当前 `osKernelStart()` 后不会回到传统 while 主循环 |
| `HAL_TIM_PeriodElapsedCallback()` 中设置 `tim6_tick_flag` 并通知 MAXtask | `Core/Src/main.c:456` 到 `Core/Src/main.c:463` |
| 只搜到 TIM7 的 `HAL_TIM_Base_Start_IT()` | `Core/Src/stm32f4xx_hal_timebase_tim.c:82`，未看到 TIM6 启动 |
| `app_rtos_mark_ready()` 只有定义和声明，未看到调用 | `Core/Src/freertos.c:69`, `Core/Inc/app_rtos.h:12` |

因此，应将当前状态描述为“RTOS 架构方向正确，但采样节拍闭环需要修复和验证”。

## 二、实时关键路径分析

### 路径清单

| 路径 | 入口 | 当前操作 | 实时性评价 |
|---|---|---|---|
| TIM6 中断路径 | `TIM6_DAC_IRQHandler()` -> `HAL_TIM_PeriodElapsedCallback()` | `max30102_mark_data_ready_from_isr()`、`app_rtos_notify_max_from_isr()` | ISR 内容很短，适合中断；但未看到 TIM6 启动和 ready 标志设置 |
| MAXtask | `StartTask02()` | 等待通知或 20 ms 超时，然后 `app_rtos_service_max()` | 是 MAX FIFO drain 和 ECG DMA 消费的核心实时任务 |
| ECG DMA | TIM2 TRGO -> ADC1 DMA2 Stream0 | DMA 写环形缓冲，任务侧消费 | 采样由硬件定时，较安全；消费延迟仍可能导致软件 overflow |
| UI task | `StartTask04()` | UART 命令、按键、OLED 刷新、USART 报告 | 正常优先级，但 OLED 和 UART 均有阻塞调用 |
| SD task | `StartTask05()` | 安全窗口判断，后台 FatFs 写入/flush | 低优先级，方向正确；FatFs 仍可能长阻塞 |
| Watchdog task | `StartTask03()` | 周期性刷新 IWDG | 只能证明 task 还在跑，不能直接证明采样质量健康 |
| USART RX ISR | `USART2_IRQHandler()` | IDLE 标志 + HAL IRQ | 不做复杂解析，基本合理 |
| SDIO IRQ | `SDIO_IRQHandler()` | `HAL_SD_IRQHandler(APP_SD_Card_GetHandle())` | 当前 SD 读写使用阻塞 API，IRQ 不是主要风险 |

### 风险排序

| 排名 | 风险点 | 文件和函数 | 为什么影响实时性 | 可能现象 | 推荐处理 |
|---:|---|---|---|---|---|
| 1 | TIM6 采样通知闭环未完成 | `Core/Src/main.c:456` `HAL_TIM_PeriodElapsedCallback()`；`Core/Src/freertos.c:69` `app_rtos_mark_ready()`；`Core/Src/freertos.c:509` `ulTaskNotifyTake()` | 设计上 MAXtask 应由 TIM6 100 Hz 唤醒，但未看到 `HAL_TIM_Base_Start_IT(&htim6)` 或 `app_rtos_mark_ready()` 调用。通知可能永远不生效，MAXtask 退化为 20 ms timeout | MAX30102 FIFO drain 不稳定，FIFO high watermark 升高，SD/OLED/USART 开启后更容易 overflow | 第一个 patch 应闭合 TIM6 -> ISR -> MAXtask 链路，并增加 heartbeat/sample-gap 计数验证 |
| 2 | OLED 全屏阻塞式 I2C 与 MAX30102 共用 I2C1 | `Core/Src/freertos.c:617` 附近 UI 刷新；`Core/Src/app_display.c` 多处 `ssd1306_UpdateScreen()`；`Core/Src/ssd1306.c:254` | OLED 一次全屏刷新会持有 I2C mutex，MAXtask 即使高优先级也必须等总线释放；SSD1306 单次 I2C timeout 为 100 ms | MAXtask 等 I2C 超时、FIFO drain 延后、PPG 断点、OLED 开启时更不稳定 | 将 OLED 刷新拆成分页/分片，限制单次 I2C 持有时间；继续保持 UI try-lock，不允许 OLED 等待 MAX |
| 3 | 串口 TX 阻塞发送长报告 | `Core/Src/app_protocol.c:354`, `Core/Src/app_protocol.c:364` | 115200 baud 下长 CSV/JSON-like 报告可能占用几十毫秒到 100 ms timeout。虽然在 UI task，仍会消耗 CPU、推迟低优先级 SD，增加系统抖动 | 串口报告期间 UI/SD 卡顿，后台日志服务延迟，严重时看门狗只能证明系统还在等 | 改成 USART TX DMA/ring buffer，或 bounded nonblocking 发送；保持报告格式不变 |
| 4 | FatFs start/stop 和底层 SD 写可能长阻塞 | `Core/Src/app_sd_file.c:223` `f_mount`；`:239`/`:242` `f_open`；`:279` `f_sync`；`:280` `f_close`；`:323` `f_write`；`Core/Src/app_sd_card.c:291` | FatFs 和 `HAL_SD_WriteBlocks()` 是阻塞 API，底层 timeout 200 ms；当前被隔离到 SDtask，但如果安全窗口判断失效或系统调度有问题，仍可能影响采样 | 启用 SD 后主观“系统不稳”、日志 backlog 增长、watchdog 复位或采样间隔异常 | 保持 SDtask 低优先级；采样活跃时禁止物理 SD I/O；记录写入耗时/backlog/drop；坏卡进入 backoff |
| 5 | MAX/I2C 恢复路径在高优先级任务中阻塞 | `Core/Src/app_measurement.c:1019` `HAL_Delay(20U)`；`max30102_init()` 内也有 delay | 恢复发生在 MAXtask，故障时会阻塞 ECG DMA 消费和后续 FIFO drain | I2C 错误后出现 ECG DMA overflow、采样长间隙、状态抖动 | 保持为故障路径，但增加恢复耗时/次数计数；后续可拆为状态机 |
| 6 | `AppState_t` 和波形缓冲多任务共享无快照 | `app_measurement.c` 写状态；`app_display.c`/`app_protocol.c` 读状态；`app_display_add_*` 写波形 | 多字段状态读取不具备一致性，波形缓冲可能一边写一边画 | OLED 显示撕裂、串口报告字段跨样本混合、难以复现实验数据 | 增加 UI/USART snapshot 或序列号，波形用双缓冲；不改变算法状态 |
| 7 | ECG DMA 消费依赖 MAXtask 调度 | `Core/Src/app_ecg.c:293` `app_ecg_process_samples()` 在 MAXtask 中调用；`Core/Src/adc.c` DMA 环形缓冲 | ADC 采样本身稳定，但消费被 MAXtask/OLED/I2C 恢复影响；若 MAXtask长时间阻塞，DMA 环形缓冲会覆盖 | ECG overflow flag、QRS/HR/PTT 状态重置、ECG 波形断续 | MAXtask 必须按 10 ms 周期运行；增加 ECG backlog/max-lag 计数 |
| 8 | 看门狗可能掩盖采样故障 | `Core/Src/freertos.c` `StartTask03()` | watchdog task 独立刷新 IWDG，只能证明 scheduler 和该 task 存活 | MAXtask 卡死或 FIFO 长期 overflow，但系统不复位，表现为“活着但数据坏” | 看门狗喂狗条件加入 MAX heartbeat、ECG overflow、FIFO stale 等 liveness 条件 |

## 三、SD / FatFs 日志风险分析

### FatFs 调用位置

以下表格只列应用层显式 FatFs 调用，不包含 FatFs 中间件内部实现。

| 调用 | 文件和位置 | 所在函数 | 说明 |
|---|---|---|---|
| `f_mount(NULL, "", 0)` | `Core/Src/app_sd_file.c:66` | `close_session_after_error()` | 错误清理，避免 `f_close` 触发额外 I/O |
| `f_findfirst()` | `Core/Src/app_sd_file.c:140` | `find_next_sequence()` | 扫描日志文件序号，目录文件多时可能慢 |
| `f_findnext()` | `Core/Src/app_sd_file.c:154` | `find_next_sequence()` | 同上 |
| `f_closedir()` | `Core/Src/app_sd_file.c:160` | `find_next_sequence()` | 关闭目录 |
| `f_mount(&fatfs, "", 1)` | `Core/Src/app_sd_file.c:223` | `APP_SdFile_StartSession()` | 挂载并触发 disk initialize |
| `f_open(... FA_OPEN_APPEND | FA_WRITE)` | `Core/Src/app_sd_file.c:239` | `APP_SdFile_StartSession()` | 打开已有日志 |
| `f_open(... FA_CREATE_NEW | FA_WRITE)` | `Core/Src/app_sd_file.c:242` | `APP_SdFile_StartSession()` | 创建新日志 |
| `f_lseek(&log_file, f_size(&log_file))` | `Core/Src/app_sd_file.c:254` | `APP_SdFile_StartSession()` | seek 到文件末尾 |
| `f_sync(&log_file)` | `Core/Src/app_sd_file.c:279` | `APP_SdFile_StopSession()` | flush 文件系统元数据和数据，可能多次 block write |
| `f_close(&log_file)` | `Core/Src/app_sd_file.c:280` | `APP_SdFile_StopSession()` | FatFs 的 close 本身也可能 sync |
| `f_mount(NULL, "", 1)` | `Core/Src/app_sd_file.c:285` | `APP_SdFile_StopSession()` | 卸载卷 |
| `f_write()` | `Core/Src/app_sd_file.c:323` | `APP_SdFile_WriteBytes()` | 实际写日志 chunk |
| `f_sync(&log_file)` | `Core/Src/app_sd_file.c:346` | `APP_SdFile_Flush()` | 当前未看到应用层调用该函数 |

### 是否可能阻塞实时采样路径

从当前源码看，实时采样路径没有直接调用 `f_write`、`f_sync`、`f_open`、`f_close`：

| 采样路径 | 是否直接 FatFs | 证据 |
|---|---|---|
| MAXtask -> `app_measurement_drain_fifo_batch()` -> `app_measurement_process_parsed_sample()` | 否 | 样本日志入口是 `APP_DataLog_PushSample()`，该函数只写 RAM ring |
| ECG DMA ISR | 否 | DMA ISR 只 `HAL_DMA_IRQHandler(&hdma_adc1)` |
| ECG 消费 `app_ecg_process_samples()` | 否 | 处理 DMA 样本和算法状态，不访问 FatFs |
| SDtask | 是 | `APP_DataLog_ServiceBudget()` 和 `ServiceDeferredStop()` 调用 `APP_SdFile_*`，低优先级后台执行 |

因此，当前 SD 日志设计的基本方向正确：不为了 SD 成功牺牲采样连续性。风险不在“采样函数直接 f_write”，而在“后台阻塞过长、系统调度闭环不完整、OLED/I2C/UART 等其他阻塞叠加后，采样任务不能按期执行”。

### 当前 SD 写入方式对系统的影响

| 影响对象 | 当前风险判断 | 依据 |
|---|---|---|
| MAX30102 FIFO overflow | 中高风险，间接风险 | SD 不直接占 I2C，但 FatFs/SDIO 可能长时间占 CPU；更关键的是 TIM6 通知未闭环和 OLED 占 I2C 会让 FIFO drain 不稳定 |
| ECG ADC 采样抖动 | 低到中风险 | ADC 由 TIM2+DMA 硬件采样，不受 SD 直接打断；但软件消费若被 MAXtask阻塞或系统负载过高，会出现 DMA 覆盖 |
| 主循环卡顿 | 当前没有传统主循环，表现为低优先级任务卡顿 | `SDtask` 低优先级阻塞会影响自身和同/低优先级任务，不应阻塞高优先级 MAXtask |
| 看门狗复位 | 中风险 | 如果 SD 底层阻塞导致调度器或高优先级 task 失活，可能复位；反过来 watchdog 独立刷新也可能掩盖采样故障 |
| 日志丢失 | 可接受但必须可观测 | `APP_DataLog_PushSample()` ring 满时丢弃最旧样本并计数，这是“保证采样优先”的合理策略，但必须在 OLED/USART 暴露 drop/overflow |

### 推荐的非阻塞 SD 日志架构

当前已有架构应保留并加强：

```text
采样任务
  -> O(1) 写 RAM ring
  -> ring 满则丢弃最旧数据，递增 drop counter
  -> 绝不等待 SD

SD 后台任务
  -> 低优先级
  -> 只在 measurement inactive / safe window 写入
  -> 每次最多写一个 512 B chunk
  -> 记录 write_ms、max_write_ms、backlog_high_watermark、error_code
  -> 卡慢/错误则进入 backoff，暂停日志，不影响采样
```

当 SD 卡写入速度跟不上时，建议策略：

| 问题 | 建议 |
|---|---|
| 是否允许丢弃最旧数据 | 允许。医学信号采集优先级是实时采样连续性，不能让 SD 反压采样任务 |
| 是否记录 overflow/drop 计数 | 必须记录。至少记录 `ring_drop_total`、`fifo_overflow_total`、`sd_write_error_total`、`sd_backoff_until`、`max_backlog` |
| 是否暂停日志但保证采样 | 必须。SD 慢或错误时暂停日志，采样和算法继续运行 |
| 是否在 OLED/USART 提示 | 必须。OLED 可显示 `LOG PAUSED`、`DROP n`、`SD ERR`；USART 报告应包含状态码和计数器 |
| 是否在测量活跃时强行 flush | 不允许。`f_sync`、`f_close` 不应进入 active measurement 窗口 |

## 四、RTOS 适用性评估

### 是否真的需要 RTOS

本项目有多个不同实时等级的活动：

| 活动 | 实时等级 | 特点 |
|---|---|---|
| MAX30102 FIFO drain | 高 | 100 Hz 级，必须及时 drain FIFO |
| ECG ADC 采样 | 高 | 硬件 TIM+DMA 采样，软件需及时消费 |
| OLED 刷新 | 低 | 可以跳帧、分页、延迟 |
| SD 日志 | 最低 | 可以暂停、丢弃旧日志、backoff |
| USART 报告 | 低 | 可以限速、丢弃过期状态 |
| 看门狗/监控 | 中高 | 需要监控采样健康，不应只监控 scheduler |

因此，RTOS 有价值。它可以清晰表达“采样高优先级、SD/OLED/USART 后台化”的架构意图。当前更合理的结论是：保留 RTOS，但先修复 RTOS 采样节拍闭环，并把阻塞外设操作严格约束在低优先级后台任务。

### 建议任务划分

| 任务 | 建议优先级 | 周期/触发 | 通信方式 | 禁止操作 |
|---|---:|---|---|---|
| MAX30102 采集任务 | High | TIM6 100 Hz notify，必要时立即补 drain | task notification，I2C mutex，RAM ring push | FatFs、OLED 刷新、阻塞 UART、长时间 printf、长时间 `HAL_Delay` |
| ECG 采样/消费任务 | High 或并入 MAXtask | ADC 由 TIM2+DMA；消费 5 到 10 ms | DMA circular buffer，计数器，必要时独立 task notification | FatFs、OLED、UART 阻塞发送、复杂非必要格式化 |
| SD 日志任务 | Low | 20 ms tick 或事件触发；只在 safe window | RAM ring pop，状态码，backoff timer | 影响采样的等待；active measurement 中 `f_write/f_sync/f_open/f_close` |
| OLED 显示任务 | Normal 或 BelowNormal | 100 到 500 ms，分片刷新 | AppState snapshot，I2C try-lock | 等待 MAX 释放 I2C、全屏长时间持锁、采样状态修改 |
| 串口调试任务 | Low/Normal | 限速事件发送 | TX ring/DMA，RX DMA idle | 长时间 blocking TX、在采样路径 printf |
| 系统监控/看门狗任务 | AboveNormal | 50 到 100 ms | heartbeat counters，error counters | 无条件喂狗、在 WDT 中做复杂 I/O |

### Mutex 使用建议

| 资源 | 是否需要 mutex | 说明 |
|---|---|---|
| I2C1 总线 | 需要 | MAX30102 和 OLED 共用 I2C1。MAXtask 可短等待，OLED 只 try-lock 或分片 |
| FatFs / SD 文件对象 | 需要或单任务拥有 | 当前只有 SDtask 使用 FatFs，单 owner 即可；若未来多任务访问，必须 mutex |
| USART TX | 建议单 owner 或 ring buffer | 多任务直接 `HAL_UART_Transmit` 会互相阻塞和串包 |
| AppState 多字段快照 | 不建议长期 mutex | 高优先级采样不应等 UI；使用 copy snapshot、sequence counter 更合适 |
| ECG DMA buffer | 不建议 mutex | DMA 和 task 之间用 producer/consumer index、水位和 overflow 标志 |
| MAX FIFO drain | 不应被低优先级 mutex 长时间阻塞 | I2C mutex 的持有时间必须受控，尤其 OLED |

### 若暂不用 RTOS的替代方案

如果为了面试项目降低复杂度，也可以采用裸机 cooperative scheduler：

```text
TIM6 ISR 只置位 sample_tick
main loop:
  if sample_tick: drain MAX FIFO + consume ECG DMA
  if display_due and i2c_free and fifo_watermark_low: refresh one OLED page
  if sd_due and measurement_inactive: write one SD chunk
  if uart_due: send small nonblocking fragment
  feed watchdog only if sample heartbeat advances
```

但当前仓库已经拆出 RTOS 任务，回退裸机会带来额外迁移成本。结论：保留 RTOS 更合理，但必须把 RTOS 采样节拍、优先级、资源占用和可观测性补齐。

## 五、DMA / 中断 / 竞态风险审查

### 中断和回调

| 中断/回调 | 文件 | 当前内容 | 是否合适 |
|---|---|---|---|
| `TIM6_DAC_IRQHandler()` | `Core/Src/stm32f4xx_it.c:243` | 进入 HAL TIM IRQ | 合适 |
| `HAL_TIM_PeriodElapsedCallback()` | `Core/Src/main.c:456` | TIM6 设置 MAX ready 标志并通知 task；TIM7 HAL tick | 内容短，合适；但 TIM6 启动缺证据 |
| `DMA2_Stream0_IRQHandler()` | `Core/Src/stm32f4xx_it.c:271` | ADC DMA IRQ -> HAL | 合适 |
| `USART2_IRQHandler()` | `Core/Src/stm32f4xx_it.c:291` | IDLE 检测 + HAL IRQ | 不解析命令，合适 |
| `DMA1_Stream5_IRQHandler()` | `Core/Src/stm32f4xx_it.c:285` | USART RX DMA IRQ | 合适 |
| `DMA1_Stream0/6_IRQHandler()` | `Core/Src/stm32f4xx_it.c:302`, `:308` | I2C DMA IRQ | 当前 MAX/OLED主要用阻塞 I2C，DMA 句柄存在但不是主要路径 |
| `HAL_GPIO_EXTI_Callback()` | `Core/Src/stm32f4xx_it.c:329` | MAX INT 模式下标记 ready | 当前 `MAX30102_USE_INT_PIN` 为 0，路径基本禁用 |
| `SDIO_IRQHandler()` | `Core/Src/stm32f4xx_it.c:342` | HAL SD IRQ | 当前 SD block I/O 是阻塞式，IRQ不是核心路径 |

当前 ISR 中没有看到 FatFs、OLED 刷新、复杂算法、`printf`、`HAL_Delay` 等明显不适合 ISR 的操作，这是优点。

### 共享变量和保护

| 共享对象 | 文件 | 当前保护 | 风险 |
|---|---|---|---|
| `max30102_data_ready_flag`, `max30102_int_seen` | `Core/Src/max30102_driver.c:33` | `volatile` | 基本合适 |
| `app_rtos_ready` | `Core/Src/freertos.c:67` | `volatile` | 变量本身合适，但未看到置 1 调用 |
| `tim6_tick_flag` | `Core/Src/main.c:74` | `volatile` | 旧裸机标志，目前可能无人消费 |
| USART RX DMA 指针和 idle flag | `Core/Src/usart.c:35` 到 `:37` | `volatile` + task poll | 基本合理；如果 UI 长时间阻塞，64 B RX buffer 可能丢命令 |
| ECG DMA ring state | `Core/Src/adc.c:29` 到 `:32` | 部分 `volatile`，软件计数 task 侧维护 | DMA buffer 本身未标 `volatile`，Cortex-M4 无 cache 通常可行；需验证覆盖计数 |
| DataLog ring | `Core/Src/app_data_log.c` | 短临界区 `__disable_irq()` | O(1) push/pop 合理；RTOS 下全局关中断应保持极短 |
| `AppState_t` | 多文件 | 无整体锁 | 多字段状态可能被 UI/USART 读到不一致快照 |
| OLED 波形缓冲 | `app_display.c` | 无双缓冲 | MAX/ECG 写入与 UI 绘制可能并发，显示可撕裂 |

### DMA 和任务交接安全性

ECG 路径的设计方向是正确的：TIM2 触发 ADC，ADC1 DMA circular buffer，软件通过 DMA NDTR 推算写入位置并消费样本。这比在 ISR 中逐样本处理可靠。

需要补强的地方：

| 风险 | 建议 |
|---|---|
| ADC DMA buffer 使用固定地址 `0x2001F800` | 必须检查 `.map` 文件，确认没有和栈、heap、其他段重叠。源码无法单独证明链接器一定按预期放置 |
| 消费频率依赖 MAXtask | 修复 TIM6 100 Hz notify 后再测 ECG backlog；必要时独立 ECG consume task |
| DMA overflow 后重置算法状态 | 这是合理保护，但应把 overflow 计数暴露给 OLED/USART，方便面试和调试说明 |

### 竞态和状态丢失位置

| 位置 | 风险 | 严重性 | 处理建议 |
|---|---|---:|---|
| `AppState_t` 被 MAXtask 更新、UI/USART/SDtask 读取 | 多字段跨样本不一致 | 中 | 增加 snapshot 或 sequence counter |
| waveform buffer 写/画并发 | OLED 波形撕裂 | 低到中 | 双缓冲或绘制前复制一帧 |
| UART RX 64 B circular buffer | UI 阻塞时命令覆盖 | 低 | 增大 buffer 或让 RX parser 独立、更频繁 |
| DataLog counters 无统一快照 | 状态显示可能瞬时不一致 | 低 | 读取时短临界区或提供 `GetSnapshot()` |
| MAX sensor recovery 持有 I2C mutex | 恢复期间 UI 被阻塞，MAXtask自身延迟 | 中 | 记录恢复耗时，后续状态机化 |

## 六、硬件-软件交互风险

### 可以从源码判断的事实

| 事实 | 证据 | 风险含义 |
|---|---|---|
| MAX30102 和 OLED 共用 I2C1 | `max30102_driver.c` 使用 `hi2c1`；`ssd1306.c` 使用 `hi2c1` | OLED 刷新会竞争 MAX FIFO drain 总线 |
| MAX30102 INT 引脚当前禁用 | `Core/Inc/main.h` 中 `MAX30102_USE_INT_PIN 0U` | 当前依赖 TIM6/任务轮询而非硬件 INT |
| AD8232 ADC 使用 TIM2 + DMA | `Core/Inc/app_ecg.h` 注释和 `Core/Src/adc.c` | ADC 采样时基独立于主任务，利于稳定采样 |
| USART2 RX 用 DMA circular + IDLE | `Core/Src/usart.c` | RX 较安全，TX 仍阻塞 |
| SDIO block I/O 使用阻塞 HAL API | `Core/Src/app_sd_card.c:254`, `:291` | SD 写慢时后台任务会长时间卡住 |
| I2C GPIO 初始化为 AF open-drain NOPULL | `Core/Src/i2c.c` | 板上需要外部上拉，源码不能证明硬件上拉存在 |

### 必须通过实验验证的问题

| 问题 | 为什么源码不能证明 | 验证方法 |
|---|---|---|
| SD 写入电流脉冲是否影响 AD8232/MAX30102 | 源码没有原理图、PCB、电源树、去耦、电流波形 | 示波器测 3.3 V、模拟前端供电、MAX30102 供电；对比 SD on/off 写入 burst |
| OLED 刷新实际占用 I2C 多久 | 源码可估算，但真实耗时取决于 I2C 速率、总线电容、HAL overhead | 在 `ssd1306_UpdateScreen()` 前后翻 GPIO 或记录 cycle/tick |
| MAX30102 FIFO 是否被及时 drain | 源码显示有 high watermark/overflow 计数，但需运行数据 | UART 输出 `fifo_high_watermark`、`fifo_overflow_total`、max sample gap |
| AD8232 ADC 采样是否稳定 | DMA 配置可见，但模拟噪声、导联、供电不可见 | 记录 ECG DMA overflow、sample interval；示波器看 PA5；导联接入/断开测试 |
| USART debug 是否造成阻塞影响 | 源码能看到 blocking TX，实际影响需测 | 记录 TX duration、MAXtask heartbeat jitter；对比关闭/开启报告 |
| Watchdog 是否掩盖故障 | 源码可判断 watchdog 独立刷新，但故障模式需注入 | 人为卡住 MAXtask 或断开 I2C，观察 watchdog 是否仍喂狗 |

### 建议验证矩阵

| 实验 | 操作 | 观测指标 | 通过标准 |
|---|---|---|---|
| TIM6 采样节拍验证 | GPIO 在 TIM6 ISR 或 MAXtask drain 前后翻转 | 频率、抖动、max gap | 100 Hz 稳定，gap 不应长期超过 20 ms |
| OLED 影响验证 | 开/关 OLED，或切换全屏刷新/分页刷新 | FIFO high watermark、overflow、I2C hold time | OLED 开启不应显著增加 FIFO overflow |
| SD 影响验证 | SD 写入启用/禁用，插入慢卡/拔卡 | ring backlog、drop count、write_ms、backoff | SD 慢时只影响日志，采样 heartbeat 正常 |
| ECG 稳定性验证 | 长时间采集，开启 SD/OLED/UART | DMA overflow count、ECG sample lag | 无持续 overflow；异常能被计数记录 |
| 电源噪声验证 | SD burst 写、OLED 刷新、MAX LED 工作 | 3.3 V 纹波、PA5 噪声、PPG raw 波动 | 电源纹波不与采样异常强相关，或可定位并整改 |
| Watchdog 真实性验证 | 人为让 MAXtask 不更新 heartbeat | watchdog 行为、诊断记录 | watchdog 不应在采样死亡时继续无条件喂狗 |

## 七、最小补丁路线图

下面是建议的后续修改拆分。每个 patch 都应单独编译、单独验证，不做一次性大重构。

| Patch | 目标 | 可能涉及文件 | 具体行为变化 | 明确不能修改 | 验证方法 | 风险 | 适合低成本代码 agent |
|---:|---|---|---|---|---|---|---|
| 1 | 闭合 TIM6 -> MAXtask 100 Hz 调度链路 | `main.c`, `freertos.c`, `app_rtos.h`, `tim.c` | 在 RTOS task/对象就绪后设置 `app_rtos_ready`，启动 TIM6 中断，让 `MAXtask` 按 10 ms notify 运行；增加 heartbeat/sample-gap 计数 | 不改 PPG/SpO2/BPM/ECG 算法；不引入 SD I/O 到采样路径 | GPIO 翻转测 100 Hz；UART/OLED 显示 MAX heartbeat、max gap、FIFO watermark | 中 | 是，但必须给清楚启动时序 |
| 2 | 增加实时性可观测性 | `app_state.h`, `freertos.c`, `app_measurement.c`, `app_data_log.c`, `app_protocol.c`, `app_display.c` | 记录 `fifo_overflow_total`、`fifo_high_watermark`、`max_task_gap_ms`、`ecg_dma_overflow`、`sd_backlog_high`、`sd_write_max_ms`、`uart_tx_ms` | 不改算法阈值；不改变日志样本语义 | 串口周期输出计数器；OLED 状态页显示核心错误码 | 低 | 是 |
| 3 | OLED 分片刷新，缩短 I2C mutex 持有时间 | `ssd1306.c/h`, `app_display.c`, `freertos.c` | 将全屏 `ssd1306_UpdateScreen()` 改为每轮 1 到 2 页，或 dirty page 分片；UI task 继续 try-lock | 不允许 UI 等待 MAX；不改显示内容含义；不改 MAX I2C 逻辑 | GPIO 测 I2C hold time；OLED 开启时 FIFO overflow 不增加 | 中 | 部分适合，建议人工 review |
| 4 | USART TX 改为非阻塞/有界发送 | `usart.c/h`, `app_protocol.c`, `stm32f4xx_it.c` | 使用 TX DMA/ring buffer，或发送忙时丢弃过期报告；保留 RX DMA | 不改协议字段、单位、算法输出；不在采样路径 printf | 压力输出 1 KB 报告，MAX heartbeat gap 不恶化 | 中 | 部分适合，需硬件验证 |
| 5 | SD 文件会话和慢卡 backoff 加固 | `app_sd_file.c`, `app_data_log.c`, `app_state.h` | 减少目录扫描阻塞，记录 session start/stop 耗时；慢卡/错误进入 backoff；必要时避免重复 `f_sync` + `f_close` | 不允许 active measurement 中写 SD；不为了 flush 阻塞采样 | 慢卡/拔卡/满卡测试；采样不中断，drop/backoff 可见 | 中高 | 不建议完全交给低成本 agent |
| 6 | UI/USART 状态快照和波形双缓冲 | `app_display.c`, `app_state.h`, `app_protocol.c`, `app_measurement.c` | UI/USART 读 snapshot，不直接读一组正在更新的多字段；波形绘制前复制或双缓冲 | 不改算法状态更新顺序；不改变采样数据 | OLED 无撕裂；串口报告字段时间一致性提高 | 中 | 是，需边界清楚 |
| 7 | Watchdog 从“任务存活”升级为“采样健康” | `freertos.c`, `app_diag.c/h`, `app_state.h` | 只有 MAX heartbeat、ECG 消费、关键错误状态满足条件才喂狗；异常前保存诊断码 | 不因无手指、无 SD 等正常状态误复位 | 注入 MAXtask 卡死/I2C 卡死，确认 watchdog 和诊断记录有效 | 中 | 不建议完全交给低成本 agent |

最应该先做的是 Patch 1。没有确定的 100 Hz 采样调度闭环，后面讨论 SD、OLED、UART 的影响都会缺少基线。

## 八、面试价值总结

### 5 个强简历 bullet points

1. 基于 STM32F407 构建多传感器生物医学信号采集系统，集成 MAX30102 PPG、AD8232 ECG、OLED、RTC、SD/FatFs、USART 和 IWDG。
2. 设计实时采样优先的 RTOS 架构，将 MAX30102 FIFO drain、ECG DMA 消费、OLED UI、SD 日志和系统监控拆分为不同优先级任务。
3. 实现 SD 日志 RAM ring buffer 和后台分片写入，采样路径不直接执行 FatFs 操作，支持慢卡/错误时暂停日志并保留采样连续性。
4. 使用 TIM2 触发 ADC1 DMA 环形采样 AD8232 ECG，并在任务侧完成样本消费、overflow 检测和 ECG/PPG 联合状态更新。
5. 建立嵌入式医疗信号系统的可观测性思路，通过 FIFO overflow、水位、DMA overflow、SD backlog、写入耗时和 watchdog 诊断定位实时性问题。

### 5 个面试中可以展开讲的技术点

1. 为什么医学信号采集必须把采样路径和存储路径解耦：SD 卡是高延迟、不可预测外设，不能反压采样。
2. MAX30102 FIFO drain 的实时性设计：100 Hz 节拍、FIFO high watermark、overflow 计数、I2C 总线竞争。
3. AD8232 ECG 使用 TIM+ADC+DMA 的理由：采样时基由硬件保证，软件只负责及时消费和质量监控。
4. RTOS 优先级和资源隔离：采样任务高优先级，OLED/USART 可降级，SD 日志低优先级并可暂停。
5. 工程化诊断：用 GPIO 翻转、UART 计数器、SD backlog、水位和示波器，把“系统不稳定”变成可测量问题。

### 当前项目最能体现工程能力的部分

最有价值的是 SD 日志已经不是直接在采样路径 `f_write`，而是通过 `APP_DataLog_PushSample()` 写 RAM ring，再由低优先级 SDtask 分片写入。这体现了实时采样优先、存储后台化、错误可降级的正确工程方向。

第二个亮点是 ECG 采用 TIM2 触发 ADC + DMA circular buffer。对于模拟生物电信号，硬件定时采样比软件轮询更适合稳定采样间隔。

### 当前最薄弱、最需要补强的部分

1. TIM6 100 Hz RTOS 通知链路缺少启动/ready 证据，是最高优先级缺口。
2. OLED 全屏阻塞式 I2C 与 MAX30102 共总线，容易解释为“UI 影响采样”。
3. USART TX 阻塞发送长报告，调试输出可能反过来影响被调试系统。
4. 看门狗只证明 task 还活着，不证明采样质量健康。
5. README 与当前源码存在不一致，例如 RTOS/主循环、SDIO 速度、MAX INT 使用状态，需要在展示前修正。

### 如何解释“SD 卡导致实时采样异常”

可以这样讲：

SD 卡本质上是高延迟、不可预测外设，`f_write/f_sync/f_close` 可能触发块写、FAT 表更新、目录项更新和内部擦写，耗时不是固定的。医学采样路径不能等待 SD，所以我把采样和存储解耦：采样任务只写 RAM ring，SDtask 在后台低优先级、分片写入。卡慢时允许日志丢弃最旧数据并记录 drop/backlog/error，但不允许停止采样去等待 SD。

当前源码已经有这个方向，但我也识别出两个需要补强的点：一是 TIM6 采样节拍闭环要验证，二是 OLED/I2C、UART 阻塞也可能和 SD 同时叠加，使问题看起来像“SD 导致”，实际是系统实时预算不足。

### 如何解释“为什么引入或不引入 RTOS”

建议解释为：这个项目保留 RTOS 更合理，因为系统有多个不同实时等级的工作：MAX30102 FIFO drain 和 ECG DMA 消费必须高优先级，OLED、USART、SD 可以延迟或降级。RTOS 能把优先级和隔离关系讲清楚。

但 RTOS 不是自动解决实时性的魔法。引入 RTOS 后必须完成三件事：采样任务的确定性触发、共享总线的短时占用、后台任务的阻塞隔离。当前项目需要先修复 TIM6 -> MAXtask 通知闭环，再把 OLED 和 USART 的长阻塞改成分片或非阻塞。

### 如何把模块讲成完整系统

可以按“信号质量优先”的系统链路讲：

MAX30102 提供 PPG，AD8232 提供 ECG，二者进入统一的采样和状态管理层。PPG 通过 FIFO 批量读取，ECG 通过 TIM+ADC+DMA 保证采样间隔。算法层输出 BPM、SpO2、HRV/RR/PTT 和质量状态。显示和串口只是状态观察窗口，不允许反过来阻塞采样。SD 卡是后台持久化模块，写不动时可以丢日志和报警，但不能牺牲采样连续性。RTC 给日志和显示提供时间基准，看门狗和诊断模块负责把故障变成可追踪的状态，而不是让系统静默失败。

## 九、最终结论

### 1. 当前架构是可接受、脆弱，还是不适合实时医学采集？

当前架构方向可接受，但实现状态仍然脆弱。它已经具备高质量项目的骨架：RTOS 任务拆分、ECG DMA、SD ring buffer、后台日志、I2C mutex 和错误计数。但因为 TIM6 采样通知闭环缺证据、OLED 阻塞式 I2C、USART 阻塞 TX、watchdog 未绑定采样健康，当前还不能说适合稳定实时医学采集。

更准确的表述是：这是一个有正确工程方向、但需要补齐实时性闭环和可观测性的医学信号采集项目。

### 2. 当前最高优先级问题是什么？

最高优先级是修复并验证 MAX30102/ECG 软件消费的确定性调度：TIM6 100 Hz 是否真正启动，`app_rtos_ready` 是否置位，`MAXtask` 是否按 10 ms 周期运行。这是所有后续优化的基线。

### 3. 下一步最应该做的一个 patch 是什么？

Patch 1：闭合 TIM6 -> ISR -> MAXtask 的 100 Hz 通知链路，并加入最小可观测性计数器，例如 MAXtask heartbeat、最大 sample gap、FIFO high watermark、FIFO overflow total。这个 patch 不应碰医学算法、不应碰 FatFs 策略。

### 4. 哪些问题必须在展示给面试官前修掉？

1. TIM6 采样调度闭环必须修掉并用数据证明。
2. OLED 全屏阻塞 I2C 至少要分片化，或用测量数据证明不会造成 FIFO backlog。
3. USART 阻塞长报告要有边界，最好改成 DMA/ring buffer。
4. SD 日志状态、drop、backlog、write_ms、backoff 必须能在 OLED/USART 中看到。
5. README 和源码不一致的部分要修正，尤其是 RTOS/裸机模型、MAX INT 状态、SDIO 速度描述。
6. Watchdog 要能监控采样 liveness，而不是只监控任务循环。

### 5. 哪些问题可以作为“已识别的工程 trade-off”解释？

| Trade-off | 可以如何解释 |
|---|---|
| SD 慢时丢弃最旧日志 | 为保证医学采样连续性，存储不能反压采样；丢弃必须计数并提示 |
| SD 写只在 safe window 执行 | 日志完整性让位于实时采样质量，这是医疗信号项目的合理优先级 |
| MAX30102 不用 INT 引脚 | 当前 PE5 被 AD8232 LO- 占用，源码证据显示 INT 模式禁用；用 TIM6 轮询需补强时序验证 |
| OLED 可跳帧/分页刷新 | UI 是观察窗口，不是实时采样路径；显示延迟可接受 |
| RTOS 保留但严格限制阻塞 | RTOS 用于隔离不同实时等级任务，不代表可以在任意 task 中长阻塞 |
| 看门狗从复位工具变成诊断工具 | 不能只追求“不死机”，更要能记录为什么采样失效 |

