# 变更记录 / Changelog

本文档按时间倒序整理项目自初始以来的关键架构决策、功能增减和重要修复。

Commits are grouped by theme rather than listed chronologically.

---

## Phase 3 — ECG DSP 增强与 OLED 显示优化 (2026-06-20)

### ECG DSP 预处理 (Phase 1)

**决策**: 在现有整数 DC+LP 链路基础上增加浮点 biquad 预处理，利用 STM32F407 Cortex-M4F 硬件 FPU，不重写 QRS 状态机。

**新增**:
- `EcgBiquad_t` Direct Form I 结构，单精度浮点，接口兼容 CMSIS-DSP `arm_biquad_casd_df1_inst_f32`
- 50 Hz 陷波器 (Q=30) + 10-20 Hz 带通滤波器 (Q≈1.4) 级联
- `APP_ECG_DSP_PREPROCESS` 编译开关 (默认 1)，设为 0 回退到纯整数链路
- biquad 延迟线在 DMA 溢出/导联脱落/检测器重置时统一清零

### ECG QRS 检测增强 (Phase 2)

**决策**: 在滤波后信号上增加标准 Pan-Tompkins 差分+MWI+双估计阈值，改"纯幅值阈值"为"signal peak / noise peak 双估计自适应阈值"，不改 HR/RR/PTT 输出接口。

**新增**:
- 两点差分 + 平方能量 + 120 ms 移动窗口积分 (MWI)，全整数 O(1)/样本
- `signal_peak` EMA (α=1/8) 跟踪已确认 R 峰能量
- 双估计阈值：信号明显时 `noise + (signal-noise)/4`，信号弱时回退 `noise×GAIN`
- `ecg_dmwi_reset()` 与检测器重置联动

### OLED ECG 波形显示优化 (Phase 3)

**决策**: OLED visual 链路独立于 QRS 检测链路，仅优化视觉输出，不回写检测器状态。ECG 波形使用独立 AGC min_scale=200（PPG 保持 32），防止低幅噪声被放大。

**新增**:
- VISUAL 低通截止从 fc≈10 Hz 降至 fc≈5 Hz (`LP_SHIFT` 2→3)
- VISUAL 增益从 3× 提升至 4×
- VISUAL 50 Hz 陷波 (`APP_ECG_VISUAL_ENABLE_NOTCH=1`)，独立 biquad 实例
- 3:1 降采样改为 3 点组内算术平均（抑制随机噪声 ~1.7×）
- 质量门控：`ecg_signal_quality < 20` 时推送中线，避免 OLED 画噪声
- `WaveformBuffer_t` 增加 `min_scale` 字段，支持每波形独立 AGC 下限
- `app_display_set_ecg_min_scale()` 供运行时动态调高

### ECG 质量与诊断字段

**决策**: 新增 8 个 ECG quality 字段作为 M 帧尾部追加 (列 102-109)，不移动现有 0-101 列，不改变 SD 二进制日志格式。

**新增 AppState 字段**:
| 字段 | 含义 |
|------|------|
| `ecg_signal_quality` | 0-100 综合评分 |
| `ecg_invalid_reason` | 0=OK, 1=LEAD_OFF, 2=ADC_SAT, 3=DMA_OVERFLOW, 4=NO_R_PEAK, 5=LOW_AMPLITUDE, 6=NOISY |
| `ecg_raw_span` | ECG 原始 ADC min/max 跨度 |
| `ecg_filtered_span` | 滤波后 min/max 跨度 |
| `ecg_noise_level` | QRS 检测器自适应噪声基线 |
| `ecg_qrs_threshold` | 动态 QRS 阈值 |
| `ecg_peak_snr_x100` | 最新 R 峰 SNR ×100 |
| `ecg_dma_available_high_watermark` | ADC DMA 积压高水位 |

**质量评分逻辑**:
- lead_off → SQ=0, reason=LEAD_OFF
- DMA overflow → SQ=0, reason=DMA_OVERFLOW
- 超过 STALE_MS 无 R 峰 → SQ=0, reason=NO_R_PEAK
- filtered_span < 25 LSB → SQ=0, reason=LOW_AMPLITUDE
- noise>500 且 SNR<80 → reason=NOISY, SQ 封顶 40
- ADC 饱和 → reason=ADC_SAT, SQ 减半
- 正常: filtered_span (0-40分) + SNR×100 (0-60分) 合计

**新增 OLED 页面**:
- `DISPLAY_PAGE_ECG_QUALITY`: 上半屏 SQ/reason + HR/RR/PTT，下半屏 ECG 波形
- `DBG_SUB_D10_ECG_Q`: SQ/reason、raw/filt span、noise/threshold、SNR、DMA hwm/overflow

**CSV 协议**: JSON_PAYLOAD_SIZE 从 1024 扩至 1280，M 帧总字段数 110。

### 文档

- README: 加入 AD8232/ECG 双采样架构、DSP 流水线说明、完整引脚表
- README: 全部章节中英双语

**Commits**: `f5bf55a`, `e132ec2`, `dd95d4c`

---

## Phase 2 — FreeRTOS 迁移与系统诊断 (2026-06-07 ~ 2026-06-18)

### FreeRTOS 迁移

**决策**: 从裸机 `main()` 超级循环切换到 FreeRTOS 任务驱动模型。利用抢占式调度将 MAX30102/ECG 实时路径与 UI/SD 慢速路径隔离，防止 OLED 阻塞式 I2C 刷新拖慢采样。

**新增任务**:
| 任务 | 优先级 | 职责 |
|------|--------|------|
| `MAXtask` | `osPriorityHigh` | TIM6 100 Hz 唤醒 → ECG DMA 消费 → MAX30102 FIFO drain |
| `watchdogtask` | `osPriorityAboveNormal` | 每 50 ms 喂 IWDG，每约 1s 写 BKP 活体快照 |
| `Uitask` | `osPriorityNormal` | UART 命令轮询、按键、OLED 刷新 (try-lock I2C)、周期上报、栈 HWM 采样 |
| `SDtask` | `osPriorityLow` | SD 安全窗口门控、二进制日志后台写入/同步 |

**关键约束**:
- TIM6 中断在调度器启动后才由 `StartTask02()` 使能，避免 ISR 在 `osKernelStart()` 前调用 FreeRTOS API
- `Uitask` 仅用 `try-lock` 获取 I2C mutex，FIFO 积压时跳过显示刷新
- `SDtask` 仅在手指离位/无信号的安全窗口执行物理 SD I/O
- `MAXtask` 通过 `ulTaskNotifyTake()` 等待 TIM6 通知，超时 20 ms

**Commits**: `1f77a3f`, `7fc69bf`

### 崩溃诊断与可观测性

**决策**: 利用 STM32F407 备份寄存器 (BKP) 实现跨复位崩溃追踪，不依赖外部存储。

**新增**:
- `crash_flag/source/task/phase/tick`: 崩溃时通过 `APP_Diag_CaptureCrash()` 写入 BKP
- `max_task_phase/ui_task_phase/sd_task_phase/wdt_task_phase`: 各任务在关键路径前更新阶段码
- `max_task_stack_hwm/ui_task_stack_hwm/sd_task_stack_hwm/wdt_task_stack_hwm`: Uitask 周期性采集
- `reboot_count`: 累计重启次数 (从 BKP 恢复)
- `reset_flags`: RCC_CSR 复位标志 (启动时捕获)
- HardFault/MemManage/BusFault/UsageFault/StackOverflow 钩子全部接入诊断链路

**Commits**: `063f8cc`

### 运行时服务重构

**决策**: 将 `main.c` 中的超长函数拆分为 `app_runtime.c` 独立模块，避免 `main.c` 持续膨胀。

**变更**:
- `app_runtime_send_report_if_due()`: 条件串口上报
- `app_runtime_refresh_display_if_needed()`: 条件 OLED 刷新
- `app_runtime_update_sd_log_status()`: SD 日志状态同步到 AppState
- `app_runtime_sd_service_safe()`: SD 安全窗口判断 (手指离位+信号稳定+无 FIFO 积压)

**Commits**: `e681345`, `efbc098`

### 调度诊断

**新增**:
- `max_task_timeout_count`: ulTaskNotifyTake 超时唤醒次数
- `max_task_gap_ms`: 两次唤醒的最大间隔
- `max_task_heartbeat/ui_task_heartbeat`: 任务心跳计数器
- D9 SCHED 调试页: TIM6 频率、MAXtask 频率、超时次数、唤醒间隔

**Commits**: `7fc69bf`

### SDIO 策略

**决策**: 从高速 12 MHz 4-bit 改为保守的 400 kHz，单次尝试 4-bit，失败后永久回退 1-bit。因为多块写入 + 高速时钟在长线缆/面包板环境下不稳定。

**变更**:
- `APP_SD_INIT_CLK_DIV = 118` (48 MHz / 118 ≈ 400 kHz)
- `APP_SD_FAST_CLK_DIV = 118` (不再切换到高速分频)
- 4-bit 失败后清零 `hsd.ErrorCode`，回退 1-bit
- 1-bit 也失败时返回 `APP_SD_CARD_ERROR` 而非死循环

**Commits**: `638f38c`

### Doxygen 注释

所有应用模块和驱动模块补全 Doxygen 格式函数注释。

**Commits**: `44127f8`

---

## Phase 1 — ECG 集成与架构重构 (2026-05-16 ~ 2026-06-06)

### ECG 链路引入

**决策**: 增加 AD8232 模拟前端作为 PPG 参考/同步通道。ECG 不作为独立诊断系统，仅用于 R 峰时间基准、ECG HR/RR 和 PTT 计算。

**新增**:
- AD8232 → PA5/ADC1_IN5 → TIM2 250 Hz TRGO → ADC1 DMA circular buffer
- 简化 Pan-Tompkins: DC 漂移消除 (1.24 Hz HPF) + 低通平滑 (13.3 Hz LPF) + 自适应阈值 QRS 状态机
- `AppEcgUpdate_t`: R 峰检测结果回调，驱动 PTT 模块
- `DISPLAY_PAGE_ECG`: 全高 ECG 波形 + HR/RR/PTT + 导联脱落状态
- D8 ECG/PPG 对比调试页: ECG HR vs PPG BPM 同屏比较

**关键设计选择**:
- 250 Hz 采样率的 4 ms 间隔由软件推算，不依赖硬件时戳
- 导联脱落检测通过 PE5/PE6 GPIO 读取 AD8232 LO-/LO+ 引脚
- `APP_ECG_ENABLE_LEAD_OFF_GATE = 0` 默认禁用导联脱落门控（待硬件验证）

**Commits**: `4de7b59`, `aacbcb9`

### 架构重构: measurement-first

**决策**: 将 MAX30102 传感器访问、FIFO 管理、恢复逻辑从 `main.c` 拆分到 `app_measurement.c`，围绕"测量状态"而非"硬件操作"组织代码。

**变更**:
- `app_measurement_init_state()` / `app_measurement_drain_fifo_batch()` / `app_measurement_recover_sensor()`
- 传感器健康状态机: OK → STALE → RECOVERING → I2C_ERR → INIT_FAIL
- 自适应手指检测 with 接触稳定倒计数

**Commits**: `e3436ae`, `ab3eaea`

### SD 日志: 从 CSV 到二进制

**决策**: CSV 格式化 (snprintf) 在 100 Hz 下 CPU 开销过高。改为二进制日志: 实时路径仅 `APP_DataLog_PushSample()` 拷贝 16B 到 RAM ring，物理 `f_write` 异步在 `SDtask` 安全窗口执行。

**新增格式**:
- 文件头 32B (`BMLG` magic + version + sample_rate + start_tick)
- 样本 16B (tick + red + ir + ecg + flags + seq)
- RAM ring 2048 样本 (~20s @ 100Hz)，写块 512B

**Commits**: `3ecb346` (原始 CSV), `e3436ae` (二进制改造)

### 高级指标流水线

**新增**:
- HRV 时域: SDNN、RMSSD、SD1、SD2、SD1/SD2、心律规整提示
- HRV 频域: LF power、HF power、LF/HF (32 拍短窗口，CMSIS-DSP RFFT)
- RR 呼吸率估计: 基于脉搏幅度调制 (需 SQ≥35、≥10 拍、≥8s 窗口)
- PI 灌注指数: (AC/DC)×1000，搏动 EMA 计算
- Motion hint: 纯 PPG 运动伪影检测 (AC RMS 尖峰 + RED/IR 失衡 + SQ 骤降)

**Commits**: `3fe376a`, `4de7b59`

### 鲁棒性修复

- 手指重新放置后 contact-settle 倒计数挂死 (`5a1fee6`, `b7b2abb`)
- SD 延迟初始化在无卡时阻塞启动 (`5a1fee6`)
- SD 日志恢复停滞 (`8438520`)
- PPG 高级指标长时间无输出的门控问题 (`8a8f058`)

---

## Phase 0 — 基础建设 (2026-04-18 ~ 2026-05-07)

### 初始 MAX30102 固件

**新增**:
- TIM6 100 Hz 精确采样节拍 + WFI 低功耗等待
- MAX30102 I2C DMA FIFO 读取
- CMSIS-DSP 4 阶 Butterworth 带通滤波 (0.5-5 Hz)
- 自相关 BPM + 时域 SpO2 双算法
- SSD1306 OLED 实时波形显示 (IR/RED)
- 开机 5s 无手指基线采集
- RTC 时间维护 + USART2 文本协议上报 + 串口校时
- IWDG 独立看门狗
- FatFs + SDIO CSV 日志
- 按键页面切换 (PE2/PE3/PE4)

**Commits**: `a2f0851`, `fbac81b`, `791be50`, `3ecb346`, `cdf0f38`

---

## 关键架构决策汇总 / Key Architectural Decisions

| 决策 | 选择 | 替代方案 (已拒绝) |
|------|------|-------------------|
| RTOS | FreeRTOS (CMSIS-RTOS v2) | 裸机超级循环 (已迁移) |
| PPG 采样率 | 100 Hz (TIM6) | 400 Hz (FIFO 溢出风险高) |
| ECG 采样率 | 250 Hz (TIM2 TRGO) | 500 Hz (DMA 缓冲压力大) |
| ECG 滤波器 | 手工 biquad (float) + MWI (int) | CMSIS-DSP biquad (接口兼容但暂未替换) |
| QRS 阈值 | 双估计 (signal_peak/noise_peak) | 纯幅值×固定增益 (Phase 1 已替换) |
| SD 日志格式 | 二进制 (16B/样本) | CSV 文本 (CPU 开销过高) |
| SDIO 速度 | 400 kHz (保守) | 12 MHz 高速 (线缆环境下不可靠) |
| OLED 刷新 | 阻塞 I2C + try-lock 跳帧 | 分页/DMA (未实现) |
| USART TX | 阻塞 HAL_UART_Transmit | DMA TX (未实现) |
| 导联脱落 | 默认禁用 (APP_ECG_ENABLE_LEAD_OFF_GATE=0) | 启用 (待硬件验证 PE5/PE6 接线) |
| MAX30102 INT | 禁用 (TIM6 轮询) | EXTI 中断 (PE5 已分配给 AD8232 LO-) |
| ECG 质量字段 | M 帧尾部追加 (列 102-109) | 独立帧或插入中间 (破坏兼容性) |
| OLED 波形 AGC | ECG min_scale=200, PPG min_scale=32 | 统一 min_scale=32 (ECG 噪声被放大) |

---

## 串口协议演进 / UART Protocol Evolution

| 版本 | 字段数 | 新增内容 |
|------|--------|---------|
| 初始 | ~12 | red, ir, baseline, finger, bpm, spo2 |
| +HRV/RR | ~30 | ibi, sdnn, rmssd, motion, hrv_freq, signal_quality, PI, spo2_ratio |
| +诊断 | ~72 | 传感器统计, RTC/UART, SD/Display, finger_detect |
| +ECG/PTT | ~86 | ecg_valid/hr/rr_ms/lead_off/r_peak/filtered, ptt, ecg counters |
| +崩溃/任务 | 102 | crash, task_phase, task_stack, task_heartbeat |
| +ECG 质量 | **110** | ecg_signal_quality, ecg_invalid_reason, raw/filt_span, noise_level, qrs_threshold, peak_snr, dma_hwm |

---

## 调试页面演进 / Debug Page Evolution

| 页面 | 内容 |
|------|------|
| D1 MAX | 传感器健康、I2C 错误、恢复状态 |
| D2 FIFO | FIFO 读写指针、溢出、批量排空统计 |
| D3 PPG RAW | RED/IR 原始值、基线、信号跨度 |
| D4 PPG Q | SQ、PI、运动、SpO2 比率 |
| D5 ALGO | HR/SpO2/RR/IBI/PTT 及其 valid 标志 |
| D6 SYS | RTC/UART/SD、崩溃、栈 HWM、阶段码 |
| D7 SD | SD 总线模式、HAL 错误、任务心跳、FIFO 高水位 |
| D8 ECG/PPG | ECG vs PPG HR 对比、导联脱落、原始/滤波范围 |
| D9 SCHED | TIM6 频率、MAXtask 频率、超时次数、唤醒间隔 |
| **D10 ECG Q** | ECG SQ/reason、raw/filt span、noise/threshold、SNR、DMA hwm |
