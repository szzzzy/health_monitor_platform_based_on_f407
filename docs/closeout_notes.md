# BME 项目收尾说明 (Feature Freeze Closeout Notes)

> 日期: 2026-06-11
> 状态: **Feature Freeze** — 不添加新功能，只做稳定性收敛与小范围可观测性增强。

---

## 1. 系统架构概览

本项目是基于 STM32F407 的生物医学信号采集原型（非医疗级产品）：
- MAX30102 PPG（红光+红外）、AD8232 ECG 同步采集
- FreeRTOS 多任务调度（MAXtask / watchdogtask / Uitask / SDtask）
- SSD1306 OLED 显示（I2C1 共享总线）、USART2 调试协议
- RTC 时间戳 + BKP 寄存器崩溃诊断
- SD 卡 FatFs 二进制日志（后台懒写入）

**核心设计原则：实时采样路径优先，后台路径可降级。**

---

## 2. 采样路径优先级

```
优先级 1 (最高, 不可阻塞): MAX30102 FIFO drain + ECG ADC DMA 消费
优先级 2:            传感器状态监控 (watchdog)
优先级 3:            OLED 显示刷新、USART 报告
优先级 4 (最低, 可暂停): SD 卡日志写入
```

实施机制：
- `measurement_active` 门控 —— 手指在传感器上时 SDtask 禁止 I/O
- ISR 中只做标志位 + 任务通知，零阻塞
- I2C1 互斥锁：MAXtask 短等待（8ms），低优先级任务放弃
- SD 日志采用环形缓冲 + 后台分片写入，不在实时路径做 f_write

---

## 3. TIM6 → MAXtask 100 Hz 调度链路

```
TIM6 (84MHz / 8400 = 10 kHz, Period=99) 
  → 每 10ms 触发 TIM6_DAC_IRQHandler()
    → HAL_TIM_IRQHandler(&htim6)
      → HAL_TIM_PeriodElapsedCallback(htim6)
        → max30102_mark_data_ready_from_isr()   [设置 FIFO 就绪标志]
        → app_rtos_notify_max_from_isr()        [vTaskNotifyGiveFromISR]
          → MAXtask 被唤醒，ulTaskNotifyTake 返回非零
            → app_rtos_service_max()
              → app_ecg_process_samples()       [ECG DMA 环形缓冲消费]
              → app_measurement_drain_fifo_batch() [MAX30102 FIFO 批量读取]
              → app_measurement_service_sensor_watchdog()
```

启动时序：
1. `main.c` → `MX_TIM6_Init()` — 配置 100 Hz 但不启动
2. `main.c` → `app_rtos_bind_state(&app)` — 绑定共享状态
3. `main.c` → `osKernelInitialize()` → `MX_FREERTOS_Init()` — 创建全部任务
4. `osKernelStart()` — 调度器启动，MAXtask 开始执行
5. `StartTask02()` 首次迭代 → `app_rtos_mark_ready()` — 允许 ISR 通知
6. `StartTask02()` 首次迭代 → `HAL_TIM_Base_Start_IT(&htim6)` — 启动 100 Hz 中断

关键：TIM6 在 scheduler 启动后才使能，确保 ISR 中 `vTaskNotifyGiveFromISR`
调用时调度器已就绪，消除竞态窗口。

降级路径：如果 TIM6 中断因故停止，MAXtask 在 20ms 超时后仍会轮询唤醒。

---

## 4. 可观测性诊断计数器

| 变量 | 位置 | 含义 | 健康值 |
|---|---|---|---|
| `tim6_isr_count` | main.c 文件级 static volatile | TIM6 中断触发次数 | 约 100/秒，调试器直接读取 |
| `max_task_heartbeat` | AppState, MAXtask 递增 | MAXtask 每轮递增 | 约 100/秒持续增长 |
| `max_task_timeout_count` | AppState, 超时时递增 | ulTaskNotifyTake 返回 0 次数 | 必须保持 0 |
| `max_task_gap_ms` | AppState, 记录峰值 | MAXtask 连续唤醒最大间隔 | 应 ≤ 20ms |
| `max_sample_gap_ms` | AppState, 已有 | 连续样本间最大间隔 | ≤ 30ms |
| `fifo_overflow_total` | AppState, 已有 | FIFO 溢出累计 | 不持续增长 |

**max_task_timeout_count 判定标准：**
- 手指放上/离开期间 **不应增长**。TIM6 notify 持续送达，与手指状态无关。
- 如果该值持续增长，说明 MAXtask 没有稳定收到 TIM6 通知或被长时间阻塞
  （I2C 锁、OLED 刷新、SD 写入等占用了本任务 CPU 时间），需要排查阻塞源。

---

## 5. SD 日志后台化原则

- SD 日志全部走 `app_data_log.c` 的环形缓冲（2048 样本 × 16B = 32KB）
- 实时路径 `PushSample()` 仅做 16B memcpy 入队，零格式化、零 I/O
- 后台 `ServiceBudget()` 每轮至多 1 chunk (512B, ~0.3ms @ 12MHz SDIO)
- 反压：ring > 25% 不启动 SD，ring > 50% 暂停 SD 写入
- 测量期间 (`measurement_active=1`) 禁止一切物理 SD I/O
- 级联清理：失败时 memset FIL → f_mount(NULL) → Deinit → STA_NOINIT

---

## 6. EEPROM 设计预留（本轮不实现）

板载 24C02（256B, I2C1 总线, 地址 0x50），兼容 24C02~24C512 全系列。

推荐存储内容（不存波形，不参与实时采样）：
- `device_id` (4B) — 设备序列号，生产时一次性烧录
- `boot_count` (4B) — 累计启动次数，每次启动 +1
- `log_sequence` (2B) — 日志文件序号，跨断电连续递增
- `config_flags` (2B) — 配置位域（LED 亮度、报警阈值等）
- `crc16` (2B) — 以上字段的 CRC 校验

访问约束（硬件限制：与 MAX30102/OLED 共享 I2C1）：
- 只在 boot、measurement_stop、manual save 等安全时机读写
- 不在 MAXtask、ECG 采样路径、中断、测量活跃期间做 EEPROM I/O
- 不要在 EEPROM 驱动中使用 HAL_Delay 或长时间阻塞

推荐 layout（16B 一个 block，方便扩展）：
```
Offset  Size  Field
0x00    4     magic "BMEP"
0x04    4     device_id
0x08    4     boot_count
0x0C    2     log_sequence
0x0E    2     config_flags
0x10    2     crc16
0x12    2     reserved
...
```

---

## 7. 明确禁止的事项

1. **禁止在实时采样路径中加入**：f_write、f_sync、f_open、f_close、printf、OLED 刷新、HAL_Delay、长时间阻塞 I2C/SPI/UART
2. **禁止在 ISR 中做**：日志、显示、串口、FatFs、复杂算法、阻塞调用
3. **禁止修改**：BPM、SpO2、HRV、RR、PTT、ECG、滤波器等医学算法公式和阈值
4. **禁止重写**：SD/FatFs 架构、OLED 驱动、RTOS 调度器
5. **禁止为了日志完整性牺牲**：MAX30102 FIFO drain 或 ECG ADC 采样连续性

---

## 8. 已验证的架构决策

- [x] TIM6 100 Hz 已配置（Prescaler=8399, Period=99 → 84MHz/8400/100 = 100Hz）
- [x] MAXtask 优先级 osPriorityHigh（高于 UI/SD/WDT）
- [x] SDtask 优先级 osPriorityLow（不影响实时路径）
- [x] I2C1 互斥锁超时 8ms（MAXtask 不长时间等待总线）
- [x] IWDG 看门狗独立任务刷新（50ms 周期）
- [x] BKP 崩溃诊断：HardFault/NMI/MemManage/BusFault/UsageFault 全覆盖
- [x] SD 日志失败退避 60s（不反复阻塞）
- [x] OLED 刷新跳过机制（display_skipped_count + 强制刷新 1s 超时）
- [x] D9 SCHED 调度诊断页（TIM6 → MAXtask 100 Hz 闭环验证）

---

## 9. Debug 页面 D9 SCHED — 调度诊断

D9 SCHED 是收尾阶段新增的调试子页，专门用于验证 TIM6 → MAXtask 100 Hz 调度闭环。
仅在用户手动切换到该页时渲染，不参与实时采样路径。

### 显示布局

```
D9 SCHED
T6 100/s M 100/s
TO     0 TGAP 10
MHB123456
UHB 67890
SD IDLE
OVF    0 SGAP 12
HWM 32
```

### 字段含义与数据来源

| 字段 | 含义 | 数据来源 | 健康值 |
|---|---|---|---|
| T6 | TIM6 ISR 每秒触发次数 | `APP_TIM6_GetIsrCount()` delta/elapsed | ≈ 100/s |
| M | MAXtask heartbeat 每秒增长次数 | `max_task_heartbeat` delta/elapsed | ≈ 100/s |
| TO | MAXtask 超时唤醒累计 | `max_task_timeout_count` | 不持续增长 |
| TGAP | MAXtask 连续唤醒最大间隔 (ms) | `max_task_gap_ms` | ≤ 20ms |
| MHB | MAXtask heartbeat 累计 | `max_task_heartbeat` | 持续增长 |
| UHB | Uitask heartbeat 累计 | `ui_task_heartbeat` | 持续增长 |
| SD | SD 日志状态机当前状态 | `sd_state` | IDLE/ACTIVE/BOFF/TRY |
| OVF | FIFO overflow 累计 | `fifo_overflow_total` | 不持续增长 |
| SGAP | 连续样本间最大间隔 (ms) | `max_sample_gap_ms` | ≤ 30ms |
| HWM | FIFO backlog 峰值样本数 | `fifo_high_watermark` | 用于 backlog 门控参考 |

### TGAP 与 SGAP 的区别

- **TGAP** (task gap): MAXtask 连续两次唤醒之间的时间间隔。反映调度抖动。
  如果 TIM6 100 Hz 通知正常送达，TGAP 应稳定在 10ms 左右（唤醒频率 100 Hz）。
- **SGAP** (sample gap): MAX30102 FIFO 中连续样本的时间间隔。反映 FIFO drain 的及时性。
  SGAP 过大说明 FIFO drain 出现了滞后（I2C 总线竞争、MAXtask 被阻塞等）。

### 速率计算方式

T6/s 和 M/s 在 D9 页面渲染路径中低频计算（约每秒一次），使用无浮点整数运算：
```
rate = delta * 1000 / elapsed_ms
```
不在 ISR 或 MAXtask 中做除法。

### 设计约束

- D9 只在用户手动切到该 Debug 子页时渲染
- 不增加 OLED 整体刷新频率
- T6/s 和 M/s 的 static 变量仅在该函数内可见
- 页面渲染走低优先级 UI task，不阻塞 TIM6 ISR、MAXtask、ECG 消费路径
- 无 USART 输出、无 printf、无 FATFS
