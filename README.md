# BME STM32F407 生理信号固件 / Physiological Signal Firmware

本仓库是基于 STM32F407ZGTx 的生理信号采集固件，面向工程验证、信号采集和趋势观察。
它不是医疗诊断设备；PTT 仅作为 ECG 到 PPG 的时间差指标，不等同于血压估计。

This repository contains STM32F407ZGTx firmware for physiological signal acquisition,
engineering validation, and trend observation. It is not a medical diagnostic device.
PTT is treated only as an ECG-to-PPG timing metric, not as a blood-pressure estimate.

主要外设 / Main peripherals:

- MAX30102 PPG
- AD8232 ECG
- SSD1306 128x64 I2C OLED
- M24C02 EEPROM
- RTC
- USART2 reporting
- SD/FatFs binary logging
- FreeRTOS task scheduling

## 当前架构 / Current Architecture

固件使用 FreeRTOS 任务模型，实时采样、显示、串口和 SD 写入分离，避免低优先级 I/O 阻塞采样路径。

The firmware uses a FreeRTOS task model. Real-time sampling, display, UART, and SD
writes are separated so low-priority I/O does not block the sampling path.

| Task | Priority | 中文职责 | English role |
| --- | --- | --- | --- |
| `MAXtask` | High | 100 Hz MAX30102 节拍、FIFO 排空、ECG DMA 消费、传感器恢复 | 100 Hz MAX30102 tick, FIFO drain, ECG DMA consumer, sensor recovery |
| `Uitask` | Normal | UART 命令、按键、OLED 刷新、周期上报 | UART commands, buttons, OLED refresh, periodic reports |
| `SDtask` | Low | 在安全窗口后台写入二进制日志 | Background binary log writes during safe windows |
| `watchdogtask` | Above normal | IWDG 喂狗和任务存活快照 | IWDG refresh and liveness snapshots |

主信号路径 / Main signal paths:

```text
MAX30102 FIFO
  -> TIM6 100 Hz scheduling tick
  -> MAXtask
  -> PPG filtering / SQI / BPM / SpO2 / HRV / RR / PTT
  -> AppState
  -> OLED + USART2 + SD RAM ring

AD8232 analog ECG
  -> TIM2 250 Hz TRGO
  -> ADC1 DMA circular buffer
  -> MAXtask ECG consumer
  -> ECG filter / QRS / ECG HR / RR / PTT timing
  -> AppState
```

## 文献驱动算法更新 / Literature-Guided Algorithm Update

本分支根据本地文献和 `docs/literature_algorithm_improvement_prompts.md` 的提示词，
在不替换现有 PPG/ECG 主检测器的前提下，增加了保守的 PPG SQI 门控和 PTT 置信门控。

This branch uses the local literature and `docs/literature_algorithm_improvement_prompts.md`
to add a conservative PPG SQI gate and PTT confidence gate without replacing the existing
PPG or ECG production detectors.

参考依据 / Evidence used:

- Pan and Tompkins 1985, `A_Real-Time_QRS_Detection_Algorithm.pdf`: ECG QRS filtering, nonlinear energy, moving-window integration, adaptive thresholding, refractory logic, and search-back.
- Christov 2004, `1475-925X-3-28.pdf`: adaptive ECG thresholds based on slope, noise, and beat expectation.
- Elgendi 2013, `file.pdf`: PPG systolic peak detection with moving averages and offset thresholding.
- Park et al. 2022, `fphys-12-808451.pdf`: PPG artifact checklist covering motion, baseline wander, hypoperfusion, preprocessing, peak detection, and SQI.
- Mohagheghian et al. 2022, `Optimized_Signal_Quality_Assessment_for_Photoplethysmogram_Signals_Using_Feature_Selection.pdf`: lightweight PPG SQA features such as IBI variability and morphology/statistical features.
- Rahman et al. 2022, `rsif.2022.0012.pdf`: avoid universal fixed SQI thresholds where local adaptive baselines are available.

## PPG SQI 门控 / PPG SQI Gate

新增模块 / New module:

- `Core/Inc/app_ppg_sqi.h`
- `Core/Src/app_ppg_sqi.c`

编译开关 / Compile-time switches:

- `APP_PPG_SQI_EXPERIMENTAL = 1U`
- `APP_PPG_SQI_GATE_OUTPUTS = 1U`

SQI 模块维护 IR/RED AC RMS 和 PI 的局部自适应基线。基线只在无运动、无接触过渡、
无明显质量标志的稳定窗口中更新。坏窗口可以降低 `app->signal_quality`，但不会提高它。

The SQI module maintains adaptive local baselines for IR/RED AC RMS and PI. Baselines update
only during stable, non-motion, non-transition windows. Bad windows can lower
`app->signal_quality`, but they never raise it.

| Flag | Bit | 中文含义 | English meaning |
| --- | ---: | --- | --- |
| `APP_PPG_SQI_FLAG_LOW_PERFUSION` | `0x01` | IR AC/PI 低于局部基线或绝对下限 | IR AC/PI is too low versus local baseline or absolute floor |
| `APP_PPG_SQI_FLAG_MOTION` | `0x02` | 现有运动检测器处于激活状态 | Existing PPG motion detector is active |
| `APP_PPG_SQI_FLAG_BALANCE` | `0x04` | RED/IR 比值或平衡状态不可靠 | RED/IR ratio or balance is unreliable |
| `APP_PPG_SQI_FLAG_TRANSITION` | `0x08` | 手指接触仍在稳定期 | Finger contact is still settling |
| `APP_PPG_SQI_FLAG_BEAT_UNSTABLE` | `0x10` | IBI 或 beat 幅度出现突跳 | IBI or beat amplitude jump was detected |

影响的输出 / Outputs affected:

- HR/BPM：SQI 过低、运动或接触稳定期时抑制输出。
- SpO2：低灌注、RED/IR 平衡异常、运动、接触过渡或 beat 不稳定时抑制输出。
- PTT：PPG 峰时刻不可信时抑制匹配。

- HR/BPM: suppressed when the SQI score is too low or contact/motion gates are active.
- SpO2: suppressed for low perfusion, RED/IR imbalance, motion, contact transition, or beat instability.
- PTT: suppressed when PPG timing is not trustworthy.

## PPG 脉搏检测 / PPG Pulse Detector

现有流式 PPG 脉搏状态机仍是主检测器，保留自适应迟滞、IBI 生理范围、IBI 中位数突跳检查、
幅度跌落/尖峰检查、运动和信号质量检查。新增 SQI 只记录被拒绝候选 beat 的原因，并把
IBI/幅度拒绝计数暴露到日志。

The existing streaming PPG pulse state machine remains the production detector. It keeps adaptive
hysteresis, physiological IBI range checks, median IBI jump checks, amplitude drop/spike checks,
and motion/quality checks. The new SQI path records why candidate beats were rejected and exposes
IBI/amplitude rejection counters in logs.

## PPG 侧路 A/B 检测 / PPG Side A/B Detector

新增 `Core/Inc/app_ppg_side_elgendi.h` 和 `Core/Src/app_ppg_side_elgendi.c`，实现
Elgendi-style 双移动平均侧路检测。该路径使用 100 Hz 下约 11 点 `MApeak` 与 67 点
`MAbeat`、绝对能量、动态 offset 阈值、最小 block 宽度、不应期和 IBI 范围检查。

The side detector uses an Elgendi-style dual-moving-average path with 11-sample `MApeak`,
67-sample `MAbeat`, absolute energy, bounded offset thresholding, block-width checks,
refractory logic, and IBI-range checks.

它只输出 A/B 诊断计数：主/侧路峰 +/-150 ms 匹配数、主峰未匹配数、侧路未匹配数、
短 block/不应期/范围拒绝数、侧路最近 IBI/HR/匹配 delta。它不写 `bpm_valid`、
`spo2_valid`、`ptt_valid`，也不替换生产检测器。

It only emits A/B diagnostics and never writes `bpm_valid`, `spo2_valid`, or `ptt_valid`.

## ECG QRS 链路 / ECG QRS Path

ECG 链路保持原有实现：250 Hz ADC DMA、去直流、平滑、可选 50 Hz 陷波和 10-20 Hz 带通、
导数能量、120 ms 移动窗积分、自适应阈值、动态不应期、T 波保护、search-back，以及导联脱落、
ADC 饱和、DMA 溢出、平线和无 R 峰质量状态。

The ECG path is unchanged: 250 Hz ADC DMA, DC removal, smoothing, optional 50 Hz notch and
10-20 Hz bandpass, derivative energy, 120 ms moving-window integration, adaptive thresholds,
dynamic refractory handling, T-wave guard, search-back, and quality states for lead-off,
ADC saturation, DMA overflow, flatline, and missing R peaks.

## PTT 置信门控 / PTT Confidence Gate

PTT 只有在 ECG 和 PPG 两侧时序锚点都可信时才输出。门控条件包括 ECG valid、无导联脱落、
ECG SQI 达标、无近期 ADC 饱和/DMA 溢出原因、PPG SQI 允许 PTT、无 PPG 运动/接触/低灌注门控、
存在已接受 IBI、PPG 峰时刻未陈旧、PTT 位于 `80-350 ms` 可信范围，以及相对短历史中位数无过大突跳。

PTT is accepted only when both timing anchors are trustworthy: ECG valid, no ECG lead-off,
ECG SQI above threshold, no recent ADC saturation or DMA overflow reason, PPG SQI permits PTT,
no PPG motion/contact/low-perfusion gate, accepted IBI is present, the PPG peak is not stale,
PTT is in the trusted `80-350 ms` range, and the value does not jump excessively from the recent median.

拒绝计数 / Rejection counters:

- `ptt_reject_ecg_count`
- `ptt_reject_ppg_count`
- `ptt_reject_range_count`
- `ptt_reject_jump_count`

## USART2 测量帧 / USART2 Measurement Frame

USART2 使用追加式 CSV 风格 `M` 帧。旧字段顺序保持不变，本次只在
`ecg_dma_available_high_watermark` 之后追加诊断字段，旧上位机可继续按稳定前缀解析。
当前完整帧为 147 列（列 0 为 `M`），最后三列为 `schema_version=2`、
`field_count=147` 和递增 `frame_seq`。

USART2 sends append-only CSV-style `M` frames. Existing field order is unchanged. This branch
only appends diagnostics after `ecg_dma_available_high_watermark`, so older host tools can keep
parsing the stable prefix. The full frame now has 147 columns including column 0 `M`; the last
three fields are `schema_version=2`, `field_count=147`, and a monotonic `frame_seq`.

| Index | Field | 中文说明 / Description |
| ---: | --- | --- |
| 110 | `ppg_sqi_score` | SQI 后 PPG 质量分 / SQI-adjusted PPG quality score |
| 111 | `ppg_sqi_flags` | SQI 标志位 / SQI flags |
| 112 | `ppg_sqi_low_perfusion_count` | 低灌注计数 / low-perfusion count |
| 113 | `ppg_sqi_motion_count` | 运动门控计数 / motion gate count |
| 114 | `ppg_sqi_balance_count` | RED/IR 平衡异常计数 / RED/IR balance count |
| 115 | `ppg_sqi_transition_count` | 接触过渡计数 / contact-transition count |
| 116 | `ppg_sqi_ibi_reject_count` | IBI 拒绝计数 / IBI reject count |
| 117 | `ppg_sqi_amp_reject_count` | beat 幅度拒绝计数 / amplitude reject count |
| 118 | `ppg_sqi_suppressed_count` | SQI 降分计数 / SQI suppression count |
| 119 | `ptt_reject_ecg_count` | ECG 侧 PTT 拒绝计数 / ECG-side PTT rejects |
| 120 | `ptt_reject_ppg_count` | PPG 侧 PTT 拒绝计数 / PPG-side PTT rejects |
| 121 | `ptt_reject_range_count` | PTT 范围拒绝计数 / PTT range rejects |
| 122 | `ptt_reject_jump_count` | PTT 突跳拒绝计数 / PTT jump rejects |
| 123 | `bpm_invalid_reason` | BPM 最近一次无效原因 / latest BPM invalid reason |
| 124 | `spo2_invalid_reason` | SpO2 最近一次无效原因 / latest SpO2 invalid reason |
| 125 | `ptt_invalid_reason` | PTT 最近一次拒绝原因 / latest PTT reject reason |
| 126 | `ppg_last_gate_flags` | 最近 PPG SQI 门控 flags / latest PPG SQI gate flags |
| 127 | `bpm_age_ms` | BPM 距最近接受更新的年龄 / BPM age since accepted update |
| 128 | `spo2_age_ms` | SpO2 距最近接受更新的年龄 / SpO2 age since accepted update |
| 129 | `ptt_match_age_ms` | PTT 匹配时 PPG 峰年龄 / PTT PPG-peak age at match |
| 130 | `output_stale_flags` | 输出陈旧 flags：bit0 BPM, bit1 SpO2, bit2 PTT |
| 131 | `ppg_output_sample` | 最近主 PPG beat 样本号 / latest production PPG beat sample |
| 132 | `ppg_side_peak_count` | Elgendi 侧路接受峰计数 / accepted side peaks |
| 133 | `ppg_side_current_peak_count` | 主检测器接受峰计数 / production accepted peaks |
| 134 | `ppg_side_match_count` | 主/侧路 +/-150ms 匹配计数 / matched peaks |
| 135 | `ppg_side_missed_current_count` | 主峰未被侧路匹配计数 / production peaks missed by side |
| 136 | `ppg_side_unmatched_count` | 侧路峰未匹配主峰计数 / side peaks unmatched to production |
| 137 | `ppg_side_reject_short_count` | 侧路短 block 拒绝计数 / short-block rejects |
| 138 | `ppg_side_reject_refractory_count` | 侧路不应期拒绝计数 / refractory rejects |
| 139 | `ppg_side_reject_range_count` | 侧路 IBI 范围拒绝计数 / IBI-range rejects |
| 140 | `ppg_side_last_ibi_ms` | 侧路最近 IBI / latest side IBI |
| 141 | `ppg_side_last_hr` | 侧路最近 HR / latest side HR |
| 142 | `ppg_side_last_delta_ms` | 侧路峰与主峰匹配时间差 / side-production peak delta |
| 143 | `ppg_side_last_block_ms` | 侧路最近接受 block 宽度 / latest accepted side block width |
| 144 | `schema_version` | 协议 schema 版本，当前为 2 / protocol schema version |
| 145 | `field_count` | 完整列数，当前为 147 / complete column count |
| 146 | `frame_seq` | M 帧递增序号 / monotonic M-frame sequence |

输出原因码 / output reason codes:

| Code | Meaning |
| ---: | --- |
| 0 | OK |
| 1 | no finger |
| 2 | contact settling |
| 3 | low signal quality |
| 4 | motion |
| 5 | low perfusion |
| 6 | RED/IR balance |
| 7 | beat unstable |
| 8 | stale/no recent update |
| 9 | ECG-side PTT reject |
| 10 | PTT range reject |
| 11 | PTT jump reject |

## SD 二进制日志 / SD Binary Log

SD 日志是二进制格式，不是 CSV。文件以 32 字节 `BMLG` 头开始，后接重复的 16 字节原始样本。
实时路径只把记录压入 RAM ring，物理写卡由 `SDtask` 延后执行。

SD logging is binary, not CSV. A file starts with a 32-byte `BMLG` header followed by repeated
16-byte raw samples. The real-time path only pushes records into a RAM ring; physical writes are
performed later by `SDtask`.

```c
typedef struct __attribute__((packed)) {
  uint8_t  magic[4];      /* "BMLG" */
  uint16_t version;       /* 1 */
  uint16_t sample_rate_hz;
  uint32_t start_tick;
  uint8_t  reserved[20];
} DataLogFileHeader_t;

typedef struct __attribute__((packed)) {
  uint32_t tick;
  uint32_t red;
  uint32_t ir;
  int16_t  ecg;
  uint8_t  flags;         /* bit0: finger, bit1: contact stable, bit2: FIFO overflow */
  uint8_t  seq;
} DataLogRawSample_t;
```

## 构建 / Build

Keil project:

```text
MDK-ARM/BME.uvprojx
```

本机 CLI 重新构建 / CLI rebuild on this machine:

```powershell
cd D:\CUBEMX\template\BME\MDK-ARM
& 'D:\Keil_v5\UV4\UV4.exe' -b BME.uvprojx -t BME -o build_ecg.log
```

期望结果 / Expected result:

```text
0 Error(s), 0 Warning(s)
```

## 验证计划 / Validation Plan

替换任何主检测器之前，应先用项目 SD 日志回放验证。建议覆盖安静佩戴、弱灌注、运动、
手指放上/拿开、IBI 不规则片段、ECG 导联脱落、ADC 饱和/平线、FIFO 积压或样本陈旧等场景。

Before replacing any production detector, validate with project SD log replay. Cover quiet finger,
weak perfusion, motion, attach/remove transitions, irregular IBI segments, ECG lead-off, ADC
saturation or flatline, FIFO backlog, and stale samples.

推荐指标 / Recommended metrics:

- PPG valid-rate
- SpO2 valid-rate
- HR/IBI jump count
- PPG SQI suppression count
- PPG IBI and amplitude reject counts
- PTT median and jitter
- PTT rejection reason counts
- ECG/PPG HR agreement

## 集成规则 / Integration Rules

- 实时路径不使用动态分配。 / Do not add dynamic allocation to real-time paths.
- MAX/ECG 实时路径不加入 OLED、UART printf、SD、EEPROM 写入或长 HAL delay。 / Do not add OLED, UART printf, SD, EEPROM writes, or long HAL delays to MAX/ECG real-time paths.
- 每样本更新尽量保持 O(1)。 / Keep per-sample updates O(1) where practical.
- 手指离开、FIFO 溢出、传感器恢复或测量重启时重置自适应信号状态。 / Reset adaptive signal state on finger loss, FIFO overflow, sensor recovery, or measurement restart.
- USART `M` 帧尽量只追加字段。 / Keep USART `M` frame changes append-only whenever possible.
- 除非同步重标定算法和文档，不要改变 `TIM6` 100 Hz 和 ECG 250 Hz 采样配置。 / Keep `TIM6` at 100 Hz and ECG sampling at 250 Hz unless algorithms and documentation are rescaled together.
