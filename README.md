# BME STM32F407 Physiological Signal Firmware

This repository contains the STM32F407ZGTx firmware for a physiological signal acquisition board built around:

- MAX30102 PPG
- AD8232 ECG
- SSD1306 128x64 I2C OLED
- M24C02 EEPROM
- RTC
- USART2 reporting
- SD/FatFs binary logging
- FreeRTOS task scheduling

The firmware is an engineering prototype for signal acquisition and trend observation. It is not a medical diagnostic device. PTT is treated only as an ECG-to-PPG timing metric and is not a blood-pressure estimate.

## Current Architecture

The firmware uses a FreeRTOS task model rather than a single blocking main loop.

| Task | Priority | Role |
| --- | --- | --- |
| `MAXtask` | High | 100 Hz MAX30102 service tick, FIFO drain, ECG DMA sample consumption, sensor recovery |
| `Uitask` | Normal | UART command polling, buttons, OLED refresh, periodic reports |
| `SDtask` | Low | Background binary log drain during safe windows |
| `watchdogtask` | Above normal | IWDG refresh and liveness snapshots |

Main signal paths:

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

## Literature-Guided Algorithm Update

This branch implements the first firmware-side algorithm improvement derived from the local literature and prompt pack in `docs/literature_algorithm_improvement_prompts.md`.

Evidence used:

- Pan and Tompkins 1985, `A_Real-Time_QRS_Detection_Algorithm.pdf`: ECG QRS processing pattern using filtering, nonlinear energy, moving-window integration, adaptive thresholds, refractory logic, and search-back.
- Christov 2004, `1475-925X-3-28.pdf`: adaptive ECG threshold concepts using slope, noise, and beat-expectation terms.
- Elgendi 2013, `file.pdf`: PPG systolic peak detection using two moving averages and offset thresholding.
- Park et al. 2022, `fphys-12-808451.pdf`: PPG engineering checklist for motion artifact, baseline wander, hypoperfusion, preprocessing, peak detection, and SQI.
- Mohagheghian et al. 2022, `Optimized_Signal_Quality_Assessment_for_Photoplethysmogram_Signals_Using_Feature_Selection.pdf`: lightweight PPG SQA features such as IBI variability, skewness/kurtosis, and entropy/template features.
- Rahman et al. 2022, `rsif.2022.0012.pdf`: caution against universal fixed SQI thresholds; prefer adaptive local baselines.

The implementation keeps the production PPG and ECG detectors. It adds a conservative PPG SQI gate and PTT confidence gate around the existing outputs.

## PPG SQI Gate

New module:

- `Core/Inc/app_ppg_sqi.h`
- `Core/Src/app_ppg_sqi.c`

Compile-time switches:

- `APP_PPG_SQI_EXPERIMENTAL = 1U`
- `APP_PPG_SQI_GATE_OUTPUTS = 1U`

The SQI module maintains adaptive local baselines for IR/RED AC RMS and PI. Baselines update only during stable, non-motion, non-transition windows. Bad windows can lower `app->signal_quality`, but they never raise it.

SQI flags:

| Flag | Bit | Meaning |
| --- | ---: | --- |
| `APP_PPG_SQI_FLAG_LOW_PERFUSION` | `0x01` | IR AC/PI is too low versus local baseline or absolute floor |
| `APP_PPG_SQI_FLAG_MOTION` | `0x02` | Existing PPG motion detector is active |
| `APP_PPG_SQI_FLAG_BALANCE` | `0x04` | RED/IR ratio or balance is unreliable |
| `APP_PPG_SQI_FLAG_TRANSITION` | `0x08` | Finger contact is still settling |
| `APP_PPG_SQI_FLAG_BEAT_UNSTABLE` | `0x10` | IBI or beat amplitude jump was detected |

Outputs affected:

- HR/BPM: suppressed when the SQI score is too low or contact/motion gates are active.
- SpO2: suppressed when low perfusion, RED/IR balance, motion, contact transition, or beat instability is present.
- PTT: suppressed when PPG timing is not trustworthy.

The gate uses fixed thresholds only for physiological or safety floors. Signal-quality decisions are primarily relative to local baselines.

## PPG Pulse Detector

The existing streaming PPG pulse detector remains the production detector:

- adaptive hysteresis from beat amplitude EMA
- physiological IBI range
- IBI median jump check
- beat amplitude drop/spike check
- motion and signal-quality checks

The detector now reports rejected IBI and amplitude candidates into the SQI counters. It does not allocate memory and remains O(1) per sample.

## ECG QRS Path

The ECG path already follows the literature-supported pattern:

- 250 Hz ADC DMA samples
- DC removal
- smoothing
- optional 50 Hz notch and 10-20 Hz bandpass
- derivative energy
- 120 ms moving-window integration
- adaptive signal/noise threshold
- dynamic refractory
- T-wave guard
- search-back
- lead-off, ADC saturation, DMA overflow, flatline, and no-R-peak quality states

This branch does not replace the ECG QRS detector. It keeps the current detector and uses ECG quality as an input to PTT confidence.

## PTT Confidence Gate

PTT is accepted only when both timing anchors are trustworthy:

- ECG valid
- no ECG lead-off
- ECG SQI above threshold
- no recent ECG ADC saturation or DMA overflow reason
- PPG SQI permits PTT
- PPG motion/contact/low-perfusion gates are clear
- accepted IBI is present
- estimated PPG peak age is not stale
- PTT is in the trusted range `80-350 ms`
- PTT jump from recent median is not excessive

Rejected PTT pairs are counted by reason:

- ECG gate
- PPG gate
- range gate
- jump gate

## USART2 Measurement Frame

USART2 sends append-only CSV-style `M` frames. Existing fields keep their original order. This branch appends diagnostics after `ecg_dma_available_high_watermark`.

New tail fields:

| Index | Field |
| ---: | --- |
| 110 | `ppg_sqi_score` |
| 111 | `ppg_sqi_flags` |
| 112 | `ppg_sqi_low_perfusion_count` |
| 113 | `ppg_sqi_motion_count` |
| 114 | `ppg_sqi_balance_count` |
| 115 | `ppg_sqi_transition_count` |
| 116 | `ppg_sqi_ibi_reject_count` |
| 117 | `ppg_sqi_amp_reject_count` |
| 118 | `ppg_sqi_suppressed_count` |
| 119 | `ptt_reject_ecg_count` |
| 120 | `ptt_reject_ppg_count` |
| 121 | `ptt_reject_range_count` |
| 122 | `ptt_reject_jump_count` |

Older host tools can continue parsing the stable prefix. New tools should use the appended counters to explain invalid HR, SpO2, and PTT states.

## SD Binary Log

SD logging is binary, not CSV. A file starts with a 32-byte `BMLG` header, followed by repeated 16-byte raw samples.

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

Sampling never waits for physical SD I/O. The real-time path only pushes records into a RAM ring; `SDtask` performs physical writes later.

## Build

Keil project:

```text
MDK-ARM/BME.uvprojx
```

CLI rebuild on this machine:

```powershell
cd D:\CUBEMX\template\BME\MDK-ARM
& 'D:\Keil_v5\UV4\UV4.exe' -b BME.uvprojx -t BME -o build_ecg.log
```

Expected clean result:

```text
0 Error(s), 0 Warning(s)
```

## Validation Plan

Before replacing any production detector, validate against project SD logs:

- quiet finger
- weak perfusion
- motion
- attach/remove transitions
- irregular IBI segments
- ECG lead-off
- ADC saturation or flatline
- FIFO backlog or stale samples

Recommended metrics:

- PPG valid-rate
- SpO2 valid-rate
- HR/IBI jump count
- PPG SQI suppression count
- PPG IBI and amplitude reject counts
- PTT median and jitter
- PTT rejection reason counts
- ECG/PPG HR agreement

Keep the current production detector if results are mixed.

## Integration Rules

- Do not add dynamic allocation to real-time paths.
- Do not add OLED, UART printf, SD, EEPROM writes, or long HAL delays to MAX/ECG real-time paths.
- Keep per-sample updates O(1) where practical.
- Reset adaptive signal state on finger loss, FIFO overflow, stale sensor recovery, or measurement restart.
- Keep USART `M` frame changes append-only whenever possible.
- Keep `TIM6` at 100 Hz and ECG sampling at 250 Hz unless all algorithms and documentation are rescaled together.
