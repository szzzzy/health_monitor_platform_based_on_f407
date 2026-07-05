# Literature-Based Algorithm Improvement Prompts

Branch: `codex/literature-algorithm-prompts`

Scope: prompts only. This document converts the literature into engineering tasks. Do not modify firmware source files unless the user explicitly approves implementation.

## Evidence Map

- ECG QRS baseline: Pan and Tompkins, 1985. Use as the reference pattern for bandpass, derivative/energy transform, moving-window integration, adaptive thresholds, refractory logic, and search-back.
- ECG lightweight variant: Neri et al., Sensors 2023 AMPT. Use only as a simplification reference for single-lead real-time QRS detection. Treat reported performance cautiously because of possible product-linked interest.
- ECG adaptive threshold alternative: Christov, 2004. Use as the main ECG side-detector candidate: adaptive steep-slope threshold `M`, noise-integrating threshold `F`, and beat-expectation threshold `R`.
- ECG SQI caution: Rahman et al., J. R. Soc. Interface 2022. Do not trust universal fixed SQI thresholds. Prefer adaptive baselines, segment-local statistics, and project-log calibration.
- PPG peak detector candidate: Elgendi et al., PLOS ONE 2013. Use as the PPG systolic-peak side-detector candidate. Rescale all windows for this project's `100 Hz` PPG sample rate.
- PPG engineering review: Park et al., Frontiers in Physiology 2022. Use as the checklist for motion artifact, baseline wandering, low perfusion, contact transition, preprocessing, peak detection, and SQI gating.
- PPG SQA feature selection: Mohagheghian et al., IEEE TBME 2022. Use for lightweight SQI feature selection: IBI variability, skewness/kurtosis when cheap, sample entropy only offline or if bounded, wavelet/template features only if memory and CPU cost stay low.
- PPG broad review: Castaneda et al., IJBSBE 2018. Use as background only. Do not use it as implementation evidence.

## Global Algorithm Policy

Use the literature to improve reliability, not to chase paper accuracy numbers.

Hard rules:
- Keep the current production algorithms until project SD logs prove a candidate is better.
- Add side scores, side detectors, and debug counters before replacing any output path.
- Bad signal should clear or suppress validity flags instead of producing confident wrong HR, SpO2, RR, or PTT.
- Validity flags are first-class outputs. A stable invalid result is safer than a smooth but false number.
- Do not add dynamic allocation.
- Do not add blocking I2C, OLED, UART, SD, EEPROM, or long HAL delays to MAX/ECG real-time paths.
- Keep per-sample updates O(1) where practical.
- Reset new adaptive state on finger loss, ECG lead-off, ADC saturation, stale sensor data, or measurement restart.
- Fixed thresholds are allowed only for physiological safety limits. Signal-quality thresholds should adapt to local baselines whenever possible.

Implementation order:
1. PPG signal-quality gate using existing features.
2. PPG dual-moving-average side detector for A/B only.
3. ECG QRS/SQI side detector for A/B only.
4. PTT confidence gate using ECG and PPG quality.
5. Offline SD-log validation before replacing production behavior.

## Prompt 1: PPG SQI And Valid-Flag Gate

You are working in the STM32F407 BME firmware. Design a low-risk PPG signal-quality gate based on Frontiers 2022 and IEEE TBME 2022.

Read first:
- `Core/Src/app_ppg_pulse.c`
- `Core/Src/max30102_spo2.c`
- `Core/Src/max30102_bpm.c`
- `Core/Src/app_motion.c`
- `Core/Src/app_measurement.c`
- `Core/Inc/app_state.h`
- `Core/Src/app_protocol.c`

Goal:
- Add a bounded experimental PPG SQI score and rejection counters.
- Keep the current pulse detector as the production detector.
- Do not change HR, SpO2, HRV, RR, or PTT validity until logs prove the gate is better, unless the user explicitly approves gating behavior.

Use low-cost features first:
- `signal_ir_ac_rms`
- `signal_red_ac_rms`
- `signal_ir_pi_x1000`
- `signal_red_pi_x1000`
- RED/IR AC balance and RED/IR ratio consistency
- beat amplitude EMA, drop ratio, and spike ratio
- IBI physiological range, median IBI, and IBI jump ratio
- motion flag and motion persistence
- finger transition state and no-finger baseline stability

Optional features only if cheap:
- pulse width, rise time, fall time, and duty ratio from the existing state machine
- fixed-point short beat-template similarity
- bounded-window skewness/kurtosis, only if CPU cost is acceptable

Gate policy to design:
- Low perfusion: suppress confidence when PI/AC RMS is too low relative to local baseline.
- Contact transition: suppress confidence for a short settling window after finger attach/remove.
- Motion: suppress confidence when motion persists or coincides with IBI/amplitude jumps.
- Beat instability: reject beats whose IBI or amplitude jumps outside adaptive local limits.
- RED/IR inconsistency: suppress SpO2 confidence before suppressing HR.

Deliverable:
- Patch plan with fields, counters, update rates, reset points, and compile-time switch `APP_PPG_SQI_EXPERIMENTAL`.
- Initial threshold strategy separating fixed physiological bounds from adaptive signal-quality baselines.
- Debug outputs for valid-rate, suppressed windows, rejected IBI count, amplitude-jump count, motion-gated count, and CPU/RAM cost.

## Prompt 2: PPG Dual-Moving-Average Side Detector

Design an experimental PPG systolic-peak side detector based on Elgendi et al. 2013.

Project facts:
- PPG sample rate is `MAX30102_ALGO_SAMPLE_RATE_HZ = 100U`.
- Current PPG pulse detection is a streaming state machine in `Core/Src/app_ppg_pulse.c`.
- The side detector must not replace the existing detector until A/B validation proves better behavior.

Algorithm sketch:
- Use the existing filtered IR PPG signal or add a bounded integer preprocessing stage.
- Apply square or absolute-energy transform.
- Maintain two moving averages:
  - `MApeak`: 111 ms initial window -> about 11 samples at 100 Hz.
  - `MAbeat`: 667 ms initial window -> about 67 samples at 100 Hz.
- Dynamic threshold: `MAbeat + offset`, where offset is a bounded fraction of recent energy.
- Candidate blocks begin when `MApeak > threshold`.
- Reject blocks shorter than the systolic-width minimum.
- Choose the maximum filtered IR sample inside each accepted block as the systolic peak.
- Enforce refractory and physiological IBI limits.

Constraints:
- Fixed-size ring buffers only.
- Prefer integer arithmetic.
- O(1) update per sample.
- No production output change at first.
- State must reset on finger loss, FIFO stale data, or measurement restart.

Validation:
- Compare side peaks with current peaks over SD logs.
- Metrics: matched peaks within +/-150 ms, unmatched current peaks, unmatched side peaks, IBI median difference, HR difference, PTT peak-time jitter, noisy-window rejection count.
- Test quiet finger, weak perfusion, motion, attach/remove transitions, and irregular interval segments.

Deliverable:
- Patch plan with state location, reset logic, debug counters, and compile-time switch `APP_PPG_SIDE_ELGENDI`.

## Prompt 3: ECG QRS Side Detector And Adaptive SQI

Design an experimental ECG QRS side detector and ECG SQI upgrade using Pan-Tompkins, Christov 2004, AMPT, and Rahman 2022.

Read first:
- `Core/Src/app_ecg.c`
- `Core/Inc/app_ecg.h`
- `Core/Src/app_ptt.c`
- `Core/Src/app_protocol.c`
- `Core/Inc/app_state.h`

Project facts:
- ECG sample rate is `250 Hz`.
- Current ECG already uses DC removal, notch/bandpass, derivative energy, 120 ms MWI, adaptive threshold, T-wave guard, search-back, HR/RR/PTT update, and ECG quality scoring.

Candidate improvements:
- Keep current ECG detector as production.
- Add optional Christov-style side detector:
  - `M`: adaptive steep-slope threshold
  - `F`: noise-integrating threshold
  - `R`: beat-expectation threshold
  - dynamic refractory and RR expectation
- Improve ECG SQI with adaptive local baselines rather than universal fixed thresholds.
- Keep lead-off, ADC saturation, DMA overflow, stale R-peak handling, and no-peak timeout behavior intact.

Failure modes to target:
- T wave detected as R peak
- low-amplitude R peak missed
- noise spike detected as QRS
- long no-peak interval after missed beats
- lead-off or flatline producing false HR

Validation:
- Side detector agreement with current detector.
- False early detections after T waves.
- Missed low-amplitude beats.
- Behavior under high noise, flatline, lead-off, ADC saturation, and long no-peak intervals.
- Impact on PTT stability.
- CPU/RAM delta.

Deliverable:
- Risk-ranked patch plan with compile-time switches `APP_ECG_QRS_SIDE_CHRISTOV` and `APP_ECG_SQI_EXPERIMENTAL`.
- Debug fields for side/current peak agreement, missed side/current peaks, T-wave rejects, search-back events, noise rejects, and SQI state.

## Prompt 4: PTT Confidence Gate

Design a PTT reliability gate. PTT must be treated as a dependent metric: it is trustworthy only when both ECG R-peak timing and PPG systolic-peak timing are trustworthy.

Read first:
- `Core/Src/app_ptt.c`
- `Core/Src/app_ecg.c`
- `Core/Src/app_ppg_pulse.c`
- `Core/Src/app_measurement.c`
- `Core/Inc/app_state.h`
- `Core/Src/app_protocol.c`

Goal:
- Stop PTT from jumping when either ECG or PPG is low quality.
- Do not use PTT as a blood-pressure estimate.
- Prefer freezing or clearing PTT validity over updating from a suspicious peak pair.

Gate inputs:
- ECG quality score and lead-off status
- R-peak freshness and RR plausibility
- PPG SQI, motion state, and finger contact state
- PPG beat amplitude and IBI plausibility
- PTT physiological range, initial trusted range 80-350 ms
- PTT jump from recent median

Gate policy:
- If ECG is lead-off, saturated, stale, or low-SQI: do not update PTT.
- If PPG is motion-gated, low-perfusion, finger-transitioning, or stale: do not update PTT.
- If the ECG/PPG pair gives PTT outside the trusted range: reject it.
- If PTT differs sharply from recent median without matching physiological context: reject or mark low confidence.
- Use a small median filter over accepted PTT values.
- Track rejected-by-ECG, rejected-by-PPG, rejected-by-range, and rejected-by-jump counters.

Deliverable:
- Patch plan with reset points, median-filter state, debug counters, and optional switch `APP_PTT_CONFIDENCE_GATE`.
- Exact rules for when to keep old PTT value, when to clear validity, and when to accept a new PTT.

## Prompt 5: Offline A/B Harness Before Firmware Replacement

Create an offline validation plan for comparing current and candidate algorithms using SD binary logs.

Read:
- `README.md` SD binary log section
- `Core/Src/app_data_log.c`
- `Core/Inc/app_data_log.h`
- `Core/Src/app_protocol.c`

Goal:
- Decode `.BIN` logs with `BMLG` headers.
- Re-run current-like PPG/ECG logic and candidate side detectors offline.
- Produce CSV summaries and plots for manual inspection.

Metrics:
- PPG peak count, ECG R-peak count, matched peak count, unmatched peak count
- IBI/RR distribution and jump count
- HR difference between current and candidate outputs
- SpO2 valid-rate and suspicious-update count
- SQI suppression count
- PTT median, PTT jitter, and PTT rejection reason counts
- Segment labels: quiet, motion, low perfusion, baseline wander, finger transition, ECG lead-off
- Estimated CPU operation count and RAM cost for firmware implementation

Rules:
- Do not treat literature-reported accuracy as ground truth.
- Prefer project logs over paper numbers.
- Do not recommend replacing production behavior unless the candidate is better across quiet, weak, noisy, and transition segments.

Deliverable:
- Script interface and output schema.
- Pass/fail report template.
- No firmware changes unless explicitly approved later.

## Acceptance Criteria For Replacing Any Production Path

A candidate may replace production behavior only if project logs show:
- Quiet-finger valid-rate does not drop materially.
- Motion or low-perfusion false-valid suspicion decreases.
- HR/IBI jump count decreases or stays equal.
- SpO2 suspicious-update count decreases or stays equal.
- PTT jitter decreases, or PTT validity becomes more conservative with clear rejection reasons.
- Finger attach/remove transitions produce fewer confident wrong values.
- CPU and RAM deltas fit STM32F407 real-time constraints.
- No new blocking work appears in MAX/ECG real-time paths.
- Keil target `BME` builds with `0 Error(s), 0 Warning(s)`.

If results are mixed, keep the candidate as debug-only and collect more logs.

## Minimal First Experiment Prompt

If only one implementation is approved, do this first:

Add `APP_PPG_SQI_EXPERIMENTAL` as a debug-only PPG quality score using existing fields only: IR/RED AC RMS, IR/RED PI, RED/IR balance, beat amplitude EMA, IBI median/jump check, motion flag, and finger-contact state.

Do not change HR, SpO2, HRV, RR, or PTT outputs yet. Expose only counters and optional debug-page/UART diagnostic fields. Validate on quiet finger, weak perfusion, motion, and finger transition logs. Recommend gating production valid flags only after false-valid cases decrease without damaging quiet-finger valid-rate.
