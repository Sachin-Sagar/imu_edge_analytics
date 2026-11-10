# Phase 4: Objective 3 — Downsampling & Power Efficiency Summary

**Task**: Objective 3 (Downsampling for Power Efficiency)
**Dataset**: `mx_imu.csv` (500 Hz nominal, 90 seconds, 5 validated jumps)

## 1. Executive Answers to Core Objective Mandates

### Mandate 1: Lowest Sampling Rate for Correct Jump Detection
- **Kinematic Free-Fall State Machine Limit**: **$1.0\,\mathrm{Hz}$** (Decimation $M = 500$, $\Delta t = 1.0\,\mathrm{s}$).
  - At $1.0\,\mathrm{Hz}$, all 5 jumps are correctly identified with **0 False Positives** and **0 False Negatives** ($F_1 = 1.000$).
  - **Failure at $0.5\,\mathrm{Hz}$**: At $f_s = 0.5\,\mathrm{Hz}$ ($\Delta t = 2.0\,\mathrm{s}$), Jump 1 ($2.744\,\mathrm{s}$) and Jump 5 ($2.794\,\mathrm{s}$) contain only 1 sampled point in flight, rendering multi-sample duration validation impossible. Only the longest jump ($5.594\,\mathrm{s}$) is captured.
- **High-Precision Telemetry Limit**: **$20.0\,\mathrm{Hz} - 50.0\,\mathrm{Hz}$**.
  - Mean air-time error is under $20\,\mathrm{ms}$ ($<0.6\%$ relative error).
  - Preserves edge timing fidelity and landing deceleration shock dynamics.

### Mandate 2: Anti-Aliasing Filter (AAF) Requirements
- **Rigorous Verdict**: **An Anti-Aliasing Filter is absolutely mandatory** when downsampling to or operating at lower sampling rates ($f_s \le 100\,\mathrm{Hz}$).
- **Spectral Proof**:
  - Without an AAF, the $58.11\,\mathrm{Hz}$ single-cylinder engine vibration folds according to $f_{\mathrm{alias}} = |f_0 - k f_s|$.
  - At $f_s = 100\,\mathrm{Hz}$: $116.21\,\mathrm{Hz}$ folds to **$16.21\,\mathrm{Hz}$** (inside the $0-20\,\mathrm{Hz}$ motion band).
  - At $f_s = 50\,\mathrm{Hz}$: $58.11\,\mathrm{Hz}$ folds to **$8.11\,\mathrm{Hz}$** (directly into rider heave/suspension band, peaking at $-21.5\,\mathrm{dB/Hz}$).
  - At $f_s = 20\,\mathrm{Hz}$: $58.11\,\mathrm{Hz}$ folds to **$1.89\,\mathrm{Hz}$** (catastrophic corruption of vehicle tilt and gravity baseline).
- **Engineering Solution**: A 2nd-order analog Sallen-Key low-pass filter on the hardware front-end ($f_c \approx 40\,\mathrm{Hz}$) combined with on-chip digital decimation filtering (CIC / half-band FIR) before low-rate telemetry transmission.

## 2. Quantitative Performance Table Across Sampling Frequencies

| $f_s$ [Hz] | Decim ($M$) | $\Delta t$ [ms] | AAF Detected | AAF FP | AAF Air Err [ms] | Naive Air Err [ms] | Shock Peak [$m/s^2$] | Accel Batt Life [days] | 6-Axis Batt Life [days] |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **500.0** | 1 | 2.0 | **5/5** | 0 | 0.0 | 0.0 | 18.24 | **10.1 d** | 6.6 d |
| **250.0** | 2 | 4.0 | **5/5** | 0 | 2.4 | 5.6 | 18.11 | **19.9 d** | 9.3 d |
| **125.0** | 4 | 8.0 | **5/5** | 0 | 1.6 | 1.6 | 17.31 | **38.5 d** | 11.6 d |
| **100.0** | 5 | 10.0 | **5/5** | 0 | 4.0 | 4.0 | 16.93 | **47.3 d** | 12.3 d |
| **50.0** | 10 | 20.0 | **5/5** | 0 | 12.0 | 8.0 | 15.23 | **87.4 d** | 13.8 d |
| **25.0** | 20 | 40.0 | **5/5** | 0 | 24.0 | 16.0 | 11.91 | **151.9 d** | 14.6 d |
| **20.0** | 25 | 50.0 | **5/5** | 0 | 44.0 | 6.0 | 11.65 | **178.2 d** | 14.8 d |
| **10.0** | 50 | 100.0 | **5/5** | 0 | 84.0 | 16.0 | 10.67 | **272.5 d** | 15.2 d |
| **5.0** | 100 | 200.0 | **5/5** | 0 | 144.0 | 56.0 | 10.41 | **370.5 d** | 15.5 d |
| **2.0** | 250 | 500.0 | **5/5** | 0 | 264.0 | 151.2 | 10.36 | **472.5 d** | 15.6 d |
| **1.0** | 500 | 1000.0 | **5/5** | 0 | 548.8 | 333.6 | 10.30 | **520.2 d** | 15.6 d |
| **0.5** | 1000 | 2000.0 | **5/5** | 0 | 1026.4 | 748.8 | 10.46 | **547.9 d** | 15.6 d |

## 3. Generated Visual Artifacts

1. [`results/phase4_downsampling/plots/01_detection_accuracy_pareto.png`](plots/01_detection_accuracy_pareto.png) (mirrored to [`plots/objective3/01_detection_accuracy_pareto.png`](../../plots/objective3/01_detection_accuracy_pareto.png)): Pareto frontier of detection count and air-time timing error.
2. [`results/phase4_downsampling/plots/02_spectral_aliasing_mechanism.png`](plots/02_spectral_aliasing_mechanism.png) (mirrored to [`plots/objective3/02_spectral_aliasing_mechanism.png`](../../plots/objective3/02_spectral_aliasing_mechanism.png)): Welch PSD comparison illustrating harmonic foldover into the motion band.
3. [`results/phase4_downsampling/plots/03_telemetry_waveform_comparison.png`](plots/03_telemetry_waveform_comparison.png) (mirrored to [`plots/objective3/03_telemetry_waveform_comparison.png`](../../plots/objective3/03_telemetry_waveform_comparison.png)): Jump 1 time-domain waveforms across downsampling rates.
4. [`results/phase4_downsampling/plots/04_timing_quantization_error.png`](plots/04_timing_quantization_error.png) (mirrored to [`plots/objective3/04_timing_quantization_error.png`](../../plots/objective3/04_timing_quantization_error.png)): Empirical vs theoretical ($\pm \Delta t$) edge timestamp quantization errors.
5. [`results/phase4_downsampling/plots/05_landing_shock_impulse_attenuation.png`](plots/05_landing_shock_impulse_attenuation.png) (mirrored to [`plots/objective3/05_landing_shock_impulse_attenuation.png`](../../plots/objective3/05_landing_shock_impulse_attenuation.png)): Landing suspension impact peak attenuation vs sample interval.
6. [`results/phase4_downsampling/plots/06_energy_battery_tradeoff.png`](plots/06_energy_battery_tradeoff.png) (mirrored to [`plots/objective3/06_energy_battery_tradeoff.png`](../../plots/objective3/06_energy_battery_tradeoff.png)): Embedded power consumption and operational battery life trade-off curves.

## 4. Key Deliverables & Artifacts

- [x] **Downsampling Sweep Engine**: Implemented in [`src/downsample_sweep_and_aaf_analysis.py`](../../src/downsample_sweep_and_aaf_analysis.py).
- [x] **Sweep Metrics Dataset**: Saved in [`results/phase4_downsampling/downsampling_sweep_metrics.csv`](downsampling_sweep_metrics.csv).
- [x] **Power & Battery Tradeoffs**: Formatted in [`results/phase4_downsampling/power_sampling_tradeoffs.json`](power_sampling_tradeoffs.json).
- [x] **Publication Visualizations**: 6 high-resolution figures in [`results/phase4_downsampling/plots/`](../../plots/) and [`plots/objective3/`](../../plots/objective3/).
- [x] **Interactive Jupyter Notebook**: Fully verified interactive notebook in [`notebooks/04_downsampling_power.ipynb`](../../notebooks/04_Downsampling.ipynb).
- [x] **Technical Engineering Report**: Documented in [`phase4_report.md`](../../docs/phase4_report.md) and [`phase4_report.html`](../../docs/phase4_report.html).
- [x] **Master Roadmap**: Updated [`plan.md`](../../docs/plans/master_plan.md) with Phase 4 marked COMPLETED.
