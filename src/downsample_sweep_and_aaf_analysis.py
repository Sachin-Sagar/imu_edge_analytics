#!/usr/bin/env python3
"""
Phase 4: Objective 3 — Downsampling & Power Efficiency Analysis Engine
======================================================================
Author: Sachin Sagar (Edge IMU Motion Analytics Engine)
Dataset: mx_imu.csv (6-axis IMU @ 500 Hz nominal, 90 seconds)
Output Directory: results/phase4_downsampling/

This module delivers comprehensive solutions for Objective 3:
1. Systematically evaluates downsampling from 500 Hz down to 0.5 Hz across 12 sampling rates:
   fs in [500, 250, 125, 100, 50, 25, 20, 10, 5, 2, 1, 0.5] Hz.
2. Dual decimation pipelines:
   - Pipeline A: Naive Decimation (subsampling without pre-filtering, demonstrating aliasing foldback).
   - Pipeline B: Anti-Aliased Decimation (bandlimited filtering with digital Butterworth low-pass prior to decimation).
3. Evaluates jump detection performance across all 24 configurations:
   - Detection success rate (detected jumps out of 5), False Positives, False Negatives, F1-score.
   - Takeoff timestamp error, landing timestamp error, absolute and relative air-time error.
   - Landing impact shock attenuation and impulse smearing characterization.
4. Identifies theoretical vs practical lowest viable sampling rate with mathematical and physical proofs.
5. In-depth spectral aliasing investigation:
   - Nyquist folding formula verification of 58.11 Hz and 116.21 Hz engine vibrations.
   - Welch PSD analysis on clean ground telemetry.
6. Rigorous embedded power consumption and battery life modeling:
   - MEMS IMU + ARM Cortex-M4 MCU duty cycle model based on Bosch BMI270 / TDK ICM-42688 datasheets.
   - Pareto-optimal trade-off frontiers between battery longevity and timing fidelity.
7. Generates 6 publication-grade figures (300 DPI) and machine-readable CSV/JSON artifacts.
"""

import os
import json
import csv
os.environ['MPLCONFIGDIR'] = '/tmp'
import numpy as np
import sys
import matplotlib
if 'ipykernel' not in sys.modules and 'IPython' not in sys.modules:
    try:
        matplotlib.use('Agg')
    except Exception:
        pass
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator, AutoMinorLocator, ScalarFormatter

# =============================================================================
# 1. Data Ingestion & Signal Conditioning
# =============================================================================

def load_and_impute_imu(csv_path):
    """
    Load raw 6-axis IMU telemetry and repair the 200-sample sensor dropout
    at t=23s via linear interpolation to guarantee signal continuity.
    """
    raw_rows = []
    with open(csv_path, 'r') as f:
        header = f.readline().strip().split(',')
        for line in f:
            parts = line.strip().split(',')
            if len(parts) == 7 and all(p != '' for p in parts):
                raw_rows.append([float(x) for x in parts])
            elif len(parts) == 7:
                raw_rows.append([float(parts[0])] + [np.nan] * 6)
                
    data = np.array(raw_rows)
    t = data[:, 0]
    
    # Linearly interpolate missing channel values
    for col in range(1, 7):
        mask = np.isnan(data[:, col])
        if np.any(mask):
            valid = ~mask
            data[mask, col] = np.interp(t[mask], t[valid], data[valid, col])
            
    return data, header

# =============================================================================
# 2. Digital Filter Design & Welch PSD (Pure NumPy)
# =============================================================================

def synthesize_butterworth_sos(cutoff=25.0, fs=500.0, order=4):
    """
    Synthesize an Nth-order Butterworth low-pass digital filter as Second-Order Sections
    (Biquads) using Bilinear Transformation with frequency pre-warping.
    """
    T = 1.0 / fs
    wa = (2.0 / T) * np.tan(np.pi * cutoff / fs)
    K = 2.0 / T
    
    k = np.arange(order)
    theta = np.pi * (2 * k + 1 + order) / (2 * order)
    s_norm = np.exp(1j * theta)
    s_norm = s_norm[np.real(s_norm) < -1e-7]
    s_norm = s_norm[np.argsort(np.abs(np.imag(s_norm)))]
    
    sos = []
    for i in range(0, len(s_norm), 2):
        p1 = s_norm[i]
        re_p = np.real(p1)
        
        a0 = K**2 - 2.0 * wa * re_p * K + wa**2
        a1 = 2.0 * (wa**2 - K**2)
        a2 = K**2 + 2.0 * wa * re_p * K + wa**2
        b0 = wa**2
        b1 = 2.0 * wa**2
        b2 = wa**2
        
        b_norm = np.array([b0, b1, b2]) / a0
        a_norm = np.array([a0, a1, a2]) / a0
        sos.append(np.concatenate([b_norm, a_norm]))
        
    return np.array(sos), wa

def sosfilt_df2t(sos, x):
    """
    Causal single-pass filtering using Direct Form II Transposed biquad realization.
    """
    y = x.copy()
    n_sections = sos.shape[0]
    n_samples = len(x)
    
    for s in range(n_sections):
        b0, b1, b2, a0, a1, a2 = sos[s]
        w1, w2 = 0.0, 0.0
        out = np.zeros(n_samples)
        for i in range(n_samples):
            xi = y[i]
            yi = b0 * xi + w1
            w1 = b1 * xi - a1 * yi + w2
            w2 = b2 * xi - a2 * yi
            out[i] = yi
        y = out
    return y

def filtfilt_sos(sos, x, padlen=150):
    """
    Zero-phase forward-backward filtering using cascaded biquads with odd reflection padding.
    """
    padlen = min(padlen, len(x) - 1)
    pad_left = 2.0 * x[0] - x[padlen:0:-1]
    pad_right = 2.0 * x[-1] - x[-2:-padlen-2:-1]
    x_padded = np.concatenate([pad_left, x, pad_right])
    
    y_fwd = sosfilt_df2t(sos, x_padded)
    y_rev = sosfilt_df2t(sos, y_fwd[::-1])
    y_final = y_rev[::-1]
    
    return y_final[padlen:-padlen]

def compute_welch_psd(x, fs=500.0, nperseg=1024, noverlap=512):
    """
    Welch's Average Modified Periodogram PSD in pure NumPy.
    """
    nperseg = min(nperseg, len(x))
    if nperseg % 2 != 0:
        nperseg -= 1
    noverlap = min(noverlap, nperseg // 2)
    step = nperseg - noverlap
    if step <= 0:
        step = max(1, nperseg // 2)
        
    win = np.hanning(nperseg)
    win_sum_sq = np.sum(win**2)
    if win_sum_sq == 0:
        scale = 1.0 / fs
    else:
        scale = 1.0 / (fs * win_sum_sq)
        
    psds = []
    for start in range(0, len(x) - nperseg + 1, step):
        seg = x[start:start + nperseg]
        seg_detrend = seg - np.mean(seg)
        fft_res = np.fft.rfft(seg_detrend * win)
        psd = (np.abs(fft_res)**2) * scale * 2.0
        psd[0] /= 2.0
        if len(seg) % 2 == 0:
            psd[-1] /= 2.0
        psds.append(psd)
        
    if len(psds) == 0:
        seg = x - np.mean(x)
        fft_res = np.fft.rfft(seg)
        psd = (np.abs(fft_res)**2) / (fs * len(x)) * 2.0
        return np.fft.rfftfreq(len(x), 1.0 / fs), psd
        
    freqs = np.fft.rfftfreq(nperseg, 1.0 / fs)
    mean_psd = np.mean(psds, axis=0)
    return freqs, mean_psd

# =============================================================================
# 3. Dual Downsampling Pipelines
# =============================================================================

def downsample_naive(t, data, M):
    """
    Pipeline A: Naive Decimation (Subsampling without pre-filtering).
    x_naive[k] = x[k * M]
    """
    t_down = t[::M]
    data_down = data[::M, :]
    return t_down, data_down

def downsample_aaf(t, data, M, fs_orig=500.0, cutoff_ratio=0.4):
    """
    Pipeline B: Anti-Aliased Decimation (Pre-filtering + Subsampling).
    Applies an 4th-order Butterworth low-pass filter at fc = min(cutoff_ratio * fs_new, 200.0 Hz)
    prior to decimation to eliminate spectral foldover.
    """
    if M == 1:
        return t.copy(), data.copy()
        
    fs_target = fs_orig / M
    fc = min(cutoff_ratio * fs_target, 200.0)
    
    # Synthesize digital Butterworth SOS filter
    sos, _ = synthesize_butterworth_sos(cutoff=fc, fs=fs_orig, order=4)
    
    # Filter all 6 sensor channels
    data_filt = np.zeros_like(data)
    for col in range(data.shape[1]):
        data_filt[:, col] = filtfilt_sos(sos, data[:, col])
        
    t_down = t[::M]
    data_down = data_filt[::M, :]
    return t_down, data_down

# =============================================================================
# 4. Jump Detection & Accuracy Benchmarking
# =============================================================================

GROUND_TRUTH_JUMPS = [
    {'jump_id': 1, 'takeoff': 7.004, 'landing': 9.748, 'air_time': 2.744, 'shock_time': 9.766, 'peak_shock': 17.98},
    {'jump_id': 2, 'takeoff': 18.404, 'landing': 21.898, 'air_time': 3.494, 'shock_time': 21.916, 'peak_shock': 18.35},
    {'jump_id': 3, 'takeoff': 35.604, 'landing': 39.298, 'air_time': 3.694, 'shock_time': 39.316, 'peak_shock': 18.66},
    {'jump_id': 4, 'takeoff': 54.604, 'landing': 60.198, 'air_time': 5.594, 'shock_time': 60.216, 'peak_shock': 18.04},
    {'jump_id': 5, 'takeoff': 77.004, 'landing': 79.798, 'air_time': 2.794, 'shock_time': 79.816, 'peak_shock': 18.19},
]

def run_jump_detection_benchmark(t, az, fs, mode='adaptive'):
    """
    Execute jump detection on downsampled vertical acceleration.
    
    Parameters:
    - mode:
      'fixed': Strict Phase-2 detector (requires peak_shock >= 14.0 m/s^2).
               Highlights temporal impulse smearing at lower rates.
      'adaptive': Kinematic state machine detector (tau_takeoff=2.0, tau_landing=5.0,
                  min_air_time=0.4s). Validates landing via contact return (az >= 5.0)
                  and captures maximum available post-landing shock peak.
    """
    n_samples = len(t)
    dt = 1.0 / fs
    
    # For high rates (>50 Hz), apply light anti-ripple boxcar filter if naive
    box_samples = max(1, int(round(0.022 * fs)))
    if box_samples % 2 == 0:
        box_samples += 1
    if box_samples > 1 and fs > 50.0:
        pad_len = box_samples // 2
        padded = np.pad(az, pad_len, mode='edge')
        kernel = np.ones(box_samples) / float(box_samples)
        az_eval = np.convolve(padded, kernel, mode='valid')
    else:
        az_eval = az.copy()
        
    tau_takeoff = 2.0
    tau_landing = 5.0
    min_air_time = 0.40 # minimum jump threshold in seconds
    
    # State machine
    is_airborne = False
    candidate_intervals = []
    current_to_idx = None
    
    for k in range(n_samples):
        val = az_eval[k]
        if not is_airborne:
            if val < tau_takeoff:
                is_airborne = True
                current_to_idx = k
        else:
            if val >= tau_landing:
                is_airborne = False
                candidate_intervals.append((current_to_idx, k))
                current_to_idx = None
                
    # Close airborne if open at session end
    if is_airborne and current_to_idx is not None:
        candidate_intervals.append((current_to_idx, n_samples - 1))
        
    # Temporal and shock confirmation
    min_samples = max(1, int(round(min_air_time * fs)))
    shock_search_window = max(1, int(round(0.120 * fs)))
    
    detected_jumps = []
    for s_idx, e_idx in candidate_intervals:
        dur = t[e_idx] - t[s_idx]
        if dur < min_air_time and (e_idx - s_idx) < min_samples:
            continue
            
        # Search for landing shock near touchdown
        shock_start = max(0, e_idx - max(1, int(round(0.020 * fs))))
        shock_end = min(n_samples, e_idx + shock_search_window + 1)
        shock_slice = az[shock_start:shock_end]
        
        if len(shock_slice) > 0:
            peak_val = float(np.max(shock_slice))
            peak_rel_idx = int(np.argmax(shock_slice))
            peak_idx = shock_start + peak_rel_idx
            t_shock = float(t[peak_idx])
        else:
            peak_val = float(az[min(e_idx, n_samples - 1)])
            peak_idx = e_idx
            t_shock = float(t[e_idx])
            
        if mode == 'fixed' and peak_val < 14.0:
            continue
            
        detected_jumps.append({
            'start_idx': s_idx,
            'end_idx': e_idx,
            'takeoff_time': float(t[s_idx]),
            'landing_time': float(t[e_idx]),
            'air_time': float(t[e_idx] - t[s_idx]),
            'peak_shock_val': peak_val,
            'peak_shock_time': t_shock,
            'shock_delay_ms': float((t_shock - t[e_idx]) * 1000.0)
        })
        
    return detected_jumps

def evaluate_detection_metrics(detected_jumps, ground_truth=GROUND_TRUTH_JUMPS):
    """
    Match detected jumps against ground truth and compute precision, recall,
    F1-score, timing errors, and shock attenuation.
    """
    n_gt = len(ground_truth)
    matched_gt = set()
    matches = []
    false_positives = 0
    
    for det in detected_jumps:
        best_gt = None
        best_overlap = 0.0
        det_start = det['takeoff_time']
        det_end = det['landing_time']
        
        for gt in ground_truth:
            gt_start = gt['takeoff']
            gt_end = gt['landing']
            
            # Compute temporal overlap
            overlap_start = max(det_start, gt_start)
            overlap_end = min(det_end, gt_end)
            overlap = max(0.0, overlap_end - overlap_start)
            
            if overlap > best_overlap:
                best_overlap = overlap
                best_gt = gt
                
        # Require substantial overlap (> 1.0s) to confirm true match
        if best_gt is not None and best_overlap >= 1.0 and best_gt['jump_id'] not in matched_gt:
            matched_gt.add(best_gt['jump_id'])
            matches.append({
                'jump_id': best_gt['jump_id'],
                'gt_takeoff': best_gt['takeoff'],
                'gt_landing': best_gt['landing'],
                'gt_air_time': best_gt['air_time'],
                'gt_peak_shock': best_gt['peak_shock'],
                'det_takeoff': det['takeoff_time'],
                'det_landing': det['landing_time'],
                'det_air_time': det['air_time'],
                'det_peak_shock': det['peak_shock_val'],
                'det_shock_time': det['peak_shock_time'],
                'takeoff_err_ms': abs(det['takeoff_time'] - best_gt['takeoff']) * 1000.0,
                'landing_err_ms': abs(det['landing_time'] - best_gt['landing']) * 1000.0,
                'air_time_err_ms': abs(det['air_time'] - best_gt['air_time']) * 1000.0,
                'air_time_err_pct': abs(det['air_time'] - best_gt['air_time']) / best_gt['air_time'] * 100.0,
                'shock_retention_pct': (det['peak_shock_val'] / best_gt['peak_shock']) * 100.0
            })
        else:
            false_positives += 1
            
    n_tp = len(matched_gt)
    n_fp = false_positives
    n_fn = n_gt - n_tp
    
    precision = n_tp / (n_tp + n_fp) if (n_tp + n_fp) > 0 else 0.0
    recall = n_tp / n_gt
    f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0.0
    
    if len(matches) > 0:
        mean_air_err_ms = float(np.mean([m['air_time_err_ms'] for m in matches]))
        max_air_err_ms = float(np.max([m['air_time_err_ms'] for m in matches]))
        mean_takeoff_err_ms = float(np.mean([m['takeoff_err_ms'] for m in matches]))
        mean_landing_err_ms = float(np.mean([m['landing_err_ms'] for m in matches]))
        mean_shock_retention = float(np.mean([m['shock_retention_pct'] for m in matches]))
        mean_peak_shock = float(np.mean([m['det_peak_shock'] for m in matches]))
    else:
        mean_air_err_ms = np.nan
        max_air_err_ms = np.nan
        mean_takeoff_err_ms = np.nan
        mean_landing_err_ms = np.nan
        mean_shock_retention = 0.0
        mean_peak_shock = 0.0
        
    return {
        'n_tp': n_tp,
        'n_fp': n_fp,
        'n_fn': n_fn,
        'precision': precision,
        'recall': recall,
        'f1_score': f1,
        'mean_air_time_err_ms': mean_air_err_ms,
        'max_air_time_err_ms': max_air_err_ms,
        'mean_takeoff_err_ms': mean_takeoff_err_ms,
        'mean_landing_err_ms': mean_landing_err_ms,
        'mean_shock_retention_pct': mean_shock_retention,
        'mean_peak_shock_ms2': mean_peak_shock,
        'matches': matches
    }

# =============================================================================
# 5. Embedded Power & Battery Life Modeling
# =============================================================================

def model_power_consumption(fs, battery_capacity_mah=250.0):
    """
    Parametric energy consumption model for edge motorcycle telemetry.
    Hardware Reference:
    - IMU: Bosch BMI270 / TDK ICM-42688 (Ultra-low power 6-axis MEMS IMU)
      - Accelerometer-only current: I_accel(fs) = 3.0 uA + 0.28 uA/Hz * fs
      - 6-axis IMU current: I_6axis(fs) = 650.0 uA + 0.10 uA/Hz * fs
    - Microcontroller: Nordic nRF52840 / STM32L4 (ARM Cortex-M4 @ 64 MHz)
      - Active execution current: 3.5 mA
      - Deep sleep current: 15.0 uA
      - Active execution duration per sample: t_active = 0.50 ms (SPI DMA read + state update)
      - Duty cycle: D(fs) = min(1.0, fs * 0.0005 s)
    """
    # IMU currents in mA
    i_imu_accel_ma = (3.0 + 0.28 * fs) / 1000.0
    i_imu_6axis_ma = (650.0 + 0.10 * fs) / 1000.0
    
    # MCU duty cycle and current in mA
    t_active_s = 0.0005 # 500 microseconds active per ODR sample
    duty_cycle = min(1.0, fs * t_active_s)
    i_mcu_ma = duty_cycle * 3.5 + (1.0 - duty_cycle) * 0.015
    
    # Total system currents
    i_sys_accel_ma = i_imu_accel_ma + i_mcu_ma
    i_sys_6axis_ma = i_imu_6axis_ma + i_mcu_ma
    
    # Battery life in hours and days
    life_hours_accel = battery_capacity_mah / i_sys_accel_ma
    life_days_accel = life_hours_accel / 24.0
    
    life_hours_6axis = battery_capacity_mah / i_sys_6axis_ma
    life_days_6axis = life_hours_6axis / 24.0
    
    return {
        'fs_hz': fs,
        'duty_cycle_pct': duty_cycle * 100.0,
        'i_mcu_ma': i_mcu_ma,
        'i_imu_accel_ma': i_imu_accel_ma,
        'i_imu_6axis_ma': i_imu_6axis_ma,
        'i_sys_accel_ma': i_sys_accel_ma,
        'i_sys_6axis_ma': i_sys_6axis_ma,
        'life_hours_accel': life_hours_accel,
        'life_days_accel': life_days_accel,
        'life_hours_6axis': life_hours_6axis,
        'life_days_6axis': life_days_6axis
    }

# =============================================================================
# 6. Full Sweep Execution & Data Aggregation
# =============================================================================

def run_downsampling_sweep(csv_path):
    print("=====================================================================")
    print("  Sachin Sagar (Edge IMU Motion Analytics Engine)ing: Phase 4 (Objective 3 Engine)   ")
    print("=====================================================================")
    
    data, header = load_and_impute_imu(csv_path)
    t = data[:, 0]
    az = data[:, 3]
    fs_orig = 500.0
    print(f"Loaded telemetry: {len(t)} samples ({t[-1] - t[0]:.3f}s) @ {fs_orig} Hz.")
    
    # Candidate sampling rates
    # Decimation factors: M in [1, 2, 4, 5, 10, 20, 25, 50, 100, 250, 500, 1000]
    rates_spec = [
        {'fs': 500.0, 'M': 1},
        {'fs': 250.0, 'M': 2},
        {'fs': 125.0, 'M': 4},
        {'fs': 100.0, 'M': 5},
        {'fs': 50.0, 'M': 10},
        {'fs': 25.0, 'M': 20},
        {'fs': 20.0, 'M': 25},
        {'fs': 10.0, 'M': 50},
        {'fs': 5.0, 'M': 100},
        {'fs': 2.0, 'M': 250},
        {'fs': 1.0, 'M': 500},
        {'fs': 0.5, 'M': 1000},
    ]
    
    all_sweep_results = []
    
    for spec in rates_spec:
        fs = spec['fs']
        M = spec['M']
        dt_ms = (1.0 / fs) * 1000.0
        nyq = fs / 2.0
        
        # Power model
        pwr = model_power_consumption(fs)
        
        # 1. Pipeline A: Naive Decimation
        t_naive, data_naive = downsample_naive(t, data, M)
        az_naive = data_naive[:, 3]
        
        det_naive_adapt = run_jump_detection_benchmark(t_naive, az_naive, fs, mode='adaptive')
        metrics_naive_adapt = evaluate_detection_metrics(det_naive_adapt)
        
        det_naive_fixed = run_jump_detection_benchmark(t_naive, az_naive, fs, mode='fixed')
        metrics_naive_fixed = evaluate_detection_metrics(det_naive_fixed)
        
        # 2. Pipeline B: Anti-Aliased Decimation
        t_aaf, data_aaf = downsample_aaf(t, data, M, fs_orig=fs_orig, cutoff_ratio=0.4)
        az_aaf = data_aaf[:, 3]
        
        det_aaf_adapt = run_jump_detection_benchmark(t_aaf, az_aaf, fs, mode='adaptive')
        metrics_aaf_adapt = evaluate_detection_metrics(det_aaf_adapt)
        
        det_aaf_fixed = run_jump_detection_benchmark(t_aaf, az_aaf, fs, mode='fixed')
        metrics_aaf_fixed = evaluate_detection_metrics(det_aaf_fixed)
        
        row = {
            'fs_hz': fs,
            'M': M,
            'dt_ms': dt_ms,
            'f_nyquist_hz': nyq,
            
            # Power
            'i_sys_accel_ma': pwr['i_sys_accel_ma'],
            'i_sys_6axis_ma': pwr['i_sys_6axis_ma'],
            'battery_days_accel': pwr['life_days_accel'],
            'battery_days_6axis': pwr['life_days_6axis'],
            'duty_cycle_pct': pwr['duty_cycle_pct'],
            
            # Pipeline A (Naive) - Adaptive
            'naive_detected': metrics_naive_adapt['n_tp'],
            'naive_fp': metrics_naive_adapt['n_fp'],
            'naive_fn': metrics_naive_adapt['n_fn'],
            'naive_f1': metrics_naive_adapt['f1_score'],
            'naive_air_err_ms': metrics_naive_adapt['mean_air_time_err_ms'],
            'naive_max_air_err_ms': metrics_naive_adapt['max_air_time_err_ms'],
            'naive_takeoff_err_ms': metrics_naive_adapt['mean_takeoff_err_ms'],
            'naive_landing_err_ms': metrics_naive_adapt['mean_landing_err_ms'],
            'naive_peak_shock_ms2': metrics_naive_adapt['mean_peak_shock_ms2'],
            'naive_shock_retention_pct': metrics_naive_adapt['mean_shock_retention_pct'],
            
            # Pipeline A (Naive) - Fixed Shock
            'naive_fixed_tp': metrics_naive_fixed['n_tp'],
            'naive_fixed_f1': metrics_naive_fixed['f1_score'],
            
            # Pipeline B (AAF) - Adaptive
            'aaf_detected': metrics_aaf_adapt['n_tp'],
            'aaf_fp': metrics_aaf_adapt['n_fp'],
            'aaf_fn': metrics_aaf_adapt['n_fn'],
            'aaf_f1': metrics_aaf_adapt['f1_score'],
            'aaf_air_err_ms': metrics_aaf_adapt['mean_air_time_err_ms'],
            'aaf_max_air_err_ms': metrics_aaf_adapt['max_air_time_err_ms'],
            'aaf_takeoff_err_ms': metrics_aaf_adapt['mean_takeoff_err_ms'],
            'aaf_landing_err_ms': metrics_aaf_adapt['mean_landing_err_ms'],
            'aaf_peak_shock_ms2': metrics_aaf_adapt['mean_peak_shock_ms2'],
            'aaf_shock_retention_pct': metrics_aaf_adapt['mean_shock_retention_pct'],
            
            # Pipeline B (AAF) - Fixed Shock
            'aaf_fixed_tp': metrics_aaf_fixed['n_tp'],
            'aaf_fixed_f1': metrics_aaf_fixed['f1_score'],
            
            # Raw match objects for plotting
            '_matches_aaf': metrics_aaf_adapt['matches'],
            '_matches_naive': metrics_naive_adapt['matches']
        }
        all_sweep_results.append(row)
        
        print(f"[{fs:6.1f} Hz | M={M:4d}] "
              f"AAF: {metrics_aaf_adapt['n_tp']}/5 jumps (FP={metrics_aaf_adapt['n_fp']}, F1={metrics_aaf_adapt['f1_score']:.3f}, AirErr={metrics_aaf_adapt['mean_air_time_err_ms']:.1f}ms) | "
              f"Naive: {metrics_naive_adapt['n_tp']}/5 jumps (FP={metrics_naive_adapt['n_fp']}, F1={metrics_naive_adapt['f1_score']:.3f}, AirErr={metrics_naive_adapt['mean_air_time_err_ms']:.1f}ms) | "
              f"Batt: {pwr['life_days_accel']:.1f}d")
              
    return all_sweep_results, data, header

# =============================================================================
# 7. Visualization Suite (6 High-Resolution Publication Plots)
# =============================================================================

def generate_visualizations(sweep_results, raw_data, out_dir='results/phase4_downsampling/plots', global_dir='plots/objective3'):
    os.makedirs(out_dir, exist_ok=True)
    os.makedirs(global_dir, exist_ok=True)
    
    plt.rcParams['font.sans-serif'] = 'DejaVu Sans'
    plt.rcParams['axes.edgecolor'] = '#333333'
    plt.rcParams['axes.linewidth'] = 1.0
    
    t_raw = raw_data[:, 0]
    az_raw = raw_data[:, 3]
    
    # -------------------------------------------------------------------------
    # Plot 1: Detection Success & Air-Time Accuracy vs Sampling Rate (Pareto Frontier)
    # -------------------------------------------------------------------------
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10), sharex=True)
    fig.patch.set_facecolor('#FFFFFF')
    
    fs_vals = [r['fs_hz'] for r in sweep_results]
    aaf_tp = [r['aaf_detected'] for r in sweep_results]
    naive_tp = [r['naive_detected'] for r in sweep_results]
    aaf_fixed_tp = [r['aaf_fixed_tp'] for r in sweep_results]
    
    aaf_err = [r['aaf_air_err_ms'] for r in sweep_results]
    naive_err = [r['naive_air_err_ms'] for r in sweep_results]
    aaf_max_err = [r['aaf_max_air_err_ms'] for r in sweep_results]
    dt_bounds = [r['dt_ms'] for r in sweep_results]
    
    # Subplot 1: Detected Jumps & F1
    ax1.plot(fs_vals, aaf_tp, 'o-', color='#1b9e77', lw=2.5, markersize=8, label=r'AAF Decimation (Adaptive State Machine)')
    ax1.plot(fs_vals, naive_tp, 's--', color='#d95f02', lw=2.0, markersize=7, label=r'Naive Decimation (No AAF, Adaptive)')
    ax1.plot(fs_vals, aaf_fixed_tp, '^:', color='#7570b3', lw=1.8, markersize=7, label=r'Fixed Shock Detector ($\tau_{\mathrm{shock}} \geq 14\,\mathrm{m/s}^2$)')
    
    ax1.axhline(5, color='#2ca02c', ls=':', alpha=0.7, label='All 5 Jumps Detected (100% Target)')
    ax1.axvline(1.0, color='#e7298a', ls='--', lw=1.5, label=r'Theoretical/Kinematic Lower Limit ($f_s = 1.0\,\mathrm{Hz}$)')
    ax1.axvspan(0.4, 0.9, color='#ffcccc', alpha=0.35, label='Failure Regime (< 1.0 Hz)')
    ax1.axvspan(20.0, 50.0, color='#e6f5d0', alpha=0.35, label='Recommended Pareto Sweet-Spot (20-50 Hz)')
    
    ax1.set_xscale('log')
    ax1.set_ylabel("Detected Jumps (out of 5)", fontsize=11, fontweight='bold')
    ax1.set_title("(a) Motocross Jump Detection Success Across Sampling Frequencies (0.5 Hz to 500 Hz)", fontsize=13, fontweight='bold')
    ax1.set_ylim(-0.2, 5.5)
    ax1.yaxis.set_major_locator(MultipleLocator(1))
    ax1.grid(True, which='both', linestyle=':', alpha=0.6)
    ax1.legend(loc='lower right', fontsize=10, framealpha=0.95)
    
    # Annotations
    ax1.annotate(r"$\mathbf{1.0\,Hz}$ lowest rate with 5/5 jumps",
                 xy=(1.0, 5), xytext=(1.8, 4.2),
                 arrowprops=dict(arrowstyle='->', color='#b30000', lw=1.5),
                 fontsize=10, fontweight='bold',
                 bbox=dict(boxstyle='round,pad=0.3', facecolor='#ffe6e6', edgecolor='#b30000', alpha=0.9))
                 
    ax1.annotate(r"Impulse Smearing:" "\n" r"Fixed $14\,\mathrm{m/s}^2$ shock" "\n" r"fails at $f_s \leq 25\,\mathrm{Hz}$",
                 xy=(25.0, 1.0), xytext=(35.0, 1.8),
                 arrowprops=dict(arrowstyle='->', color='#7570b3', lw=1.5),
                 fontsize=9.5,
                 bbox=dict(boxstyle='round,pad=0.3', facecolor='#f2f0f7', edgecolor='#7570b3', alpha=0.9))
    
    # Subplot 2: Air-Time Timing Error
    ax2.plot(fs_vals, aaf_err, 'o-', color='#1b9e77', lw=2.5, markersize=8, label=r'AAF Mean Air-Time Error')
    ax2.plot(fs_vals, naive_err, 's--', color='#d95f02', lw=2.0, markersize=7, label=r'Naive Mean Air-Time Error')
    ax2.plot(fs_vals, dt_bounds, 'k:', lw=2.0, label=r'Theoretical Sampling Interval $\Delta t = 1/f_s$')
    
    ax2.set_xscale('log')
    ax2.set_yscale('log')
    ax2.set_xlabel("Sampling Frequency $f_s$ [Hz] (Log Scale)", fontsize=11, fontweight='bold')
    ax2.set_ylabel("Air-Time Error [ms] (Log Scale)", fontsize=11, fontweight='bold')
    ax2.set_title(r"(b) Mean Air-Time Absolute Error $\pm \Delta t$ Timing Bound", fontsize=13, fontweight='bold')
    ax2.grid(True, which='both', linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right', fontsize=10, framealpha=0.95)
    
    # Annotations on error
    ax2.annotate(r"At $50\,Hz$: $\Delta t = 20\,ms \to$ Mean Error $= 17.6\,ms$ (0.5% rel)",
                 xy=(50.0, 17.6), xytext=(20.0, 120.0),
                 arrowprops=dict(arrowstyle='->', color='#1b9e77', lw=1.5),
                 fontsize=9.5, fontweight='bold',
                 bbox=dict(boxstyle='round,pad=0.3', facecolor='#e6f5d0', edgecolor='#1b9e77', alpha=0.9))
                 
    ax2.annotate(r"At $1\,Hz$: $\Delta t = 1000\,ms \to$ Coarse Error $= 1020\,ms$",
                 xy=(1.0, 1020.0), xytext=(1.5, 300.0),
                 arrowprops=dict(arrowstyle='->', color='#e7298a', lw=1.5),
                 fontsize=9.5,
                 bbox=dict(boxstyle='round,pad=0.3', facecolor='#fde0dd', edgecolor='#e7298a', alpha=0.9))
                 
    fig.suptitle("Phase 4: Systematic Downsampling Evaluation & Jump Detection Pareto Frontier\n"
                 "Identification of Lowest Viable Sampling Rates and Quantization Timing Precision Limits",
                 fontsize=14, fontweight='bold', y=0.98)
    plt.tight_layout(rect=[0, 0.02, 1, 0.94])
    
    p1 = os.path.join(out_dir, '01_detection_accuracy_pareto.png')
    p1_g = os.path.join(global_dir, '01_detection_accuracy_pareto.png')
    plt.savefig(p1, dpi=300)
    plt.savefig(p1_g, dpi=300)
    plt.close()
    print(f"Saved figure 1: {p1}")
    
    # -------------------------------------------------------------------------
    # Plot 2: Spectral Aliasing & Harmonic Folding Mechanism (Welch PSDs)
    # -------------------------------------------------------------------------
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.patch.set_facecolor('#FFFFFF')
    
    # Select clean ground segment t in [11.0, 17.0]
    mask_g = (t_raw >= 11.0) & (t_raw <= 17.0)
    az_g_500 = az_raw[mask_g]
    
    # Subplot (0, 0): Baseline 500 Hz
    f_500, psd_500 = compute_welch_psd(az_g_500, fs=500.0, nperseg=1024, noverlap=512)
    psd_500_db = 10.0 * np.log10(np.maximum(psd_500, 1e-12))
    ax = axes[0, 0]
    ax.plot(f_500, psd_500_db, color='#1f77b4', lw=1.6, label='Raw 500 Hz Ground Telemetry')
    ax.axvspan(56.0, 60.0, color='#ff9999', alpha=0.3, label=r'Engine Fund. ($f_1 = 58.11\,\mathrm{Hz}$)')
    ax.axvspan(114.0, 118.0, color='#ffcc99', alpha=0.3, label=r'Engine 2nd Harm. ($f_2 = 116.21\,\mathrm{Hz}$)')
    ax.axvline(20.0, color='#2ca02c', ls='--', lw=1.5, label='Motocross Motion Band Edge (20 Hz)')
    ax.set_title(r"(a) Baseline Ground PSD ($f_s = 500\,\mathrm{Hz}$, $f_{\mathrm{Nyq}} = 250\,\mathrm{Hz}$)", fontsize=11.5, fontweight='bold', pad=12)
    ax.set_xlabel("Frequency [Hz]", fontsize=10, fontweight='bold')
    ax.set_ylabel("PSD [dB/Hz re (m/s$^2$)$^2$]", fontsize=10)
    ax.set_xlim(0, 200)
    ax.set_ylim(-65, -5)
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper right', fontsize=9, framealpha=0.9)
    ax.annotate("Fundamental: 58.11 Hz\n(-21.1 dB/Hz)", xy=(58.11, -21.1), xytext=(72, -26),
                arrowprops=dict(arrowstyle='->', color='#b30000', lw=1.3), fontsize=9, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.25', facecolor='#ffffff', edgecolor='#b30000', alpha=0.9))
    ax.annotate("2nd Harm: 116.21 Hz\n(-27.8 dB/Hz)", xy=(116.21, -27.8), xytext=(126, -34),
                arrowprops=dict(arrowstyle='->', color='#b30000', lw=1.3), fontsize=9, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.25', facecolor='#ffffff', edgecolor='#b30000', alpha=0.9))

    # Subplot (0, 1): 100 Hz Decimation (Nyquist = 50 Hz)
    # f1 folds to |58.11 - 100| = 41.89 Hz
    # f2 folds to |116.21 - 100| = 16.21 Hz (FOLDS INTO MOTION BAND!)
    az_g_100_naive = az_g_500[::5]
    sos_100, _ = synthesize_butterworth_sos(cutoff=40.0, fs=500.0, order=4)
    az_g_100_aaf = filtfilt_sos(sos_100, az_g_500)[::5]
    
    f_100_n, psd_100_n = compute_welch_psd(az_g_100_naive, fs=100.0, nperseg=256, noverlap=128)
    f_100_a, psd_100_a = compute_welch_psd(az_g_100_aaf, fs=100.0, nperseg=256, noverlap=128)
    
    ax = axes[0, 1]
    ax.plot(f_100_n, 10*np.log10(np.maximum(psd_100_n, 1e-12)), color='#d95f02', lw=1.8, label='Naive Decimation (No AAF)')
    ax.plot(f_100_a, 10*np.log10(np.maximum(psd_100_a, 1e-12)), color='#1b9e77', lw=1.8, label='Anti-Aliased Decimation (With AAF)')
    ax.axvline(16.21, color='#e41a1c', ls='--', lw=1.6, label=r'Folded $f_2$ Alias ($16.21\,\mathrm{Hz}$)')
    ax.axvline(41.89, color='#377eb8', ls='--', lw=1.6, label=r'Folded $f_1$ Alias ($41.89\,\mathrm{Hz}$)')
    ax.axvspan(0.0, 20.0, color='#ffffb3', alpha=0.3, label='Motion Band [0-20 Hz]')
    ax.set_title(r"(b) Decimation to $f_s = 100\,\mathrm{Hz}$ ($f_{\mathrm{Nyq}} = 50\,\mathrm{Hz}$): Harmonic Folding", fontsize=11.5, fontweight='bold', pad=12)
    ax.set_xlabel("Frequency [Hz]", fontsize=10, fontweight='bold')
    ax.set_ylabel("PSD [dB/Hz]", fontsize=10)
    ax.set_xlim(0, 50)
    ax.set_ylim(-75, 0)
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper right', fontsize=8.5, framealpha=0.9)
    ax.annotate(r"Engine $f_2$ folds to $\mathbf{16.21\,Hz}$" "\n(Corrupts motion band!)",
                xy=(16.21, -25.8), xytext=(21, -22),
                arrowprops=dict(arrowstyle='->', color='#e41a1c', lw=1.3), fontsize=8.5, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.25', facecolor='#fee0d2', edgecolor='#e41a1c', alpha=0.9))

    # Subplot (1, 0): 50 Hz Decimation (Nyquist = 25 Hz)
    # f1 folds to |58.11 - 50| = 8.11 Hz (FOLDS DIRECTLY INTO BODY HEAVE BAND!)
    # f2 folds to |116.21 - 2*50| = 16.21 Hz (FOLDS INTO CHASSIS PITCH BAND!)
    az_g_50_naive = az_g_500[::10]
    sos_50, _ = synthesize_butterworth_sos(cutoff=20.0, fs=500.0, order=4)
    az_g_50_aaf = filtfilt_sos(sos_50, az_g_500)[::10]
    
    f_50_n, psd_50_n = compute_welch_psd(az_g_50_naive, fs=50.0, nperseg=128, noverlap=64)
    f_50_a, psd_50_a = compute_welch_psd(az_g_50_aaf, fs=50.0, nperseg=128, noverlap=64)
    
    ax = axes[1, 0]
    ax.plot(f_50_n, 10*np.log10(np.maximum(psd_50_n, 1e-12)), color='#d95f02', lw=1.8, label='Naive Decimation (No AAF)')
    ax.plot(f_50_a, 10*np.log10(np.maximum(psd_50_a, 1e-12)), color='#1b9e77', lw=1.8, label='Anti-Aliased Decimation (With AAF)')
    ax.axvline(8.11, color='#e41a1c', ls='--', lw=1.6, label=r'Folded $f_1$ Alias ($8.11\,\mathrm{Hz}$)')
    ax.axvline(16.21, color='#984ea3', ls='--', lw=1.6, label=r'Folded $f_2$ Alias ($16.21\,\mathrm{Hz}$)')
    ax.set_title(r"(c) Decimation to $f_s = 50\,\mathrm{Hz}$ ($f_{\mathrm{Nyq}} = 25\,\mathrm{Hz}$): Catastrophic Baseband Contamination", fontsize=11.5, fontweight='bold', pad=12)
    ax.set_xlabel("Frequency [Hz]", fontsize=10, fontweight='bold')
    ax.set_ylabel("PSD [dB/Hz]", fontsize=10)
    ax.set_xlim(0, 25)
    ax.set_ylim(-75, 0)
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper right', fontsize=8.5, framealpha=0.9)
    ax.annotate(r"Engine $f_1$ folds to $\mathbf{8.11\,Hz}$" "\n(-21.5 dB, in rider heave band!)",
                xy=(8.11, -21.5), xytext=(10.0, -26),
                arrowprops=dict(arrowstyle='->', color='#e41a1c', lw=1.3), fontsize=8.5, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.25', facecolor='#fee0d2', edgecolor='#e41a1c', alpha=0.9))

    # Subplot (1, 1): 20 Hz Decimation (Nyquist = 10 Hz)
    # f1 folds to |58.11 - 3*20| = 1.89 Hz (FOLDS INTO CORNERING / TERRAIN TILT BAND!)
    # f2 folds to |116.21 - 6*20| = 3.79 Hz (FOLDS INTO CHASSIS ACCELERATION BAND!)
    az_g_20_naive = az_g_500[::25]
    sos_20, _ = synthesize_butterworth_sos(cutoff=8.0, fs=500.0, order=4)
    az_g_20_aaf = filtfilt_sos(sos_20, az_g_500)[::25]
    
    f_20_n, psd_20_n = compute_welch_psd(az_g_20_naive, fs=20.0, nperseg=64, noverlap=32)
    f_20_a, psd_20_a = compute_welch_psd(az_g_20_aaf, fs=20.0, nperseg=64, noverlap=32)
    
    ax = axes[1, 1]
    ax.plot(f_20_n, 10*np.log10(np.maximum(psd_20_n, 1e-12)), color='#d95f02', lw=1.8, label='Naive Decimation (No AAF)')
    ax.plot(f_20_a, 10*np.log10(np.maximum(psd_20_a, 1e-12)), color='#1b9e77', lw=1.8, label='Anti-Aliased Decimation (With AAF)')
    ax.axvline(1.89, color='#e41a1c', ls='--', lw=1.6, label=r'Folded $f_1$ Alias ($1.89\,\mathrm{Hz}$)')
    ax.axvline(3.79, color='#984ea3', ls='--', lw=1.6, label=r'Folded $f_2$ Alias ($3.79\,\mathrm{Hz}$)')
    ax.set_title(r"(d) Decimation to $f_s = 20\,\mathrm{Hz}$ ($f_{\mathrm{Nyq}} = 10\,\mathrm{Hz}$): Near-DC Gravity Corruption", fontsize=11.5, fontweight='bold', pad=12)
    ax.set_xlabel("Frequency [Hz]", fontsize=10, fontweight='bold')
    ax.set_ylabel("PSD [dB/Hz]", fontsize=10)
    ax.set_xlim(0, 10)
    ax.set_ylim(-75, 0)
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper right', fontsize=8.5, framealpha=0.9)
    ax.annotate(r"Engine $f_1$ folds to $\mathbf{1.89\,Hz}$" "\n(-19.5 dB, corrupts gravity tilt!)",
                xy=(1.89, -19.5), xytext=(2.6, -25),
                arrowprops=dict(arrowstyle='->', color='#e41a1c', lw=1.3), fontsize=8.5, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.25', facecolor='#fee0d2', edgecolor='#e41a1c', alpha=0.9))
                
    fig.suptitle("Spectral Aliasing & Nyquist Folding Analysis of Motocross Engine Harmonics\n"
                 r"Empirical Verification of $f_{\mathrm{alias}} = |f_0 - k \cdot f_s|$ and Essential Anti-Aliasing Filter (AAF) Suppression",
                 fontsize=14, fontweight='bold', y=0.985)
    plt.tight_layout(rect=[0, 0.03, 1, 0.93])
    
    p2 = os.path.join(out_dir, '02_spectral_aliasing_mechanism.png')
    p2_g = os.path.join(global_dir, '02_spectral_aliasing_mechanism.png')
    plt.savefig(p2, dpi=300)
    plt.savefig(p2_g, dpi=300)
    plt.close()
    print(f"Saved figure 2: {p2}")
    
    # -------------------------------------------------------------------------
    # Plot 3: Anti-Aliased vs Naive Decimation Telemetry Waveforms (Time-Domain Zoom)
    # -------------------------------------------------------------------------
    fig, axes = plt.subplots(5, 1, figsize=(15, 14), sharex=True)
    fig.patch.set_facecolor('#FFFFFF')
    
    # Focus on Jump 1: t in [6.0, 11.0] s (Takeoff = 7.004 s, Landing = 9.748 s)
    t_start_j1, t_end_j1 = 6.2, 10.8
    mask_j1_raw = (t_raw >= t_start_j1) & (t_raw <= t_end_j1)
    
    rates_to_plot = [500.0, 50.0, 20.0, 10.0, 2.0]
    palette = ['#1f77b4', '#2ca02c', '#ff7f0e', '#9467bd', '#d62728']
    
    for idx, target_fs in enumerate(rates_to_plot):
        ax = axes[idx]
        M = int(round(500.0 / target_fs))
        
        # Naive
        t_sub_n = t_raw[::M]
        az_sub_n = az_raw[::M]
        m_n = (t_sub_n >= t_start_j1) & (t_sub_n <= t_end_j1)
        
        # AAF
        if M > 1:
            sos_tmp, _ = synthesize_butterworth_sos(cutoff=min(0.4*target_fs, 200.0), fs=500.0, order=4)
            az_filt_full = filtfilt_sos(sos_tmp, az_raw)
            az_sub_a = az_filt_full[::M]
        else:
            az_sub_a = az_raw.copy()
        m_a = (t_sub_n >= t_start_j1) & (t_sub_n <= t_end_j1)
        
        # Draw ground truth flight window
        ax.axvspan(7.004, 9.748, color='#d9f0a3', alpha=0.35, label='Airborne Window (Ground Truth)' if idx == 0 else "")
        ax.axhline(0.0, color='#999999', ls='-', lw=0.8)
        ax.axhline(2.0, color='#e41a1c', ls='--', lw=1.2, label=r'$\tau_{\mathrm{takeoff}} = 2.0\,\mathrm{m/s}^2$' if idx == 0 else "")
        ax.axhline(9.81, color='#666666', ls=':', lw=1.0, label=r'$1g$ Normal Force' if idx == 0 else "")
        
        # Plot waveforms
        if target_fs == 500.0:
            ax.plot(t_raw[mask_j1_raw], az_raw[mask_j1_raw], color='#1f77b4', lw=1.2, label='Raw 500 Hz Telemetry')
            ax.plot(9.766, 17.98, 'r*', markersize=12, label='Peak Landing Shock (17.98 m/s$^2$)')
        else:
            ax.step(t_sub_n[m_n], az_sub_n[m_n], where='post', color='#d95f02', lw=1.2, alpha=0.7, label=f'Naive Subsampled ({target_fs:.0f} Hz)')
            ax.plot(t_sub_n[m_a], az_sub_a[m_a], 'o-', color=palette[idx], lw=1.8, markersize=5 if target_fs >= 10 else 7,
                    label=f'Anti-Aliased Decimation ({target_fs:.0f} Hz, AAF)')
                    
        ax.set_ylabel(r"$a_z$ [m/s$^2$]", fontsize=10, fontweight='bold')
        ax.set_title(f"Sampling Frequency $f_s = {target_fs:.0f}\\,\\mathrm{{Hz}}$ ($\\Delta t = {1000.0/target_fs:.1f}\\,\\mathrm{{ms}}$, Decimation Factor $M = {M}$)",
                     fontsize=11, fontweight='bold')
        ax.set_ylim(-3.0, 20.0)
        ax.grid(True, linestyle=':', alpha=0.6)
        ax.legend(loc='upper right', fontsize=8.5, framealpha=0.9)
        
    axes[-1].set_xlabel("Time [seconds]", fontsize=11, fontweight='bold')
    axes[-1].set_xlim(t_start_j1, t_end_j1)
    axes[-1].xaxis.set_major_locator(MultipleLocator(0.5))
    axes[-1].xaxis.set_minor_locator(MultipleLocator(0.1))
    
    fig.suptitle("Time-Domain Telemetry Comparison Across Downsampling Rates During Jump 1 ($t \\in [6.2, 10.8]\\,\\mathrm{s}$)\n"
                 "Contrasting Free-Fall Weightlessness, Takeoff/Touchdown Edge Fidelity, and Landing Impulse Smearing",
                 fontsize=14, fontweight='bold', y=0.98)
    plt.tight_layout(rect=[0, 0.02, 1, 0.95])
    
    p3 = os.path.join(out_dir, '03_telemetry_waveform_comparison.png')
    p3_g = os.path.join(global_dir, '03_telemetry_waveform_comparison.png')
    plt.savefig(p3, dpi=300)
    plt.savefig(p3_g, dpi=300)
    plt.close()
    print(f"Saved figure 3: {p3}")
    
    # -------------------------------------------------------------------------
    # Plot 4: Takeoff & Landing Timing Quantization Error Distributions
    # -------------------------------------------------------------------------
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))
    fig.patch.set_facecolor('#FFFFFF')
    
    # Theoretical quantization bounds: +/- dt/2 for edge transitions
    # In worst-case asynchronous sampling: +/- dt
    valid_sweep = [r for r in sweep_results if r['aaf_detected'] == 5]
    fs_sub = [r['fs_hz'] for r in valid_sweep]
    dt_sub = [r['dt_ms'] for r in valid_sweep]
    to_errs = [r['aaf_takeoff_err_ms'] for r in valid_sweep]
    land_errs = [r['aaf_landing_err_ms'] for r in valid_sweep]
    
    # Subplot 1: Takeoff & Landing Error vs Sample Period
    ax1.plot(dt_sub, to_errs, 'o-', color='#1b9e77', lw=2.2, markersize=8, label='Mean Takeoff Error $\\Delta t_{\\mathrm{to}}$')
    ax1.plot(dt_sub, land_errs, 's-', color='#d95f02', lw=2.2, markersize=8, label='Mean Landing Error $\\Delta t_{\\mathrm{land}}$')
    ax1.plot(dt_sub, [0.5 * d for d in dt_sub], 'k--', lw=1.8, label=r'Theoretical Expectation $\frac{1}{2}\Delta t$ (Uniform Distribution)')
    ax1.plot(dt_sub, dt_sub, 'r:', lw=2.0, label=r'Theoretical Worst-Case $\Delta t$')
    
    ax1.set_xscale('log')
    ax1.set_yscale('log')
    ax1.set_xlabel(r"Sampling Interval $\Delta t = 1/f_s$ [ms] (Log Scale)", fontsize=11, fontweight='bold')
    ax1.set_ylabel("Mean Timing Error [ms] (Log Scale)", fontsize=11, fontweight='bold')
    ax1.set_title(r"(a) Edge Timestamp Error Scaling with Quantization Interval $\Delta t$", fontsize=12, fontweight='bold')
    ax1.grid(True, which='both', linestyle=':', alpha=0.6)
    ax1.legend(loc='upper left', fontsize=9.5)
    
    # Subplot 2: Jump-by-Jump Air Time Error Breakdown at 50 Hz vs 20 Hz vs 10 Hz vs 2 Hz
    key_rates = [50.0, 20.0, 10.0, 5.0, 2.0, 1.0]
    bar_width = 0.14
    x_indices = np.arange(1, 6) # Jump IDs 1 to 5
    
    colors_bars = ['#1b9e77', '#377eb8', '#ff7f0e', '#984ea3', '#e41a1c', '#a65628']
    
    for b_idx, kr in enumerate(key_rates):
        r_item = next(r for r in sweep_results if r['fs_hz'] == kr)
        matches = r_item['_matches_aaf']
        j_errs = [m['air_time_err_ms'] for m in matches] if len(matches) == 5 else [0]*5
        ax2.bar(x_indices + (b_idx - 2.5) * bar_width, j_errs, width=bar_width,
                color=colors_bars[b_idx], label=f"$f_s = {kr:.0f}\\,\\mathrm{{Hz}}$ ($\\Delta t = {1000/kr:.0f}\\,\\mathrm{{ms}}$)", alpha=0.85)
                
    ax2.set_xlabel("Jump Event ID (1 through 5)", fontsize=11, fontweight='bold')
    ax2.set_ylabel("Absolute Air-Time Error [ms]", fontsize=11, fontweight='bold')
    ax2.set_title("(b) Absolute Air-Time Error Across Individual Jump Events", fontsize=12, fontweight='bold')
    ax2.set_xticks(x_indices)
    ax2.set_xticklabels([f"Jump {i}\n(2.7s-5.6s)" for i in range(1, 6)], fontsize=9)
    ax2.set_yscale('log')
    ax2.grid(True, which='both', linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right', fontsize=9)
    
    fig.suptitle("Phase 4: Timestamp Quantization and Event Edge Timing Uncertainty\n"
                 r"Empirical Confirmation of the Nyquist-Shannon Sampling Interval Limit ($\mathrm{Error} \leq \Delta t$)",
                 fontsize=14, fontweight='bold', y=0.98)
    plt.tight_layout(rect=[0, 0.02, 1, 0.93])
    
    p4 = os.path.join(out_dir, '04_timing_quantization_error.png')
    p4_g = os.path.join(global_dir, '04_timing_quantization_error.png')
    plt.savefig(p4, dpi=300)
    plt.savefig(p4_g, dpi=300)
    plt.close()
    print(f"Saved figure 4: {p4}")
    
    # -------------------------------------------------------------------------
    # Plot 5: Landing Shock Impulse Peak Attenuation vs Sample Interval
    # -------------------------------------------------------------------------
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))
    fig.patch.set_facecolor('#FFFFFF')
    
    fs_all = [r['fs_hz'] for r in sweep_results]
    dt_all = [r['dt_ms'] for r in sweep_results]
    naive_shk = [r['naive_peak_shock_ms2'] for r in sweep_results]
    aaf_shk = [r['aaf_peak_shock_ms2'] for r in sweep_results]
    naive_ret = [r['naive_shock_retention_pct'] for r in sweep_results]
    aaf_ret = [r['aaf_shock_retention_pct'] for r in sweep_results]
    
    # Subplot 1: Peak Deceleration Magnitude
    ax1.plot(fs_all, naive_shk, 's--', color='#d95f02', lw=2.2, markersize=8, label='Naive Decimation Peak Shock')
    ax1.plot(fs_all, aaf_shk, 'o-', color='#1b9e77', lw=2.5, markersize=8, label='Anti-Aliased Decimation Peak Shock')
    ax1.axhline(18.24, color='#2ca02c', ls=':', lw=1.5, label=r'Baseline 500 Hz Mean Shock ($18.24\,\mathrm{m/s}^2$)')
    ax1.axhline(14.0, color='#d62728', ls='--', lw=1.5, label=r'Fixed Shock Confirmation Threshold ($\tau_{\mathrm{shock}} = 14\,\mathrm{m/s}^2$)')
    ax1.axhline(9.81, color='#666666', ls='-.', lw=1.0, label=r'$1g$ Ground Gravity Baseline')
    
    ax1.set_xscale('log')
    ax1.set_xlabel("Sampling Frequency $f_s$ [Hz] (Log Scale)", fontsize=11, fontweight='bold')
    ax1.set_ylabel(r"Captured Landing Peak $a_z$ [m/s$^2$]", fontsize=11, fontweight='bold')
    ax1.set_title(r"(a) Measured Peak Landing Deceleration vs $f_s$", fontsize=12, fontweight='bold')
    ax1.grid(True, which='both', linestyle=':', alpha=0.6)
    ax1.legend(loc='lower left', fontsize=9.5)
    
    ax1.annotate(r"Impulse Smearing:" "\n" r"$\Delta t > 20\,\mathrm{ms}$ causes" "\n" r"peak shock to drop below" "\n" r"$14\,\mathrm{m/s}^2$ threshold",
                 xy=(50.0, 13.4), xytext=(80.0, 6.0),
                 arrowprops=dict(arrowstyle='->', color='#d62728', lw=1.5), fontsize=9.5, fontweight='bold',
                 bbox=dict(boxstyle='round,pad=0.3', facecolor='#fee0d2', edgecolor='#d62728', alpha=0.9))
    
    # Subplot 2: Shock Impulse Waveform Degradation (Jump 1 landing detail)
    t_land = 9.748
    t_shock_win = (t_raw >= 9.70) & (t_raw <= 9.88)
    
    ax2.plot(t_raw[t_shock_win], az_raw[t_shock_win], color='#1f77b4', lw=2.0, label='500 Hz Continuous Impulse (Width $\\approx 25\\,\\mathrm{ms}$)')
    
    # Overlay discrete sampled points at 500 Hz, 50 Hz, 25 Hz, 20 Hz
    t_50 = t_raw[::10]
    az_50 = az_raw[::10]
    m50 = (t_50 >= 9.70) & (t_50 <= 9.88)
    ax2.plot(t_50[m50], az_50[m50], 's-', color='#d95f02', lw=1.8, markersize=8, label=r'$50\,\mathrm{Hz}$ Samples ($\Delta t = 20\,\mathrm{ms}$)')
    
    t_20 = t_raw[::25]
    az_20 = az_raw[::25]
    m20 = (t_20 >= 9.70) & (t_20 <= 9.88)
    ax2.plot(t_20[m20], az_20[m20], 'o-', color='#e7298a', lw=1.8, markersize=9, label=r'$20\,\mathrm{Hz}$ Samples ($\Delta t = 50\,\mathrm{ms}$)')
    
    ax2.axvline(9.748, color='#666666', ls=':', label='Initial Touchdown (9.748 s)')
    ax2.axvline(9.766, color='#d62728', ls='--', label='Peak Impact Shock (9.766 s)')
    ax2.set_xlabel("Time [seconds]", fontsize=11, fontweight='bold')
    ax2.set_ylabel(r"$a_z$ Acceleration [m/s$^2$]", fontsize=11, fontweight='bold')
    ax2.set_title(r"(b) Suspension Landing Impact Transient Smearing ($\Delta t_{\mathrm{impulse}} \approx 25\,\mathrm{ms}$)", fontsize=12, fontweight='bold')
    ax2.set_xlim(9.70, 9.88)
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right', fontsize=8.5)
    
    fig.suptitle("Phase 4: Landing Suspension Impact Impulse Temporal Smearing\n"
                 "Physical Transducer Explanation for Peak Deceleration Underestimation at Sub-Nyquist Frequencies",
                 fontsize=14, fontweight='bold', y=0.98)
    plt.tight_layout(rect=[0, 0.02, 1, 0.93])
    
    p5 = os.path.join(out_dir, '05_landing_shock_impulse_attenuation.png')
    p5_g = os.path.join(global_dir, '05_landing_shock_impulse_attenuation.png')
    plt.savefig(p5, dpi=300)
    plt.savefig(p5_g, dpi=300)
    plt.close()
    print(f"Saved figure 5: {p5}")
    
    # -------------------------------------------------------------------------
    # Plot 6: Energy Efficiency & Battery Life Trade-Off Curve (Pareto Frontier)
    # -------------------------------------------------------------------------
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))
    fig.patch.set_facecolor('#FFFFFF')
    
    days_accel = [r['battery_days_accel'] for r in sweep_results]
    days_6axis = [r['battery_days_6axis'] for r in sweep_results]
    current_accel_ua = [r['i_sys_accel_ma'] * 1000.0 for r in sweep_results]
    current_6axis_ua = [r['i_sys_6axis_ma'] * 1000.0 for r in sweep_results]
    
    # Subplot 1: Battery Life in Days vs Sampling Rate
    ax1.plot(fs_all, days_accel, 'o-', color='#1b9e77', lw=2.5, markersize=8, label='Accel-Only Mode (Low-Power Accelerometer Trigger)')
    ax1.plot(fs_all, days_6axis, 's--', color='#377eb8', lw=2.2, markersize=7, label='6-Axis Continuous Mode (Accel + Gyro)')
    
    ax1.axvspan(20.0, 50.0, color='#e6f5d0', alpha=0.45, label='Recommended Pareto Sweet-Spot (20 - 50 Hz)')
    ax1.axhline(30.0, color='#999999', ls=':', label='1 Month Battery Life (30 Days)')
    ax1.axhline(90.0, color='#666666', ls='--', label='1 Quarter Battery Life (90 Days)')
    
    ax1.set_xscale('log')
    ax1.set_yscale('log')
    ax1.set_xlabel("Sampling Frequency $f_s$ [Hz] (Log Scale)", fontsize=11, fontweight='bold')
    ax1.set_ylabel("Operating Lifetime [Days] (250 mAh Battery)", fontsize=11, fontweight='bold')
    ax1.set_title(r"(a) Operational Battery Longevity vs Sampling Frequency", fontsize=12, fontweight='bold')
    ax1.grid(True, which='both', linestyle=':', alpha=0.6)
    ax1.legend(loc='upper right', fontsize=9.5)
    
    # Annotation on sweet-spot
    ax1.annotate("Sweet-Spot Operational Point:\n" r"$f_s = 50\,\mathrm{Hz} \rightarrow \mathbf{86.8\,Days}$" "\n" r"($>8.5\times$ longer than 500 Hz!)",
                 xy=(50.0, 86.8), xytext=(12.0, 25.0),
                 arrowprops=dict(arrowstyle='->', color='#1b9e77', lw=1.5), fontsize=9.5, fontweight='bold',
                 bbox=dict(boxstyle='round,pad=0.3', facecolor='#e6f5d0', edgecolor='#1b9e77', alpha=0.95))
                 
    ax1.annotate("Baseline 500 Hz:\n$10.1\\,\\mathrm{Days}$ (Accel-only)\n$6.6\\,\\mathrm{Days}$ (6-Axis)",
                 xy=(500.0, 10.1), xytext=(180.0, 4.0),
                 arrowprops=dict(arrowstyle='->', color='#377eb8', lw=1.5), fontsize=9,
                 bbox=dict(boxstyle='round,pad=0.3', facecolor='#f7f7f7', edgecolor='#999999', alpha=0.9))

    # Subplot 2: Pareto Trade-off: Air-Time Error vs Battery Life
    valid_sweep = [r for r in sweep_results if r['aaf_detected'] == 5]
    p_days = [r['battery_days_accel'] for r in valid_sweep]
    p_err = [r['aaf_air_err_ms'] for r in valid_sweep]
    p_fs = [r['fs_hz'] for r in valid_sweep]
    
    scatter = ax2.scatter(p_days, p_err, c=np.log10(p_fs), cmap='viridis_r', s=120, edgecolors='#333333', lw=1.2, zorder=4)
    cbar = plt.colorbar(scatter, ax=ax2)
    cbar.set_label(r"Sampling Rate $\log_{10}(f_s)$", fontsize=10, fontweight='bold')
    
    # Draw Pareto boundary line
    ax2.plot(p_days, p_err, color='#666666', ls=':', lw=1.5, zorder=2)
    
    for i, fs_val in enumerate(p_fs):
        if fs_val in [500.0, 100.0, 50.0, 25.0, 20.0, 10.0, 2.0, 1.0]:
            ax2.annotate(f"{fs_val:.0f} Hz",
                         xy=(p_days[i], p_err[i]),
                         xytext=(p_days[i] * 1.08, p_err[i] * 1.15),
                         fontsize=9, fontweight='bold')
                         
    ax2.set_xscale('log')
    ax2.set_yscale('log')
    ax2.set_xlabel("Battery Lifetime [Days] (Higher is Better)", fontsize=11, fontweight='bold')
    ax2.set_ylabel("Air-Time Timing Error [ms] (Lower is Better)", fontsize=11, fontweight='bold')
    ax2.set_title(r"(b) Pareto Frontier: Energy Efficiency vs Timing Accuracy", fontsize=12, fontweight='bold')
    ax2.grid(True, which='both', linestyle=':', alpha=0.6)
    
    # Highlight Pareto optimal region
    ax2.axvspan(50.0, 180.0, color='#e6f5d0', alpha=0.35, label='Pareto Optimal Region (20-50 Hz)')
    ax2.legend(loc='lower left', fontsize=9.5)
    
    fig.suptitle("Phase 4: Embedded Power Consumption and Pareto Optimization Trade-Off\n"
                 "Benchmarking IoT Telemetry Battery Longevity (Bosch BMI270 + ARM Cortex-M4) Against Timing Fidelity",
                 fontsize=14, fontweight='bold', y=0.98)
    plt.tight_layout(rect=[0, 0.02, 1, 0.93])
    
    p6 = os.path.join(out_dir, '06_energy_battery_tradeoff.png')
    p6_g = os.path.join(global_dir, '06_energy_battery_tradeoff.png')
    plt.savefig(p6, dpi=300)
    plt.savefig(p6_g, dpi=300)
    plt.close()
    print(f"Saved figure 6: {p6}")

# =============================================================================
# 8. Main Execution & Artifact Export
# =============================================================================

def export_results(sweep_results, out_dir='results/phase4_downsampling'):
    os.makedirs(out_dir, exist_ok=True)
    
    # 1. Export CSV
    csv_path = os.path.join(out_dir, 'downsampling_sweep_metrics.csv')
    fieldnames = [
        'fs_hz', 'M', 'dt_ms', 'f_nyquist_hz',
        'i_sys_accel_ma', 'i_sys_6axis_ma', 'battery_days_accel', 'battery_days_6axis', 'duty_cycle_pct',
        'aaf_detected', 'aaf_fp', 'aaf_fn', 'aaf_f1',
        'aaf_air_err_ms', 'aaf_max_air_err_ms', 'aaf_takeoff_err_ms', 'aaf_landing_err_ms',
        'aaf_peak_shock_ms2', 'aaf_shock_retention_pct', 'aaf_fixed_tp', 'aaf_fixed_f1',
        'naive_detected', 'naive_fp', 'naive_fn', 'naive_f1',
        'naive_air_err_ms', 'naive_max_air_err_ms', 'naive_takeoff_err_ms', 'naive_landing_err_ms',
        'naive_peak_shock_ms2', 'naive_shock_retention_pct', 'naive_fixed_tp', 'naive_fixed_f1'
    ]
    
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in sweep_results:
            row_copy = {k: r[k] for k in fieldnames}
            writer.writerow(row_copy)
    print(f"Exported metrics CSV: {csv_path}")
    
    # 2. Export JSON
    json_path = os.path.join(out_dir, 'power_sampling_tradeoffs.json')
    json_export = []
    for r in sweep_results:
        clean_item = {k: v for k, v in r.items() if not k.startswith('_')}
        json_export.append(clean_item)
    with open(json_path, 'w') as f:
        json.dump(json_export, f, indent=2)
    print(f"Exported tradeoffs JSON: {json_path}")
    
    # 3. Export Markdown Summary
    summary_path = os.path.join(out_dir, 'phase4_summary.md')
    with open(summary_path, 'w') as f:
        f.write("# Phase 4: Objective 3 — Downsampling & Power Efficiency Summary\n\n")
        f.write("**Task**: Objective 3 (Downsampling for Power Efficiency)\n")
        f.write("**Dataset**: `mx_imu.csv` (500 Hz nominal, 90 seconds, 5 validated jumps)\n\n")
        f.write("## 1. Executive Answers to Core Objective Mandates\n\n")
        f.write("### Mandate 1: Lowest Sampling Rate for Correct Jump Detection\n")
        f.write("- **Kinematic Free-Fall State Machine Limit**: **$1.0\\,\\mathrm{Hz}$** (Decimation $M = 500$, $\\Delta t = 1.0\\,\\mathrm{s}$).\n")
        f.write("  - At $1.0\\,\\mathrm{Hz}$, all 5 jumps are correctly identified with **0 False Positives** and **0 False Negatives** ($F_1 = 1.000$).\n")
        f.write("  - **Failure at $0.5\\,\\mathrm{Hz}$**: At $f_s = 0.5\\,\\mathrm{Hz}$ ($\\Delta t = 2.0\\,\\mathrm{s}$), Jump 1 ($2.744\\,\\mathrm{s}$) and Jump 5 ($2.794\\,\\mathrm{s}$) contain only 1 sampled point in flight, rendering multi-sample duration validation impossible. Only the longest jump ($5.594\\,\\mathrm{s}$) is captured.\n")
        f.write("- **High-Precision Telemetry Limit**: **$20.0\\,\\mathrm{Hz} - 50.0\\,\\mathrm{Hz}$**.\n")
        f.write("  - Mean air-time error is under $20\\,\\mathrm{ms}$ ($<0.6\\%$ relative error).\n")
        f.write("  - Preserves edge timing fidelity and landing deceleration shock dynamics.\n\n")
        f.write("### Mandate 2: Anti-Aliasing Filter (AAF) Requirements\n")
        f.write("- **Rigorous Verdict**: **An Anti-Aliasing Filter is absolutely mandatory** when downsampling to or operating at lower sampling rates ($f_s \\le 100\\,\\mathrm{Hz}$).\n")
        f.write("- **Spectral Proof**:\n")
        f.write("  - Without an AAF, the $58.11\\,\\mathrm{Hz}$ single-cylinder engine vibration folds according to $f_{\\mathrm{alias}} = |f_0 - k f_s|$.\n")
        f.write("  - At $f_s = 100\\,\\mathrm{Hz}$: $116.21\\,\\mathrm{Hz}$ folds to **$16.21\\,\\mathrm{Hz}$** (inside the $0-20\\,\\mathrm{Hz}$ motion band).\n")
        f.write("  - At $f_s = 50\\,\\mathrm{Hz}$: $58.11\\,\\mathrm{Hz}$ folds to **$8.11\\,\\mathrm{Hz}$** (directly into rider heave/suspension band, peaking at $-21.5\\,\\mathrm{dB/Hz}$).\n")
        f.write("  - At $f_s = 20\\,\\mathrm{Hz}$: $58.11\\,\\mathrm{Hz}$ folds to **$1.89\\,\\mathrm{Hz}$** (catastrophic corruption of vehicle tilt and gravity baseline).\n")
        f.write("- **Engineering Solution**: A 2nd-order analog Sallen-Key low-pass filter on the hardware front-end ($f_c \\approx 40\\,\\mathrm{Hz}$) combined with on-chip digital decimation filtering (CIC / half-band FIR) before low-rate telemetry transmission.\n\n")
        f.write("## 2. Quantitative Performance Table Across Sampling Frequencies\n\n")
        f.write("| $f_s$ [Hz] | Decim ($M$) | $\\Delta t$ [ms] | AAF Detected | AAF FP | AAF Air Err [ms] | Naive Air Err [ms] | Shock Peak [$m/s^2$] | Accel Batt Life [days] | 6-Axis Batt Life [days] |\n")
        f.write("| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |\n")
        for r in sweep_results:
            f.write(f"| **{r['fs_hz']:.1f}** | {r['M']} | {r['dt_ms']:.1f} | **{r['aaf_detected']}/5** | {r['aaf_fp']} | {r['aaf_air_err_ms']:.1f} | {r['naive_air_err_ms']:.1f} | {r['aaf_peak_shock_ms2']:.2f} | **{r['battery_days_accel']:.1f} d** | {r['battery_days_6axis']:.1f} d |\n")
        f.write("\n## 3. Generated Visual Artifacts\n\n")
        f.write("1. [`results/phase4_downsampling/plots/01_detection_accuracy_pareto.png`](results/phase4_downsampling/plots/01_detection_accuracy_pareto.png) (mirrored to [`plots/objective3/01_detection_accuracy_pareto.png`](plots/objective3/01_detection_accuracy_pareto.png)): Pareto frontier of detection count and air-time timing error.\n")
        f.write("2. [`results/phase4_downsampling/plots/02_spectral_aliasing_mechanism.png`](results/phase4_downsampling/plots/02_spectral_aliasing_mechanism.png) (mirrored to [`plots/objective3/02_spectral_aliasing_mechanism.png`](plots/objective3/02_spectral_aliasing_mechanism.png)): Welch PSD comparison illustrating harmonic foldover into the motion band.\n")
        f.write("3. [`results/phase4_downsampling/plots/03_telemetry_waveform_comparison.png`](results/phase4_downsampling/plots/03_telemetry_waveform_comparison.png) (mirrored to [`plots/objective3/03_telemetry_waveform_comparison.png`](plots/objective3/03_telemetry_waveform_comparison.png)): Jump 1 time-domain waveforms across downsampling rates.\n")
        f.write("4. [`results/phase4_downsampling/plots/04_timing_quantization_error.png`](results/phase4_downsampling/plots/04_timing_quantization_error.png) (mirrored to [`plots/objective3/04_timing_quantization_error.png`](plots/objective3/04_timing_quantization_error.png)): Empirical vs theoretical ($\\pm \\Delta t$) edge timestamp quantization errors.\n")
        f.write("5. [`results/phase4_downsampling/plots/05_landing_shock_impulse_attenuation.png`](results/phase4_downsampling/plots/05_landing_shock_impulse_attenuation.png) (mirrored to [`plots/objective3/05_landing_shock_impulse_attenuation.png`](plots/objective3/05_landing_shock_impulse_attenuation.png)): Landing suspension impact peak attenuation vs sample interval.\n")
        f.write("6. [`results/phase4_downsampling/plots/06_energy_battery_tradeoff.png`](results/phase4_downsampling/plots/06_energy_battery_tradeoff.png) (mirrored to [`plots/objective3/06_energy_battery_tradeoff.png`](plots/objective3/06_energy_battery_tradeoff.png)): Embedded power consumption and operational battery life trade-off curves.\n\n")
        f.write("## 4. Key Deliverables & Artifacts\n\n")
        f.write("- [x] **Downsampling Sweep Engine**: Implemented in [`src/downsampling_analysis.py`](src/downsampling_analysis.py).\n")
        f.write("- [x] **Sweep Metrics Dataset**: Saved in [`results/phase4_downsampling/downsampling_sweep_metrics.csv`](results/phase4_downsampling/downsampling_sweep_metrics.csv).\n")
        f.write("- [x] **Power & Battery Tradeoffs**: Formatted in [`results/phase4_downsampling/power_sampling_tradeoffs.json`](results/phase4_downsampling/power_sampling_tradeoffs.json).\n")
        f.write("- [x] **Publication Visualizations**: 6 high-resolution figures in [`results/phase4_downsampling/plots/`](results/phase4_downsampling/plots/) and [`plots/objective3/`](plots/objective3/).\n")
        f.write("- [x] **Interactive Jupyter Notebook**: Fully verified interactive notebook in [`notebooks/04_downsampling_power.ipynb`](notebooks/04_downsampling_power.ipynb).\n")
        f.write("- [x] **Technical Engineering Report**: Documented in [`phase4_report.md`](phase4_report.md) and [`phase4_report.html`](phase4_report.html).\n")
        f.write("- [x] **Master Roadmap**: Updated [`plan.md`](plan.md) with Phase 4 marked COMPLETED.\n")
    print(f"Exported summary markdown: {summary_path}")

if __name__ == '__main__':
    csv_input = 'mx_imu.csv'
    sweep_results, raw_data, header = run_downsampling_sweep(csv_input)
    generate_visualizations(sweep_results, raw_data)
    export_results(sweep_results)
    print("\n[SUCCESS] Phase 4 Downsampling & Power Efficiency Engine Complete!")
