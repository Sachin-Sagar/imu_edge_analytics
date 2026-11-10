#!/usr/bin/env python3
"""
Regression Curve Fitting & Error Assessment for IMU Fault Mitigation
---------------------------------------------------------------------
Author: Antigravity Motion Algorithm Engineer
Dataset: mx_imu.csv (Jump 4 inflight segment)

This module:
1. Fits a quadratic/cubic kinematic regression parabola through all valid
   in-flight samples of Jump 4 (excluding the 0.58s fault window).
2. Computes the baseline trajectory through the outage with R^2 = 0.9955.
3. Quantitatively evaluates RMSE, MAE, Max Error, and Integrated Angle Error
   for all 6 interpolation and substitution methods.
4. Generates a 4-panel publication-grade figure saved to:
   results/phase2_jump_detection/plots/11_regression_curve_error_assessment.png
"""

import os
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

def run_regression_assessment():
    # Load Jump 4 flight data
    data = np.genfromtxt('results/phase2_jump_detection/snipped_jumps/jump_4_inflight.csv', delimiter=',', skip_header=1)
    t = data[:, 0]
    gy = data[:, 5]
    dt = 0.002
    
    t0 = 56.198
    t1 = 56.780
    delta_t = t1 - t0
    
    # Exclude the 8.0 rad/s glitch window
    valid_mask = (t < t0) | (t > t1)
    t_val = t[valid_mask]
    gy_val = gy[valid_mask]
    
    # 1. Fit Quadratic Regression Curve through valid flight data
    coeffs = np.polyfit(t_val, gy_val, 2)
    p_reg = np.poly1d(coeffs)
    r2_score = 1.0 - (np.sum((gy_val - p_reg(t_val))**2) / np.sum((gy_val - np.mean(gy_val))**2))
    
    # Fault window samples
    fault_mask = (t >= t0) & (t <= t1)
    t_f = t[fault_mask]
    N_f = len(t_f)
    w_reg = p_reg(t_f) # Ground-truth kinematic baseline
    
    # Boundary values and slopes
    w0 = 0.5553
    w1 = 0.7004
    slope_0 = np.polyfit(t[(t >= 56.10) & (t <= 56.198)] - t0, gy[(t >= 56.10) & (t <= 56.198)], 1)[0]
    slope_1 = np.polyfit(t[(t >= 56.78) & (t <= 56.88)] - t1, gy[(t >= 56.78) & (t <= 56.88)], 1)[0]
    u = (t_f - t0) / delta_t
    
    # Candidate methods
    preds = {}
    preds['Raw Saturated (8.0)'] = np.full(N_f, 8.0)
    preds['ZOH (LOCF)'] = np.full(N_f, w0)
    preds['Causal FOH Extrap'] = w0 + slope_0 * (t_f - t0)
    preds['Non-Causal FOH Interp'] = w0 + (w1 - w0) * u
    
    # Cubic Hermite Spline (PCHIP)
    h00 = 2*u**3 - 3*u**2 + 1
    h10 = u**3 - 2*u**2 + u
    h01 = -2*u**3 + 3*u**2
    h11 = u**3 - u**2
    preds['Cubic Hermite Spline'] = h00*w0 + h10*delta_t*slope_0 + h01*w1 + h11*delta_t*slope_1
    preds['Aero Damping Decay'] = w0 * np.exp(-(t_f - t0) / 2.0)
    
    # Compute error statistics
    theta_true = np.sum(w_reg) * dt * (180.0 / np.pi)
    stats = []
    
    colors = {
        'Raw Saturated (8.0)': '#b71c1c',
        'ZOH (LOCF)': '#ff9800',
        'Causal FOH Extrap': '#ab47bc',
        'Non-Causal FOH Interp': '#1e88e5',
        'Cubic Hermite Spline': '#2e7d32',
        'Aero Damping Decay': '#795548'
    }
    
    for name, pred in preds.items():
        err = pred - w_reg
        rmse = float(np.sqrt(np.mean(err**2)))
        mae = float(np.mean(np.abs(err)))
        max_err = float(np.max(np.abs(err)))
        theta_pred = float(np.sum(pred) * dt * (180.0 / np.pi))
        theta_err = float(theta_pred - theta_true)
        rel_err = float((theta_err / theta_true) * 100.0)
        stats.append({
            'name': name,
            'rmse': rmse,
            'mae': mae,
            'max_err': max_err,
            'theta_err': theta_err,
            'rel_err': rel_err
        })

    # -----------------------------------------------------
    # Visualization: 4-Panel Diagnostic Assessment Figure
    # -----------------------------------------------------
    fig = plt.figure(figsize=(16, 13))
    fig.patch.set_facecolor('#ffffff')
    gs = fig.add_gridspec(2, 2, hspace=0.30, wspace=0.22)
    
    # Subplot (a): Full Jump 4 Flight Trajectory with Fitted Regression
    ax_a = fig.add_subplot(gs[0, 0])
    ax_a.set_facecolor('#fafafa')
    
    ax_a.scatter(t_val, gy_val, color='#90a4ae', s=6, alpha=0.6, label='Valid In-Flight IMU Samples')
    t_full = np.linspace(t[0], t[-1], 500)
    ax_a.plot(t_full, p_reg(t_full), color='#1565c0', lw=2.4,
              label=f'Kinematic Regression Parabola ($R^2 = {r2_score:.4f}$)')
    
    ax_a.axvspan(t0, t1, color='#ffebee', alpha=0.5, hatch='//', label='Fault Window (0.582s)')
    ax_a.text(0.5*(t0+t1), 0.35, '0.58s Outage\n(8.0 rad/s omitted)', ha='center', va='center',
              fontsize=8.5, fontweight='bold', color='#c62828',
              bbox=dict(boxstyle='round,pad=0.2', facecolor='#ffebee', edgecolor='#c62828'))

    ax_a.set_xlabel(r'Session Time $[\mathrm{seconds}]$', fontsize=11, fontweight='bold')
    ax_a.set_ylabel(r'Pitch Rate $\omega_y$ $[\mathrm{rad/s}]$', fontsize=11, fontweight='bold')
    ax_a.set_title(r'(a) Full Jump 4 Flight Trajectory: Parabolic Aerodynamic Fit ($R^2 = 0.9955$)',
                   fontsize=12, fontweight='bold', pad=8)
    ax_a.grid(True, linestyle=':', alpha=0.6)
    ax_a.legend(loc='lower center', framealpha=0.95, fontsize=8.5)
    ax_a.set_ylim([0.2, 0.85])

    # Subplot (b): Zoomed Comparison in Fault Window
    ax_b = fig.add_subplot(gs[0, 1])
    ax_b.set_facecolor('#fafafa')
    
    # Plot regression baseline
    ax_b.plot(t_f, w_reg, color='#000000', lw=3.0, linestyle='-', label='True Regression Baseline (Ground Truth)')
    
    for name in ['Cubic Hermite Spline', 'Non-Causal FOH Interp', 'Causal FOH Extrap', 'ZOH (LOCF)', 'Aero Damping Decay']:
        ls = '-' if 'Cubic' in name else ('--' if 'Interp' in name else ('-.' if 'Extrap' in name else ':'))
        ax_b.plot(t_f, preds[name], color=colors[name], lw=2.2, linestyle=ls, label=name)
        
    ax_b.set_xlabel(r'Time $[\mathrm{seconds}]$', fontsize=11, fontweight='bold')
    ax_b.set_ylabel(r'Pitch Rate $\omega_y$ $[\mathrm{rad/s}]$', fontsize=11, fontweight='bold')
    ax_b.set_title(r'(b) Microscopic Zoom in Fault Window: Reconstructed Trajectories vs Baseline',
                   fontsize=12, fontweight='bold', pad=8)
    ax_b.grid(True, linestyle=':', alpha=0.6)
    ax_b.legend(loc='lower right', framealpha=0.95, fontsize=8.2)
    ax_b.set_xlim([t0, t1])
    ax_b.set_ylim([0.42, 0.76])

    # Subplot (c): Instantaneous Error Residuals e(t) = pred(t) - reg(t)
    ax_c = fig.add_subplot(gs[1, 0])
    ax_c.set_facecolor('#fafafa')
    
    for name in ['Cubic Hermite Spline', 'Non-Causal FOH Interp', 'Causal FOH Extrap', 'ZOH (LOCF)', 'Aero Damping Decay']:
        ls = '-' if 'Cubic' in name else ('--' if 'Interp' in name else ('-.' if 'Extrap' in name else ':'))
        err = preds[name] - w_reg
        ax_c.plot(t_f, err, color=colors[name], lw=2.2, linestyle=ls, label=f"{name}")
        
    ax_c.axhline(0.0, color='#000000', lw=1.2, linestyle='-')
    ax_c.set_xlabel(r'Time $[\mathrm{seconds}]$', fontsize=11, fontweight='bold')
    ax_c.set_ylabel(r'Error Residual $e(t)$ $[\mathrm{rad/s}]$', fontsize=11, fontweight='bold')
    ax_c.set_title(r'(c) Instantaneous Error Residuals: $\hat{\omega}_m(t) - \omega_{\mathrm{reg}}(t)$ Across Fault Window',
                   fontsize=12, fontweight='bold', pad=8)
    ax_c.grid(True, linestyle=':', alpha=0.6)
    ax_c.legend(loc='lower left', framealpha=0.95, fontsize=8.2)
    ax_c.set_xlim([t0, t1])
    ax_c.set_ylim([-0.28, 0.05])

    # Subplot (d): Error Metrics Bar Chart
    ax_d = fig.add_subplot(gs[1, 1])
    ax_d.set_facecolor('#fafafa')
    
    clean_schemes = [s for s in stats if 'Raw' not in s['name']]
    scheme_names = [s['name'].replace(' (LOCF)', '').replace(' (Linear Extrap)', '').replace(' (Linear Interp)', '') for s in clean_schemes]
    x_pos = np.arange(len(scheme_names))
    width = 0.35
    
    rmse_vals = [s['rmse'] * 1000.0 for s in clean_schemes] # in mrad/s
    angle_err_vals = [abs(s['theta_err']) for s in clean_schemes] # in degrees
    
    rects1 = ax_d.bar(x_pos - width/2, rmse_vals, width, label='RMSE (mrad/s)', color='#42a5f5', edgecolor='#1565c0')
    
    ax_d_twin = ax_d.twinx()
    rects2 = ax_d_twin.bar(x_pos + width/2, angle_err_vals, width, label='Angle Error (|Δθ| deg)', color='#ef5350', edgecolor='#b71c1c')
    
    for rect in rects1:
        h = rect.get_height()
        ax_d.text(rect.get_x() + rect.get_width()/2., h + 3, f'{h:.1f}', ha='center', va='bottom',
                  fontsize=8.0, fontweight='bold', color='#1565c0')

    for rect in rects2:
        h = rect.get_height()
        ax_d_twin.text(rect.get_x() + rect.get_width()/2., h + 0.1, f'{h:.2f}°', ha='center', va='bottom',
                       fontsize=8.0, fontweight='bold', color='#b71c1c')

    ax_d.set_xticks(x_pos)
    ax_d.set_xticklabels(scheme_names, rotation=20, ha='right', fontsize=9, fontweight='bold')
    ax_d.set_ylabel(r'Velocity RMSE $[\mathrm{mrad/s}]$', fontsize=11, fontweight='bold', color='#1565c0')
    ax_d_twin.set_ylabel(r'Integrated Angle Error $|\Delta\theta| \; [\mathrm{degrees}]$', fontsize=11, fontweight='bold', color='#b71c1c')
    ax_d.set_title(r'(d) Quantitative Performance Benchmark: RMSE & Integrated Pitch Angle Error',
                   fontsize=12, fontweight='bold', pad=8)
    ax_d.grid(True, axis='y', linestyle=':', alpha=0.6)
    ax_d.set_ylim([0, 200])
    ax_d_twin.set_ylim([0, 6.5])

    out_dir = 'results/phase2_jump_detection/plots'
    os.makedirs(out_dir, exist_ok=True)
    out_fig = os.path.join(out_dir, '11_regression_curve_error_assessment.png')
    fig.savefig(out_fig, dpi=300)
    plt.close(fig)
    print(f"Saved {out_fig}")
    
    # Mirror to plots/objective1/
    import shutil
    shutil.copy2(out_fig, 'plots/objective1/11_regression_curve_error_assessment.png')
    print("Mirrored to plots/objective1/11_regression_curve_error_assessment.png")
    
    return stats, theta_true

if __name__ == '__main__':
    stats, theta_true = run_regression_assessment()

