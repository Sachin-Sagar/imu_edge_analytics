#!/usr/bin/env python3
"""
Interpolation Schemes Comparative Analysis for IMU Sensor Fault Mitigation
-------------------------------------------------------------------------
Author: Antigravity Motion Algorithm Engineer
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

def generate_interpolation_comparison(out_path):
    # Load Jump 4 flight data
    data = np.genfromtxt('results/phase2_jump_detection/snipped_jumps/jump_4_inflight.csv', delimiter=',', skip_header=1)
    t = data[:, 0]
    gy = data[:, 5]
    dt = 0.002
    
    t0 = 56.198
    t1 = 56.780
    w0 = 0.5553
    w1 = 0.7004
    delta_t = t1 - t0
    
    # Pre-fault slope from t in [56.10, 56.198]
    pre_mask = (t >= 56.10) & (t <= 56.198)
    slope_0 = np.polyfit(t[pre_mask] - t0, gy[pre_mask], 1)[0]
    post_mask = (t >= 56.78) & (t <= 56.88)
    slope_1 = np.polyfit(t[post_mask] - t1, gy[post_mask], 1)[0]
    
    # Time vector for window [55.8, 57.2]
    win_mask = (t >= 55.8) & (t <= 57.2)
    t_win = t[win_mask]
    
    # Fault indices in t_win
    fault_mask_win = (t_win >= t0) & (t_win <= t1)
    t_fault = t_win[fault_mask_win]
    u = (t_fault - t0) / delta_t # normalized [0, 1]
    
    # 1. Raw Saturated
    gy_raw_win = gy[win_mask].copy()
    
    # 2. ZOH (Zero-Order Hold)
    gy_zoh = gy_raw_win.copy()
    gy_zoh[fault_mask_win] = w0
    
    # 3. Causal FOH (Linear Extrapolation)
    gy_foh_extrap = gy_raw_win.copy()
    gy_foh_extrap[fault_mask_win] = w0 + slope_0 * (t_fault - t0)
    
    # 4. Non-Causal FOH (Linear Interpolation)
    gy_foh_interp = gy_raw_win.copy()
    gy_foh_interp[fault_mask_win] = w0 + (w1 - w0) * u
    
    # 5. Cubic Hermite Spline (PCHIP)
    gy_cubic = gy_raw_win.copy()
    h00 = 2*u**3 - 3*u**2 + 1
    h10 = u**3 - 2*u**2 + u
    h01 = -2*u**3 + 3*u**2
    h11 = u**3 - u**2
    gy_cubic[fault_mask_win] = h00*w0 + h10*delta_t*slope_0 + h01*w1 + h11*delta_t*slope_1
    
    # 6. Aero Decay Model (tau = 2.0s)
    tau_aero = 2.0
    gy_decay = gy_raw_win.copy()
    gy_decay[fault_mask_win] = w0 * np.exp(-(t_fault - t0) / tau_aero)
    
    # Plotting
    fig, axes = plt.subplots(3, 1, figsize=(14, 12), sharex=True)
    fig.patch.set_facecolor('#ffffff')
    
    # Subplot 1: Angular Velocity Trajectories (Zoomed to candidate schemes)
    ax1 = axes[0]
    ax1.set_facecolor('#fafafa')
    # Omit raw 8.0 rad/s line as requested to allow full dynamic visibility of candidate schemes
    ax1.plot(t_win[~fault_mask_win], gy_raw_win[~fault_mask_win], color='#333333', lw=1.8, label='Valid Pre/Post Flight Rate')
    ax1.plot(t_fault, gy_zoh[fault_mask_win], color='#ff9800', lw=2.4, linestyle=':', label=r'Zero-Order Hold (ZOH / LOCF): $\hat{\omega} = \omega_0$ ($0.555\,\mathrm{rad/s}$)')
    ax1.plot(t_fault, gy_foh_extrap[fault_mask_win], color='#ab47bc', lw=2.2, linestyle='-.', label=r'Causal FOH (Linear Extrap): $\hat{\omega} = \omega_0 + \dot{\omega}_0 \Delta t$')
    ax1.plot(t_fault, gy_foh_interp[fault_mask_win], color='#1e88e5', lw=2.5, linestyle='--', label=r'Non-Causal FOH (Linear Interp): 2-Point Bridge ($C^0$)')
    ax1.plot(t_fault, gy_cubic[fault_mask_win], color='#2e7d32', lw=2.8, linestyle='-', label=r'Cubic Hermite Spline (PCHIP): Smooth Accel ($C^1$)')
    ax1.plot(t_fault, gy_decay[fault_mask_win], color='#795548', lw=2.0, linestyle='--', alpha=0.85, label=r'Aero Damping Decay: $\omega_0 e^{-t/\tau}$')
    
    ax1.axvspan(t0, t1, color='#e8f5e9', alpha=0.35, hatch='//')
    ax1.text(0.5 * (t0 + t1), 0.74, 'Fault Window (0.582s): Raw 8.0 rad/s Saturated Value Omitted for Clarity',
             ha='center', va='center', fontsize=9.5, fontweight='bold', color='#c62828',
             bbox=dict(boxstyle='round,pad=0.3', facecolor='#ffebee', edgecolor='#c62828', alpha=0.9))

    ax1.set_ylabel(r'Pitch Rate $\omega_y$ $[\mathrm{rad/s}]$', fontsize=11, fontweight='bold')
    ax1.set_title(r'(a) Candidate Angular Velocity Trajectories Zoomed ($0.45 - 0.78\,\mathrm{rad/s}$, 8.0 rad/s Omitted)', fontsize=12, fontweight='bold', pad=8)
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='lower right', framealpha=0.95, fontsize=8.5, ncol=2)
    ax1.set_ylim([0.45, 0.78])
    
    # Subplot 2: Accumulated Angle Delta Theta(t) during fault
    ax2 = axes[1]
    ax2.set_facecolor('#fafafa')
    
    # Compute relative angle integrated from t0
    # Create high-res grid for smooth plotting
    t_plot = np.linspace(t0, t1, 300)
    u_p = (t_plot - t0) / delta_t
    
    theta_zoh_cum = w0 * (t_plot - t0) * (180.0 / np.pi)
    theta_foh_extrap_cum = (w0*(t_plot - t0) + 0.5*slope_0*(t_plot - t0)**2) * (180.0 / np.pi)
    theta_foh_interp_cum = (w0*(t_plot - t0) + 0.5*(w1 - w0)*delta_t*(u_p**2)) * (180.0 / np.pi)
    
    # Cubic integral
    theta_cubic_cum = np.zeros_like(t_plot)
    for i, up in enumerate(u_p):
        int_h00 = up - up**3 + 0.5*up**4
        int_h10 = delta_t * (0.5*up**2 - (2.0/3.0)*up**3 + 0.25*up**4)
        int_h01 = up**3 - 0.5*up**4
        int_h11 = delta_t * (-(1.0/3.0)*up**3 + 0.25*up**4)
        theta_cubic_cum[i] = (w0*int_h00 + slope_0*int_h10 + w1*int_h01 + slope_1*int_h11) * (180.0 / np.pi)
        
    theta_decay_cum = (w0 * tau_aero * (1.0 - np.exp(-(t_plot - t0) / tau_aero))) * (180.0 / np.pi)
    
    # Omit raw 266.8 deg line as requested to allow micro-scale comparison
    ax2.plot(t_plot, theta_zoh_cum, color='#ff9800', lw=2.4, linestyle=':', label=r'ZOH: $\Delta\theta = \mathbf{18.5^\circ}$ (underestimates by $-2.4^\circ$)')
    ax2.plot(t_plot, theta_foh_extrap_cum, color='#ab47bc', lw=2.2, linestyle='-.', label=r'Causal FOH Extrap: $\Delta\theta = \mathbf{20.7^\circ}$ (error: $-0.2^\circ$)')
    ax2.plot(t_plot, theta_foh_interp_cum, color='#1e88e5', lw=2.6, linestyle='--', label=r'Non-Causal FOH Interp: $\Delta\theta = \mathbf{20.9^\circ}$ (Reference)')
    ax2.plot(t_plot, theta_cubic_cum, color='#2e7d32', lw=2.8, linestyle='-', label=r'Cubic Spline PCHIP: $\Delta\theta = \mathbf{21.1^\circ}$ (error: $+0.2^\circ$)')
    ax2.plot(t_plot, theta_decay_cum, color='#795548', lw=2.0, linestyle='--', label=r'Aero Damping Decay: $\Delta\theta = \mathbf{16.1^\circ}$')

    ax2.axvspan(t0, t1, color='#e8f5e9', alpha=0.3, hatch='//')
    ax2.text(0.5 * (t0 + t1), 22.5, 'Raw Saturated Curve (266.8°) Omitted to Resolve Physical Differences',
             ha='center', va='center', fontsize=9.5, fontweight='bold', color='#c62828',
             bbox=dict(boxstyle='round,pad=0.3', facecolor='#ffebee', edgecolor='#c62828', alpha=0.9))

    ax2.set_ylabel(r'Integrated Angle $\Delta\theta(t)$ $[\mathrm{degrees}]$', fontsize=11, fontweight='bold')
    ax2.set_title(r'(b) Cumulative Pitch Angle Zoomed ($0^\circ - 25^\circ$): High-Resolution Scheme Comparison',
                  fontsize=12, fontweight='bold', pad=8)
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='lower right', framealpha=0.95, fontsize=8.5)
    ax2.set_ylim([0, 25.0])

    # Subplot 3: Angular Acceleration Profile (Continuity Check)
    ax3 = axes[2]
    ax3.set_facecolor('#fafafa')
    
    # Plot angular acceleration for ZOH, FOH, Cubic
    # FOH interp slope is constant
    slope_foh = (w1 - w0) / delta_t
    ax3.plot([t0, t1], [slope_foh, slope_foh], color='#1e88e5', lw=2.2, linestyle='--',
             label=f'Non-Causal FOH Acceleration: Constant $\\dot{{\\omega}} = {slope_foh:+.3f}\\,\\mathrm{{rad/s^2}}$ ($C^0$)')
    
    # Cubic Hermite acceleration is linear: d2gy/dt2
    acc_cubic = np.zeros_like(t_plot)
    for i, up in enumerate(u_p):
        dh00 = (6*up**2 - 6*up) / delta_t
        dh10 = (3*up**2 - 4*up + 1)
        dh01 = (-6*up**2 + 6*up) / delta_t
        dh11 = (3*up**2 - 2*up)
        acc_cubic[i] = w0*dh00 + slope_0*dh10 + w1*dh01 + slope_1*dh11
    ax3.plot(t_plot, acc_cubic, color='#2e7d32', lw=2.6, linestyle='-',
             label=f'Cubic Hermite Spline Acceleration: Smooth Linear Transition ($C^1$: {slope_0:+.2f} to {slope_1:+.2f} rad/s²)')

    ax3.scatter([t0], [slope_0], color='#2e7d32', s=50, zorder=5)
    ax3.scatter([t1], [slope_1], color='#2e7d32', s=50, zorder=5)

    # ZOH has delta impulse at t1
    ax3.annotate('ZOH Discontinuity Impulse\nat fault recovery (+0.145 rad/s in 2ms = +72.5 rad/s²)',
                 xy=(t1, 0.0), xytext=(56.55, -0.2),
                 arrowprops=dict(facecolor='#ff9800', arrowstyle='->', lw=1.5),
                 fontsize=8.5, fontweight='bold', color='#e65100',
                 bbox=dict(boxstyle='round,pad=0.2', facecolor='#fff3e0', edgecolor='#ff9800'))

    ax3.axvspan(t0, t1, color='#ffebee', alpha=0.3, hatch='//')
    ax3.set_xlabel(r'Session Time $[\mathrm{seconds}]$', fontsize=11, fontweight='bold')
    ax3.set_ylabel(r'Angular Accel $\dot{\omega}_y$ $[\mathrm{rad/s^2}]$', fontsize=11, fontweight='bold')
    ax3.set_title(r'(c) Continuity Analysis: ZOH ($C^{-1}$ step impulse) vs FOH ($C^0$ kink) vs Cubic Hermite ($C^1$ smooth)',
                  fontsize=12, fontweight='bold', pad=8)
    ax3.grid(True, linestyle=':', alpha=0.6)
    ax3.legend(loc='upper left', framealpha=0.95, fontsize=8.5)
    ax3.set_xlim([55.8, 57.2])
    ax3.set_ylim([-0.3, 0.5])

    plt.tight_layout()
    fig.savefig(out_path, dpi=300)
    plt.close(fig)
    print(f"Saved {out_path}")

if __name__ == '__main__':
    out_dir = 'results/phase2_jump_detection/plots'
    os.makedirs(out_dir, exist_ok=True)
    out_fig = os.path.join(out_dir, '10_interpolation_schemes_comparison.png')
    generate_interpolation_comparison(out_fig)
    
    # Mirror
    import shutil
    shutil.copy2(out_fig, 'plots/objective1/10_interpolation_schemes_comparison.png')
    print("Mirrored to plots/objective1/10_interpolation_schemes_comparison.png")

