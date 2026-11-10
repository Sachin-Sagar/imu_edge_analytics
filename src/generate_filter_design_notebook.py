#!/usr/bin/env python3
"""
Generate interactive Jupyter Notebook for Phase 3: Objective 2 (Filter Design & Spectral Analysis).
Notebook path: notebooks/03_filter_design.ipynb
"""

import json
import os

def create_phase3_notebook():
    os.makedirs('notebooks', exist_ok=True)
    nb_path = 'notebooks/03_filter_design.ipynb'
    
    cells = []
    
    # ----------------------------------------------------
    # Cell 1: Markdown Title & Executive Overview
    # ----------------------------------------------------
    cells.append({
        'cell_type': 'markdown',
        'metadata': {},
        'source': [
            "# Phase 3: Objective 2 — Power Spectral Density & Butterworth Low-Pass Filter Design\n",
            "\n",
            "**Task**: Motocross Jump Analytics — Objective 2  \n",
            "**Dataset**: `../mx_imu.csv` (6-axis IMU @ 500 Hz nominal, 90 seconds)  \n",
            "**Author**: Sachin Sagar (Edge IMU Motion Analytics Engine)  \n",
            "\n",
            "---\n",
            "\n",
            "## Objective Requirements\n",
            "1. **Ground Segment Power Spectral Density (PSD)**:\n",
            "   - Compute and plot the PSD of all 3 accelerometer axes ($a_x, a_y, a_z$) and all 3 gyroscope axes ($\\omega_x, \\omega_y, \\omega_z$) during a uniform ground riding segment.\n",
            "   - Accurately identify and label the single-cylinder 2-stroke engine vibration peaks ($58.11\\,\\text{Hz}$ fundamental and $116.21\\,\\text{Hz}$ second harmonic).\n",
            "   - Provide a rigorous physical transducer mechanism explaining why accelerometer channels are severely contaminated while gyroscope channels remain largely immune.\n",
            "2. **Butterworth Low-Pass Filter Design & Justification**:\n",
            "   - Design a discrete-time Butterworth low-pass filter to eradicate engine vibration harmonics while preserving vehicle motion dynamics below $20\\,\\text{Hz}$.\n",
            "   - Mathematically derive and justify the filter order ($N = 4$) and cutoff frequency ($f_c = 25.0\\,\\text{Hz}$) based on passband retention ($A_p \\le 1.0\\,\\text{dB}$ at $20\\,\\text{Hz}$) and stopband attenuation ($A_s \\ge 30.0\\,\\text{dB}$ at $58\\,\\text{Hz}$).\n",
            "   - Implement the filter using Second-Order Sections (Biquads / SOS) via Bilinear Transformation with frequency pre-warping to guarantee numerical stability.\n",
            "   - Evaluate both causal real-time filtering (`sosfilt`) and zero-phase forward-backward filtering (`filtfilt`).\n",
            "   - Verify filtering performance in both frequency and time domains."
        ]
    })
    
    # ----------------------------------------------------
    # Cell 2: Imports & Environment Configuration
    # ----------------------------------------------------
    cells.append({
        'cell_type': 'code',
        'execution_count': None,
        'metadata': {},
        'outputs': [],
        'source': [
            "import os\n",
            "import numpy as np\n",
            "import matplotlib.pyplot as plt\n",
            "from matplotlib.ticker import MultipleLocator\n",
            "\n",
            "# Configure publication-grade inline matplotlib aesthetics\n",
            "%matplotlib inline\n",
            "plt.rcParams['figure.dpi'] = 120\n",
            "plt.rcParams['font.size'] = 10\n",
            "plt.rcParams['axes.grid'] = True\n",
            "plt.rcParams['grid.linestyle'] = ':'\n",
            "plt.rcParams['grid.alpha'] = 0.6\n",
            "print('Environment initialized successfully.')"
        ]
    })
    
    # ----------------------------------------------------
    # Cell 3: Data Ingestion & Imputation
    # ----------------------------------------------------
    cells.append({
        'cell_type': 'markdown',
        'metadata': {},
        'source': [
            "## 1. Telemetry Ingestion & Missing Data Imputation\n",
            "\n",
            "We load `mx_imu.csv` and linearly impute the 200-sample ($0.400\\,\\text{s}$) sensor dropout at $t \\in [23.000, 23.398]\\,\\text{s}$ identified in Phase 1 EDA."
        ]
    })
    
    cells.append({
        'cell_type': 'code',
        'execution_count': None,
        'metadata': {},
        'outputs': [],
        'source': [
            "csv_path = '../mx_imu.csv' if os.path.exists('../mx_imu.csv') else 'mx_imu.csv'\n",
            "\n",
            "raw_rows = []\n",
            "with open(csv_path, 'r') as f:\n",
            "    header = f.readline().strip().split(',')\n",
            "    for line in f:\n",
            "        parts = line.strip().split(',')\n",
            "        if len(parts) == 7 and all(p != '' for p in parts):\n",
            "            raw_rows.append([float(x) for x in parts])\n",
            "        elif len(parts) == 7:\n",
            "            raw_rows.append([float(parts[0])] + [np.nan] * 6)\n",
            "\n",
            "data = np.array(raw_rows)\n",
            "t = data[:, 0]\n",
            "fs = 500.0\n",
            "\n",
            "# Linear interpolation across dropout\n",
            "for col in range(1, 7):\n",
            "    mask = np.isnan(data[:, col])\n",
            "    if np.any(mask):\n",
            "        valid = ~mask\n",
            "        data[mask, col] = np.interp(t[mask], t[valid], data[valid, col])\n",
            "\n",
            "print(f'Telemetry successfully ingested: {len(t):,} samples ({t[-1] - t[0]:.3f} s) at {fs} Hz.')"
        ]
    })
    
    # ----------------------------------------------------
    # Cell 4: Ground Segment Selection & Welch PSD Formulation
    # ----------------------------------------------------
    cells.append({
        'cell_type': 'markdown',
        'metadata': {},
        'source': [
            "## 2. Ground Segment Selection & Multi-Channel Power Spectral Density (PSD)\n",
            "\n",
            "### 2.1 Segment Selection Criteria\n",
            "To measure steady engine vibration without transient distortion from airborne free fall or landing shocks, we extract:\n",
            "$$t \\in [11.000, 17.000]\\,\\text{s} \\quad (N = 3,001\\,\\text{samples}, \\Delta t = 6.000\\,\\text{s})$$\n",
            "This window provides a wide-sense stationary process under active dirt tracking.\n",
            "\n",
            "### 2.2 Welch's Method Formulation (Pure NumPy)\n",
            "Using $N_{\\text{fft}} = 1,024$ samples (frequency bin spacing $\\Delta f = \\frac{500}{1024} = 0.4883\\,\\text{Hz}$), periodic Hanning window $w[n]$, and $50\\%$ overlap:\n",
            "$$\\hat{S}_{xx}(f_k) = \\frac{2}{K \\cdot f_s \\sum w[n]^2} \\sum_{m=1}^{K} \\left| \\sum_{n=0}^{N_{\\text{fft}}-1} (x_m[n] - \\bar{x}_m) w[n] e^{-j 2\\pi k n / N_{\\text{fft}}} \\right|^2$$"
        ]
    })
    
    cells.append({
        'cell_type': 'code',
        'execution_count': None,
        'metadata': {},
        'outputs': [],
        'source': [
            "def compute_welch_psd(x, fs=500.0, nperseg=1024, noverlap=512):\n",
            "    \"\"\"Pure NumPy implementation of Welch's Average Modified Periodogram PSD.\"\"\"\n",
            "    step = nperseg - noverlap\n",
            "    win = np.hanning(nperseg)\n",
            "    scale = 1.0 / (fs * np.sum(win**2))\n",
            "    psds = []\n",
            "    for start in range(0, len(x) - nperseg + 1, step):\n",
            "        seg = x[start:start + nperseg]\n",
            "        seg_detrend = seg - np.mean(seg)\n",
            "        fft_res = np.fft.rfft(seg_detrend * win)\n",
            "        psd = (np.abs(fft_res)**2) * scale * 2.0\n",
            "        psd[0] /= 2.0\n",
            "        if len(seg) % 2 == 0:\n",
            "            psd[-1] /= 2.0\n",
            "        psds.append(psd)\n",
            "    freqs = np.fft.rfftfreq(nperseg, 1.0 / fs)\n",
            "    return freqs, np.mean(psds, axis=0)\n",
            "\n",
            "# Extract ground segment\n",
            "ground_mask = (t >= 11.0) & (t <= 17.0)\n",
            "channel_names = ['accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z']\n",
            "channel_labels = [r'$a_x$ (Forward)', r'$a_y$ (Lateral)', r'$a_z$ (Vertical)',\n",
            "                  r'$\\omega_x$ (Roll)', r'$\\omega_y$ (Pitch)', r'$\\omega_z$ (Yaw)']\n",
            "channel_units = ['m/s^2', 'm/s^2', 'm/s^2', 'rad/s', 'rad/s', 'rad/s']\n",
            "\n",
            "ground_psds = {}\n",
            "print('Detected Spectral Peaks on Ground Segment (t in [11.0, 17.0] s):')\n",
            "print('-' * 70)\n",
            "for idx, name in enumerate(channel_names, start=1):\n",
            "    sig = data[ground_mask, idx]\n",
            "    f_axis, p_data = compute_welch_psd(sig, fs=fs, nperseg=1024, noverlap=512)\n",
            "    ground_psds[name] = (f_axis, p_data)\n",
            "    \n",
            "    # Fundamental (50-65 Hz)\n",
            "    b1 = (f_axis >= 50.0) & (f_axis <= 65.0)\n",
            "    f_p1 = f_axis[b1][np.argmax(p_data[b1])]\n",
            "    p_db1 = 10.0 * np.log10(np.max(p_data[b1]))\n",
            "    \n",
            "    # Second harmonic (110-125 Hz)\n",
            "    b2 = (f_axis >= 110.0) & (f_axis <= 125.0)\n",
            "    f_p2 = f_axis[b2][np.argmax(p_data[b2])]\n",
            "    p_db2 = 10.0 * np.log10(np.max(p_data[b2]))\n",
            "    print(f\"{name:8s} | Fund: {f_p1:6.2f} Hz ({p_db1:6.2f} dB) | 2nd Harm: {f_p2:6.2f} Hz ({p_db2:6.2f} dB)\")"
        ]
    })
    
    # ----------------------------------------------------
    # Cell 5: Plotting Ground Segment PSD
    # ----------------------------------------------------
    cells.append({
        'cell_type': 'markdown',
        'metadata': {},
        'source': [
            "### 2.3 6-Axis Ground PSD Visualization\n",
            "Plotting the Power Spectral Density across all 6 channels highlights the massive ingress on $a_x$ and $a_z$, the $29\\,\\text{dB}$ attenuation on $a_y$ due to sagittal polarization, and the flat noise floor of the gyroscopes."
        ]
    })
    
    cells.append({
        'cell_type': 'code',
        'execution_count': None,
        'metadata': {},
        'outputs': [],
        'source': [
            "fig, axes = plt.subplots(2, 3, figsize=(16, 9), sharex=True)\n",
            "colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']\n",
            "\n",
            "for i, name in enumerate(channel_names):\n",
            "    r, c = divmod(i, 3)\n",
            "    ax = axes[r, c]\n",
            "    f_axis, p_data = ground_psds[name]\n",
            "    p_db = 10.0 * np.log10(np.maximum(p_data, 1e-12))\n",
            "    ax.plot(f_axis, p_db, color=colors[i], lw=1.5, label=f\"{channel_labels[i]} PSD\")\n",
            "    ax.axvspan(55.0, 61.0, color='#ff9999', alpha=0.3, label='Engine Fund. (~58 Hz)' if i == 0 else '')\n",
            "    ax.axvspan(113.0, 119.0, color='#ffcc99', alpha=0.3, label='Engine 2nd Harm. (~116 Hz)' if i == 0 else '')\n",
            "    ax.axvline(25.0, color='#d62728', ls='--', lw=1.4, label='Filter Cutoff fc = 25 Hz' if i == 0 else '')\n",
            "    ax.set_title(f\"{channel_labels[i]} PSD\", fontsize=11, fontweight='bold')\n",
            "    ax.set_ylabel(f\"PSD [dB/Hz re {channel_units[i]}$^2$]\", fontsize=9)\n",
            "    ax.set_xlim(0, 200)\n",
            "    ax.xaxis.set_major_locator(MultipleLocator(50))\n",
            "    if r == 1:\n",
            "        ax.set_xlabel('Frequency [Hz]', fontsize=10, fontweight='bold')\n",
            "\n",
            "fig.suptitle(r'Power Spectral Density Across All 6 IMU Channels on Ground Telemetry ($t \\in [11.0, 17.0]\\,\\mathrm{s}$)'\n",
            "             '\\nIdentification of Engine Reciprocating Harmonics (58.11 Hz & 116.21 Hz)', fontsize=13, fontweight='bold')\n",
            "fig.legend(loc='lower center', bbox_to_anchor=(0.5, -0.02), ncol=4, fontsize=9.5, frameon=True)\n",
            "plt.tight_layout()\n",
            "plt.show()"
        ]
    })
    
    # ----------------------------------------------------
    # Cell 6: Physical Mechanism Justification Markdown
    # ----------------------------------------------------
    cells.append({
        'cell_type': 'markdown',
        'metadata': {},
        'source': [
            "### 2.4 Physical Transducer Mechanism: Accelerometer Contamination vs Gyroscope Immunity\n",
            "\n",
            "1. **Reciprocating Piston Kinematics**:\n",
            "   A 2-stroke single-cylinder engine generates unbalance primary and secondary reciprocating inertial forces along the cylinder bore:\n",
            "   $$F_{\\text{recip}}(t) = m_{\\text{recip}} r \\omega^2 \\left( \\cos(\\omega t) + \\frac{r}{l} \\cos(2\\omega t) \\right)$$\n",
            "   At $3,487\\,\\text{RPM}$, $\\omega = 365.1\\,\\text{rad/s}$, producing $f_1 = 58.11\\,\\text{Hz}$ and $f_2 = 116.21\\,\\text{Hz}$. With the cylinder angled at $56.7^\\circ$ forward from vertical in the sagittal plane, these linear forces couple directly into $a_x$ and $a_z$, but have zero lateral projection into $a_y$ ($>25\\,\\text{dB}$ quieter).\n",
            "\n",
            "2. **Chassis Torsional Stiffness**:\n",
            "   The frame's massive polar moments of inertia ($I_{xx}, I_{yy}, I_{zz}$) and high torsional rigidity prevent angular twisting at $58\\,\\text{Hz}$. The chassis displaces linearly (producing $\\Delta a \\approx \\pm 4\\,\\text{m/s}^2$), but high-frequency angular rate oscillations remain negligible ($\\Delta\\omega < 0.005\\,\\text{rad/s}$).\n",
            "\n",
            "3. **Transducer Operating Principles (Proof-Mass vs Coriolis Sensing)**:\n",
            "   - **MEMS Accelerometer**: Suspended silicon proof-mass is directly displaced by translational acceleration $\\ddot{x} + 2\\zeta\\omega_0 \\dot{x} + \\omega_0^2 x = -a_{\\text{frame}}$, registering the full structural vibration.\n",
            "   - **MEMS Gyroscope**: Operates via the **Coriolis effect** ($F_c = 2m(\\mathbf{v}_{\\text{drive}} \\times \\mathbf{\\Omega})$). The mass is driven into high-frequency resonance ($20-30\\,\\text{kHz}$), and angular rate is detected via synchronous demodulation, rejecting baseband linear vibrations. Furthermore, differential tuning-fork geometry mechanically cancels linear common-mode acceleration."
        ]
    })
    
    # ----------------------------------------------------
    # Cell 7: Butterworth Filter Mathematical Synthesis
    # ----------------------------------------------------
    cells.append({
        'cell_type': 'markdown',
        'metadata': {},
        'source': [
            "## 3. Mathematical Butterworth Low-Pass Filter Design\n",
            "\n",
            "### 3.1 Analytical Order Derivation\n",
            "Given:\n",
            "- Passband edge $f_p = 20.0\\,\\text{Hz}$ with attenuation $A_p \\le 1.0\\,\\text{dB}$ ($>89\\%$ retention).\n",
            "- Cutoff frequency $f_c = 25.0\\,\\text{Hz}$ ($-3.01\\,\\text{dB}$ half-power bandwidth).\n",
            "- Stopband edge $f_s = 58.0\\,\\text{Hz}$ with minimum rejection $A_s \\ge 30.0\\,\\text{dB}$ ($>96.8\\%$ suppression).\n",
            "\n",
            "Butterworth magnitude constraint:\n",
            "$$|H(j\\omega)|^2 = \\frac{1}{1 + (\\omega / \\omega_c)^{2N}}$$\n",
            "Solving for minimum integer order $N$:\n",
            "$$N \\ge \\frac{\\log_{10}\\left( \\frac{10^{A_s/10} - 1}{10^{A_p/10} - 1} \\right)}{2 \\log_{10}(f_s / f_p)} = \\frac{\\log_{10}\\left( \\frac{999}{0.2589} \\right)}{2 \\log_{10}(2.900)} = \\frac{3.5864}{0.9248} = \\mathbf{3.878} \\implies \\mathbf{N = 4}$$\n",
            "\n",
            "### 3.2 Bilinear Transformation with Pre-Warping\n",
            "$$\\omega_a = 2 f_s \\tan\\left( \\frac{\\pi f_c}{f_s} \\right) = 1,000 \\cdot \\tan(0.05\\pi) = \\mathbf{158.384440\\,\\text{rad/s}}$$\n",
            "Factoring into two Second-Order Sections (Biquads) guarantees numerical stability."
        ]
    })
    
    cells.append({
        'cell_type': 'code',
        'execution_count': None,
        'metadata': {},
        'outputs': [],
        'source': [
            "def synthesize_butterworth_sos(cutoff=25.0, fs=500.0, order=4):\n",
            "    \"\"\"Synthesize Nth-order Butterworth low-pass digital filter as Biquad SOS.\"\"\"\n",
            "    T = 1.0 / fs\n",
            "    wa = (2.0 / T) * np.tan(np.pi * cutoff / fs)\n",
            "    K = 2.0 / T\n",
            "    \n",
            "    k = np.arange(order)\n",
            "    theta = np.pi * (2 * k + 1 + order) / (2 * order)\n",
            "    s_norm = np.exp(1j * theta)\n",
            "    s_norm = s_norm[np.real(s_norm) < -1e-7]\n",
            "    s_norm = s_norm[np.argsort(np.abs(np.imag(s_norm)))]\n",
            "    \n",
            "    sos = []\n",
            "    for i in range(0, len(s_norm), 2):\n",
            "        re_p = np.real(s_norm[i])\n",
            "        a0 = K**2 - 2.0 * wa * re_p * K + wa**2\n",
            "        a1 = 2.0 * (wa**2 - K**2)\n",
            "        a2 = K**2 + 2.0 * wa * re_p * K + wa**2\n",
            "        b0 = wa**2\n",
            "        b1 = 2.0 * wa**2\n",
            "        b2 = wa**2\n",
            "        sos.append(np.array([b0/a0, b1/a0, b2/a0, 1.0, a1/a0, a2/a0]))\n",
            "    return np.array(sos), wa\n",
            "\n",
            "sos_4, wa = synthesize_butterworth_sos(cutoff=25.0, fs=500.0, order=4)\n",
            "print(f'Pre-warped analog cutoff: wa = {wa:.6f} rad/s')\n",
            "for i, sec in enumerate(sos_4, 1):\n",
            "    print(f'Section {i}:')\n",
            "    print(f'  b = [{sec[0]:.8f}, {sec[1]:.8f}, {sec[2]:.8f}]')\n",
            "    print(f'  a = [{sec[3]:.8f}, {sec[4]:.8f}, {sec[5]:.8f}]')"
        ]
    })
    
    # ----------------------------------------------------
    # Cell 8: Bode Plot & Group Delay
    # ----------------------------------------------------
    cells.append({
        'cell_type': 'markdown',
        'metadata': {},
        'source': [
            "### 3.3 Frequency Response & Group Delay Characterization\n",
            "Evaluating magnitude, phase, and group delay across critical frequencies."
        ]
    })
    
    cells.append({
        'cell_type': 'code',
        'execution_count': None,
        'metadata': {},
        'outputs': [],
        'source': [
            "def evaluate_sos_response(sos, freqs, fs=500.0):\n",
            "    w = 2.0 * np.pi * freqs / fs\n",
            "    z = np.exp(1j * w)\n",
            "    H = np.ones(len(freqs), dtype=complex)\n",
            "    for sec in sos:\n",
            "        b, a = sec[:3], sec[3:]\n",
            "        num = b[0] + b[1] * (z**(-1)) + b[2] * (z**(-2))\n",
            "        den = a[0] + a[1] * (z**(-1)) + a[2] * (z**(-2))\n",
            "        H *= (num / den)\n",
            "    mag = np.abs(H)\n",
            "    with np.errstate(divide='ignore'):\n",
            "        mag_db = 20.0 * np.log10(mag)\n",
            "    phase_deg = np.unwrap(np.angle(H)) * 180.0 / np.pi\n",
            "    dw = np.gradient(w)\n",
            "    group_delay_ms = (-np.gradient(np.unwrap(np.angle(H))) / dw / fs) * 1000.0\n",
            "    return H, mag, mag_db, phase_deg, group_delay_ms\n",
            "\n",
            "f_eval = np.linspace(0.1, 150.0, 1500)\n",
            "_, _, mag_db, phase_deg, gd_ms = evaluate_sos_response(sos_4, f_eval, fs=fs)\n",
            "\n",
            "fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 9), sharex=True)\n",
            "ax1.plot(f_eval, mag_db, color='#1f77b4', lw=2.0, label='Causal Gain |H(f)| [dB]')\n",
            "ax1.plot(f_eval, 2.0 * mag_db, color='#2ca02c', lw=1.8, ls='--', label='Zero-Phase Gain |H(f)|^2 [dB]')\n",
            "ax1.axvline(20.0, color='#ff7f0e', ls=':', lw=1.4, label='fp = 20 Hz (-0.67 dB)')\n",
            "ax1.axvline(25.0, color='#d62728', ls='--', lw=1.4, label='fc = 25 Hz (-3.01 dB)')\n",
            "ax1.axvline(58.11, color='#b30000', ls='-.', lw=1.4, label='f1 = 58.11 Hz (-30.54 dB)')\n",
            "ax1.set_ylabel('Gain [dB]', fontweight='bold')\n",
            "ax1.set_ylim(-80, 5)\n",
            "ax1.legend(loc='upper right', fontsize=9)\n",
            "\n",
            "ax2.plot(f_eval, phase_deg, color='#e377c2', lw=1.8, label='Causal Phase Shift [deg]')\n",
            "ax2.plot(f_eval, np.zeros_like(f_eval), color='#2ca02c', lw=1.8, ls='--', label='Zero-Phase (0 deg)')\n",
            "ax2.set_ylabel('Phase [deg]', fontweight='bold')\n",
            "ax2.legend(loc='lower left', fontsize=9)\n",
            "\n",
            "ax3.plot(f_eval, gd_ms, color='#17becf', lw=1.8, label='Causal Group Delay [ms]')\n",
            "ax3.plot(f_eval, np.zeros_like(f_eval), color='#2ca02c', lw=1.8, ls='--', label='Zero-Phase (0 ms)')\n",
            "ax3.set_ylabel('Group Delay [ms]', fontweight='bold')\n",
            "ax3.set_xlabel('Frequency [Hz]', fontweight='bold')\n",
            "ax3.set_xlim(0, 150)\n",
            "ax3.set_ylim(-5, 40)\n",
            "ax3.legend(loc='upper right', fontsize=9)\n",
            "\n",
            "fig.suptitle(r'Bode Response: 4th-Order Butterworth Low-Pass Filter ($f_c = 25.0\\,\\mathrm{Hz}$)', fontsize=13, fontweight='bold')\n",
            "plt.tight_layout()\n",
            "plt.show()"
        ]
    })
    
    # ----------------------------------------------------
    # Cell 9: Quantitative Order Trade-off Comparison
    # ----------------------------------------------------
    cells.append({
        'cell_type': 'markdown',
        'metadata': {},
        'source': [
            "### 3.4 Filter Order Trade-Off Analysis ($N \\in \\{2, 4, 6, 8\\}$)\n",
            "Demonstrating why $N = 4$ is the exact optimal compromise between stopband attenuation and real-time latency."
        ]
    })
    
    cells.append({
        'cell_type': 'code',
        'execution_count': None,
        'metadata': {},
        'outputs': [],
        'source': [
            "orders = [2, 4, 6, 8]\n",
            "print(f\"{'Order':<6} | {'Slope':<12} | {'Atten @ 20Hz':<14} | {'Atten @ 25Hz':<14} | {'Atten @ 58Hz':<14} | {'Delay @ 10Hz':<14}\")\n",
            "print('-' * 85)\n",
            "for o in orders:\n",
            "    s_mat, _ = synthesize_butterworth_sos(cutoff=25.0, fs=fs, order=o)\n",
            "    _, _, m_db, _, g_ms = evaluate_sos_response(s_mat, f_eval, fs=fs)\n",
            "    a20 = abs(m_db[np.argmin(np.abs(f_eval - 20.0))])\n",
            "    a25 = abs(m_db[np.argmin(np.abs(f_eval - 25.0))])\n",
            "    a58 = abs(m_db[np.argmin(np.abs(f_eval - 58.11))])\n",
            "    d10 = g_ms[np.argmin(np.abs(f_eval - 10.0))]\n",
            "    print(f\"N = {o:<2} | {o*20} dB/dec    | -{a20:<11.2f} dB | -{a25:<11.2f} dB | -{a58:<11.2f} dB | {d10:<11.1f} ms\")"
        ]
    })
    
    # ----------------------------------------------------
    # Cell 10: Filtering Execution (Causal vs Zero-Phase)
    # ----------------------------------------------------
    cells.append({
        'cell_type': 'markdown',
        'metadata': {},
        'source': [
            "## 4. Digital Filtering Execution: Causal vs Zero-Phase\n",
            "\n",
            "- **Causal (`sosfilt_df2t`)**: Direct Form II Transposed biquad realization for real-time embedded edge deployment.\n",
            "- **Zero-Phase (`filtfilt_sos`)**: Forward-backward pass with odd reflection padding for zero-latency post-session telemetry scoring."
        ]
    })
    
    cells.append({
        'cell_type': 'code',
        'execution_count': None,
        'metadata': {},
        'outputs': [],
        'source': [
            "def sosfilt_df2t(sos, x):\n",
            "    y = x.copy()\n",
            "    for s in range(sos.shape[0]):\n",
            "        b0, b1, b2, a0, a1, a2 = sos[s]\n",
            "        w1, w2 = 0.0, 0.0\n",
            "        out = np.zeros(len(x))\n",
            "        for i in range(len(x)):\n",
            "            xi = y[i]\n",
            "            yi = b0 * xi + w1\n",
            "            w1 = b1 * xi - a1 * yi + w2\n",
            "            w2 = b2 * xi - a2 * yi\n",
            "            out[i] = yi\n",
            "        y = out\n",
            "    return y\n",
            "\n",
            "def filtfilt_sos(sos, x, padlen=150):\n",
            "    padlen = min(padlen, len(x) - 1)\n",
            "    pad_left = 2.0 * x[0] - x[padlen:0:-1]\n",
            "    pad_right = 2.0 * x[-1] - x[-2:-padlen-2:-1]\n",
            "    x_padded = np.concatenate([pad_left, x, pad_right])\n",
            "    y_fwd = sosfilt_df2t(sos, x_padded)\n",
            "    y_rev = sosfilt_df2t(sos, y_fwd[::-1])\n",
            "    return y_rev[::-1][padlen:-padlen]\n",
            "\n",
            "# Filter vertical acceleration\n",
            "az_raw = data[:, 3]\n",
            "az_causal = sosfilt_df2t(sos_4, az_raw)\n",
            "az_zerophase = filtfilt_sos(sos_4, az_raw)\n",
            "print('Filtering complete across all 45,000 samples.')"
        ]
    })
    
    # ----------------------------------------------------
    # Cell 11: Verification in Frequency Domain (PSD Overlay)
    # ----------------------------------------------------
    cells.append({
        'cell_type': 'markdown',
        'metadata': {},
        'source': [
            "## 5. Verification & Validation\n",
            "\n",
            "### 5.1 Frequency Domain: Proving Stopband Elimination\n",
            "Overlaying PSD before and after filtering on the ground segment demonstrates $>30\\,\\text{dB}$ attenuation at $58\\,\\text{Hz}$ in causal mode and $>60\\,\\text{dB}$ in zero-phase mode."
        ]
    })
    
    cells.append({
        'cell_type': 'code',
        'execution_count': None,
        'metadata': {},
        'outputs': [],
        'source': [
            "f_psd, p_raw = compute_welch_psd(az_raw[ground_mask])\n",
            "_, p_causal = compute_welch_psd(az_causal[ground_mask])\n",
            "_, p_zp = compute_welch_psd(az_zerophase[ground_mask])\n",
            "\n",
            "plt.figure(figsize=(14, 6))\n",
            "plt.plot(f_psd, 10*np.log10(p_raw), color='#888888', lw=1.4, label='Raw $a_z$')\n",
            "plt.plot(f_psd, 10*np.log10(p_causal), color='#ff7f0e', lw=1.6, ls='--', label='Causal Filtered (Δ30.5 dB @ 58 Hz)')\n",
            "plt.plot(f_psd, 10*np.log10(p_zp), color='#1f77b4', lw=2.0, label='Zero-Phase Filtered (Δ61.1 dB @ 58 Hz)')\n",
            "plt.axvline(20.0, color='#2ca02c', ls=':', lw=1.4, label='Motion Band Limit (20 Hz)')\n",
            "plt.axvline(25.0, color='#d62728', ls='--', lw=1.4, label='Cutoff fc = 25 Hz')\n",
            "plt.axvline(58.11, color='#b30000', ls='-.', lw=1.4, label='Engine Peak (58.11 Hz)')\n",
            "plt.axvline(116.21, color='#800080', ls='-.', lw=1.4, label='Engine Peak (116.21 Hz)')\n",
            "plt.xlim(0, 150)\n",
            "plt.ylim(-120, 0)\n",
            "plt.ylabel('Power Spectral Density [dB/Hz]', fontweight='bold')\n",
            "plt.xlabel('Frequency [Hz]', fontweight='bold')\n",
            "plt.title(r'Power Spectral Density Before vs After Filtering on Ground Segment ($a_z$)', fontsize=13, fontweight='bold')\n",
            "plt.legend(loc='lower left', fontsize=10)\n",
            "plt.show()"
        ]
    })
    
    # ----------------------------------------------------
    # Cell 12: Time Domain Waveform Comparison
    # ----------------------------------------------------
    cells.append({
        'cell_type': 'markdown',
        'metadata': {},
        'source': [
            "### 5.2 Time Domain: Clean Kinematics & Transient Preservation\n",
            "Comparison of raw vs filtered waveforms during steady ground riding and Jump 1 event."
        ]
    })
    
    cells.append({
        'cell_type': 'code',
        'execution_count': None,
        'metadata': {},
        'outputs': [],
        'source': [
            "fig, (ax_g, ax_j) = plt.subplots(2, 1, figsize=(15, 8))\n",
            "\n",
            "# Ground detail\n",
            "m_g = (t >= 11.5) & (t <= 13.5)\n",
            "ax_g.plot(t[m_g], az_raw[m_g], color='#bbbbbb', lw=1.0, label='Raw $a_z$ (Corrupted by 58 Hz Ripple)')\n",
            "ax_g.plot(t[m_g], az_causal[m_g], color='#ff7f0e', lw=1.6, ls='--', label='Causal Filtered (df2t)')\n",
            "ax_g.plot(t[m_g], az_zerophase[m_g], color='#1f77b4', lw=2.0, label='Zero-Phase Filtered (filtfilt)')\n",
            "ax_g.set_title(r'Ground Telemetry ($t \\in [11.5, 13.5]\\,\\mathrm{s}$): Total Removal of Engine Oscillation', fontsize=11, fontweight='bold')\n",
            r"ax_g.set_ylabel(r'$a_z$ [$\mathrm{m/s^2}$]', fontweight='bold')" + "\n",
            "ax_g.set_xlim(11.5, 13.5)\n",
            "ax_g.legend(loc='upper right')\n",
            "\n",
            "# Jump 1 detail\n",
            "m_j = (t >= 6.8) & (t <= 10.2)\n",
            "ax_j.plot(t[m_j], az_raw[m_j], color='#bbbbbb', lw=1.0, label='Raw $a_z$')\n",
            "ax_j.plot(t[m_j], az_zerophase[m_j], color='#1f77b4', lw=2.0, label='Zero-Phase Filtered $a_z$')\n",
            "ax_j.axvspan(7.004, 9.748, color='#d6f5d6', alpha=0.4, label='Airborne Window (2.744 s)')\n",
            "ax_j.axvline(7.004, color='#2ca02c', ls='--', label='Takeoff (t = 7.004 s)')\n",
            "ax_j.axvline(9.748, color='#d62728', ls='--', label='Landing (t = 9.748 s)')\n",
            "ax_j.set_title(r'Jump 1 Event ($t \\in [6.8, 10.2]\\,\\mathrm{s}$): Pristine Preservation of Free-Fall Plunge & Landing Impact', fontsize=11, fontweight='bold')\n",
            r"ax_j.set_ylabel(r'$a_z$ [$\mathrm{m/s^2}$]', fontweight='bold')" + "\n",
            "ax_j.set_xlabel('Time [seconds]', fontweight='bold')\n",
            "ax_j.set_xlim(6.8, 10.2)\n",
            "ax_j.legend(loc='upper right')\n",
            "\n",
            "plt.tight_layout()\n",
            "plt.show()"
        ]
    })
    
    # ----------------------------------------------------
    # Assemble Notebook Structure
    # ----------------------------------------------------
    notebook = {
        'cells': cells,
        'metadata': {
            'kernelspec': {
                'display_name': 'Python 3',
                'language': 'python',
                'name': 'python3'
            },
            'language_info': {
                'codemirror_mode': {'name': 'ipython', 'version': 3},
                'file_extension': '.py',
                'mimetype': 'text/x-python',
                'name': 'python',
                'nbconvert_exporter': 'python',
                'pygments_lexer': 'ipython3',
                'version': '3.12.0'
            }
        },
        'nbformat': 4,
        'nbformat_minor': 5
    }
    
    with open(nb_path, 'w') as f:
        json.dump(notebook, f, indent=1)
        
    print(f'Successfully generated valid Jupyter notebook: {nb_path}')

if __name__ == '__main__':
    create_phase3_notebook()
