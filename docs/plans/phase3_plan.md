# Detailed Technical Plan: Phase 3 — Objective 2 (Filter Design & Spectral Analysis)

## 1. Executive Summary & Problem Formulation

In motocross telemetry using frame-mounted inertial measurement units (IMUs), mechanical vibration from the internal combustion engine couples strongly into the accelerometer channels. The single-cylinder 2-stroke engine operates at an idle/mid-throttle range of $\approx 3,487\,\text{RPM}$, generating a fundamental reciprocating inertial vibration peak at **$f_1 \approx 58.11\,\text{Hz}$** and a prominent second harmonic at **$f_2 \approx 116.21\,\text{Hz}$**.

### Objective Requirements (from Task Specification):
1. **Ground Segment Power Spectral Density (PSD)**:
   - Compute and plot the PSD of all 3 accelerometer axes ($a_x, a_y, a_z$) and all 3 gyroscope axes ($\omega_x, \omega_y, \omega_z$) during a stationary/uniform ground segment.
   - Accurately identify and label the engine vibration peaks ($58\,\text{Hz}$ and $116\,\text{Hz}$).
   - Physically explain why accelerometer channels exhibit severe contamination while gyroscope channels remain largely immune.
2. **Butterworth Low-Pass Filter Design & Justification**:
   - Design a discrete-time Butterworth low-pass filter with cutoff frequency $f_c = 25\,\text{Hz}$ to strip out engine vibration harmonics while preserving vehicle motion dynamics below $20\,\text{Hz}$.
   - Rigorously justify the filter order ($N = 4$) and cutoff frequency ($f_c = 25\,\text{Hz}$) through transfer function equations, passband ripple/attenuation, stopband rejection constraints, and group delay bounds.
   - Implement the filter using Second-Order Sections (Biquads / SOS) to guarantee numerical stability.
   - Evaluate both zero-phase forward-backward filtering (`filtfilt`) for telemetry/offline scoring and causal single-pass filtering (`lfilter`) for embedded real-time triggers.

---

## 2. Ground Segment Selection & Spectral Methodology

### 2.1 Segment Selection Criteria
To measure steady-state engine noise without distortion from free-fall weightlessness or transient jump impacts, we select the ground segment:
$$t \in [11.000, 17.000]\,\text{s} \quad (N = 3,000\,\text{samples}, \Delta t = 6.000\,\text{s})$$
- **Location**: Situated between Jump 1 touchdown ($t = 9.748\,\text{s}$) and Jump 2 takeoff ($t = 18.404\,\text{s}$).
- **Physical State**: The bike is under active throttle on the dirt track ($a_z \approx 9.8\,\text{m/s}^2$), far from the sensor dropout at $t = 23.0\,\text{s}$.
- **Ergodicity & Stationarity**: Provides a stationary random process for Welch PSD estimation.

### 2.2 Welch Power Spectral Density Formulation (Pure NumPy)
Given sampling frequency $f_s = 500.0\,\text{Hz}$:
- Segment length: $N_{\text{fft}} = 1,024\,\text{samples}$, yielding frequency resolution $\Delta f = \frac{f_s}{N_{\text{fft}}} = \frac{500}{1024} = \mathbf{0.4883\,\text{Hz}}$.
- Window: Periodic Hanning (Hann) window $w[n] = 0.5 - 0.5\cos\left(\frac{2\pi n}{N_{\text{fft}}}\right)$ with power normalization $S_2 = \sum_{n=0}^{N_{\text{fft}}-1} w[n]^2$.
- Overlap: $50\%$ overlap ($N_{\text{overlap}} = 512\,\text{samples}$, step size $L = 512$).
- One-Sided PSD Scaling:
  $$\hat{S}_{xx}(f_k) = \frac{2}{K \cdot f_s \cdot S_2} \sum_{m=1}^{K} \left| \sum_{n=0}^{N_{\text{fft}}-1} (x_m[n] - \bar{x}_m) w[n] e^{-j 2\pi k n / N_{\text{fft}}} \right|^2$$
  for $0 < f_k < f_s / 2$, where $K$ is the number of overlapping segments.

---

## 3. Mathematical Filter Design & Justification

### 3.1 Design Specifications & Constraints
- **Passband Edge ($f_p$)**: $20.0\,\text{Hz}$. Must preserve human-rider maneuvers, suspension roll/pitch/yaw motions, and takeoff transitions. Maximum permissible attenuation: $A_p \le 1.0\,\text{dB}$ ($> 89\%$ amplitude retention).
- **Cutoff Frequency ($f_c$)**: $25.0\,\text{Hz}$ (the $-3.01\,\text{dB}$ half-power bandwidth).
- **Stopband Edge ($f_s$)**: $58.0\,\text{Hz}$ (the fundamental 2-stroke engine harmonic). Minimum required attenuation: $A_s \ge 30.0\,\text{dB}$ (suppressing $> 97\%$ of vibration power).
- **Second Harmonic ($2 f_s$)**: $116.0\,\text{Hz}$. Expected attenuation: $> 60.0\,\text{dB}$ (suppressing $99.9\%$ of harmonic power).

### 3.2 Analytical Derivation of Minimum Filter Order ($N$)
The continuous-time Butterworth power gain is:
$$|H(j\omega)|^2 = \frac{1}{1 + (\omega / \omega_c)^{2N}}$$
Expressed in decibels:
$$A(f) = 10 \log_{10}\left(1 + \left(\frac{f}{f_c}\right)^{2N}\right) \quad [\text{dB}]$$

To simultaneously meet passband tolerance $A(f_p) \le A_p$ and stopband attenuation $A(f_s) \ge A_s$:
$$\left(\frac{f_p}{f_c}\right)^{2N} \le 10^{A_p/10} - 1 \quad \text{and} \quad \left(\frac{f_s}{f_c}\right)^{2N} \ge 10^{A_s/10} - 1$$

Dividing the two constraints eliminates $f_c$ and solves for the minimum theoretical order $N$:
$$N \ge \frac{\log_{10}\left( \frac{10^{A_s/10} - 1}{10^{A_p/10} - 1} \right)}{2 \log_{10}(f_s / f_p)} = \frac{\log_{10}\left( \frac{10^{3.0} - 1}{10^{0.1} - 1} \right)}{2 \log_{10}(58 / 20)} = \frac{\log_{10}(999 / 0.25892)}{2 \log_{10}(2.9000)} = \frac{3.5864}{0.9248} = \mathbf{3.878} \implies \mathbf{N = 4}$$

Thus, **order $N = 4$ is the exact mathematical minimum** required to bridge the steep transition from $20\,\text{Hz}$ to $58\,\text{Hz}$.

### 3.3 Quantitative Trade-off Across Filter Orders

| Filter Order ($N$) | Roll-off Slope | Attenuation at $20\,\text{Hz}$ | Attenuation at $25\,\text{Hz}$ ($f_c$) | Attenuation at $58\,\text{Hz}$ ($f_1$) | Attenuation at $116\,\text{Hz}$ ($f_2$) | Passband Group Delay ($\tau_g$) | Engineering Verdict |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **$N = 2$** | $-40\,\text{dB/dec}$ | $-1.49\,\text{dB}$ | $-3.01\,\text{dB}$ | $-14.73\,\text{dB}$ | $-26.69\,\text{dB}$ | $12.7\,\text{ms}$ | **Underdamped**: leaves $18.3\%$ of engine vibration |
| **$N = 4$** | **$-80\,\text{dB/dec}$** | **$-0.67\,\text{dB}$** | **$-3.01\,\text{dB}$** | **$-30.54\,\text{dB}$** | **$-60.09\,\text{dB}$** | **$25.5\,\text{ms}$** | **Optimal**: meets all passband/stopband constraints |
| **$N = 6$** | $-120\,\text{dB/dec}$ | $-0.32\,\text{dB}$ | $-3.01\,\text{dB}$ | $-44.85\,\text{dB}$ | $-93.58\,\text{dB}$ | $38.2\,\text{ms}$ | **Excessive Lag**: large group delay degrades real-time triggers |
| **$N = 8$** | $-160\,\text{dB/dec}$ | $-0.15\,\text{dB}$ | $-3.01\,\text{dB}$ | $-59.20\,\text{dB}$ | $-127.1\,\text{dB}$ | $50.9\,\text{ms}$ | **Overdesigned**: severe latency, sensitive to coefficient quantization |

### 3.4 Discrete-Time Synthesis via Bilinear Transform
To prevent frequency distortion near the Nyquist limit ($f_{\text{Nyq}} = 250\,\text{Hz}$), the analog cutoff frequency $\omega_c = 2\pi f_c$ is pre-warped:
$$\omega_a = \frac{2}{T} \tan\left(\frac{\omega_c T}{2}\right) = 2 f_s \tan\left(\frac{\pi f_c}{f_s}\right) = 1,000 \cdot \tan\left(\frac{25\pi}{500}\right) = \mathbf{158.3844\,\text{rad/s}}$$

The 4 analog poles lie on the radius $\omega_a$ in the left-half s-plane:
$$p_k = \omega_a \exp\left(j \frac{2k + 3}{8}\pi\right), \quad k \in \{1, 2, 3, 4\}$$
Factoring into two complex conjugate pole pairs yields two Second-Order Sections (Biquads):
$$H(z) = H_1(z) \cdot H_2(z) = \frac{b_{0,1} + b_{1,1}z^{-1} + b_{2,1}z^{-2}}{1 + a_{1,1}z^{-1} + a_{2,1}z^{-2}} \cdot \frac{b_{0,2} + b_{1,2}z^{-1} + b_{2,2}z^{-2}}{1 + a_{1,2}z^{-1} + a_{2,2}z^{-2}}$$

Exact discrete coefficients:
- **Section 1**:
  $$b = [0.01903683,\; 0.03807366,\; 0.01903683], \quad a = [1.0,\; -1.47967422,\; 0.55582154]$$
- **Section 2**:
  $$b = [0.02188385,\; 0.04376770,\; 0.02188385], \quad a = [1.0,\; -1.70096433,\; 0.78849974]$$
- **Total DC Gain**: $\prod_{i=1}^{2} \frac{\sum b_i}{\sum a_i} = 1.0000000000$ ($0.000\,\text{dB}$).

---

## 4. Work Breakdown & Deliverables

### 4.1 Script: `src/design_butterworth_filter_and_psd.py`
- Self-contained, pure NumPy + Matplotlib execution.
- Functions:
  - `welch_psd(x, fs, nfft, noverlap)`: Welch spectral estimator with Hanning windowing and one-sided power scaling.
  - `butterworth_lowpass_sos(cutoff, fs, order)`: Analytical biquad pole synthesis and bilinear transformation.
  - `sosfilt_np(sos, x)`: Direct Form II Transposed filtering.
  - `filtfilt_sos_np(sos, x)`: Zero-phase forward-backward filtering with odd reflection padding.
  - `evaluate_filter_specs()`: Frequency response analysis, Bode magnitude/phase, group delay.
  - `run_filter_design_pipeline()`: End-to-end execution, saving metrics and figures.

### 4.2 Output Folder: `results/phase3_filter_design/`
- `ground_psd_peaks.csv`: Table of detected spectral peaks for all 6 IMU channels.
- `filter_specs.json`: Synthesized transfer function coefficients and gain/attenuation metrics.
- `phase3_summary.md`: Engineering documentation with embedded figures and interpretation guides.
- `plots/`:
  1. `01_ground_segment_imu_psd.png`: 6-axis ground PSD highlighting $58.11\,\text{Hz}$ and $116.21\,\text{Hz}$ peaks.
  2. `02_butterworth_bode_response.png`: 4th-order Butterworth magnitude, phase, and group delay with critical frequency markers.
  3. `03_filter_order_tradeoff_comparison.png`: Comparison of orders $N \in \{2, 4, 6, 8\}$ across attenuation and delay.
  4. `04_filtered_vs_raw_time_domain.png`: Detailed time-series overlay of raw vs filtered signals on ground and during Jump 1.
  5. `05_filtered_vs_raw_psd_comparison.png`: Spectral overlay proving complete eradication of the $58\,\text{Hz}$ and $116\,\text{Hz}$ peaks.
  6. `06_causal_vs_zerophase_comparison.png`: Causal real-time delay ($25.5\,\text{ms}$) vs Zero-Phase offline processing.

### 4.3 Reproducible Jupyter Notebook: `notebooks/03_Filter_and_PSD.ipynb`
- Verified valid JSON structure.
- Interactive cells for PSD estimation, filter coefficient calculation, filtering execution, and plotting.

---

## 5. Verification & Acceptance Criteria
1. **Spectral Precision**: PSD correctly isolates $58.11\,\text{Hz}$ ($\pm 0.5\,\text{Hz}$) and $116.21\,\text{Hz}$ ($\pm 0.5\,\text{Hz}$) peaks on accelerometer axes, and verifies their absence on gyroscope axes.
2. **Attenuation Constraints**: Synthesized filter achieves $\le 0.67\,\text{dB}$ loss at $20\,\text{Hz}$, exactly $-3.01\,\text{dB}$ at $25\,\text{Hz}$, $\ge 30.5\,\text{dB}$ rejection at $58\,\text{Hz}$, and $\ge 60.0\,\text{dB}$ rejection at $116\,\text{Hz}$.
3. **Phase & Group Delay**: Analytical verification that zero-phase filtering produces $\Delta\phi = 0^\circ$ and $\tau_g = 0\,\text{ms}$, while causal filtering maintains a flat $25.5\,\text{ms}$ delay in $[0, 15]\,\text{Hz}$.
4. **Reproducibility**: Script runs cleanly via CLI with exit code 0; notebook is valid JSON.

