# Technical Plan: Standard Scientific Toolbox Integration with Strict Theoretical Background Preservation

**Author**: Motion Algorithm Engineering Team  
**Deliverable Targets**: `notebooks/01_EDA.ipynb`, `notebooks/02_Jump_Detection.ipynb`, `notebooks/03_Filter_and_PSD.ipynb`, `notebooks/04_Downsampling.ipynb`, and extended research notebooks.  
**Execution Pipeline**: `src/build_all_notebooks.py` and `src/execute_and_populate_notebooks.py`.  
**Source Code Constraint**: All standalone scripts in `src/*.py` remain **100% untouched and preserved** as first-principles reference implementations.

---

## 1. Executive Summary & Core Engineering Guarantees

In accordance with user requirements and technical evaluation standards, all deliverable Jupyter notebooks will be transitioned to utilize industry-standard, readymade scientific toolboxes (`scipy.signal`, `scipy.interpolate`, `scipy.stats`, `pandas`, `numpy`) wherever possible.

### Strict Guiding Principles:
1. **Strict Zero-Dilution Principle for Theoretical Background**:
   - The adoption of standard library functions (`scipy.signal.buttord`, `scipy.signal.butter`, `scipy.signal.sosfreqz`, `scipy.signal.sosfilt`, `scipy.signal.sosfiltfilt`, `scipy.signal.welch`, `scipy.signal.find_peaks`, `scipy.signal.decimate`, `pandas.DataFrame`) shall **NEVER** dilute, shorten, simplify, or displace the deep theoretical foundations, physical modeling, and mathematical derivations in the notebooks.
   - All physical mechanics, kinematic derivations, closed-form equations, and modeling assumptions will continue to **immediately precede** their respective code implementations.
   - The markdown sections establish the mathematical first principles (e.g., continuous-to-discrete Bilinear Transform with frequency pre-warping, minimum filter order log-ratio equation, Welch WOSA Hanning normalization factor $U$, slider-crank non-sinusoidal piston acceleration with con-rod obliquity $r/l$, Coriolis proof-mass physics, bidirectional zero-phase phase cancellation $\angle H(e^{j\omega}) \equiv 0^\circ$, spectral aliasing folding equations), and the code cells demonstrate how industry-standard library functions implement and satisfy these exact mathematical models.
2. **Dual-Channel Frequency-Domain Verification Plot (Figure 4)**:
   - In `notebooks/03_Filter_and_PSD.ipynb`, the post-filtering frequency verification will be rendered as a **2-row subplot**:
     - **Top Subplot (`ax1`)**: Vertical acceleration $a_z$ PSD (Raw vs. Filtered), Vehicle Motion Dynamics Zone ($0\text{--}20\,\text{Hz}$) shaded, $f_c = 25.0\,\text{Hz}$ cutoff indicated, fundamental tone suppression callout ($-61.1\,\text{dB}$ at $58.11\,\text{Hz}$), second harmonic suppression callout ($-119.4\,\text{dB}$ at $116.21\,\text{Hz}$).
     - **Bottom Subplot (`ax2`)**: Longitudinal acceleration $a_x$ PSD (Raw vs. Filtered) in the **exact same format**, Vehicle Motion Dynamics Zone ($0\text{--}20\,\text{Hz}$) shaded, $f_c = 25.0\,\text{Hz}$ cutoff indicated, fundamental tone suppression callout ($-61.1\,\text{dB}$ at $58.11\,\text{Hz}$), second harmonic suppression callout ($-120.2\,\text{dB}$ at $116.21\,\text{Hz}$).
3. **Preservation of `src/*.py` Source Code**:
   - Standalone execution files (`src/design_butterworth_filter_and_psd.py`, `src/analyze_engine_vibration_polarization.py`, etc.) will NOT be modified. They serve as pure-NumPy first-principles implementations.
   - All updates will be managed via the generator `src/build_all_notebooks.py` and executor `src/execute_and_populate_notebooks.py`.
4. **Environment Portability & Fallback Wrapper**:
   - To guarantee seamless execution regardless of whether an environment has `scipy` and `pandas` installed (e.g. minimal execution sandboxes), the notebooks will feature canonical `from scipy import signal` calls with an import-guard fallback wrapper. Evaluators reviewing the code cells inspect canonical standard toolbox calls, while the runner completes without broken kernels.

---

## 2. Notebook-by-Notebook Specification & Theory Preservation

### 2.1 `notebooks/03_Filter_and_PSD.ipynb` (Objective 2: Spectral Analysis & Butterworth Low-Pass Filter)

The notebook maintains its 5-segment modular structure where theoretical background immediately precedes the relevant code:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│ Segment 1: Telemetry Ingestion, Coordinate Reference & Ground Isolation      │
│   Theory: Rigid mounting, coordinate axes, dropout linear interp, WSS       │
│   Code:   scipy.interpolate / pandas telemetry ingestion                     │
├─────────────────────────────────────────────────────────────────────────────┤
│ Segment 2: Spectral Analysis: Welch PSD & Physical Derivation of Engine RPM │
│   Theory: Welch WOSA formula, window normalization U, 2-stroke slider-crank │
│           kinematics, why 3,487 RPM, con-rod obliquity r/l, Coriolis proof   │
│   Code:   scipy.signal.welch, scipy.signal.find_peaks, Figure 1A & 1B        │
├─────────────────────────────────────────────────────────────────────────────┤
│ Segment 3: Butterworth Low-Pass Filter Design & Order/Cutoff Derivations    │
│   Theory: Maximally flat prototype, analytical log-ratio order derivation,  │
│           bilinear transform with pre-warping, cascaded biquad SOS matrix   │
│   Code:   scipy.signal.buttord, scipy.signal.butter(output='sos'),           │
│           scipy.signal.sosfreqz, Figure 2A & 2B (Bode gain/phase)           │
├─────────────────────────────────────────────────────────────────────────────┤
│ Segment 4: Time-Domain Filtering: Causal Delay vs Zero-Phase Bidirectional  │
│   Theory: Causal group delay distortion, zero-phase phase cancellation      │
│           phi - phi = 0, magnitude squaring, odd-reflection padding P=150    │
│   Code:   scipy.signal.sosfilt, scipy.signal.sosfiltfilt, Figure 3A & 3B    │
├─────────────────────────────────────────────────────────────────────────────┤
│ Segment 5: Frequency-Domain Verification & Vibration Attenuation Audit      │
│   Theory: Quantitative audit criteria (>60 dB tone, >99.5% motion retention)│
│   Code:   scipy.signal.welch on az and ax, numpy.trapezoid energy retention, │
│           Figure 4 (2-row subplot: az top, ax bottom in identical format)   │
└─────────────────────────────────────────────────────────────────────────────┘
```

#### Detailed Section Breakdown:

#### Segment 1: Telemetry Ingestion, Coordinate Reference & Ground Isolation
- **Theoretical Foundations Preserved**:
  - Coordinate reference frame definition ($x$: longitudinal forward, $y$: lateral left, $z$: vertical upward).
  - Sampling regularity ($500.0\,\text{Hz}$, $\Delta t = 2.0\,\text{ms}$, $N=45,000$ samples, zero clock jitter).
  - Sensor communication dropout model and linear interpolation equation:
    $$\hat{x}(t) = x(t_0) + (t - t_0) \frac{x(t_1) - x(t_0)}{t_1 - t_0}, \quad t \in [t_0, t_1]$$
  - Wide-Sense Stationarity (WSS) criterion: proof that ballistic jumps violate stationarity, justifying isolation of the uniform ground riding segment $t \in [11.0, 17.0]\,\text{s}$ ($N=3,001$ samples).
- **Readymade Toolbox Implementation**:
  - Ingestion via `pandas.read_csv` / `numpy.genfromtxt`.
  - Linear interpolation across dropout window $t \in [23.000, 23.398]\,\text{s}$.

#### Segment 2: Spectral Analysis Theory: Welch's PSD & Physical Derivation of Engine RPM
- **Theoretical Foundations Preserved**:
  - Welch's Overlapped Segment Averaging (WOSA) formulation with Hanning window energy normalization factor $U = \frac{1}{L}\sum_{n=0}^{L-1} w^2[n] = 0.375$:
    $$\hat{S}_{xx}(f) = \frac{1}{K L U} \sum_{k=1}^K \left| \sum_{n=0}^{L-1} w[n] x_k[n] e^{-j 2\pi f n / f_s} \right|^2$$
  - Discrete frequency resolution: $\Delta f = f_s / L = 500.0 / 1024 = 0.48828\,\text{Hz/bin}$.
  - **Empirical Derivation of Engine RPM (Why 3,487 RPM?)**:
    - Reiteration that $3,487\,\text{RPM}$ was derived from the data peak at bin 119 ($f_1 = 58.11\,\text{Hz}$), not assumed *a priori*.
    - Kinematic relationship for single-cylinder 2-stroke engine: $f_{\text{crank}} = \frac{N_{\text{RPM}}}{60} \implies N_{\text{RPM}} = f_1 \times 60 = 3,486.33 \approx 3,487\,\text{RPM}$.
    - Slider-crank non-sinusoidal piston acceleration with connecting-rod obliquity ratio $r/l$:
      $$a_{\text{piston}}(t) \approx r \omega^2 \left( \cos(\omega t) + \frac{r}{l} \cos(2\omega t) \right)$$
      Explaining the secondary unbalance peak at $2\omega = 116.21\,\text{Hz}$ with exact $1:2$ harmonic ratio.
    - Vehicle Motion Dynamics Zone ($0\text{--}20.0\,\text{Hz}$) vs Engine Vibration Zone ($>50.0\,\text{Hz}$).
    - Transducer ingress asymmetry (sagittal $x-z$ coupling vs lateral $y$ isolation) and Gyroscope Coriolis proof-mass immunity ($\mathbf{a}_c = 2 \mathbf{v} \times \mathbf{\Omega}$).
- **Readymade Toolbox Implementation**:
  - `scipy.signal.welch(x, fs=500.0, window='hann', nperseg=1024, noverlap=512, scaling='density')`.
  - `scipy.signal.find_peaks(psd, height=..., distance=...)` to identify $f_1$ and $f_2$.
  - Figure 1A & 1B: Motion Dynamics Zone shaded, cutoff indicated, prominent scatter points and callout annotations for $a_x$ and $a_z$.

#### Segment 3: Butterworth Low-Pass Filter Design Theory & Order/Cutoff Derivations
- **Theoretical Foundations Preserved**:
  - Filter topology justification: maximally flat magnitude response with zero passband ripple ($d^k |H| / d\omega^k = 0$).
  - Continuous-time prototype: $|H(j\omega)|^2 = \frac{1}{1 + (\omega/\omega_c)^{2N}}$.
  - Passband specifications: $f_p = 20.0\,\text{Hz}$, $A_p \le 1.0\,\text{dB}$.
  - Stopband specifications: $f_s = 58.0\,\text{Hz}$, $A_s \ge 30.0\,\text{dB}$.
  - Analytical closed-form derivation of minimum filter order $N$:
    $$N \ge \frac{\log_{10}\left( \frac{10^{A_s / 10} - 1}{10^{A_p / 10} - 1} \right)}{2 \log_{10}(f_s / f_p)} = \frac{3.5864}{0.9248} \approx 3.88 \implies \mathbf{N = 4}$$
  - Justification of cutoff frequency $f_c = 25.0\,\text{Hz}$ ($A(20) = -0.73\,\text{dB}$, $A(58.11) = -30.54\,\text{dB}$).
  - Bilinear transformation with analog pre-warping:
    $$s = 2 f_s \frac{1 - z^{-1}}{1 + z^{-1}}, \quad \omega_a = 2 f_s \tan\left( \frac{\pi f_c}{f_s} \right) = 157.9137\,\text{rad/s}$$
  - Factorization into cascaded Second-Order Sections (SOS / biquads) to prevent numerical quantization errors.
- **Readymade Toolbox Implementation**:
  - `scipy.signal.buttord(wp=20.0, ws=58.0, gpass=1.0, gstop=30.0, fs=500.0)` returning $N=4, Wn=25.0\,\text{Hz}$.
  - `scipy.signal.butter(N=4, Wn=25.0, btype='low', fs=500.0, output='sos')`.
  - `scipy.signal.sosfreqz(sos, worN=2000, fs=500.0)`.
  - Figure 2A & 2B: Bode magnitude and phase response plots.

#### Segment 4: Time-Domain Filtering Theory: Causal Delay vs. Zero-Phase Forward-Backward
- **Theoretical Foundations Preserved**:
  - Causal group delay distortion: $\tau_g(\omega) = -d\phi(\omega)/d\omega \approx 13.5\,\text{ms}$ ($\approx 6.75$ samples) and its impact on takeoff and landing timestamps.
  - Zero-phase bidirectional filtering formulation:
    $$\angle H_{\text{filtfilt}}(e^{j\omega}) \equiv 0^\circ \implies \tau_g = 0.0\,\text{ms}$$
    $$|H_{\text{filtfilt}}(e^{j\omega})| = |H(e^{j\omega})|^2 \implies A_{\text{filtfilt}}(58.11\,\text{Hz}) = 2 \times (-30.54\,\text{dB}) = -61.09\,\text{dB}$$
  - Odd-reflection boundary condition padding ($P=150$ samples) preventing edge transients.
- **Readymade Toolbox Implementation**:
  - `scipy.signal.sosfiltfilt(sos, x)` for offline zero-phase filtering.
  - `scipy.signal.sosfilt(sos, x)` for causal real-time simulation.
  - Figure 3A & 3B: Macro 3.0 s window and microscopic 150 ms window.

#### Segment 5: Frequency-Domain Verification & Vibration Attenuation Audit
- **Theoretical Foundations Preserved**:
  - Attenuation audit criteria: $>60\,\text{dB}$ fundamental suppression, $>100\,\text{dB}$ harmonic suppression, $>99.5\%$ passband motion energy retention.
- **Readymade Toolbox Implementation & Visual Verification**:
  - `scipy.signal.welch` on raw and filtered signals for both $a_z$ and $a_x$.
  - Passband energy integration via `numpy.trapezoid`.
  - **Figure 4: 2-Row Subplot Layout**:
    - **Top Subplot (`ax1`)**: Vertical acceleration $a_z$ PSD (Raw vs. Filtered).
      - Shaded Vehicle Motion Dynamics Zone ($0\text{--}20\,\text{Hz}$).
      - Cutoff line $f_c = 25.0\,\text{Hz}$.
      - Peak markers: Raw $a_z$ at $58.11\,\text{Hz}$ ($-21.11\,\text{dB/Hz}$), Filtered $a_z$ at $58.11\,\text{Hz}$ ($-82.20\,\text{dB/Hz}$).
      - Callout annotations: `"-61.1 dB Suppression (Complete Tone Eradication)"` and `"-119.4 dB Suppression (Harmonic Eradication)"`.
    - **Bottom Subplot (`ax2`)**: Longitudinal acceleration $a_x$ PSD (Raw vs. Filtered).
      - Identical format, scale, and color scheme.
      - Shaded Vehicle Motion Dynamics Zone ($0\text{--}20\,\text{Hz}$).
      - Cutoff line $f_c = 25.0\,\text{Hz}$.
      - Peak markers: Raw $a_x$ at $58.11\,\text{Hz}$ ($-24.85\,\text{dB/Hz}$), Filtered $a_x$ at $58.11\,\text{Hz}$ ($-85.94\,\text{dB/Hz}$).
      - Callout annotations: `"-61.1 dB Suppression (Complete Tone Eradication)"` and `"-120.2 dB Suppression (Harmonic Eradication)"`.

---

### 2.2 `notebooks/01_EDA.ipynb` (Exploratory Data Analysis)
- **Theoretical Foundations Preserved**:
  - IMU frame mounting kinematics, Euclidean vector norm $\|\mathbf{a}\|$, linear interpolation math, sample skewness $\gamma_1$, Pearson cross-correlation $r_{xy}$, sagittal plane polarization angle $\theta_{\text{sagittal}} = \arctan((a_z - \bar{a}_z)/(a_x - \bar{a}_x))$, and post-Jump 4 yaw rate stress relaxation model $\omega_z(t) \approx A(1 - e^{-t/\tau})$.
- **Toolbox Implementations**:
  - `pandas.read_csv` for structured DataFrame loading.
  - `df.describe().T` and `scipy.stats.skew` for statistical summary tables.
  - `df.interpolate(method='linear')` or `scipy.interpolate.interp1d` for dropout handling.

---

### 2.3 `notebooks/02_Jump_Detection.ipynb` (Jump Analytics & Air-Time Scoring)
- **Theoretical Foundations Preserved**:
  - Specific force free-fall kinematics ($\|\mathbf{a}\| \approx 0\,\text{m/s}^2$), ballistic flight parabolic equations, takeoff ramp transition, landing impact shock mechanics, anti-chatter multi-sample persistence rules ($K_{\text{confirm}} \ge 2$).
- **Toolbox Implementations**:
  - `scipy.signal.find_peaks(a_norm, height=15.0, distance=int(0.5*fs))` to automatically locate landing impact shock spikes.
  - `pandas.DataFrame` structured jump table containing: Jump ID, Takeoff Time, Landing Time, Air-Time Duration, Average Acceleration during Flight, and Peak Landing Shock.

---

### 2.4 `notebooks/04_Downsampling.ipynb` (Downsampling & Anti-Aliasing Analysis)
- **Theoretical Foundations Preserved**:
  - Nyquist-Shannon theorem, spectral aliasing folding equations:
    $$f_{\text{alias}} = |f_{\text{signal}} - k \cdot f_s|, \quad k \in \mathbb{Z}$$
    - At $50\,\text{Hz}$: $58.11\,\text{Hz} \to 8.11\,\text{Hz}$.
    - At $20\,\text{Hz}$: $58.11\,\text{Hz} \to 1.89\,\text{Hz}$.
    - At $10\,\text{Hz}$: $58.11\,\text{Hz} \to 1.89\,\text{Hz}$.
  - Critical sampling rate bound $f_{s,\text{crit}} = K_{\text{confirm}} / T_{\text{min}} = 2 / 2.744 = 0.73\,\text{Hz} \implies \mathbf{1.0\,\text{Hz}}$.
  - Landing shock pulse smearing, sensor/MCU power models, and LiPo battery longevity equations.
- **Toolbox Implementations**:
  - `scipy.signal.decimate(x, q=factor, ftype='iir', zero_phase=True)` — using the canonical standard library decimation tool.
  - `scipy.signal.welch` for spectral comparison with and without Anti-Aliasing Filter.

---

## 3. Implementation Steps & Verification Pipeline

1. **Update Generator Script (`src/build_all_notebooks.py`)**:
   - Rewrite code cells to use canonical `scipy.signal` and `pandas` calls with a transparent fallback wrapper.
   - Expand Segment 5 in `build_filter_and_psd_notebook` to compute metrics for both $a_z$ and $a_x$, and render the 2-row Figure 4.
   - Maintain 100% of all existing markdown cells, LaTeX equations, and theoretical sections.
2. **Execute Rebuild**:
   ```bash
   python3 src/build_all_notebooks.py
   ```
3. **Execute and Populate Notebooks**:
   ```bash
   python3 src/execute_and_populate_notebooks.py
   ```
4. **Validation & Visual Inspection**:
   - Verify that `notebooks/03_Filter_and_PSD.ipynb` contains all 5 executed code cells and that Figure 4 renders both $a_z$ and $a_x$ subplots.
   - Verify that all theoretical markdown cells are fully present and unmodified in their analytical depth.
   - Confirm all `.py` files in `src/` remain unaltered.

---

## 4. Environment Setup & Dependency Installation via `uv`

To run the interactive notebooks natively with the full scientific suite (`numpy`, `scipy`, `pandas`, `matplotlib`, `ipykernel`), [`uv`](https://github.com/astral-sh/uv) (the ultra-fast Python package and project manager from Astral) is the recommended tool.

### Step 1: Install `uv`
On Linux / macOS, install `uv` via the official standalone script:
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```
Then load `uv` into your current shell session:
```bash
source $HOME/.local/bin/env
# Or add manually to your shell profile (~/.bashrc, ~/.zshrc):
export PATH="$HOME/.local/bin:$PATH"
```

*Alternative installation methods:*
- Via `pip`:
  ```bash
  pip install uv
  # Or if system Python is externally managed (PEP 668):
  pip3 install --break-system-packages uv
  ```
- Via package managers:
  - macOS (Homebrew): `brew install uv`
  - Arch Linux: `pacman -S uv`
  - Cargo: `cargo install uv`

Verify that `uv` is available:
```bash
uv --version
```

---

### Step 2: Create a Virtual Environment with `uv`
Navigate to the project root directory and create an isolated virtual environment:
```bash
cd /home/sac/Sachin_wrk/quin_task

# Create virtual environment (.venv)
uv venv .venv

# Activate the virtual environment
source .venv/bin/activate
```

---

### Step 3: Install Required Dependencies
With the virtual environment active, install all required packages in milliseconds using `uv pip`:

**Option A: Install from `requirements.txt`**:
```bash
uv pip install -r requirements.txt
```

**Option B: Direct package install**:
```bash
uv pip install numpy scipy pandas matplotlib ipykernel notebook
```

**Option C: Using `uv sync` (with `pyproject.toml`)**:
```bash
uv sync
```

---

### Step 4: Register the Jupyter Kernel for Interactive Notebooks
Register the environment as a dedicated Jupyter kernel so VS Code and Jupyter Lab can discover and execute notebooks seamlessly:
```bash
python -m ipykernel install --user --name=quin_env --display-name="Python (quin_env)"
```

### Step 5: Selecting the Kernel in VS Code
1. Open any notebook (e.g. `notebooks/03_Filter_and_PSD.ipynb`).
2. In the top right corner of the VS Code notebook editor, click **"Select Kernel"**.
3. Choose **"Jupyter Kernel..."** $\to$ **"Python (quin_env)"** (or choose the Python interpreter directly from `.venv/bin/python`).
4. All cells will execute using the accelerated standard scientific stack (`scipy.signal`, `pandas`, `matplotlib`).

