# Edge IMU Motion Analytics & Vibration DSP Engine
**High-Frequency 6-Axis Telemetry Processing, Ballistic Jump Kinematics, Powertrain Vibration Isolation, and Edge Decimation**  
*Author: Sachin Sagar*  
*Portfolio Project: Sensor Fusion, Digital Signal Processing & Embedded Motion Analytics*

---

## 1. Project Overview

This repository provides a production-grade signal processing, kinematics estimation, and edge power optimization engine designed for two-wheeled vehicles (motocross motorcycles) using high-frequency 6-axis Inertial Measurement Unit (IMU) telemetry.

Rigidly mounting an IMU near the steering headstock of a high-performance motorcycle subjects sensors to extreme mechanical environments: strong reciprocating internal combustion vibration, harsh terrain chattering, and high-$g$ landing shocks. This engine solves the multi-faceted problem of:
1. **Kinematic Event Detection**: Robustly identifying ballistic jump flight intervals and takeoff/landing timestamps using strictly accelerometer data in the presence of continuous mechanical vibration.
2. **Spectral Characterization & Vibration Isolation**: Isolating single-cylinder 2-stroke engine unbalance harmonics and synthesizing digital filters (Butterworth LP, Parametric SysID ARX, and State-Space Kalman filters) to attenuate engine noise while preserving rider motion dynamics.
3. **Edge Downsampling & IoT Power Optimization**: Establishing analytical lower bounds for sampling frequency, proving the necessity of Anti-Aliasing Filtering (AAF), and optimizing battery longevity for constrained IoT hardware.

### Hardware & Telemetry Context:
- **Sensor**: 6-axis IMU (triaxial accelerometer and triaxial gyroscope) mounted rigidly to the frame near the steering headstock.
- **Powertrain**: Single-cylinder 2-stroke engine producing strong reciprocating inertial unbalance forces.
- **Telemetry Specifications**: $90.0\,\text{seconds}$ duration, $500.0\,\text{Hz}$ nominal Output Data Rate (ODR), $N = 45,000$ samples.
- **Reference Coordinate Frame**:
  - **X-axis**: Forward along the longitudinal chassis centerline.
  - **Y-axis**: Lateral directed out the left side of the frame.
  - **Z-axis**: Vertical directed upwards orthogonal to the chassis plane.

---

## 2. Physics-Based Synthetic Telemetry Generator

To enable reproducible benchmarking without reliance on proprietary datasets, this project includes a first-principles **Synthetic Telemetry Generator** ([`src/generate_synthetic_telemetry.py`](src/generate_synthetic_telemetry.py)):

```
+-------------------------------------------------------------------------------------------------+
|                                 FIRST-PRINCIPLES SIMULATION ENGINE                              |
+-------------------------------------------------------------------------------------------------+
                                                 |
         +---------------------------------------+---------------------------------------+
         |                                       |                                       |
         v                                       v                                       v
+-----------------------------+         +-----------------------------+         +-----------------------------+
| 2-Stroke Engine Unbalance   |         | Terrain & Chassis Dynamics  |         | Ballistic Flight & Impacts  |
| • Crankshaft: 58.11 Hz      |         | • Filtered Brownian dirt    |         | • 5 Free-fall jump plateaus |
| • Harmonic: 116.21 Hz       |         |   undulation (fc < 15 Hz)   |         | • Landing shock: 18.2 m/s²  |
| • 56.7° Sagittal lean tilt  |         | • az = 9.81 m/s² gravity    |         |   half-sine + exp damping   |
+-----------------------------+         +-----------------------------+         +-----------------------------+
```

The synthetic generator precisely models:
- **Reciprocating Piston Kinematics**: Primary ($58.11\,\text{Hz} / 3,487\,\text{RPM}$) and secondary ($116.21\,\text{Hz}$) harmonic excitation forces projected along a $56.7^\circ$ cylinder stroke tilt in the sagittal ($x-z$) plane.
- **Ballistic Free-Fall Trajectories**: Unconstrained free-fall intervals where specific force collapses toward zero ($\lVert \mathbf{a} \rVert < 0.5 - 2.0\,\text{m/s}^2$).
- **Landing Shock Physics**: Asymmetric compression spikes reaching $18.0 - 18.7\,\text{m/s}^2$ ($1.84 - 1.90\,g$) occurring 16 ms post-touchdown, followed by exponential suspension damping ($\tau \approx 18\,\text{ms}$).
- **Sensor Packaging Artifacts**: Post-shock viscoelastic stress relaxation in the MEMS yaw rate sensor ($\omega_z(t) = 0.445(1 - e^{-(t-60.2)/9.5})\,\text{rad/s}$) and bus communication dropouts (0.400s gap at $t \in [23.0, 23.4]\,\text{s}$).

To generate or re-synthesize the primary telemetry file:
```bash
python3 src/generate_synthetic_telemetry.py --output data/motocross_imu_telemetry.csv
```

---

## 3. Repository Architecture

```
edge-imu-motion-analytics/
├── README.md                                  # Repository overview and technical documentation
├── requirements.txt                           # Core dependencies (numpy, scipy, pandas, matplotlib, ipykernel)
├── pyproject.toml                             # Package metadata and build configuration
├── TOOLBOX_MIGRATION_PLAN.md                  # Standard scientific toolbox migration plan (Zero-Dilution)
├── mx_imu.csv                                 # Backward-compatible symlink to data/motocross_imu_telemetry.csv
│
├── data/                                      # Synthesized first-principles telemetry datasets
│   └── motocross_imu_telemetry.csv            # 6-axis 500 Hz IMU telemetry dataset (45,000 samples)
│
├── archive/                                   # Historical assessment archive & provenance
│   └── quin_assessment_original/              # Preserved original assessment specs, drafts, & dataset
│       ├── README.md                          # Archive documentation & provenance statement
│       ├── mx_imu.csv                         # Original reference telemetry dataset
│       ├── specifications/                    # Historical assessment prompt PDFs
│       └── submission_reports/                # Historical submission drafts & reports
│
├── docs/                                      # Comprehensive technical documentation & reports
│   ├── final_report.md                        # Master Technical Report
│   ├── final_report.html                      # Standalone responsive HTML report with MathJax
│   ├── eda_summary.md                         # Exploratory data analysis & 2-stroke noise characterization
│   ├── phase3_report.md                       # Spectral Analysis & Filter Design report
│   ├── phase3_report.html                     # HTML report for Filter Design
│   ├── phase4_report.md                       # Downsampling & Power Optimization report
│   ├── phase4_report.html                     # HTML report for Downsampling
│   ├── MATHEMATICAL_AND_ENGINEERING_CONCEPTS.md # Complete compendium of mathematical derivations
│   ├── walkthrough.md                         # Project walkthrough and execution milestones
│   └── plans/                                 # Engineering design plans
│
├── notebooks/                                 # Interactive Jupyter Notebooks
│   ├── README.md                              # Guide to core analytics and research notebooks
│   ├── [CORE MOTION ANALYTICS SUITE]
│   ├── 01_EDA.ipynb                           # Data inspection, dropout repair, and full 90s telemetry
│   ├── 02_Jump_Detection.ipynb                # Ballistic free-fall jump detection & air times
│   ├── 03_Filter_and_PSD.ipynb                # Ground PSD & Butterworth LP filter synthesis
│   ├── 04_Downsampling.ipynb                  # Multi-rate decimation sweep & Anti-Aliasing Filter
│   ├── 05_Data_Generating_Process.ipynb # Data Generating Process: Throttle & braking ODE, fork dive, mid-air brake tap & 3D vibration injection
│   ├── [ADVANCED RESEARCH & KINEMATICS SUITE]
│   ├── Filter_and_PSD_extended.ipynb          # Extended: SysID ARX, Kalman filter, and SOS latency
│   └── EDA_and_Downsampling_extended.ipynb    # Extended: 3D polarization, battery model, shock smearing
│
├── src/                                       # Modular Python source code
│   ├── generate_synthetic_telemetry.py        # First-principles physics simulation & data generator
│   ├── generate_throttle_rpm_notebook.py      # Notebook generator for dynamic throttle & RPM sweeps
│   ├── jump_analytics_pipeline.py             # Standalone unified CLI engine (ingestion, detection, DSP)
│   ├── detect_jumps_and_airtime.py            # Jump detection state machine & metrics extractor
│   ├── design_butterworth_filter_and_psd.py   # Butterworth SOS synthesis & Welch PSD engine
│   ├── downsample_sweep_and_aaf_analysis.py   # Multi-rate decimation & AAF foldover analyzer
│   ├── eda_sensor_telemetry.py                # Exploratory data analysis scripts
│   ├── analyze_engine_vibration_polarization.py # 3D vibration polarization & 56.7° cylinder tilt
│   ├── sysid_arx_modeling_experiment.py       # Parametric ARX system identification modeling
│   ├── kalman_filter_state_estimation.py      # Discrete Kalman state-space estimator
│   ├── estimate_attitude_complementary_filter.py # Attitude estimation (pitch/roll complementary filter)
│   ├── benchmark_dropout_interpolation.py     # Comparison of linear vs spline vs PCHIP interpolation
│   ├── assess_interpolation_regression_errors.py # Sensor regression curve error assessment
│   ├── render_html_reports.py                 # HTML generator with MathJax & GitHub markdown styling
│   ├── build_all_notebooks.py                 # Notebook generation & validation pipeline
│   └── execute_and_populate_notebooks.py      # Automated headless execution & figure populator
│
├── results/                                   # Processed CSV metrics, JSON specs, and snipped datasets
│   ├── phase1_eda/                            # EDA summary statistics and correlation matrices
│   ├── phase2_jump_detection/                 # Detected jump intervals, metrics, and snipped CSVs
│   ├── phase3_filter_design/                  # Ground PSD peaks CSV, filter specs JSON, SysID metrics
│   └── phase4_downsampling/                   # Decimation sweep metrics and power tradeoff JSON
│
└── plots/                                     # Publication-grade figures (300 DPI)
    ├── eda/                                   # EDA overview, dropout inspection, 3D polar kinematics
    ├── objective1/                            # Full-session jump overview and 5 zoomed jump profiles
    ├── objective2/                            # Ground IMU PSD, Butterworth Bode, filtered vs raw
    └── objective3/                            # Detection Pareto frontier, aliasing foldover, battery life
```

---

## 4. Key Engineering Findings & Quantitative Results

### 4.1 Kinematic Jump Detection (Accelerometer-Only)
- **Mathematical Detection Criterion**:
  $$\lVert \mathbf{a}(t) \rVert = \sqrt{a_x(t)^2 + a_y(t)^2 + a_z(t)^2} \le 3.50\,\text{m/s}^2 \quad \text{for duration } \Delta t \ge 0.50\,\text{s}$$
- **Physical Principle**:
  *During airborne flight, external ground reaction forces vanish and the vehicle undergoes an unconstrained ballistic free-fall trajectory, collapsing the net frame acceleration vector from its gravitational baseline ($9.81\,\text{m/s}^2$) toward near-zero ($<3.5\,\text{m/s}^2$). Enforcing a minimum temporal persistence threshold of $\tau_{\text{min}} = 0.50\,\text{s}$ robustly discriminates authentic flight intervals from transient suspension unloading over track crests and high-frequency chassis vibration.*
- **Quantitative Jump Metrics**:
  | Jump # | Takeoff Time $t_{\text{takeoff}}$ (s) | Landing Time $t_{\text{landing}}$ (s) | Air Time $\Delta t_{\text{air}}$ (s) | Avg Accel $\Vert\mathbf{a}\Vert$ ($\text{m/s}^2$) | Avg Vertical $a_z$ ($\text{m/s}^2$) | Peak Landing Deceleration ($a_z$) |
  |:---:|:---:|:---:|:---:|:---:|:---:|:---:|
  | **Jump 1** | $7.000$ | $9.748$ | $\mathbf{2.748}$ | $0.131\,\text{m/s}^2$ ($0.013\,g$) | $+0.003\,\text{m/s}^2$ | $17.91\,\text{m/s}^2$ ($1.83\,g$) |
  | **Jump 2** | $18.400$ | $21.898$ | $\mathbf{3.498}$ | $0.129\,\text{m/s}^2$ ($0.013\,g$) | $+0.005\,\text{m/s}^2$ | $18.42\,\text{m/s}^2$ ($1.88\,g$) |
  | **Jump 3** | $35.600$ | $39.298$ | $\mathbf{3.698}$ | $0.126\,\text{m/s}^2$ ($0.013\,g$) | $-0.001\,\text{m/s}^2$ | $18.54\,\text{m/s}^2$ ($1.89\,g$) |
  | **Jump 4** | $54.600$ | $60.198$ | $\mathbf{5.598}$ | $0.126\,\text{m/s}^2$ ($0.013\,g$) | $+0.004\,\text{m/s}^2$ | $18.07\,\text{m/s}^2$ ($1.84\,g$) |
  | **Jump 5** | $77.000$ | $79.798$ | $\mathbf{2.798}$ | $0.125\,\text{m/s}^2$ ($0.013\,g$) | $+0.003\,\text{m/s}^2$ | $18.25\,\text{m/s}^2$ ($1.86\,g$) |
  | **Total / Mean** | — | — | $\mathbf{18.340\,\text{s}}$ (Mean: $3.668\,\text{s}$) | $\mathbf{0.127\,\text{m/s}^2}$ ($0.013\,g$) | $\mathbf{+0.003\,\text{m/s}^2}$ | $18.24\,\text{m/s}^2$ ($1.86\,g$) |

### 4.2 Spectral Analysis & Digital Filter Synthesis
- **Powertrain Harmonic Peaks (Welch PSD on Ground Segment)**:
  - Fundamental: $f_1 = \mathbf{58.11\,\text{Hz}}$ ($3,487\,\text{RPM}$); $-20.90\,\text{dB/Hz}$ on $a_z$, $-24.47\,\text{dB/Hz}$ on $a_x$.
  - Second Harmonic: $f_2 = 2 f_1 = \mathbf{116.21\,\text{Hz}}$; $-26.29\,\text{dB/Hz}$ on $a_z$, $-29.94\,\text{dB/Hz}$ on $a_x$.
  - Gyroscopes show zero engine vibration peaks (flat noise floor at $-65.3\,\text{dB/Hz}$) due to differential Coriolis tuning-fork CMRR.
- **Butterworth Low-Pass Filter Design**:
  - Specifications: Passband ripple $A_p \le 1.0\,\text{dB}$ at $f_p = 20.0\,\text{Hz}$, stopband attenuation $A_s \ge 30.0\,\text{dB}$ at $f_s = 58.0\,\text{Hz}$.
  - Derived Minimal Order: $N \ge \frac{\log_{10}[(10^3 - 1)/(10^{0.1} - 1)]}{2 \log_{10}(58 / 20)} \approx 3.88 \implies \mathbf{N = 4}$.
  - Cutoff: $f_c = \mathbf{25.0\,\text{Hz}}$ (achieves $-0.73\,\text{dB}$ at $20\,\text{Hz}$, $-30.54\,\text{dB}$ at $58.11\,\text{Hz}$, and $-53.48\,\text{dB}$ at $116.21\,\text{Hz}$).

### 4.3 Multi-Rate Decimation & Anti-Aliasing Filter Analysis
- **Lowest Sampling Rate**:
  - **Kinematic Detection Bound**: $\mathbf{1.0\,\text{Hz}}$ ($100\%$ precision, $100\%$ recall, $F_1 = 1.000$).
  - *Why $1.0\,\text{Hz}$ works*: Shortest jump ($2.748\,\text{s}$) contains $\approx 3$ discrete samples, satisfying multi-sample confirmation ($K \ge 2$).
  - *Why $<1.0\,\text{Hz}$ fails*: At $0.5\,\text{Hz}$ ($\Delta t = 2.0\,\text{s}$), the shortest jump registers only 1 sample, failing anti-chatter confirmation.
  - **Landing Shock Dynamic Bound**: $\mathbf{20.0 - 50.0\,\text{Hz}}$ is required if the $18.2\,\text{m/s}^2$ landing shock impulse ($\tau \approx 20\,\text{ms}$) must be resolved without $>45\%$ temporal peak smearing.
- **Anti-Aliasing Filter (AAF) Verdict**:
  - **MANDATORY for all sampling rates $f_s \le 100.0\,\text{Hz}$**.
  - Without AAF, $58.11\,\text{Hz}$ folds into $8.11\,\text{Hz}$ at $50\,\text{Hz}$ ODR (rider motion band) and into $1.89\,\text{Hz}$ at $20\,\text{Hz}$ ODR (gravity orientation corruption). Pre-filtering with $f_c = 25\,\text{Hz}$ eliminates aliasing tones ($>30\,\text{dB}$ suppression).

---

## 5. Advanced Research & Engineering Deep-Dives

1. **3D Engine Vibration Polarization & Cylinder Angle**:
   Resolving the triaxial acceleration covariance matrix in the sagittal plane confirms that 2-stroke vibration is strongly polarized along the cylinder stroke axis inclined at $\mathbf{56.7^\circ}$ forward lean ($a_z / a_x \approx \tan(56.7^\circ)$).
2. **Parametric System Identification (SysID ARX)**:
   Estimating discrete ARX transfer functions from ground IMU data demonstrates empirical pole clustering at $58.1\,\text{Hz}$, validating the Butterworth digital filter order against data-driven models.
3. **State-Space Discrete Kalman Filter**:
   Formulates a discrete-time kinematic state-space estimator to track chassis acceleration, benchmarking causal phase delay ($22.0\,\text{ms}$) against zero-phase offline smoothing.
4. **IoT Battery Longevity Modeling**:
   Models system current draw across sampling rates for a $250\,\text{mAh}$ LiPo battery. Proves that downsampling to $50\,\text{Hz}$ extends battery life by $>8.5\times$ while preserving full landing shock dynamics.

---

## 6. Quickstart Guide

### 1. Environment Setup:
```bash
# Clone or navigate to the workspace
cd edge-imu-motion-analytics

# Create isolated virtual environment
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### 2. Generate Synthetic Telemetry:
```bash
python3 src/generate_synthetic_telemetry.py --output data/motocross_imu_telemetry.csv
```

### 3. Run Unified Analytics Pipeline:
```bash
python3 src/jump_analytics_pipeline.py --data data/motocross_imu_telemetry.csv
```

### 4. Compile Standalone HTML Reports:
```bash
python3 src/render_html_reports.py
```
This compiles `docs/final_report.html`, `docs/phase3_report.html`, and `docs/phase4_report.html` with full MathJax rendering.

### 5. Rebuild and Validate All Jupyter Notebooks:
```bash
python3 src/build_all_notebooks.py
python3 src/execute_and_populate_notebooks.py
```

---

## 7. Key Documentation Links

- **Master Technical Report**: [`docs/final_report.md`](docs/final_report.md) | [HTML Version](docs/final_report.html)
- **Detailed Phase Reports**:
  - [Spectral Analysis & Filter Design Report](docs/phase3_report.md)
  - [Downsampling & Power Optimization Report](docs/phase4_report.md)
  - [EDA & 2-Stroke Vibration Ingress Summary](docs/eda_summary.md)
- **Mathematical Compendium**:
  - [`docs/MATHEMATICAL_AND_ENGINEERING_CONCEPTS.md`](docs/MATHEMATICAL_AND_ENGINEERING_CONCEPTS.md)
- **Historical Assessment Archive**:
  - [`archive/quin_assessment_original/README.md`](archive/quin_assessment_original/README.md)
