# Detailed Plan: Phase 1 — Exploratory Data Analysis (EDA)

This document specifies the technical plan for **Phase 1: Exploratory Data Analysis (EDA)** on the motocross IMU dataset (`data/motocross_imu_telemetry.csv`). 

---

## 1. Objectives of EDA

1. **Verify Data Integrity & Health**: Audit timestamp uniformity, identify missing/corrupted values, check for ADC clipping/saturation, and assess sensor noise floors.
2. **Establish Physical Domain Mapping**: Correlate each IMU axis with motorcycle dynamics (longitudinal acceleration/braking, cornering lean, pitch angle changes, normal force, free fall, and landing shock).
3. **Statistical State Separation**: Quantify the distributional divergence between ground-riding states ($a_z \approx 9.81\,\text{m/s}^2$) and airborne free-fall states ($a_z \approx 0\,\text{m/s}^2$).
4. **Characterize Sensor Artifacts & Engine Signature**: Formulate baseline observations regarding engine vibration frequencies and amplitude characteristics across axes before formal filter design.
5. **Produce Publication-Grade Visual Assets**: Generate an initial suite of exploratory plots to guide algorithm design in subsequent phases.

---

## 2. Sensor & Coordinate System Specification

The sensor setup consists of a 6-axis IMU rigidly mounted to the motocross frame near the steering headstock.

```
                  +Z (Up)
                   ^
                   |    +X (Forward)
                   |   /
                   |  /
                   | /
                   +--------> +Y (Left)
```

| Sensor Channel | Axis | Physical Orientation | Primary Motion Captured | Units |
| :--- | :---: | :--- | :--- | :---: |
| `accel_x` | $X$ | Longitudinal (Forward) | Throttle acceleration ($+X$), braking deceleration ($-X$) | $\text{m/s}^2$ |
| `accel_y` | $Y$ | Lateral (Leftward) | Cornering centripetal acceleration, bike lean forces | $\text{m/s}^2$ |
| `accel_z` | $Z$ | Vertical (Upward) | Ground normal force ($+1g$), free-fall weightlessness ($0g$), landing impact ($>1g$) | $\text{m/s}^2$ |
| `gyro_x` | $\omega_x$ | Roll Axis | Roll angular velocity (leaning into corners) | $\text{rad/s}$ |
| `gyro_y` | $\omega_y$ | Pitch Axis | Pitch angular velocity (launch ramp elevation, airborne nose trim) | $\text{rad/s}$ |
| `gyro_z` | $\omega_z$ | Yaw Axis | Yaw angular velocity (steering turns, directional heading changes) | $\text{rad/s}$ |

---

## 3. Data Integrity & Anomaly Audit Plan

### 3.1 Sampling Uniformity & Timebase Audit
- Compute timestamp differences $\Delta t_i = t_{i} - t_{i-1}$ across all samples.
- Confirm nominal Output Data Rate (ODR) of $500.0\,\text{Hz}$ ($\Delta t = 2.000\,\text{ms}$).
- Check for dropped frames, timestamp jitter, or clock drift.

### 3.2 Missing Data / Sensor Dropout Audit
- **Identified Anomaly**: Rows $11,500$ to $11,699$ ($200$ samples, $t \in [23.000, 23.398]\,\text{s}$, exact duration $\Delta t = 0.400\,\text{s}$) contain empty string tokens (`23.0,,,,,,`).
- **Context Assessment**: This dropout is situated between Jump 2 (ends at $21.900\,\text{s}$) and Jump 3 (starts at $35.600\,\text{s}$). The bike is on flat ground before ($t = 22.998\,\text{s}$, $a_z = 9.63\,\text{m/s}^2$) and after ($t = 23.400\,\text{s}$, $a_z = 10.21\,\text{m/s}^2$).
- **Imputation Strategy for EDA & Downstream Tasks**:
  - Compare **Linear Interpolation**, **Forward Fill (Last Observation Carried Forward)**, and **Cubic Spline**.
  - Standardize on linear interpolation across the $0.4\,\text{s}$ gap to maintain signal continuity for IIR filter stability while clearly flagging this synthetic bridge.

### 3.3 Dynamic Range & Sensor Saturation Check
- Inspect minimum and maximum values across all channels to detect potential ADC clipping:
  - Typical accelerometer ranges: $\pm 2g$ ($\approx \pm 19.6\,\text{m/s}^2$), $\pm 4g$ ($\approx \pm 39.2\,\text{m/s}^2$), $\pm 8g$, $\pm 16g$.
  - Check whether landing peaks ($a_z \approx 18.66\,\text{m/s}^2$) approached the sensor's full-scale limit.
  - Gyroscope limits: check maximum angular rates against typical $\pm 250^\circ/\text{s}$ ($\approx 4.36\,\text{rad/s}$) or $\pm 500^\circ/\text{s}$ limits.

---

## 4. Statistical Characterization Plan

We will compute two distinct sets of statistical metrics: **Global Statistics** (across all 90 seconds) and **Segmented State Statistics** (Ground vs Airborne).

### 4.1 Global Metric Profile
For each channel ($a_x, a_y, a_z, g_x, g_y, g_z$) and vector magnitudes ($\|\vec{a}\|, \|\vec{\omega}\|$):
- Parametric: Mean, Standard Deviation, Variance, Skewness, Kurtosis.
- Non-parametric: Minimum, 5th Percentile ($P_5$), Median ($P_{50}$), 95th Percentile ($P_{95}$), Maximum, Interquartile Range (IQR).

### 4.2 Segmented State Decomposition
Partition the dataset into:
1. **Ground Riding Segments**: $\sim 71.65\,\text{s}$ total (inter-jump riding).
2. **Airborne Jump Segments**: Exactly $18.35\,\text{s}$ total across 5 jumps.
3. **Dropout Segment**: $0.40\,\text{s}$.

Compute and contrast:
- Mean and standard deviation of $a_z$ on ground ($\approx 9.81\,\text{m/s}^2$, $\sigma \approx 0.21\,\text{m/s}^2$) versus in air ($\approx 0.00\,\text{m/s}^2$, $\sigma \approx 0.13\,\text{m/s}^2$).
- Pitch angular velocity ($g_y$) dynamics: Mean on ground ($\approx 0.00\,\text{rad/s}$) versus persistent positive bias during flight ($\approx +0.28\,\text{rad/s}$ due to ballistic parabolic pitching).
- Cross-axis Pearson correlation matrix to reveal physical coupling between channels.

---

## 5. Physical Dynamics & Jump Footprint Analysis

### 5.1 Free-Fall Kinematic Signature
- During flight, the external contact forces vanish:
  $$\vec{F}_{\text{net}} = m \vec{g} + \vec{F}_{\text{contact}} = m \vec{a}_{\text{inertial}}$$
  Since an accelerometer measures specific force $\vec{f} = \frac{\vec{F}_{\text{contact}}}{m} = \vec{a}_{\text{inertial}} - \vec{g}$, in free fall $\vec{F}_{\text{contact}} = \vec{0} \implies \vec{f} \approx \vec{0}$.
- Vertical channel drops precipitously from $\sim 9.81\,\text{m/s}^2$ to near $0.0\,\text{m/s}^2$.

### 5.2 Takeoff & Landing Transition Profiles
- **Takeoff Edge**: Transition duration, slope $da_z/dt$, presence of launch ramp pre-compression.
- **Landing Impact**: Impact duration ($\sim 30\,\text{ms}$), peak acceleration ($18.0 - 18.7\,\text{m/s}^2$), suspension damping rebound oscillations.

---

## 6. Planned EDA Visualizations

We will generate 5 high-resolution figures saved in `plots/eda/`:

1. **`01_full_session_overview.png`**:
   - 6-panel stacked time-series plot of all IMU channels for the full 90 seconds.
   - Highlights the 5 jump regions and the sensor dropout region with distinct background shading.
2. **`02_sensor_dropout_inspection.png`**:
   - Detailed zoom-in of $t \in [22.8, 23.6]\,\text{s}$ displaying the $0.4\,\text{s}$ missing data window.
   - Compares raw missing points against the linear interpolation reconstruction.
3. **`03_ground_vs_air_distributions.png`**:
   - Kernel Density Estimates (KDE) and histograms of $a_z$, $\|\vec{a}\|$, and $g_y$ comparing ground vs flight states.
   - Clearly illustrates the bimodal separation between $0g$ free-fall and $1g$ ground contact.
4. **`04_imu_correlation_matrix.png`**:
   - Pearson correlation heatmap between all 6 channels and magnitude vectors.
5. **`05_phase_plane_trajectories.png`**:
   - Phase-plane portrait: $a_z(t)$ vs $g_y(t)$ (Vertical Acceleration vs Pitch Rate).
   - Visualizes the characteristic trajectory loop that defines a complete jump cycle: Approach $\to$ Takeoff $\to$ Air $\to$ Landing Impact $\to$ Settling.

---

## 7. Deliverables & Execution Plan for Phase 1

1. **Analysis Script**: [`src/eda_sensor_telemetry.py`](../../src/eda_sensor_telemetry.py) implementing all data ingestion, metrics calculation, and figure generation routines.
2. **Figure Directory**: `plots/eda/` containing all 5 generated 300-DPI charts.
3. **Summary Document**: A structured markdown table summarizing all statistical findings, data anomalies, and engineering takeaways.
4. **User Review Checkpoint**: Present the Phase 1 EDA findings to the user for sign-off before proceeding to Phase 2 (Objective 1: Jump Detection Algorithm).

