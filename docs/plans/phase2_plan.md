# Detailed Plan: Phase 2 — Objective 1: Jump Detection Algorithm

This document outlines the engineering plan, physical assumptions, mathematical equations, caveats, and execution strategy for **Phase 2: Objective 1 (Jump Detection Algorithm)** as specified in the **Motocross Jump Analytics & Powertrain Vibration Attenuation** specification.

---

## 1. Objective 1 Requirements & Deliverables

According to the specification:
1. **Accelerometer-Only Detection**: Detect all jumps in `data/motocross_imu_telemetry.csv` using *only* accelerometer channels ($a_x, a_y, a_z$ or vector magnitude $\|\mathbf{a}\|$); no gyroscope channels may be used for detection.
2. **Mathematical Definition & Justification**: Define the detection criterion mathematically and justify it in **exactly two sentences**.
3. **Event Graphs**: Generate high-resolution takeoff and landing graphs for all identified jumps.
4. **Air Times Table**: Compute and report exact takeoff time ($s$), landing time ($s$), and air time ($\Delta t_{\text{air}}$ in seconds) for each jump.
5. **Jump Data Snipping**: Snip out individual jump intervals from the session data and export clean CSV datasets for each jump (both pure in-flight window and context-buffered window covering approach and landing rebound).
6. **Jump Dynamics Distribution Analysis**: Compute complete statistical distribution properties (mean, std, variance, median, IQR, skewness, kurtosis, percentiles, integrated pitch angle, peak impact shock) across all channels for each jump and aggregated.
7. **Dedicated Output Structure**: Store all generated artifacts (tabular metrics, snipped datasets, event plots, distribution analysis) in [`results/phase2_jump_detection/`](../../results/phase2_jump_detection/).
8. **Interactive Jupyter Notebook**: Provide a fully executable Jupyter Notebook [`notebooks/02_Jump_Detection.ipynb`](../../notebooks/02_Jump_Detection.ipynb) alongside the modular Python production script [`src/detect_jumps_and_airtime.py`](../../src/detect_jumps_and_airtime.py).

---

## 2. Physical Principles & Assumptions

### 2.1 Accelerometer Specific Force Principle
MEMS accelerometers measure **proper acceleration** (specific contact force per unit mass $\mathbf{f}$), not inertial coordinate acceleration $\mathbf{a}_{\text{inertial}}$:
$$\mathbf{f}(t) = \frac{\mathbf{F}_{\text{contact}}(t)}{m} = \mathbf{a}_{\text{inertial}}(t) - \mathbf{g}$$
where $\mathbf{g} = [0, 0, -9.81]^T\,\text{m/s}^2$ in the local Earth frame.

### 2.2 Ground Riding Contact State
- **Wheel Contact**: While riding on dirt, tires and suspension transmit normal reaction forces supporting the motorcycle and rider against gravity:
  $$\mathbf{F}_{\text{contact}} \approx m (-\mathbf{g}) \implies \mathbf{f} \approx +9.81\,\text{m/s}^2 \quad (\text{along } +Z \text{ axis})$$
- **Measured Envelope**: In ground riding, $a_z(t) = 9.818 \pm 0.336\,\text{m/s}^2$. Engine vibration adds a $\sim 1.0\,\text{m/s}^2$ peak-to-peak ripple ($58\,\text{Hz}$), causing the ground acceleration envelope to oscillate strictly within $[8.5, 11.2]\,\text{m/s}^2$.

### 2.3 Airborne Free-Fall State
- **Loss of Contact**: The instant tires lose contact with the launch lip:
  $$\mathbf{F}_{\text{contact}}(t) = \mathbf{0} \implies \mathbf{f}(t) = \mathbf{0}\,\text{m/s}^2$$
- **Orientation Invariance**: Because specific force is identically zero in free fall, this zero-reading holds regardless of whether the motorcycle pitches forward, rolls, or whips sideways:
  $$\|\mathbf{a}(t)\| = \sqrt{a_x(t)^2 + a_y(t)^2 + a_z(t)^2} \approx 0.0\,\text{m/s}^2$$
- **Measured Envelope**: In airborne flight, $a_z(t) = 0.009 \pm 0.262\,\text{m/s}^2$, and $\|\mathbf{a}(t)\| = 0.132 \pm 0.237\,\text{m/s}^2$.

### 2.4 Landing Impact Dynamics
- **Touchdown Shock**: Landing re-establishes contact. Within $16\,\text{ms}$ of tire contact, the suspension compresses rapidly to absorb vertical kinetic energy:
  $$F_{\text{contact}} = k \Delta z + c \dot{z} \gg mg \implies a_z \in [18.0, 18.7]\,\text{m/s}^2$$
- **Rebound & Settling**: After peak shock, suspension rebound damping dissipates residual energy over $150 - 300\,\text{ms}$, settling back into steady-state ground riding ($9.81\,\text{m/s}^2$).

---

## 3. Mathematical Detection Criteria & State Machine

### 3.1 Signal Conditioning (Anti-Vibration Boxcar Pre-filter)
To eliminate high-frequency $58.11\,\text{Hz}$ engine ripple ($T_{\text{engine}} = 17.2\,\text{ms}$) without distorting jump transition edges, raw acceleration $a_z(t)$ is passed through a moving average boxcar filter of length $W = 11$ samples ($22.0\,\text{ms} \approx 1.28\,T_{\text{engine}}$):
$$\tilde{a}_z[k] = \frac{1}{W} \sum_{j = -\lfloor W/2 \rfloor}^{\lfloor W/2 \rfloor} a_z[k + j]$$
This attenuates the $58\,\text{Hz}$ ripple by $> 20\,\text{dB}$ while introducing a group delay of only $\frac{W-1}{2} = 5$ samples ($10.0\,\text{ms}$), which is compensated symmetrically during offline batch processing via two-pass zero-phase filtering.

### 3.2 Dual-Threshold Hysteresis State Machine
To avoid chatter near the transition thresholds, we implement a 2-state discrete automaton:

$$\text{State}[k] = \begin{cases}
\text{AIRBORNE} & \text{if } \text{State}[k-1] = \text{GROUND} \text{ and } \tilde{a}_z[k] < \tau_{\text{takeoff}} \\
\text{GROUND} & \text{if } \text{State}[k-1] = \text{AIRBORNE} \text{ and } \tilde{a}_z[k] \ge \tau_{\text{landing}} \\
\text{State}[k-1] & \text{otherwise}
\end{cases}$$

- **Takeoff Threshold**: $\tau_{\text{takeoff}} = 2.0\,\text{m/s}^2$
  - Separation from ground mean ($9.82\,\text{m/s}^2$): $\Delta = 7.82\,\text{m/s}^2 > 23\sigma_{\text{ground}}$.
  - Separation from airborne mean ($0.01\,\text{m/s}^2$): $\Delta = 1.99\,\text{m/s}^2 > 7.6\sigma_{\text{air}}$.
- **Landing Threshold**: $\tau_{\text{landing}} = 5.0\,\text{m/s}^2$
  - Hysteresis band $\Delta \tau = \tau_{\text{landing}} - \tau_{\text{takeoff}} = 3.0\,\text{m/s}^2$ guarantees chatter immunity.
  - A secondary impact condition detects the suspension shock peak: $\max(\tilde{a}_z) > 12.0\,\text{m/s}^2$ within $50\,\text{ms}$ of touchdown.

### 3.3 Temporal Validity Constraints
A candidate airborne interval $[t_{\text{start}}, t_{\text{end}}]$ is confirmed as a valid motocross jump if and only if:
1. **Minimum Air Time**:
   $$\Delta t_{\text{air}} = t_{\text{end}} - t_{\text{start}} \ge T_{\text{min}} = 0.500\,\text{s} \quad (250 \text{ samples})$$
   (Rejects brief wheel hops, curb drops, and terrain chatter).
2. **Landing Shock Confirmation**:
   $$\max_{t \in [t_{\text{end}}, t_{\text{end}} + 0.1\,\text{s}]} a_z(t) \ge \tau_{\text{shock}} = 14.0\,\text{m/s}^2$$
   (Confirms touchdown was accompanied by vehicle landing deceleration).

### 3.4 Exact Two-Sentence Physical Justification (Prompt Requirement)
> *"During airborne flight, contact forces vanish and the IMU is in free fall, causing the accelerometer's measured proper acceleration (specific force) to plunge from the $1g$ ($9.81\,\text{m/s}^2$) ground normal force down to near zero ($< 0.5g$). Takeoff is identified at the rapid transition below the free-fall threshold, while landing is marked by the re-establishment of ground contact and the subsequent high-$g$ suspension compression shock."*

---

## 4. Caveats, Edge Cases & Robustness Considerations

| Caveat / Edge Case | Physical Origin | Failure Risk | Algorithmic Mitigation |
| :--- | :--- | :--- | :--- |
| **Engine Vibration Ripple** | 2-stroke unbalance ($58\,\text{Hz}$) produces $\pm 0.5\,\text{m/s}^2$ sinusoidal oscillation on $a_z$. | High threshold ($\tau > 8\,\text{m/s}^2$) could trigger false takeoff on trough. | Zero-phase boxcar filter eliminates ripple; $\tau = 2.0\,\text{m/s}^2$ provides $>7\sigma$ clearance. |
| **Sensor Dropout ($t = 23\,\text{s}$)** | 200 rows of missing/blank tokens in `data/motocross_imu_telemetry.csv`. | Unhandled `NaN` or zero-fill could be mistaken for $0g$ free-fall. | Linear interpolation bridges the gap; flagged as ground segment ($a_z \approx 9.8\,\text{m/s}^2$ before and after). |
| **Launch Ramp Pre-load** | Rider pre-loads suspension before takeoff lip ("seat bounce"). | Rapid $a_z$ spike ($> 12\,\text{m/s}^2$) prior to takeoff. | Hysteresis state machine ignores pre-load upward spikes; triggers strictly on the subsequent drop below $2.0\,\text{m/s}^2$. |
| **Airborne Body Attitude Changes** | Pitching forward or whipping sideways alters body-frame $Z$ alignment. | $a_z$ could deviate if gravitational vector rotated. | In true free fall, contact force is $\mathbf{0}$ along *all* axes; vector norm $\Vert\mathbf{a}\Vert \approx 0$ provides attitude-invariant backup. |
| **Touchdown Timing Ambiguity** | Tire initial contact precedes suspension bottoming shock peak by $\approx 16\,\text{ms}$. | Inconsistent air-time definition across algorithms. | Explicitly define and report **Initial Touchdown** ($t$ where $a_z$ crosses $\tau_{\text{landing}}$) and **Peak Shock Impact** ($t$ of max $a_z$). Air time is computed to Initial Touchdown. |
| **Terrain Bumps & Rut Chatter** | High-speed terrain chop can cause momentary wheel unweighting ($< 150\,\text{ms}$). | Spurious micro-jumps detected on rough straightaways. | Temporal duration filter enforces $\Delta t_{\text{air}} \ge 0.500\,\text{s}$. |

---

## 5. Output Architecture & Directory Organization

All Phase 2 artifacts will be generated into a self-contained results directory:

```
results/phase2_jump_detection/
├── jump_metrics.csv                 # Machine-readable tabular metrics (takeoff, landing, air time, shock peak)
├── jump_metrics.json                # JSON serialization of jump intervals and algorithm parameters
├── jump_dynamics_distributions.csv  # Statistical distribution properties (parametric & non-parametric) for all jumps
├── snipped_jumps/                   # Snipped jump intervals
│   ├── jump_1_inflight.csv          # Pure airborne samples
│   ├── jump_1_context.csv           # Flight + 1s approach and 1s landing context
│   ├── jump_2_inflight.csv
│   ├── jump_2_context.csv
│   ├── jump_3_inflight.csv
│   ├── jump_3_context.csv
│   ├── jump_4_inflight.csv
│   ├── jump_4_context.csv
│   ├── jump_5_inflight.csv
│   └── jump_5_context.csv
├── plots/
│   ├── 01_full_session_detections.png       # 90-second session overview showing all 5 detected jumps
│   ├── 02_jump_1_detail.png                 # Zoomed view of Jump 1 takeoff, flight, landing
│   ├── 03_jump_2_detail.png                 # Zoomed view of Jump 2
│   ├── 04_jump_3_detail.png                 # Zoomed view of Jump 3
│   ├── 05_jump_4_detail.png                 # Zoomed view of Jump 4
│   ├── 06_jump_5_detail.png                 # Zoomed view of Jump 5
│   ├── 07_jump_comparison_overlay.png      # Multi-panel aligned comparison of all 5 jumps
│   └── 08_jump_dynamics_distributions.png   # Violin / boxplot distributions of jump dynamics across channels
└── phase2_summary.md                # Markdown technical summary of detection results
```

In addition:
- Production Script: [`src/detect_jumps_and_airtime.py`](../../src/detect_jumps_and_airtime.py) (CLI-callable, modular, with snipping and distribution calculations).
- Jupyter Notebook: [`notebooks/02_Jump_Detection.ipynb`](../../notebooks/02_Jump_Detection.ipynb) (Self-contained, interactive, executable).

---

## 6. Execution & Verification Steps

1. **Implement `src/detect_jumps_and_airtime.py`**:
   - Clean data loader with dropout imputation.
   - Dual-threshold hysteresis state machine with boxcar pre-filter.
   - High-precision timestamp extraction for takeoff and landing.
   - Matplotlib visualization routines generating 7 distinct high-resolution plots.
   - Export metrics to CSV, JSON, and Markdown.
2. **Execute Script & Verify Output**:
   - Confirm exactly 5 jumps detected.
   - Verify zero false positives across all 71.65 seconds of ground riding.
   - Check millisecond-level precision of takeoff and landing transitions.
3. **Generate Jupyter Notebook `notebooks/02_Jump_Detection.ipynb`**:
   - Construct valid JSON `.ipynb` structure with code and documentation cells.
   - Test notebook execution.
4. **Compile Phase 2 Summary Document**:
   - Document quantitative findings and visual evidence in `phase2_summary.md`.

