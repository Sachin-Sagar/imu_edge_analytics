# Mathematical and Engineering Concepts Compendium
**Motocross Jump Analytics & Embedded Telemetry Signal Processing Engine**  
*Rigidly-Mounted Frame IMU Telemetry on a 2-Stroke Single-Cylinder Motocross Motorcycle*

---

## 1. Executive Overview & System Architecture

This compendium synthesizes every mathematical derivation, physical principle, digital signal processing (DSP) architecture, and embedded hardware trade-off developed throughout the **Motocross Jump Analytics Engine** (`quin_task`). 

```
                                  +-------------------------------------------------------+
                                  | 2-Stroke Single-Cylinder Engine & Reciprocating Mass  |
                                  | Primary: 58.11 Hz | Secondary: 116.21 Hz              |
                                  +-------------------------------------------------------+
                                                              |
                                                              | Structural Sagittal Projection
                                                              v
+------------------------+        +-------------------------------------------------------+
| Free-Fall Ballistics   | -----> | Rigidly Mounted 6-Axis Frame IMU (Headstock @ 500 Hz) |
| ||a|| -> 0 m/s^2       |        | a_x, a_y, a_z (Translational) | w_x, w_y, w_z (Rot)   |
+------------------------+        +-------------------------------------------------------+
                                         |                                  |
               +-------------------------+                                  +----------------------+
               |                                                                                   |
               v                                                                                   v
+-------------------------------+                                           +------------------------------------+
|  Kinematic Analytics Engine   |                                           | Transducer Physics Rejection       |
|  - Euclidean Norm Invariance  |                                           | - Proof-Mass vs Coriolis Sensing   |
|  - Dual-Threshold Hysteresis  |                                           | - Differential Tuning-Fork CMRR    |
|  - Air Time & Shock Profiling |                                           | - Synchronous Demodulation Carrier |
|  - Attitude KF & Comp Filter  |                                           | - Torsional Rigidity Isolation     |
|  - GPS/INS 15-State ESKF      |                                           | - Viscoelastic Stress Relaxation   |
+-------------------------------+                                           +------------------------------------+
               |                                                                                   |
               v                                                                                   v
+-------------------------------+                                           +------------------------------------+
| Digital Filter Synthesis      |                                           | Downsampling & Edge Optimization   |
| - Welch PSD Estimation        |                                           | - Shannon-Nyquist vs Event Limits  |
| - Bilinear Butterworth LP     |                                           | - Nyquist Folding Aliasing Math    |
| - DF2T Zero-Phase filtfilt    |                                           | - Shock Impulse Smearing Model     |
| - SysID Parametric Notch      |                                           | - Battery Longevity Pareto Model   |
| - State-Space Kalman & RTS    |                                           | - IoT Edge Pareto Sweet Spot       |
| - Adaptive Harmonic EKF (RPM) |                                           | - Dynamic Pole-Placement Engine    |
+-------------------------------+                                           +------------------------------------+
```

### Telemetry & Coordinate Frame Specification
- **Sampling Frequency**: Nominal $f_s = 500.00\,\text{Hz}$ ($\Delta t = 2.000\,\text{ms}$, $\sigma_{\Delta t} = 0\,\mu\text{s}$ across $N = 45{,}000$ samples, $T = 90.0\,\text{s}$).
- **Sensor Placement**: Bolted rigidly to the aluminum perimeter frame near the steering headstock.
- **Right-Handed Coordinate Reference Frame**:
  - **$X$-axis (Longitudinal)**: Directed forward along the chassis centerline.
  - **$Y$-axis (Lateral / Transverse)**: Directed out the left-hand side of the motorcycle.
  - **$Z$-axis (Vertical / Normal)**: Directed upwards, orthogonal to the chassis plane.

---

## 2. Kinematics, Dynamics & Jump Mechanics (Objective 1)

### 2.1 Gravitational Proper Acceleration vs Coordinate Acceleration
An inertial measurement unit does not measure kinematic coordinate acceleration $\ddot{\mathbf{r}}(t) = \frac{d^2 \mathbf{r}}{dt^2}$ directly. Instead, it measures **proper acceleration** (specific force) $\mathbf{a}(t)$:
$$\mathbf{a}(t) = \ddot{\mathbf{r}}(t) - \mathbf{g}$$
where $\mathbf{g} = [0, 0, -9.80665]^T\,\text{m/s}^2$ is the gravitational acceleration vector.

- **Static Ground Contact**: When resting or riding on flat ground with zero vertical acceleration ($\ddot{z} = 0$):
  $$a_z = 0 - (-g) = +g \approx +9.81\,\text{m/s}^2$$
  The sensor reads the normal reaction force exerted by the ground through the motorcycle suspension.
- **Ballistic Airborne Flight (Einstein's Equivalence Principle)**: During airborne flight over a motocross obstacle, aerodynamic drag is minimal relative to chassis mass, and the motorcycle accelerates downward under gravity alone:
  $$\ddot{\mathbf{r}}(t) = \mathbf{g}$$
  Substituting this into the proper acceleration equation:
  $$\mathbf{a}(t) = \mathbf{g} - \mathbf{g} = \mathbf{0}\,\text{m/s}^2$$
  The sensor enters a state of **apparent weightlessness** where proper acceleration along all three axes vanishes.

### 2.2 Rotational Invariance via Euclidean Acceleration Norm
In motocross jumping, the rider and chassis undergo continuous pitch ($\theta$), roll ($\phi$), and yaw ($\psi$) rotations governed by dynamic rotation matrix $\mathbf{R}_b^n(t) \in \text{SO}(3)$:
$$\mathbf{a}_{\text{body}}(t) = \mathbf{R}_n^b(t) \, \mathbf{a}_{\text{nav}}(t)$$
Detecting jumps using vertical acceleration $a_z$ alone is prone to false triggers during steep hill climbs, whoops, or hard banking, where $a_z = g \cos(\theta)\cos(\phi)$ dips. 

To achieve coordinate-rotation invariance, the jump detection algorithm is formulated on the **Euclidean Norm (L2 norm)**:
$$\|\mathbf{a}(t)\| = \sqrt{a_x^2(t) + a_y^2(t) + a_z^2(t)}$$
Because the rotation matrix $\mathbf{R}_b^n(t)$ is orthogonal ($\mathbf{R}^T \mathbf{R} = \mathbf{I}$), the L2 norm is strictly invariant under any 3D body orientation:
$$\|\mathbf{a}_{\text{body}}(t)\| = \sqrt{\mathbf{a}^T \mathbf{R} \mathbf{R}^T \mathbf{a}} = \sqrt{\mathbf{a}^T \mathbf{a}} = \|\mathbf{a}_{\text{nav}}(t)\|$$
- Ground Riding: $\|\mathbf{a}\| \approx 9.81\,\text{m/s}^2$ (with transient spikes up to $\approx 15 - 18\,\text{m/s}^2$).
- Airborne Flight: $\|\mathbf{a}\| \approx 0.05 - 0.25\,\text{m/s}^2$ (sharp bimodal distribution).

### 2.3 Finite State Machine (FSM) & Dual-Threshold Hysteresis
To prevent chattering and false edge triggers caused by high-frequency engine vibration or suspension chatter, detection is governed by a **Dual-Threshold Hysteresis State Machine** operating on pre-filtered acceleration:

```
               +------------------------------------------------+
               |                  GROUND STATE                  |
               |             ||a(t)|| >= 6.0 m/s^2              |
               +------------------------------------------------+
                        |                              ^
                        | Takeoff Trigger:             | Landing Trigger:
                        | ||a(t)|| <= a_takeoff        | ||a(t)|| >= a_land
                        | persists for tau_takeoff     | persists for tau_land
                        v                              |
               +------------------------------------------------+
               |                 AIRBORNE STATE                 |
               |             ||a(t)|| <= 2.0 m/s^2              |
               +------------------------------------------------+
```

#### Deterministic Threshold Equations:
1. **Takeoff Transition ($S_{\text{ground}} \to S_{\text{air}}$)**:
   $$\|\mathbf{a}(t)\| \le a_{\text{takeoff}} \quad \forall t \in [t_0, t_0 + \tau_{\text{takeoff}}]$$
   - Threshold: $a_{\text{takeoff}} = 2.0\,\text{m/s}^2$ ($0.204\,g$, comfortably below ground mean $9.81\,\text{m/s}^2$).
   - Window: $\tau_{\text{takeoff}} = 30\,\text{ms}$ ($15$ consecutive samples @ $500\,\text{Hz}$).
2. **Landing Transition ($S_{\text{air}} \to S_{\text{ground}}$)**:
   $$\|\mathbf{a}(t)\| \ge a_{\text{landing}} \quad \forall t \in [t_1, t_1 + \tau_{\text{landing}}]$$
   - Threshold: $a_{\text{landing}} = 6.0\,\text{m/s}^2$ ($0.612\,g$).
   - Window: $\tau_{\text{landing}} = 10\,\text{ms}$ ($5$ consecutive samples @ $500\,\text{Hz}$).
3. **Temporal Validation Constraints**:
   - Minimum Air Time: $\tau_{\text{air,min}} = 300\,\text{ms}$ (filters terrain bumps and short curb drops).
   - Refractory Lockout: $\tau_{\text{refractory}} = 1.0\,\text{s}$ (prevents double-triggering on landing suspension rebound).

### 2.4 Landing Touchdown Shock & Momentum Transfer Integral
Upon touchdown, the chassis vertical velocity $\Delta v_z$ is arrested by tyre deformation and suspension damper compression over an impulse window $\Delta t_{\text{impulse}} \approx 25\,\text{ms}$. By Newton's Second Law and the Impulse-Momentum Theorem:
$$\mathbf{J} = \int_{t_{\text{touchdown}}}^{t_{\text{rebound}}} \mathbf{F}_{\text{net}}(t) \, dt = m_{\text{total}} \Delta \mathbf{v}$$
Expressed in measured proper acceleration:
$$\Delta v_z = \int_{t_{\text{touchdown}}}^{t_{\text{touchdown}} + \Delta t_{\text{impulse}}} (a_z(t) - g) \, dt$$
This delivers an intense landing shock transient:
- Peak deceleration: $a_{z,\text{peak}} \approx 18.0 - 18.66\,\text{m/s}^2$ ($1.84 - 1.90\,g$).
- Peak shock latency: Occurs exactly $16 - 20\,\text{ms}$ after initial tyre touchdown.

### 2.5 Quantitative Flight Summary
Across the $90.0\,\text{s}$ session, exactly $5$ jumps were deterministically identified:

| Jump ID | Takeoff Time ($t_{\text{to}}$) | Touchdown Time ($t_{\text{td}}$) | Peak Shock Time ($t_{\text{shock}}$) | Air Time ($\Delta t_{\text{air}}$) | Peak $a_z$ Deceleration |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **Jump 1** | $17.436\,\text{s}$ | $18.528\,\text{s}$ | $18.544\,\text{s}$ | **$1.092\,\text{s}$** | $18.15\,\text{m/s}^2$ |
| **Jump 2** | $30.824\,\text{s}$ | $31.838\,\text{s}$ | $31.854\,\text{s}$ | **$1.014\,\text{s}$** | $18.66\,\text{m/s}^2$ |
| **Jump 3** | $42.748\,\text{s}$ | $43.682\,\text{s}$ | $43.702\,\text{s}$ | **$0.934\,\text{s}$** | $18.54\,\text{m/s}^2$ |
| **Jump 4** | $55.452\,\text{s}$ | $60.198\,\text{s}$ | $60.218\,\text{s}$ | **$4.746\,\text{s}$** | $18.04\,\text{m/s}^2$ |
| **Jump 5** | $74.802\,\text{s}$ | $75.928\,\text{s}$ | $75.946\,\text{s}$ | **$1.126\,\text{s}$** | $18.42\,\text{m/s}^2$ |
| **Total** | — | — | — | **$8.912\,\text{s}$** ($9.90\%$ of session) | — |

---

## 3. Powertrain Kinematics & Vibration Polarization

### 3.1 2-Stroke Slider-Crank Reciprocating Mechanics
The motorcycle is powered by a single-cylinder 2-stroke internal combustion engine. In a slider-crank kinematic chain, piston displacement $x(t)$ relative to top dead center (TDC) is non-linear due to connecting-rod angularity:
$$x(t) = r \left( (1 - \cos(\omega t)) + \frac{l}{r} \left( 1 - \sqrt{1 - \left(\frac{r}{l}\right)^2 \sin^2(\omega t)} \right) \right)$$
Using a Taylor binomial expansion for $r/l \ll 1$ (typically $r/l \approx 0.25 - 0.30$):
$$x(t) \approx r \left( (1 - \cos(\omega t)) + \frac{r}{2l} \sin^2(\omega t) \right) = r \left( 1 - \cos(\omega t) + \frac{r}{4l} (1 - \cos(2\omega t)) \right)$$
Differentiating twice with respect to time yields the reciprocating acceleration:
$$\ddot{x}(t) = r \omega^2 \left( \cos(\omega t) + \frac{r}{l} \cos(2\omega t) \right)$$
By D'Alembert's principle, the unbalanced inertial reaction force acting on the engine cylinder block along the stroke line is:
$$F_{\text{recip}}(t) = m_{\text{recip}} \, r \, \omega^2 \left( \cos(\omega t) + \frac{r}{l} \cos(2\omega t) \right)$$
where:
- $m_{\text{recip}}$ is the combined reciprocating mass (piston, gudgeon pin, rings, small-end bearing),
- $r$ is the crankshaft throw radius ($r = \text{Stroke}/2$),
- $l$ is the connecting rod center-to-center length,
- $\omega = 2\pi f_1$ is the angular crankshaft velocity.

### 3.2 Fundamental & Harmonic Spectral Relationships
At the empirical engine operating speed of $N_{\text{rpm}} \approx 3{,}487\,\text{RPM}$:
- **Primary Fundamental ($f_1$)**:
  $$f_1 = \frac{N_{\text{rpm}}}{60} = \frac{3{,}487}{60} = \mathbf{58.11\,\text{Hz}}$$
- **Secondary Harmonic ($f_2$)**:
  $$f_2 = 2 f_1 = \mathbf{116.21\,\text{Hz}}$$
Because a single-cylinder 2-stroke engine fires once per revolution ($360^\circ$ crankshaft cycle), the primary reciprocating unbalance occurs at exactly the crankshaft speed.

### 3.3 2D Sagittal Vibration Polarization & Cylinder Tilt Angle
The engine cylinder bore is inclined within the motorcycle's longitudinal-vertical sagittal plane ($x-z$), with zero tilt in the lateral direction ($y$). 

Let $\theta_{\text{cyl}}$ represent the cylinder tilt angle measured from the horizontal chassis centerline ($X$-axis):
$$F_x(t) = F_{\text{recip}}(t) \cos(\theta_{\text{cyl}}), \quad F_z(t) = F_{\text{recip}}(t) \sin(\theta_{\text{cyl}}), \quad F_y(t) = 0$$

Isolating the fundamental engine tone via a narrowband bandpass filter ($54 - 62\,\text{Hz}$) yields the measured root-mean-square (RMS) vibration amplitudes:
- $\text{RMS}(a_{x,\text{engine}}) = 0.0511\,\text{m/s}^2$
- $\text{RMS}(a_{z,\text{engine}}) = 0.0773\,\text{m/s}^2$
- $\text{RMS}(a_{y,\text{engine}}) = 0.0031\,\text{m/s}^2$

The physical inclination angle of the cylinder bore relative to the chassis frame is derived directly:
$$\theta_{\text{incline}} = \arctan\left( \frac{\text{RMS}(a_{z,\text{engine}})}{\text{RMS}(a_{x,\text{engine}})} \right) = \arctan\left( \frac{0.0773}{0.0511} \right) = \mathbf{56.5^\circ \pm 0.2^\circ}$$
- **Sagittal Ingress**: Accelerometer axes $X$ and $Z$ experience severe harmonic line ingress ($-24.85\,\text{dB}$ and $-21.11\,\text{dB}$).
- **Transverse Decoupling**: Accelerometer axis $Y$ receives zero direct piston motion, showing **$>25.2\,\text{dB}$ to $28.9\,\text{dB}$ greater isolation**.

---

## 4. Transducer Physics & Gyroscope Noise Immunity

A foundational question in telemetry analysis: *Why do the accelerometer channels experience violent engine harmonic spikes while all three gyroscope channels exhibit a completely flat white-noise floor ($-65.3\,\text{dB/Hz}$)?*

This immunity is governed by four cascading physical and electrical mechanisms:

### 4.1 Linear Force Sensing vs Coriolis Sensing
1. **MEMS Accelerometer (Direct Proof-Mass Displacement)**:
   Measures the displacement $x_{\text{rel}}(t)$ of a suspended proof mass $m$ supported by silicon flexure beams with spring constant $k$ and damping $c$:
   $$m \ddot{x}_{\text{rel}} + c \dot{x}_{\text{rel}} + k x_{\text{rel}} = -m a_{\text{frame}}(t)$$
   Any external linear frame vibration $a_{\text{frame}}(t)$ at $58\,\text{Hz}$ directly drives $x_{\text{rel}}$, causing baseband modulation of the capacitive sensing combs.
2. **MEMS Gyroscope (Coriolis Effect Sensing)**:
   Does not sense linear translation. It senses **Coriolis acceleration**:
   $$\mathbf{a}_c = 2 \, (\mathbf{\Omega} \times \mathbf{v}_{\text{drive}})$$
   The sensor electrostatically drives a micro-machined proof mass at an active resonant velocity $\mathbf{v}_{\text{drive}}$. Coriolis acceleration is produced **only when an angular rotation $\mathbf{\Omega}$ is applied orthogonal to the drive axis**. Pure linear acceleration ($\mathbf{a}_{\text{frame}}$) produces zero Coriolis cross-product:
   $$\mathbf{\Omega} = \mathbf{0} \implies \mathbf{a}_c = \mathbf{0}$$

### 4.2 Differential Tuning-Fork Architecture & CMRR
Modern automotive/sports MEMS gyroscopes implement **counter-vibrating tuning-fork structures** consisting of two identical proof masses ($m_1 = m_2 = m$) driven in anti-phase:
$$\mathbf{v}_{\text{drive},1}(t) = \mathbf{v}_0 \cos(\omega_d t), \quad \mathbf{v}_{\text{drive},2}(t) = -\mathbf{v}_0 \cos(\omega_d t)$$

- **Under Real Chassis Rotation ($\mathbf{\Omega}$)**:
  Coriolis forces act in opposite directions (anti-phase):
  $$\mathbf{F}_{c,1} = 2m (\mathbf{\Omega} \times \mathbf{v}_{\text{drive},1}) = -\mathbf{F}_{c,2}$$
  The differential capacitive sense circuit measures the difference:
  $$\Delta S_{\text{rot}} \propto \mathbf{F}_{c,1} - \mathbf{F}_{c,2} = 2 \mathbf{F}_{c,1}$$
- **Under Engine Linear Vibration ($\mathbf{a}_{\text{vib}}$)**:
  External translational shaking acts equally and in the same direction on both proof masses (**common-mode acceleration**):
  $$\mathbf{F}_{\text{vib},1} = m \mathbf{a}_{\text{vib}}, \quad \mathbf{F}_{\text{vib},2} = m \mathbf{a}_{\text{vib}}$$
  The differential readout circuit subtracts the common-mode force:
  $$\Delta S_{\text{vib}} \propto \mathbf{F}_{\text{vib},1} - \mathbf{F}_{\text{vib},2} = m \mathbf{a}_{\text{vib}} - m \mathbf{a}_{\text{vib}} = 0$$
  This differential geometry yields **$40 - 60\,\text{dB}$ of Common-Mode Rejection Ratio (CMRR)** against linear engine vibrations.

### 4.3 High-Frequency Synchronous Demodulation Carrier
MEMS gyroscopes separate rotational signals from baseband mechanical vibrations in the frequency domain:
1. **Drive Carrier Resonance**: Proof masses are driven electrostatically at high mechanical resonance:
   $$f_{\text{drive}} \approx 20\,\text{kHz} - 35\,\text{kHz}$$
2. **Double Sideband Modulation**: True angular rate $\mathbf{\Omega}(t)$ modulates the high-frequency carrier:
   $$s_{\text{sense}}(t) \propto \Omega(t) \cdot \sin(2\pi f_{\text{drive}} t)$$
3. **Phase-Sensitive Synchronous Demodulation**:
   The raw sense signal is multiplied by a coherent reference carrier $\sin(2\pi f_{\text{drive}} t)$ and passed through a low-pass filter ($f_{\text{LPF}} \approx 100 - 400\,\text{Hz}$):
   $$s_{\text{demod}}(t) = s_{\text{sense}}(t) \cdot \sin(2\pi f_{\text{drive}} t) = \frac{1}{2}\Omega(t) \left[ 1 - \cos(4\pi f_{\text{drive}} t) \right]$$
4. **Baseband Engine Rejection**:
   External baseband vibrations ($f_{\text{engine}} = 58\,\text{Hz}$) entering the sense channel are multiplied by the carrier in the mixer:
   $$s_{\text{vib,demod}}(t) \propto a_{\text{vib}} \cos(2\pi f_{\text{engine}} t) \cdot \sin(2\pi f_{\text{drive}} t) = \frac{1}{2} a_{\text{vib}} \left[ \sin(2\pi (f_d + f_e) t) - \sin(2\pi (f_d - f_e) t) \right]$$
   The engine tone is up-converted to $\approx 25\,\text{kHz} \pm 58\,\text{Hz}$, where it is completely attenuated by the post-demodulation baseband filter.

### 4.4 Structural Chassis Rigidity (Torsional Isolation)
The IMU is bolted near the steering headstock of an aluminum perimeter frame designed with immense polar moments of inertia ($I_{xx}, I_{yy}, I_{zz}$) and high torsional stiffness ($K_{\text{torsion}} > 10{,}000\,\text{N}\cdot\text{m/rad}$) to resist rider steering forces. While the engine generates translational shaking forces ($\Delta a \approx \pm 3.5\,\text{m/s}^2$), it exerts near-zero net high-frequency torque about the headstock yaw/pitch axes. Micro-angular vibration velocities are bounded by $\Delta\omega < 0.005\,\text{rad/s}$, which sits well below the sensor's thermal noise floor.

---

## 5. Digital Filter Synthesis & Spectral Analysis (Objective 2)

```
        Raw Signal a(t) ---> [ A/D Conversion @ 500 Hz ]
                                     |
                                     v
                        [ Bilinear 4th-Order LP ]
                          Cutoff: f_c = 25.0 Hz
                          Passband: 0 - 20 Hz (Ap <= 1 dB)
                          Stopband: 58 Hz (As >= 30.6 dB)
                          2nd Harm: 116 Hz (As >= 60.2 dB)
                                     |
                                     v
                        Clean Kinematic Acceleration
```

### 5.1 Welch Power Spectral Density (PSD) Formulation
Spectral characteristics during stationary ground riding ($t \in [11.0, 17.0]\,\text{s}$, $N_{\text{stat}} = 3{,}000$) are estimated via Welch’s averaged modified periodogram method:
- Sub-window: $N_{\text{fft}} = 1{,}024$ samples ($2.048\,\text{s}$ segment length, frequency resolution $\Delta f = 0.488\,\text{Hz}$).
- Window: Hanning window $w[n] = 0.5 \left( 1 - \cos\left(\frac{2\pi n}{N_{\text{fft}} - 1}\right) \right)$.
- Overlap: $50\%$ ($N_{\text{overlap}} = 512$ samples).
- Scaling factor: $S_2 = \frac{1}{N_{\text{fft}}} \sum_{n=0}^{N_{\text{fft}}-1} w^2[n] \approx 0.375$.
- One-sided power spectral density:
  $$\hat{P}_{xx}(f_k) = \frac{2}{K \cdot f_s \cdot S_2} \sum_{m=1}^{K} \left\vert \sum_{n=0}^{N_{\text{fft}}-1} (x_m[n] - \bar{x}_m) w[n] e^{-j 2\pi k n / N_{\text{fft}}} \right\vert^2 \quad (0 < f_k < f_{\text{Nyq}})$$

### 5.2 Analytical Butterworth Low-Pass Filter Design

#### Design Specifications:
1. **Passband Boundary**: $f_p = 20.0\,\text{Hz}$ with maximum permissible ripple $A_p \le 1.0\,\text{dB}$ ($\ge 89.13\%$ amplitude preservation).
2. **Stopband Boundary**: $f_s = 58.0\,\text{Hz}$ (engine fundamental) with minimum attenuation $A_s \ge 30.0\,\text{dB}$ ($\ge 96.84\%$ suppression, $\ge 99.9\%$ power rejection).
3. **Second Harmonic Attenuation**: $A(116.0\,\text{Hz}) \ge 60.0\,\text{dB}$ ($\ge 99.9\%$ suppression).
4. **Sampling Rate**: $f_{\text{samp}} = 500.0\,\text{Hz}$ ($f_{\text{Nyq}} = 250.0\,\text{Hz}$).

#### Analytical Derivation of Minimum Filter Order ($N$):
The magnitude squared response of an analog Butterworth prototype is:
$$|H(j\Omega)|^2 = \frac{1}{1 + \left(\frac{\Omega}{\Omega_c}\right)^{2N}}$$
At passband edge $\Omega_p$ and stopband edge $\Omega_s$:
$$10^{-A_p/10} = \frac{1}{1 + (\Omega_p / \Omega_c)^{2N}} \implies \left(\frac{\Omega_p}{\Omega_c}\right)^{2N} = 10^{A_p/10} - 1$$
$$10^{-A_s/10} = \frac{1}{1 + (\Omega_s / \Omega_c)^{2N}} \implies \left(\frac{\Omega_s}{\Omega_c}\right)^{2N} = 10^{A_s/10} - 1$$
Dividing the two equations eliminates the unknown cutoff $\Omega_c$:
$$\left( \frac{\Omega_s}{\Omega_p} \right)^{2N} = \frac{10^{A_s/10} - 1}{10^{A_p/10} - 1}$$
Taking the base-10 logarithm and solving explicitly for $N$:
$$N \ge \frac{\log_{10}\left( \frac{10^{A_s/10} - 1}{10^{A_p/10} - 1} \right)}{2 \log_{10}(\Omega_s / \Omega_p)}$$

Substituting numerical specifications:
- $10^{A_s/10} - 1 = 10^{3.0} - 1 = 999.0$
- $10^{A_p/10} - 1 = 10^{0.1} - 1 = 0.2589$
- Pre-warped frequency ratio: $\frac{\Omega_s}{\Omega_p} = \frac{\tan(\pi \cdot 58 / 500)}{\tan(\pi \cdot 20 / 500)} = \frac{\tan(0.3644)}{\tan(0.1257)} = \frac{0.3813}{0.1263} = 3.018$

$$N \ge \frac{\log_{10}(999.0 / 0.2589)}{2 \log_{10}(3.018)} = \frac{\log_{10}(3858.6)}{2 \log_{10}(3.018)} = \frac{3.5864}{2 \cdot 0.4797} = \frac{3.5864}{0.9594} = \mathbf{3.738} \implies N = \mathbf{4}$$

#### Filter Order Trade-off Analysis:
| Order ($N$) | Stopband Atten @ $58\,\text{Hz}$ | Stopband Atten @ $116\,\text{Hz}$ | Passband Group Delay @ $10\,\text{Hz}$ | Landing Shock Peak Attenuation | Suitability Assessment |
| :---: | :---: | :---: | :---: | :---: | :--- |
| **$N = 2$** | $-14.73\,\text{dB}$ | $-32.41\,\text{dB}$ | $\mathbf{8.84\,\text{ms}}$ | $-0.8\%$ (Excellent) | **Rejected**: Fails $30\,\text{dB}$ stopband target |
| **$N = 4$** | **$-30.61\,\text{dB}$** | **$-60.17\,\text{dB}$** | **$17.87\,\text{ms}$** | **$-3.7\%$ (Marginal)** | **Selected**: Optimal Pareto boundary |
| **$N = 6$** | $-46.12\,\text{dB}$ | $-88.94\,\text{dB}$ | $26.85\,\text{ms}$ | $-8.4\%$ (Distorted) | **Rejected**: Excessive group delay |
| **$N = 8$** | $-61.63\,\text{dB}$ | $-117.72\,\text{dB}$ | $35.83\,\text{ms}$ | $-14.2\%$ (Unacceptable) | **Rejected**: Severe impulse smearing |

### 5.3 Bilinear Transformation with Frequency Pre-Warping
To map continuous-time Laplace $s$-plane poles to discrete-time $z$-plane poles without frequency compression distortion, cutoff frequency pre-warping is performed:
$$\Omega_c = \frac{2}{T_s} \tan\left( \frac{\omega_c T_s}{2} \right) = 2 f_s \tan\left( \frac{\pi f_c}{f_s} \right)$$
For $f_c = 25.0\,\text{Hz}$ and $f_s = 500.0\,\text{Hz}$:
$$\Omega_c = 2(500) \tan\left( \frac{25\pi}{500} \right) = 1{,}000 \tan(0.15708) = 1{,}000 \times 0.15838 = \mathbf{158.38\,\text{rad/s}}$$

The continuous prototype Butterworth transfer function:
$$H(s) = \frac{\Omega_c^4}{\prod_{k=1}^{2} (s^2 + 2\cos(\theta_k)\Omega_c s + \Omega_c^2)}, \quad \theta_k \in \left\{ \frac{5\pi}{8}, \frac{7\pi}{8} \right\}$$
Applying the Bilinear Transformation:
$$s = \frac{2}{T_s} \frac{1 - z^{-1}}{1 + z^{-1}} = 1{,}000 \frac{1 - z^{-1}}{1 + z^{-1}}$$
Factoring into cascaded **Second-Order Sections (SOS / Biquads)** to prevent coefficient quantization instability:
$$H(z) = g \cdot \prod_{k=1}^{2} \frac{b_{0,k} + b_{1,k} z^{-1} + b_{2,k} z^{-2}}{1 + a_{1,k} z^{-1} + a_{2,k} z^{-2}}$$
- Section 1: $Q_1 = \frac{1}{2\cos(\pi/8)} = 0.5412$ (Low-$Q$ stage, $\omega_{p1} = 23.4\,\text{Hz}$).
- Section 2: $Q_2 = \frac{1}{2\cos(3\pi/8)} = 1.3065$ (Resonant peak stage, $\omega_{p2} = 25.8\,\text{Hz}$).

#### 5.3.1 Second-Order Section (SOS) Direct Form II Transposed (DF2T) Topology
To prevent coefficient quantization drift and limit-cycle overflow on fixed-point or single-precision embedded platforms, the factored 4th-order transfer function $H(z)$ is realized as a cascade of two biquadratic Second-Order Sections (SOS) using the **Direct Form II Transposed (DF2T)** canonical architecture:

```
                  b0
          +----------------> (+) ----------------------> y[n]
          |                   ^
          |                   | w1[n-1]
          |                 [ z^-1 ]
          |                   ^
          |       b1          |
     x[n] +------->*--------> (+) <--------*<-------+
          |                   ^           -a1       |
          |                   | w2[n-1]             |
          |                 [ z^-1 ]                |
          |                   ^                     |
          |       b2          |           -a2       |
          +------->*--------> (+) <--------*--------+
```

For each biquad section $k \in \{1, 2\}$, the discrete difference equations are evaluated per sample:
$$y_k[n] = b_{0,k} x_k[n] + w_{1,k}[n-1]$$
$$w_{1,k}[n] = b_{1,k} x_k[n] - a_{1,k} y_k[n] + w_{2,k}[n-1]$$
$$w_{2,k}[n] = b_{2,k} x_k[n] - a_{2,k} y_k[n]$$

##### State-Space Representation of DF2T
In vector state-space notation $\mathbf{w}_k[n] = [w_{1,k}[n], w_{2,k}[n]]^T \in \mathbb{R}^2$:
$$\mathbf{w}_k[n] = \mathbf{A}_{\text{df2t}, k} \mathbf{w}_k[n-1] + \mathbf{B}_{\text{df2t}, k} x_k[n]$$
$$y_k[n] = \mathbf{C}_{\text{df2t}, k} \mathbf{w}_k[n-1] + D_{\text{df2t}, k} x_k[n]$$
where the state matrices for a normalized biquad ($a_{0,k} = 1$) are:
$$\mathbf{A}_{\text{df2t}, k} = \begin{bmatrix} -a_{1,k} & 1 \\ -a_{2,k} & 0 \end{bmatrix}, \quad \mathbf{B}_{\text{df2t}, k} = \begin{bmatrix} b_{1,k} - a_{1,k} b_{0,k} \\ b_{2,k} - a_{2,k} b_{0,k} \end{bmatrix}$$
$$\mathbf{C}_{\text{df2t}, k} = \begin{bmatrix} 1 & 0 \end{bmatrix}, \quad D_{\text{df2t}, k} = b_{0,k}$$

##### Numerical Immunity & Precision Advantages over DF1 and DF2:
1. **Canonical Memory Footprint**: Requires exactly 2 delay registers ($w_1, w_2$) per biquad (total 4 state registers for $N=4$), compared to 4 registers per section in Direct Form I (DF1).
2. **Dynamic Range Scaling & Overflow Prevention**: In Direct Form II (standard), input samples accumulate into internal states before numerator scaling: $w[n] = x[n] - a_1 w[n-1] - a_2 w[n-2]$. When resonant poles sit near the unit circle (such as Section 2 with $Q = 1.3065$), the internal state $w[n]$ experiences severe dynamic range expansion ($\|w\| \gg \|x\|$), triggering catastrophic integer overflow or floating-point mantissa cancellation. In **DF2T**, input $x[n]$ is multiplied by feedforward coefficients $b_{i}$ *before* entering state additions, maintaining state registers within the input/output dynamic envelope ($\|\mathbf{w}\| \approx \|y\|$).
3. **Minimized Sensitivity to Coefficient Quantization**: Poles and zeros are isolated in cascaded pairs rather than bundled into an $N^{\text{th}}$-order polynomial $\sum a_k z^{-k}$, preventing pole sensitivity $\frac{\partial p_i}{\partial a_k}$ from exploding exponentially with order $N$.

#### 5.3.2 Non-Causal Forward-Backward Zero-Phase Filtering (`filtfilt` via DF2T)
In offline telemetry analysis and jump trajectory reconstruction, phase distortion is unacceptable because group delay shifts touchdown timestamps and attenuates landing impact peaks. Non-causal zero-phase filtering eliminates group delay entirely through forward-backward recursion:

##### Mathematical Derivation of Zero Phase & Magnitude Squaring
1. **Forward Pass**: The input sequence $x[n]$ of length $N$ is filtered by $H(z)$ in causal time, yielding $y_f[n]$:
   $$Y_f(e^{j\omega}) = H(e^{j\omega}) X(e^{j\omega}) = |H(e^{j\omega})| e^{j \phi(\omega)} X(e^{j\omega})$$
2. **Time Reversal**: The forward output sequence is reversed in time:
   $$y_f^R[n] = y_f[N - 1 - n] \implies Y_f^R(e^{j\omega}) = Y_f^*(e^{j\omega}) e^{-j \omega (N - 1)}$$
3. **Backward Pass**: The time-reversed sequence is passed through the identical filter $H(z)$:
   $$Y_b(e^{j\omega}) = H(e^{j\omega}) Y_f^R(e^{j\omega}) = |H(e^{j\omega})| e^{j \phi(\omega)} \left[ |H(e^{j\omega})| e^{-j \phi(\omega)} X^*(e^{j\omega}) e^{-j \omega (N - 1)} \right]$$
   $$Y_b(e^{j\omega}) = |H(e^{j\omega})|^2 X^*(e^{j\omega}) e^{-j \omega (N - 1)}$$
4. **Final Time Reversal**: The backward filtered output is reversed once more:
   $$y_{\text{zero}}[n] = y_b[N - 1 - n] \implies Y_{\text{zero}}(e^{j\omega}) = Y_b^*(e^{j\omega}) e^{-j \omega (N - 1)}$$
   $$Y_{\text{zero}}(e^{j\omega}) = \left[ |H(e^{j\omega})|^2 X(e^{j\omega}) e^{+j \omega (N - 1)} \right] e^{-j \omega (N - 1)} = \mathbf{|H(e^{j\omega})|^2 X(e^{j\omega})}$$

##### Compounded Filter Characteristics:
- **Phase Response**: $\phi_{\text{zero}}(\omega) = \arg\left(|H(e^{j\omega})|^2\right) \equiv \mathbf{0.00^\circ} \quad \forall \omega \in [0, \pi]$.
- **Passband Group Delay**: $\tau_g(\omega) = -\frac{d \phi_{\text{zero}}(\omega)}{d\omega} \equiv \mathbf{0.00\,\text{ms}}$ (exact zero-phase alignment across all frequencies).
- **Compounded Attenuation**: The effective magnitude squared response doubles stopband rejection on the logarithmic scale:
  $$A_{\text{zero}}(\omega) = 20 \log_{10}|H_{\text{zero}}(e^{j\omega})| = 2 \times 20 \log_{10}|H(e^{j\omega})| = 2 \cdot A_{\text{causal}}(\omega)$$
  - Stopband Attenuation @ $58.11\,\text{Hz}$: $2 \times (-30.61\,\text{dB}) = \mathbf{-61.22\,\text{dB}}$ ($\ge 99.99\%$ power suppression).
  - Stopband Attenuation @ $116.21\,\text{Hz}$: $2 \times (-60.17\,\text{dB}) = \mathbf{-120.34\,\text{dB}}$.

##### Gustafsson's Analytical Boundary Condition Matching
Starting the backward filter with zero initial state ($\mathbf{w}[-1] = \mathbf{0}$) when the telemetry signal contains a non-zero gravitational DC offset ($a_z \approx +9.81\,\text{m/s}^2$ on ground) induces violent transient step-response ringing across the first $50-100$ samples. To eradicate edge transients, **Gustafsson’s initial condition matching** calculates the exact steady-state state vector $\mathbf{w}_0$ corresponding to a constant DC step input of amplitude $x[0]$:

$$\mathbf{w}_0 = \mathbf{A}_{\text{df2t}} \mathbf{w}_0 + \mathbf{B}_{\text{df2t}} x[0] \implies (\mathbf{I} - \mathbf{A}_{\text{df2t}}) \mathbf{w}_0 = \mathbf{B}_{\text{df2t}} x[0]$$
Solving the linear matrix system analytically for each DF2T biquad section:
$$\mathbf{w}_0 = (\mathbf{I} - \mathbf{A}_{\text{df2t}})^{-1} \mathbf{B}_{\text{df2t}} x[0]$$
$$\begin{bmatrix} 1 + a_1 & -1 \\ a_2 & 1 \end{bmatrix} \begin{bmatrix} w_{1,0} \\ w_{2,0} \end{bmatrix} = \begin{bmatrix} b_1 - a_1 b_0 \\ b_2 - a_2 b_0 \end{bmatrix} x[0]$$
Applying Cramer's rule where $\det(\mathbf{I} - \mathbf{A}_{\text{df2t}}) = 1 + a_1 + a_2 = \sum_{i=0}^2 a_i$:
$$\mathbf{w}_0 = \frac{x[0]}{1 + a_1 + a_2} \begin{bmatrix} (b_1 - a_1 b_0) + (b_2 - a_2 b_0) \\ (1 + a_1)(b_2 - a_2 b_0) - a_2 (b_1 - a_1 b_0) \end{bmatrix}$$
By initializing the forward pass with $\mathbf{w}_0(x[0])$ and the backward pass with $\mathbf{w}_0(y_f[N-1])$, boundary transient ringing is eliminated entirely.

##### Signal Reflection Padding (Odd Symmetric Extension)
To further ensure $C^0$ and $C^1$ boundary derivative continuity before time reversal, the telemetry sequence $x[n]$ is augmented on both ends with odd symmetric mirror reflections of length $L = 3 \times \max(\text{len}(b), \text{len}(a)) = 9$ samples:
$$x_{\text{pad}}[-n] = 2 x[0] - x[n], \quad x_{\text{pad}}[N - 1 + n] = 2 x[N - 1] - x[N - 1 - n] \quad (n = 1, \dots, L)$$
This guarantees that both the displacement and slope of the telemetry signal match smoothly across the artificial boundaries, preventing Gibbs phenomenon spectral leakage at the start ($t = 0.0\,\text{s}$) and end ($t = 90.0\,\text{s}$) of the dataset.

### 5.4 Data-Driven Parametric System Identification (Yule-Walker ARX)
As an alternative to physics-based low-pass filtering, an empirical System Identification model was trained on stationary ground telemetry using an Autoregressive (AR) model:
$$x[n] = -\sum_{k=1}^{p} a_k x[n-k] + e[n]$$
Solving the Yule-Walker normal equations via Toeplitz inversion:
$$\mathbf{R}_{xx} \mathbf{a} = -\mathbf{r}_{xx}$$
Root-locus extraction of the identified $p = 16$ poles revealed conjugate pairs located precisely at:
- Pole Pair 1: $f_1 = 58.11\,\text{Hz}$, pole radius $\rho_1 = 0.982$, damping ratio $\zeta_1 = 0.018$ ($Q_1 = 27.8$).
- Pole Pair 2: $f_2 = 116.21\,\text{Hz}$, pole radius $\rho_2 = 0.974$, damping ratio $\zeta_2 = 0.026$ ($Q_2 = 19.2$).

#### Data-Driven Dual-Notch IIR Filter:
$$H_{\text{notch}}(z) = \prod_{i=1}^{2} g_i \frac{1 - 2\cos(\omega_i) z^{-1} + z^{-2}}{1 - 2\rho_i \cos(\omega_i) z^{-1} + \rho_i^2 z^{-2}}$$
where $\omega_1 = \frac{2\pi \cdot 58.11}{500} = 0.7302\,\text{rad/sample}$ and $\rho_1 = 0.92$.

### 5.5 State-Space Harmonic Disturbance Kalman Filter & RTS Smoothing

To perform optimal telemetry conditioning that decouples whole-body vehicle motion from engine vibration without incurring the phase lag of IIR filters, a 5-state **Harmonic Disturbance Kalman Filter (DKF)** and backward **Rauch-Tung-Striebel (RTS) Zero-Phase Smoother** are formulated. 

#### 5.5.1 Internal Model Principle & Architecture
In signal processing and control theory, Francis & Wonham's **Internal Model Principle** establishes that persistent disturbance rejection requires embedding the dynamic generating system of the disturbance directly into the observer. The raw measured acceleration $y[k]$ (e.g., vertical axis $a_z$) is the superposition of true kinematic vehicle motion $a_{\text{kin}}[k]$, primary reciprocating engine unbalance $d_1[k]$, secondary reciprocating harmonic $d_2[k]$, and high-frequency broadband sensor noise $v[k]$:
$$y[k] = a_{\text{kin}}[k] + d_1[k] + d_2[k] + v[k]$$

By modeling the harmonic tones as autonomous stochastic oscillators inside the state vector, the Kalman filter places transmission zeros at the engine frequencies ($58.11\,\text{Hz}$ and $116.21\,\text{Hz}$) in the closed-loop transfer function from $y[k]$ to $\hat{a}_{\text{kin}}[k]$, synthesizing an optimal data-driven notch filter.

#### 5.5.2 State Vector Definition & Detailed State Descriptions
The system is parameterized by a 5-dimensional state vector $\mathbf{x}[k] \in \mathbb{R}^5$:
$$\mathbf{x}[k] = \begin{bmatrix} a_{\text{kin}}[k] & d_{1,I}[k] & d_{1,Q}[k] & d_{2,I}[k] & d_{2,Q}[k] \end{bmatrix}^T$$

1. **State $x_1[k] = a_{\text{kin}}[k]$ (Physical Kinematic Chassis Acceleration)**:
   - **Units**: $\text{m/s}^2$.
   - **Physical Phenomenon**: Represents the authentic whole-body kinematic acceleration of the motorcycle chassis along the sensing axis. This contains all low-frequency maneuvers (0–20 Hz), the zero proper acceleration condition during free-fall ballistic flight ($\approx 0.0\,\text{m/s}^2$), and the severe impact deceleration impulse upon landing touchdown ($> 18.0\,\text{m/s}^2$).
   - **Stochastic Model**: Governed by a 1st-order discrete Brownian random walk driven by zero-mean Gaussian jerk noise $w_a[k]$ with spectral intensity $q_a$.
2. **State $x_2[k] = d_{1,I}[k]$ (Fundamental Engine Unbalance — In-Phase Component)**:
   - **Units**: $\text{m/s}^2$.
   - **Physical Phenomenon**: The in-phase (cosine-projected) acceleration disturbance generated by the primary reciprocating unbalance of the single-cylinder engine at crankshaft frequency $f_1 = 58.11\,\text{Hz}$ ($N_{\text{rpm}} \approx 3{,}487\,\text{RPM}$). This is the direct projection of the slider-crank primary inertia force along the IMU sensing axis.
3. **State $x_3[k] = d_{1,Q}[k]$ (Fundamental Engine Unbalance — Quadrature Component)**:
   - **Units**: $\text{m/s}^2$.
   - **Physical Phenomenon**: The orthogonal $90^\circ$ phase-shifted (sine-projected) acceleration disturbance component of the fundamental engine unbalance. It does not appear directly in the instantaneous measurement equation, but acts as an orthogonal energy reservoir that enables continuous, autonomous rotation in the $d_{1,I}-d_{1,Q}$ state subspace.
   - **Instantaneous Polar Envelope**: The total physical amplitude and phase of the fundamental engine vibration are extracted algebraically as:
     $$A_1[k] = \sqrt{d_{1,I}^2[k] + d_{1,Q}^2[k]}, \quad \phi_1[k] = \arctan2(d_{1,Q}[k], d_{1,I}[k])$$
4. **State $x_4[k] = d_{2,I}[k]$ (Secondary Engine Harmonic — In-Phase Component)**:
   - **Units**: $\text{m/s}^2$.
   - **Physical Phenomenon**: The in-phase acceleration disturbance produced by the second harmonic reciprocating unbalance at $f_2 = 2f_1 = 116.21\,\text{Hz}$. This mode arises physically from the finite connecting-rod to crank-throw ratio ($r/l \approx 0.28$) in the slider-crank kinematic chain.
5. **State $x_5[k] = d_{2,Q}[k]$ (Secondary Engine Harmonic — Quadrature Component)**:
   - **Units**: $\text{m/s}^2$.
   - **Physical Phenomenon**: The orthogonal $90^\circ$ phase-shifted component of the second harmonic unbalance, sustaining the harmonic oscillation at angular velocity $\omega_2 = 2\omega_1$ with instantaneous amplitude $A_2[k] = \sqrt{d_{2,I}^2[k] + d_{2,Q}^2[k]}$.

#### 5.5.3 Physical & Mathematical Rationale: Why These Were Chosen As States
1. **Linearization of Arbitrary Phase and Amplitude via Orthogonal Quadrature Decomposition**:
   In experimental motorcycle operation, the engine vibration signal exhibits continuously drifting phase offsets $\phi(t)$ and time-varying envelope amplitude $A(t)$ due to engine load and structural chassis compliance:
   $$d(t) = A(t) \cos(\omega t + \phi(t))$$
   Attempting to estimate amplitude $A(t)$ and phase $\phi(t)$ directly results in an inherently non-linear measurement model ($y = a_{\text{kin}} + A \cos(\omega t + \phi)$), requiring non-convex nonlinear solvers that are vulnerable to local minima, gradient singularities, and phase slip.
   Expanding the sinusoid into orthogonal basis functions:
   $$d(t) = \underbrace{(A(t) \cos\phi(t))}_{d_I(t)} \cos(\omega t) - \underbrace{(A(t) \sin\phi(t))}_{d_Q(t)} \sin(\omega t)$$
   By assigning independent linear state variables to $d_I$ and $d_Q$, the dynamics of the 2D harmonic plane reduce to an exact linear rotation matrix:
   $$\begin{bmatrix} d_I[k+1] \\ d_Q[k+1] \end{bmatrix} = \begin{bmatrix} \cos(\omega \Delta t) & \sin(\omega \Delta t) \\ -\sin(\omega \Delta t) & \cos(\omega \Delta t) \end{bmatrix} \begin{bmatrix} d_I[k] \\ d_Q[k] \end{bmatrix}$$
   This formulation is **strictly linear and time-invariant (LTI)**, allowing the standard Kalman filter to track arbitrary instantaneous amplitude and phase variations with mathematically guaranteed convergence and optimal error variance.
2. **Subspace Orthogonality & Infinite-Depth Dynamic Notch Synthesis**:
   The measurement matrix is $\mathbf{H} = [1, 1, 0, 1, 0]$. States $d_{1,Q}$ and $d_{2,Q}$ are strictly unobserved in the instantaneous output projection, but are fully observable through system dynamics over two or more time steps. The Kalman filter partitions the measurement space into a low-frequency kinematic subspace ($a_{\text{kin}}$) and two high-frequency rotational subspaces ($d_1$ and $d_2$). By tuning process noise, the estimator achieves $-34\,\text{dB}$ causal and $-68\,\text{dB}$ smoothed attenuation at the exact harmonic lines without carving wide, phase-distorting notches in the chassis passband.
3. **Brownian Random Walk vs Higher-Order Jerk Modeling**:
   Why model $a_{\text{kin}}$ as a 1st-order random walk rather than integrating acceleration into velocity and position, or modeling kinematic jerk ($\dot{a}_{\text{kin}}$)?
   - *Observability Limits*: In 6-axis IMU telemetry without wheel speed encoders or GPS, position and velocity are unobservable from single-axis accelerometer measurements; integrating them creates unbounded random-walk divergence.
   - *Impact Transient Fidelity*: Motocross landing impacts present near-discontinuous step transitions where vertical acceleration surges from $0.0\,\text{m/s}^2$ to $18.66\,\text{m/s}^2$ within $\Delta t \approx 16-20\,\text{ms}$ (jerk $\dddot{z} > 1{,}000\,\text{m/s}^3$). Higher-order polynomial models (such as constant-acceleration or constant-jerk models) enforce artificial smoothness constraints that smear sharp landing impacts and induce significant group delay ($>15\,\text{ms}$).
   - A 1st-order random walk driven by high process noise ($q_a = 500.0\,\text{m}^2/\text{s}^4$) endows the kinematic state with high bandwidth to follow impulsive impact shocks with **$0.0\,\text{ms}$ latency and $100.0\%$ peak capture**, while the narrowband harmonic oscillators absorb the stationary engine vibration.

#### 5.5.4 Complete Dynamics Equations

##### Continuous-Time State-Space Formulation
$$\dot{\mathbf{x}}(t) = \mathbf{A}_c \mathbf{x}(t) + \mathbf{B}_c \mathbf{w}_c(t)$$
where $\mathbf{A}_c \in \mathbb{R}^{5 \times 5}$:
$$\mathbf{A}_c = \begin{bmatrix}
0 & 0 & 0 & 0 & 0 \\
0 & -\zeta_1 \omega_1 & \omega_1 & 0 & 0 \\
0 & -\omega_1 & -\zeta_1 \omega_1 & 0 & 0 \\
0 & 0 & 0 & -\zeta_2 \omega_2 & \omega_2 \\
0 & 0 & 0 & -\omega_2 & -\zeta_2 \omega_2
\end{bmatrix}$$
with damping factors $\zeta_1, \zeta_2 \ll 1$ and nominal continuous frequencies $\omega_1 = 2\pi(58.11)\,\text{rad/s}$ and $\omega_2 = 2\pi(116.21)\,\text{rad/s}$.

##### Discrete-Time State Transition Model
Discretizing over sampling interval $\Delta t = 1 / f_s = 2.0\,\text{ms}$ ($f_s = 500.0\,\text{Hz}$):
$$\mathbf{x}[k+1] = \mathbf{F} \mathbf{x}[k] + \mathbf{w}[k], \quad \mathbf{w}[k] \sim \mathcal{N}(\mathbf{0}, \mathbf{Q})$$
The state transition matrix $\mathbf{F} \in \mathbb{R}^{5 \times 5}$ is:
$$\mathbf{F} = \begin{bmatrix}
1.0 & 0 & 0 & 0 & 0 \\
0 & \rho_1 \cos(\omega_1 \Delta t) & \rho_1 \sin(\omega_1 \Delta t) & 0 & 0 \\
0 & -\rho_1 \sin(\omega_1 \Delta t) & \rho_1 \cos(\omega_1 \Delta t) & 0 & 0 \\
0 & 0 & 0 & \rho_2 \cos(\omega_2 \Delta t) & \rho_2 \sin(\omega_2 \Delta t) \\
0 & 0 & 0 & -\rho_2 \sin(\omega_2 \Delta t) & \rho_2 \cos(\omega_2 \Delta t)
\end{bmatrix}$$
where:
- $\omega_1 \Delta t = 2\pi (58.11) (0.002) = 0.7302\,\text{rad/sample}$
- $\omega_2 \Delta t = 2\pi (116.21) (0.002) = 1.4604\,\text{rad/sample}$
- $\rho_1 = \rho_2 = 0.999$ (light exponential pole contraction to ensure numerical stability and bound resonance against quantization).

##### Observation Equation
The scalar telemetry measurement $y[k]$ is mapped via measurement row vector $\mathbf{H} \in \mathbb{R}^{1 \times 5}$:
$$y[k] = \mathbf{H} \mathbf{x}[k] + v[k], \quad \mathbf{H} = \begin{bmatrix} 1.0 & 1.0 & 0.0 & 1.0 & 0.0 \end{bmatrix}, \quad v[k] \sim \mathcal{N}(0, R)$$

##### Stochastic Covariance Parameterization
1. **Process Noise Covariance ($\mathbf{Q}$)**:
   $$\mathbf{Q} = \text{diag}\left( \begin{bmatrix} q_a \Delta t & q_{d1} & q_{d1} & q_{d2} & q_{d2} \end{bmatrix} \right) \in \mathbb{R}^{5 \times 5}$$
   - $q_a = 500.0\,\text{m}^2/\text{s}^4 \implies q_a \Delta t = 1.0\,\text{m}^2/\text{s}^2$ per sample (endows $a_{\text{kin}}$ with agile shock responsiveness).
   - $q_{d1} = 1.0 \times 10^{-2}\,\text{m}^2/\text{s}^4$ (permits slow tracking of throttle-dependent fundamental amplitude shifts).
   - $q_{d2} = 1.0 \times 10^{-3}\,\text{m}^2/\text{s}^4$ (regulates secondary harmonic disturbance tracking).
2. **Measurement Noise Covariance ($R$)**:
   Calibrated from the high-frequency sensor noise floor on stationary ground riding:
   $$R = \sigma_v^2 = 0.00425\,(\text{m/s}^2)^2$$

##### Forward Discrete Kalman Recursion
At each discrete time step $k = 1, 2, \dots, N$:
1. **State Time-Update (Prediction)**:
   $$\hat{\mathbf{x}}_{k|k-1} = \mathbf{F} \hat{\mathbf{x}}_{k-1|k-1}$$
2. **Covariance Time-Update (Prediction)**:
   $$\mathbf{P}_{k|k-1} = \mathbf{F} \mathbf{P}_{k-1|k-1} \mathbf{F}^T + \mathbf{Q}$$
3. **Measurement Innovation & Innovation Covariance**:
   $$\tilde{y}_k = y[k] - \mathbf{H} \hat{\mathbf{x}}_{k|k-1}, \quad S_k = \mathbf{H} \mathbf{P}_{k|k-1} \mathbf{H}^T + R$$
4. **Optimal Kalman Gain Calculation**:
   $$\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}^T S_k^{-1} = \frac{1}{S_k} \mathbf{P}_{k|k-1} \mathbf{H}^T \in \mathbb{R}^{5 \times 1}$$
5. **State Measurement-Update (Correction)**:
   $$\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \tilde{y}_k$$
6. **Covariance Measurement-Update (Joseph Form)**:
   $$\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}) \mathbf{P}_{k|k-1} (\mathbf{I} - \mathbf{K}_k \mathbf{H})^T + \mathbf{K}_k R \mathbf{K}_k^T$$

##### Algebraic Riccati Equation & Steady-State Kalman Gain
Because the pair $(\mathbf{F}, \mathbf{H})$ is observable and $(\mathbf{F}, \mathbf{Q}^{1/2})$ is controllable, the error covariance converges rapidly ($< 50$ samples) to the unique positive-definite steady-state solution $\mathbf{P}_\infty$ of the Discrete Algebraic Riccati Equation (DARE):
$$\mathbf{P}_\infty = \mathbf{F} \mathbf{P}_\infty \mathbf{F}^T - \mathbf{F} \mathbf{P}_\infty \mathbf{H}^T \left( \mathbf{H} \mathbf{P}_\infty \mathbf{H}^T + R \right)^{-1} \mathbf{H} \mathbf{P}_\infty \mathbf{F}^T + \mathbf{Q}$$
The resulting steady-state Kalman gain vector is:
$$\mathbf{K}_\infty = \begin{bmatrix} 0.9178 & 0.0537 & 0.1153 & 0.0247 & 0.0310 \end{bmatrix}^T$$

##### Closed-Loop Steady-State Transfer Function
The equivalent discrete-time transfer function from raw measurement $y[k]$ to estimated kinematic acceleration $\hat{a}_{\text{kin}}[k] = \mathbf{C} \hat{\mathbf{x}}[k]$ (where $\mathbf{C} = [1, 0, 0, 0, 0]$) is:
$$H_{\text{KF}}(z) = \mathbf{C} \left( z \mathbf{I} - (\mathbf{I} - \mathbf{K}_\infty \mathbf{H})\mathbf{F} \right)^{-1} \mathbf{K}_\infty$$
Evaluating $H_{\text{KF}}(e^{j\omega\Delta t})$ analytically yields $-0.02\,\text{dB}$ attenuation at $20\,\text{Hz}$, $-0.11\,\text{dB}$ at $40\,\text{Hz}$, and sharp $-34.0\,\text{dB}$ notch rejections at $58.11\,\text{Hz}$ and $-29.4\,\text{dB}$ at $116.21\,\text{Hz}$, with only $2.50\,\text{ms}$ group delay at $10\,\text{Hz}$.

##### Backward Rauch-Tung-Striebel (RTS) Zero-Phase Smoother
For post-session flight analysis, backward recursive RTS smoothing propagates future telemetry information backwards in time:
$$\mathbf{C}_k = \mathbf{P}_{k|k} \mathbf{F}^T \mathbf{P}_{k+1|k}^{-1}$$
$$\hat{\mathbf{x}}_{k|N} = \hat{\mathbf{x}}_{k|k} + \mathbf{C}_k (\hat{\mathbf{x}}_{k+1|N} - \hat{\mathbf{x}}_{k+1|k})$$
$$\mathbf{P}_{k|N} = \mathbf{P}_{k|k} + \mathbf{C}_k (\mathbf{P}_{k+1|N} - \mathbf{P}_{k+1|k}) \mathbf{C}_k^T$$
- Eliminates phase delay completely: **$0.00\,\text{ms}$ group delay** ($\Delta \phi \equiv 0.0^\circ$).
- Doubles stopband attenuation: **$-68.0\,\text{dB}$ harmonic suppression at $58\,\text{Hz}$**.

---

### 5.6 Adaptive Extended Kalman Filter (AEKF) Formulation for Unknown & Varying Engine Harmonics

#### 5.6.1 Motivation & Operational Problem Statement
While the linear Harmonic Disturbance Kalman Filter (Section 5.5) achieves exceptional performance when the engine speed is static, real-world motocross racing involves severe operational non-stationarity:
- Rider modulates throttle, clutches in turns, and shifts across gears ($1^{\text{st}}$ through $5^{\text{th}}$).
- Engine fundamental speed sweeps dynamically between $2,000\,\text{RPM}$ ($f_e \approx 33.3\,\text{Hz}$) and $9,000\,\text{RPM}$ ($f_e \approx 150.0\,\text{Hz}$).
- On unfamiliar, borrowed, or uncalibrated motorcycles, the operating RPM is **completely unknown beforehand**, and many racing bikes lack digital CANbus interfaces or crankshaft position encoders.
- If the true engine speed drifts by even $\pm 1.5\,\text{Hz}$ away from a fixed notch filter, the high-$Q$ notch misses the vibration spike completely, causing severe engine noise leakage into kinematic channels.

To overcome this fundamental limitation, we formulate a **6-State Adaptive Extended Kalman Filter (AEKF)** that treats the instantaneous engine crankshaft angular velocity $\omega_e(t)$ as an unknown, time-varying hidden dynamic state, executing **joint state estimation and online parameter identification**.

#### 5.6.2 Augmented State Vector Definition & State Descriptions
The state vector is augmented to 6 dimensions $\mathbf{x}[k] \in \mathbb{R}^6$:
$$\mathbf{x}[k] = \begin{bmatrix} a_{\text{kin}}[k] & d_{1,I}[k] & d_{1,Q}[k] & d_{2,I}[k] & d_{2,Q}[k] & \omega_e[k] \end{bmatrix}^T$$

1. **State $x_1[k] = a_{\text{kin}}[k]$ (Kinematic Chassis Acceleration)**:
   - **Units**: $\text{m/s}^2$.
   - **Role**: Pure vehicle kinematic coordinate acceleration along the chassis sensing axis, completely isolated from engine harmonic pollution.
2. **State $x_2[k] = d_{1,I}[k]$ (Time-Varying Fundamental — In-Phase Component)**:
   - **Units**: $\text{m/s}^2$.
   - **Role**: In-phase acceleration disturbance component of the engine primary reciprocating unbalance, rotating at the instantaneous angular velocity $\omega_e[k]$.
3. **State $x_3[k] = d_{1,Q}[k]$ (Time-Varying Fundamental — Quadrature Component)**:
   - **Units**: $\text{m/s}^2$.
   - **Role**: Orthogonal $90^\circ$ phase-shifted component of the primary unbalance, forming the orthogonal harmonic basis with $d_{1,I}[k]$.
4. **State $x_4[k] = d_{2,I}[k]$ (Time-Varying Second Harmonic — In-Phase Component)**:
   - **Units**: $\text{m/s}^2$.
   - **Role**: In-phase acceleration disturbance component of the secondary reciprocating unbalance, constrained by slider-crank physics to rotate at exactly $2\omega_e[k]$.
5. **State $x_5[k] = d_{2,Q}[k]$ (Time-Varying Second Harmonic — Quadrature Component)**:
   - **Units**: $\text{m/s}^2$.
   - **Role**: Orthogonal $90^\circ$ phase-shifted component of the secondary harmonic unbalance, oscillating at $2\omega_e[k]$.
6. **State $x_6[k] = \omega_e[k]$ (Engine Instantaneous Crankshaft Angular Frequency)**:
   - **Units**: $\text{rad/s}$.
   - **Role**: The fundamental rotational frequency of the engine crankshaft. This provides continuous real-time engine speed tracking without a physical tachometer:
     $$f_e[k] = \frac{\omega_e[k]}{2\pi}\,\text{[Hz]}, \quad N_{\text{rpm}}[k] = \frac{60 \, \omega_e[k]}{2\pi}\,\text{[RPM]}$$

#### 5.6.3 Engineering Rationale: Why These Were Chosen As States
1. **Autonomous Self-Tuning Notch via Joint State-Parameter Estimation**:
   Promoting $\omega_e$ to a dynamic state transforms the observer from a passive filter into an active **Phase-Locked Loop (PLL) and adaptive spectral tracker**. The filter continuously seeks the spectral line that minimizes measurement innovation variance, automatically carving tracking notches at $\omega_e$ and $2\omega_e$.
2. **Physical Kinematic Coupling of Harmonics ($1\times$ and $2\times$ Rigid Lock)**:
   Why not estimate the second harmonic frequency using a separate 7th state $\omega_2$?
   From slider-crank kinematics (Section 3.1), the secondary unbalance is physically produced by connecting-rod angularity and is **strictly constrained to exactly twice the crankshaft velocity**:
   $$\omega_2(t) \equiv 2 \omega_1(t) = 2\omega_e(t)$$
   By hardcoding this kinematic relationship into the state transition function, the EKF pools the signal energy of **both** the primary ($58.11\,\text{Hz}$) and secondary ($116.21\,\text{Hz}$) vibration lines to drive the single frequency state $\omega_e$. This improves the frequency estimation signal-to-noise ratio by $>4.8\,\text{dB}$ and prevents harmonic divergence or false octave locking during aggressive throttle transients.
3. **Bilinear Coupling as an Analytical Phase Detector**:
   While the measurement function is linear ($y = a_{\text{kin}} + d_{1,I} + d_{2,I}$), the state transition function $\mathbf{f}(\mathbf{x})$ is non-linear because $\omega_e$ appears inside trigonometric terms multiplying $d_I$ and $d_Q$. The resulting cross-coupling derivatives in the Jacobian matrix act as an analytical gradient that automatically steers $\hat{\omega}_e$ toward the true excitation frequency.

#### 5.6.4 Non-Linear Dynamics & State Transition Equations

##### Continuous-Time Non-Linear State Equations
$$\dot{\mathbf{x}}(t) = \mathbf{f}_c(\mathbf{x}(t)) + \mathbf{w}_c(t)$$
$$\mathbf{f}_c(\mathbf{x}(t)) = \begin{bmatrix}
0 \\
-\zeta_1 \omega_e d_{1,I} + \omega_e d_{1,Q} \\
-\omega_e d_{1,I} - \zeta_1 \omega_e d_{1,Q} \\
-\zeta_2 (2\omega_e) d_{2,I} + (2\omega_e) d_{2,Q} \\
-(2\omega_e) d_{2,I} - \zeta_2 (2\omega_e) d_{2,Q} \\
0
\end{bmatrix}$$
where engine angular frequency $\omega_e$ follows a drift-free random walk driven by angular acceleration noise $w_\omega(t) \sim \mathcal{N}(0, q_\omega)$.

##### Discrete-Time Non-Linear Mapping ($\mathbf{f}(\mathbf{x}_k)$)
Integrating across sample period $\Delta t$:
$$\mathbf{x}[k+1] = \mathbf{f}(\mathbf{x}[k]) + \mathbf{w}[k]$$
Defining dimensionless rotation angles over one sample:
$$\theta_1[k] = \omega_e[k] \Delta t, \quad \theta_2[k] = 2 \omega_e[k] \Delta t$$
The explicit discrete transition vector $\mathbf{f}(\mathbf{x}[k]) \in \mathbb{R}^6$ is:
$$\mathbf{f}(\mathbf{x}[k]) = \begin{bmatrix}
f_1(\mathbf{x}) \\ f_2(\mathbf{x}) \\ f_3(\mathbf{x}) \\ f_4(\mathbf{x}) \\ f_5(\mathbf{x}) \\ f_6(\mathbf{x})
\end{bmatrix} = \begin{bmatrix}
a_{\text{kin}}[k] \\
\rho_1 \left( d_{1,I}[k] \cos\theta_1[k] + d_{1,Q}[k] \sin\theta_1[k] \right) \\
\rho_1 \left( -d_{1,I}[k] \sin\theta_1[k] + d_{1,Q}[k] \cos\theta_1[k] \right) \\
\rho_2 \left( d_{2,I}[k] \cos\theta_2[k] + d_{2,Q}[k] \sin\theta_2[k] \right) \\
\rho_2 \left( -d_{2,I}[k] \sin\theta_2[k] + d_{2,Q}[k] \cos\theta_2[k] \right) \\
\omega_e[k]
\end{bmatrix}$$
where $\rho_1 = \rho_2 = 0.998$ ensures robust BIBO numerical stability.

#### 5.6.5 Analytical State Jacobian Matrix ($\mathbf{F}_k = \left. \frac{\partial \mathbf{f}}{\partial \mathbf{x}} \right\vert_{\hat{\mathbf{x}}_{k|k}}$)
The Extended Kalman Filter linearizes the non-linear transition map at each iteration about the posterior estimate $\hat{\mathbf{x}}_{k|k}$. The analytical $6 \times 6$ Jacobian matrix is:
$$\mathbf{F}_k = \left. \frac{\partial \mathbf{f}}{\partial \mathbf{x}} \right\vert_{\hat{\mathbf{x}}_{k|k}} = \begin{bmatrix}
1 & 0 & 0 & 0 & 0 & 0 \\
0 & \rho_1 \cos\theta_1 & \rho_1 \sin\theta_1 & 0 & 0 & J_{26} \\
0 & -\rho_1 \sin\theta_1 & \rho_1 \cos\theta_1 & 0 & 0 & J_{36} \\
0 & 0 & 0 & \rho_2 \cos\theta_2 & \rho_2 \sin\theta_2 & J_{46} \\
0 & 0 & 0 & -\rho_2 \sin\theta_2 & \rho_2 \cos\theta_2 & J_{56} \\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}$$

##### Exact Derivation of Frequency Cross-Coupling Terms ($J_{i6} = \frac{\partial f_i}{\partial \omega_e}$):
Applying the chain rule $\frac{\partial \theta_1}{\partial \omega_e} = \Delta t$ and $\frac{\partial \theta_2}{\partial \omega_e} = 2 \Delta t$:
1. **Fundamental In-Phase Sensitivity ($J_{26}$)**:
   $$J_{26} = \frac{\partial f_2}{\partial \omega_e} = \rho_1 \Delta t \left( -d_{1,I} \sin\theta_1 + d_{1,Q} \cos\theta_1 \right)$$
2. **Fundamental Quadrature Sensitivity ($J_{36}$)**:
   $$J_{36} = \frac{\partial f_3}{\partial \omega_e} = \rho_1 \Delta t \left( -d_{1,I} \cos\theta_1 - d_{1,Q} \sin\theta_1 \right)$$
3. **Second Harmonic In-Phase Sensitivity ($J_{46}$)**:
   $$J_{46} = \frac{\partial f_4}{\partial \omega_e} = 2 \rho_2 \Delta t \left( -d_{2,I} \sin\theta_2 + d_{2,Q} \cos\theta_2 \right)$$
4. **Second Harmonic Quadrature Sensitivity ($J_{56}$)**:
   $$J_{56} = \frac{\partial f_5}{\partial \omega_e} = 2 \rho_2 \Delta t \left( -d_{2,I} \cos\theta_2 - d_{2,Q} \sin\theta_2 \right)$$

*Physical Significance of the Jacobian Cross-Terms*: The terms $J_{26}$ through $J_{56}$ quantify the sensitivity of the predicted disturbance to errors in the estimated engine frequency. In the Kalman covariance projection $\mathbf{F} \mathbf{P} \mathbf{F}^T$, these cross-terms correlate the frequency error covariance with measurement innovation, allowing any phase discrepancy in the incoming acceleration signal to directly generate an updating torque on $\hat{\omega}_e$.

#### 5.6.6 Measurement Model & Extended Kalman Recursion
- **Measurement Model**:
  $$y[k] = h(\mathbf{x}[k]) + v[k] = a_{\text{kin}}[k] + d_{1,I}[k] + d_{2,I}[k] + v[k]$$
- **Measurement Jacobian Row Vector ($\mathbf{H}$)**:
  $$\mathbf{H} = \left. \frac{\partial h}{\partial \mathbf{x}} \right\vert_{\hat{\mathbf{x}}_{k|k-1}} = \begin{bmatrix} 1.0 & 1.0 & 0.0 & 1.0 & 0.0 & 0.0 \end{bmatrix}$$
- **EKF Predict-Update Cycle**:
  1. *Nonlinear State Prediction*:
     $$\hat{\mathbf{x}}_{k|k-1} = \mathbf{f}(\hat{\mathbf{x}}_{k-1|k-1})$$
  2. *Covariance Prediction*:
     $$\mathbf{P}_{k|k-1} = \mathbf{F}_{k-1} \mathbf{P}_{k-1|k-1} \mathbf{F}_{k-1}^T + \mathbf{Q}$$
  3. *Scalar Innovation & Innovation Variance*:
     $$\tilde{y}_k = y[k] - \left( \hat{a}_{\text{kin}, k|k-1} + \hat{d}_{1,I, k|k-1} + \hat{d}_{2,I, k|k-1} \right)$$
     $$S_k = \mathbf{H} \mathbf{P}_{k|k-1} \mathbf{H}^T + R$$
  4. *Kalman Gain Vector Calculation*:
     $$\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}^T S_k^{-1} \in \mathbb{R}^{6 \times 1}$$
  5. *State Estimate Correction*:
     $$\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \tilde{y}_k$$
  6. *Covariance Update & Numerical Symmetrization*:
     $$\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}) \mathbf{P}_{k|k-1}$$
     $$\mathbf{P}_{k|k} = \frac{1}{2} \left( \mathbf{P}_{k|k} + \mathbf{P}_{k|k}^T \right) \quad (\text{Guarantees Positive Semi-Definiteness})$$

#### 5.6.7 Adaptive Noise Covariance Tuning & Stability Bounding

1. **Process Noise Covariance Matrix ($\mathbf{Q}$)**:
   $$\mathbf{Q} = \text{diag}\left( \begin{bmatrix} q_a \Delta t & q_{d1} & q_{d1} & q_{d2} & q_{d2} & q_\omega \Delta t \end{bmatrix} \right)$$
   - Kinematic acceleration noise: $q_a = 200.0\,\text{m}^2/\text{s}^4$.
   - Disturbance amplitude noise: $q_{d1} = 1.0 \times 10^{-1}\,\text{m}^2/\text{s}^4$, $q_{d2} = 1.0 \times 10^{-2}\,\text{m}^2/\text{s}^4$.
   - Crankshaft frequency agility noise: $q_\omega = (2\pi \cdot \sigma_{\dot{f}})^2$, where $\sigma_{\dot{f}} = 8.0\,\text{Hz/s}$ governs the permissible engine acceleration rate (revving dynamics).
   - Measurement noise variance: $R = 0.005\,(\text{m/s}^2)^2$.
2. **Physical Frequency Projection (State Bounding)**:
   During prolonged free-fall jump flight or quiet engine idle, lack of persistent excitation can cause the unconstrained frequency state $\omega_e$ to wander. A non-linear constraint projection operator is applied after every update step:
   $$\hat{\omega}_e[k] \leftarrow \text{clip}\left( \hat{\omega}_e[k],\, 2\pi f_{\text{min}},\, 2\pi f_{\text{max}} \right)$$
   where:
   - $f_{\text{min}} = 25.0\,\text{Hz}$ ($1{,}500\,\text{RPM}$, minimum idle threshold).
   - $f_{\text{max}} = 140.0\,\text{Hz}$ ($8{,}400\,\text{RPM}$, 2-stroke rev-limiter boundary).
3. **Shock Robustness via Innovation Gating**:
   During extreme landing impacts ($a_z > 18.0\,\text{m/s}^2$), the rapid innovation surge $\tilde{y}_k$ could be misinterpreted by the filter as a violent engine frequency shift. When the normalized innovation squared exceeds the gating threshold $\frac{\tilde{y}_k^2}{S_k} > \chi_{1, 0.999}^2 \approx 10.83$, the update on $\hat{\omega}_e$ is frozen for that sample, preventing impact shock transients from corrupting engine speed tracking.

#### 5.6.8 Blind Frequency Tracking Convergence Demonstration
To validate the autonomous adaptation capability under zero prior calibration, the AEKF was evaluated on stationary ground telemetry ($t \in [11.0, 17.0]\,\text{s}$) with an intentionally corrupted blind initial guess:
$$f_{e,0} = 45.00\,\text{Hz} \quad (N_{\text{rpm}, 0} = 2{,}700\,\text{RPM})$$
representing a large initial error of $\Delta f = -13.11\,\text{Hz}$ ($-787\,\text{RPM}$) relative to the true empirical frequency ($58.11\,\text{Hz}$).

```
Tracked Frequency [Hz]
  60 |                                     * * * * * * * True Ground Truth (58.11 Hz / 3,487 RPM)
     |                                 * * 
  55 |                             * *
     |                         * *
  50 |                     * *
     |                 * *
  45 | * * * * * * * * (Blind Initial Guess: 45.0 Hz / 2,700 RPM)
     +------------------------------------------------------------------------------------> Time [s]
     0.0             0.5             1.0             1.5             2.0             2.5
```

- **Locking Latency**: The AEKF converges to within $\pm 0.5\,\text{Hz}$ of ground truth in **$\tau_{\text{lock}} < 2.5\,\text{seconds}$**.
- **Asymptotic Tracking Accuracy**: Settles at $\hat{f}_e = 58.11\,\text{Hz}$ ($\hat{N}_{\text{rpm}} = 3{,}487\,\text{RPM}$) with standard deviation $\sigma_f < 0.08\,\text{Hz}$.
- **Autonomous Notch Carving**: Welch PSD of the resulting $\hat{a}_{\text{kin}}$ confirms $>32\,\text{dB}$ harmonic attenuation at $58.11\,\text{Hz}$ and $>28\,\text{dB}$ at $116.21\,\text{Hz}$, matching the performance of a hand-tuned notch filter without requiring any prior knowledge of engine RPM.

---

### 5.7 Multi-Paradigm Benchmark Comparison
The full comparative evaluation across raw telemetry, the Physics Butterworth LP filter, the Data-Driven SysID Dual-Notch filter, the Linear Harmonic Disturbance Kalman Filter (DKF), the Backward RTS Smoother, and the Adaptive Extended Kalman Filter (AEKF) is synthesized below:

| Metric | Raw Unfiltered | 4th-Order Butterworth LP | SysID Dual-Notch Filter | Linear Kalman Filter (DKF) | RTS Smoother (Zero-Phase) | Adaptive EKF (AEKF) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Passband Group Delay @ 10 Hz** | $0.0\,\text{ms}$ | $17.87\,\text{ms}$ | **$0.94\,\text{ms}$** | **$2.50\,\text{ms}$** | **$0.00\,\text{ms}$** | **$2.65\,\text{ms}$** |
| **Takeoff Edge Detection Lag** | $0.0\,\text{ms}$ | $20.0\,\text{ms}$ ($10$ samples) | **$0.0\,\text{ms}$** | **$0.0\,\text{ms}$** | **$0.0\,\text{ms}$** | **$0.0\,\text{ms}$** |
| **Landing Peak Shock Amplitude** | $17.98\,\text{m/s}^2$ | $17.31\,\text{m/s}^2$ ($-3.7\%$) | $18.20\,\text{m/s}^2$ ($+1.2\%$) | **$17.98\,\text{m/s}^2$ ($100.0\%$)** | **$17.96\,\text{m/s}^2$ ($99.9\%$)** | **$17.98\,\text{m/s}^2$ ($100.0\%$)** |
| **Landing Peak Timing Delay** | $0.0\,\text{ms}$ | $+20.0\,\text{ms}$ lag | $-2.0\,\text{ms}$ | **$0.0\,\text{ms}$** | **$0.0\,\text{ms}$** | **$0.0\,\text{ms}$** |
| **Engine Spike Rejection @ $58\,\text{Hz}$** | $0.0\,\text{dB}$ | $-30.61\,\text{dB}$ | **$-56.39\,\text{dB}$** | **$-34.00\,\text{dB}$** | **$-68.00\,\text{dB}$** | **$-32.40\,\text{dB}$ (Self-Tuned)** |
| **Chassis Dynamics Gain @ $40\,\text{Hz}$** | $0.0\,\text{dB}$ | $-16.87\,\text{dB}$ (Smears motion) | **$-0.47\,\text{dB}$** | **$-0.11\,\text{dB}$** | **$0.00\,\text{dB}$** | **$-0.14\,\text{dB}$** |
| **Engine RPM Tracking** | N/A | None | None | None (Fixed) | None (Fixed) | **$\pm 0.08\,\text{Hz}$ Blind Lock** |
| **Computational Complexity** | Zero | 4 MACs / sample | 4 MACs / sample | 36 MACs / sample | $2 \times$ passes (offline) | 72 MACs / sample |

---

### 5.8 Control-Theoretic Pole Placement & Root Locus of the Harmonic Kalman Filter

The extraordinary performance of the State-Space Harmonic Disturbance Kalman Filter (combining sub-$3\,\text{ms}$ latency with $>-34\,\text{dB}$ harmonic rejection) is rooted in its **closed-loop pole-zero constellation** in the complex $z$-plane. Analyzing the system from a root-locus and pole-placement perspective reveals how the Discrete Algebraic Riccati Equation (DARE) automatically achieves optimal pole placement:

```
                           Im(z)
                             ^
                             |       * Closed-Loop Harmonic Poles (r ≈ 0.952, 58.11 Hz)
                             |      o  Open-Loop Poles & Transmission Zeros (ρ = 0.999)
                             |     /
                             |    /
                        +----+---+----+ Unit Circle (|z| = 1)
                        |    |  /     |
                        |    | /      |
       -----------------+----+--------+*----------------> Re(z)
                        |    |  \     |  Kinematic Pole Migration:
                        |    |   *    |  Open-Loop: z = 1.0  --->  Closed-Loop: z = 0.0822
                        +----+---+----+
                             |    \
                             |     \
                             |      o
                             |       * (116.21 Hz Harmonics)
                             v
```

#### 5.8.1 Open-Loop vs Closed-Loop Eigenvalue Structure
The open-loop state transition matrix $\mathbf{F}$ possesses 5 eigenvalues:
1. **Kinematic Integrator Pole**: $\lambda_{\text{kin}}^{\text{OL}} = 1.0000 + j 0.0000$ (on the real axis at the unit circle boundary, modeling the discrete Brownian random walk of chassis acceleration).
2. **Fundamental Harmonic Conjugate Pair ($58.11\,\text{Hz}$)**:
   $$\lambda_{1, 1,2}^{\text{OL}} = \rho_1 e^{\pm j \omega_1 \Delta t} = 0.999 \left( \cos(0.7302) \pm j \sin(0.7302) \right) = \mathbf{0.7443 \pm j 0.6663} \quad (|\lambda| = 0.9990)$$
3. **Second Harmonic Conjugate Pair ($116.21\,\text{Hz}$)**:
   $$\lambda_{2, 1,2}^{\text{OL}} = \rho_2 e^{\pm j \omega_2 \Delta t} = 0.999 \left( \cos(1.4604) \pm j \sin(1.4604) \right) = \mathbf{0.1098 \pm j 0.9929} \quad (|\lambda| = 0.9990)$$

In closed-loop estimation, the observer error dynamics are governed by the closed-loop system matrix:
$$\mathbf{A}_{\text{cl}} = (\mathbf{I} - \mathbf{K}_\infty \mathbf{H}) \mathbf{F}$$
where the steady-state Kalman gain $\mathbf{K}_\infty = [0.9178, 0.0537, 0.1153, 0.0247, 0.0310]^T$ acts as an optimal pole-placement feedback matrix.

#### 5.8.2 Kinematic Pole Migration & Transient Response Speed
The open-loop kinematic pole at $z = 1.0$ is shifted dramatically along the real axis deep into the interior of the unit circle:
$$z_{\text{cl, kin}} \approx 1.0 - K_{\infty, 1} = 1.0 - 0.9178 = \mathbf{0.0822}$$

##### Mathematical & Physical Implications:
- **Equivalent Continuous Pole Location**: Mapping back to the Laplace $s$-plane via $z = e^{s \Delta t}$:
  $$s_{\text{kin}} = \frac{\ln(z_{\text{cl, kin}})}{\Delta t} = \frac{\ln(0.0822)}{0.002\,\text{s}} = \frac{-2.4986}{0.002} = \mathbf{-1{,}249.3\,\text{rad/s}} \quad (\omega_{\text{bandwidth}} \approx 198.8\,\text{Hz})$$
- **Transient Decay Time Constant**:
  $$\tau_{\text{kin}} = \frac{1}{|s_{\text{kin}}|} = \frac{1}{1{,}249.3\,\text{rad/s}} = \mathbf{0.80\,\text{ms}} < 1\,\text{sample}$$
- **Zero Overshoot & Impulse Fidelity**: Because $z_{\text{cl, kin}}$ sits strictly on the positive real axis with zero imaginary component, the state transition response is **critically overdamped**. When a violent touchdown landing shock ($>18.0\,\text{m/s}^2$) hits the chassis, the kinematic state $\hat{a}_{\text{kin}}$ converges to the true peak within a single time step without any oscillatory Gibbs ringing or phase lag, explaining the empirical **$100.0\%$ peak capture ratio**.

#### 5.8.3 Harmonic Disturbance Pole Inward Contraction & Notch Bandwidth
Under the influence of the DARE solution, the open-loop resonant poles on the unit circle ($\rho = 0.999$) contract radially inward toward the origin:
- **Closed-Loop Fundamental Poles**: $z_{\text{cl}, 1} = r_1 e^{\pm j \tilde{\omega}_1 \Delta t}$, where $r_1 \approx \mathbf{0.952}$ and $\tilde{\omega}_1 \approx 2\pi(58.08)\,\text{rad/s}$.
- **Closed-Loop Second Harmonic Poles**: $z_{\text{cl}, 2} = r_2 e^{\pm j \tilde{\omega}_2 \Delta t}$, where $r_2 \approx \mathbf{0.971}$ and $\tilde{\omega}_2 \approx 2\pi(116.18)\,\text{rad/s}$.

##### Control Rationale & Notch Bandwidth Law:
The radial contraction factor $(1 - r_{\text{cl}})$ directly determines the $-3\,\text{dB}$ notch transition bandwidth of the filter:
$$\Delta f_{-3\text{dB}} \approx \frac{f_s}{\pi} (1 - r_{\text{cl}})$$
- For the fundamental engine mode: $\Delta f_{-3\text{dB}} \approx \frac{500}{\pi} (1 - 0.952) = \mathbf{7.64\,\text{Hz}}$.
- *Operational Significance*: Rather than creating an infinitely sharp Dirac notch that fails if engine RPM drifts slightly, the Riccati optimization places the closed-loop poles at $r_1 = 0.952$. This creates a robust $-3\,\text{dB}$ notch window of $\pm 3.8\,\text{Hz}$ around $58.11\,\text{Hz}$ ($3{,}250 - 3{,}720\,\text{RPM}$), absorbing micro-fluctuations in motorcycle idle speed without leaking vibration into kinematics.

#### 5.8.4 Transmission Zeros & The Internal Model Principle
Evaluating the closed-loop transfer function from measurement $y[k]$ to estimated kinematics $\hat{a}_{\text{kin}}[k]$:
$$H_{\text{KF}}(z) = \mathbf{C} (z\mathbf{I} - \mathbf{A}_{\text{cl}})^{-1} \mathbf{K}_\infty = \frac{b_4 z^4 + b_3 z^3 + b_2 z^2 + b_1 z + b_0}{\det(z\mathbf{I} - \mathbf{A}_{\text{cl}})}$$
By Francis & Wonham's theorem, the zeros of $H_{\text{KF}}(z)$ coincide identically with the **open-loop disturbance poles**:
$$z_{\text{zero}, 1} = \rho_1 e^{\pm j \omega_1 \Delta t}, \quad z_{\text{zero}, 2} = \rho_2 e^{\pm j \omega_2 \Delta t}$$
At exactly $f = 58.11\,\text{Hz}$ and $f = 116.21\,\text{Hz}$, the numerator evaluates identically to zero ($H_{\text{KF}}(e^{j\omega_1 \Delta t}) \equiv 0$). This creates infinite-depth notch transmission zeros, while the denominator closed-loop poles at $r_1 = 0.952$ restrict the notch depth attenuation to $-34.0\,\text{dB}$ causal and $-68.0\,\text{dB}$ smoothed, preserving full transmission ($>-0.11\,\text{dB}$) across the adjacent $0-40\,\text{Hz}$ chassis motion spectrum.

#### 5.8.5 Root Locus Tuning via Process-to-Measurement Noise Ratios
The root-locus trajectories of the Kalman filter are parameterized by the stochastic noise ratios:
1. **Kinematic Root Locus Parameter**: $\gamma_{\text{kin}} = \frac{q_a \Delta t}{R} = \frac{1.0}{0.00425} \approx \mathbf{235.3}$.
   - As $q_a / R \to \infty$: The kinematic closed-loop pole moves to $z \to 0$ (instantaneous tracking, deadbeat observer).
   - As $q_a / R \to 0$: The kinematic closed-loop pole moves to $z \to 1$ (heavy filtering, large group delay).
2. **Harmonic Disturbance Root Locus Parameter**: $\gamma_{\text{dist}} = \frac{q_{d1}}{R} = \frac{0.01}{0.00425} \approx \mathbf{2.35}$.
   - Larger $q_{d1}$: Pulls harmonic poles deeper into the unit circle ($r \to 0.90$), widening the notch and accelerating tracking of rapid throttle revs.
   - Smaller $q_{d1}$: Pushes harmonic poles toward the unit circle ($r \to 0.999$), narrowing the notch to an ultra-sharp knife edge but slowing transient settling time.

---

## 6. Sampling Theory, Aliasing & Downsampling (Objective 3)

```
                            [ Continuous IMU Signal ]
                                       |
                   +-------------------+-------------------+
                   |                                       |
                   v                                       v
        [ Pipeline A: Naive ]                   [ Pipeline B: Anti-Aliased ]
        Subsampling without AAF                 Pre-filter with Butterworth LP
                   |                                       |
                   v                                       v
         Engine Harmonics Fold                   Engine Harmonics Eradicated
        f_alias = |f_engine - k*fs|                     Prior to Subsampling
                   |                                       |
                   +-------------------+-------------------+
                                       |
                                       v
                         [ Sweep: 500 Hz -> 0.5 Hz ]
```

### 6.1 Shannon-Nyquist Sampling Criterion vs Event Detection Limits
A crucial theoretical distinction must be drawn between **waveform reconstruction** and **kinematic event detection**:
1. **Continuous Signal Reconstruction (Shannon-Nyquist Theorem)**:
   A continuous-time bandlimited signal $x(t)$ with maximum spectral frequency $f_{\text{max}}$ can be reconstructed perfectly without distortion if and only if:
   $$f_s \ge 2 f_{\text{max}}$$
   Because the reciprocating engine produces tones at $58.11\,\text{Hz}$ and $116.21\,\text{Hz}$, full signal reconstruction requires $f_s > 232.42\,\text{Hz}$.
2. **Discrete Kinematic Event Detection**:
   A jump event is not an arbitrary high-frequency sinusoid; it is a **macroscopic state transition** parameterized by a sustained ballistic dwell time $\tau_{\text{air}} \in [0.934, 4.746]\,\text{s}$.
   By the fundamental theorem of discrete event sampling, an event of duration $\tau_{\text{event}}$ is captured if at least one sample falls strictly within the interval:
   $$\Delta t = \frac{1}{f_s} < \tau_{\text{event}} \implies f_s > \frac{1}{\tau_{\text{event}}}$$
   For the shortest jump in the dataset (Jump 3, $\tau_{\text{jump}} = 0.934\,\text{s}$):
   $$f_{s,\text{min,event}} > \frac{1}{0.934\,\text{s}} = \mathbf{1.071\,\text{Hz}}$$
   At $f_s = 1.0\,\text{Hz}$ ($\Delta t = 1.000\,\text{s}$), all 5 jumps contain at least one free-fall sample ($\|\mathbf{a}\| \le 2.0\,\text{m/s}^2$), yielding a **$100\%$ detection rate ($F_1 = 1.0$)**. At $f_s = 0.5\,\text{Hz}$ ($\Delta t = 2.000\,\text{s}$), the sampling interval exceeds the duration of Jump 1, Jump 2, Jump 3, and Jump 5, causing complete detection failure.

### 6.2 Spectral Aliasing & The Nyquist Folding Formula
When telemetry is sampled at $f_s$ without an Anti-Aliasing Filter (AAF), high-frequency spectral components above the Nyquist boundary $f_{\text{Nyq}} = f_s / 2$ fold back into the baseband $[0, f_{\text{Nyq}}]$. The apparent aliased frequency $f_{\text{alias}}$ is governed by:
$$f_{\text{alias}} = \left\vert f_{\text{source}} - k \cdot f_s \right\vert \le \frac{f_s}{2}, \quad k = \text{round}\left( \frac{f_{\text{source}}}{f_s} \right) \in \mathbb{Z}$$

#### Theoretical vs Measured Nyquist Folding Frequencies:
| Sampling Rate ($f_s$) | Nyquist Limit ($f_{\text{Nyq}}$) | Fundamental ($58.11\,\text{Hz}$) Fold | Second Harmonic ($116.21\,\text{Hz}$) Fold | Verification Status |
| :---: | :---: | :---: | :---: | :--- |
| **$500.0\,\text{Hz}$** | $250.0\,\text{Hz}$ | $58.11\,\text{Hz}$ (True) | $116.21\,\text{Hz}$ (True) | Unaliased Ground Truth |
| **$250.0\,\text{Hz}$** | $125.0\,\text{Hz}$ | $58.11\,\text{Hz}$ (True) | $\vert 116.21 - 250 \vert = 133.79 \to \mathbf{116.21\,\text{Hz}}$ | Unaliased |
| **$125.0\,\text{Hz}$** | $62.5\,\text{Hz}$ | $58.11\,\text{Hz}$ (True) | $\vert 116.21 - 125 \vert = \mathbf{8.79\,\text{Hz}}$ | Aliased into Low Band |
| **$100.0\,\text{Hz}$** | $50.0\,\text{Hz}$ | $\vert 58.11 - 100 \vert = \mathbf{41.89\,\text{Hz}}$ | $\vert 116.21 - 100 \vert = \mathbf{16.21\,\text{Hz}}$ | Both Harmonics Aliased |
| **$50.0\,\text{Hz}$** | $25.0\,\text{Hz}$ | $\vert 58.11 - 50 \vert = \mathbf{8.11\,\text{Hz}}$ | $\vert 116.21 - 2(50) \vert = \mathbf{16.21\,\text{Hz}}$ | Aliased into Chassis Band |
| **$25.0\,\text{Hz}$** | $12.5\,\text{Hz}$ | $\vert 58.11 - 2(25) \vert = \mathbf{8.11\,\text{Hz}}$ | $\vert 116.21 - 5(25) \vert = \mathbf{8.79\,\text{Hz}}$ | Aliased into Rider Heave Band |
| **$20.0\,\text{Hz}$** | $10.0\,\text{Hz}$ | $\vert 58.11 - 3(20) \vert = \mathbf{1.89\,\text{Hz}}$ | $\vert 116.21 - 6(20) \vert = \mathbf{3.79\,\text{Hz}}$ | Aliased into Pitch Dynamic Band |

### 6.3 Anti-Aliasing Filter (AAF) Engineering Verdict
- **For Jump Event Detection**: **AAF is strictly NOT required**.
  *Mathematical Proof*: In flight, the engine uncouples from ground load and drops to idle or closed throttle. Even if the engine continues vibrating, its acceleration amplitude is envelope-modulated:
  $$a_z(t) = a_{\text{kin}}(t) + A_{\text{vib}} \cos(2\pi f_e t)$$
  During free-fall, $a_{\text{kin}}(t) = 0\,\text{m/s}^2$. The peak vibration amplitude is $A_{\text{vib}} \le 0.45\,\text{m/s}^2$. When folded to any aliased frequency $f_{\text{alias}}$, the total signal magnitude remains bounded by:
  $$\|a(t)\| \le 0 + A_{\text{vib}} \le 0.45\,\text{m/s}^2 \ll a_{\text{takeoff}} = 2.0\,\text{m/s}^2$$
  The signal cannot cross the $2.0\,\text{m/s}^2$ threshold while airborne, guaranteeing that naive decimation achieves identical $100\%$ detection accuracy down to $1.0\,\text{Hz}$.
- **For Waveform Reconstruction & Shock Capture**: **AAF is mandatory**.
  Without an AAF, high-frequency aliasing introduces spurious low-frequency oscillations into continuous kinematics and causes severe landing shock distortion.

### 6.4 Timestamp Quantization Uncertainty
Downsampling introduces discrete temporal quantization jitter into takeoff and landing timestamps. Modeled as a uniform random variable $e_t \sim \mathcal{U}\left(-\frac{\Delta t}{2}, \frac{\Delta t}{2}\right)$:
- Mean error: $\mathbb{E}[e_t] = 0\,\text{s}$
- Variance: $\sigma_t^2 = \frac{\Delta t^2}{12}$
- Standard Deviation:
  $$\sigma_t = \frac{\Delta t}{\sqrt{12}} = \frac{1}{f_s \sqrt{12}}$$
At $f_s = 20\,\text{Hz}$ ($\Delta t = 50\,\text{ms}$), timestamp jitter is $\sigma_t = \frac{0.050}{\sqrt{12}} = 14.4\,\text{ms}$, which represents less than $1.5\%$ error relative to a $1.0\,\text{s}$ jump.

### 6.5 Landing Shock Impulse Smearing & Capture Ratio
The landing shock is an impulsive transient with half-width $\Delta t_{\text{impulse}} \approx 25\,\text{ms}$. The empirical peak capture ratio is defined as:
$$\eta(f_s) = \frac{\max(a_{z, f_s})}{\max(a_{z, 500\,\text{Hz}})} \times 100\%$$

| Sampling Rate ($f_s$) | Sample Spacing ($\Delta t$) | Naive Peak Shock ($a_{z,\text{max}}$) | Naive Capture ($\eta_{\text{naive}}$) | AAF Peak Shock ($a_{z,\text{max}}$) | AAF Capture ($\eta_{\text{aaf}}$) |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **$500.0\,\text{Hz}$** | $2.0\,\text{ms}$ | $18.66\,\text{m/s}^2$ | **$100.0\%$** | $17.31\,\text{m/s}^2$ | $92.8\%$ |
| **$250.0\,\text{Hz}$** | $4.0\,\text{ms}$ | $18.66\,\text{m/s}^2$ | **$100.0\%$** | $17.31\,\text{m/s}^2$ | $92.8\%$ |
| **$100.0\,\text{Hz}$** | $10.0\,\text{ms}$ | $18.32\,\text{m/s}^2$ | **$98.2\%$** | $17.20\,\text{m/s}^2$ | $92.2\%$ |
| **$50.0\,\text{Hz}$** | $20.0\,\text{ms}$ | $17.84\,\text{m/s}^2$ | **$95.6\%$** | $16.92\,\text{m/s}^2$ | $90.7\%$ |
| **$25.0\,\text{Hz}$** | $40.0\,\text{ms}$ | $16.12\,\text{m/s}^2$ | **$86.4\%$** | $15.40\,\text{m/s}^2$ | $82.5\%$ |
| **$20.0\,\text{Hz}$** | $50.0\,\text{ms}$ | $15.48\,\text{m/s}^2$ | **$83.0\%$** | $14.80\,\text{m/s}^2$ | $79.3\%$ |
| **$10.0\,\text{Hz}$** | $100.0\,\text{ms}$ | $12.80\,\text{m/s}^2$ | **$68.6\%$** | $12.10\,\text{m/s}^2$ | $64.8\%$ |
| **$1.0\,\text{Hz}$** | $1000.0\,\text{ms}$ | $9.82\,\text{m/s}^2$ | **$52.6\%$** | $9.81\,\text{m/s}^2$ | $52.6\%$ |

*Takeaway*: Downsampling below $25\,\text{Hz}$ severely smears the landing shock impulse because $\Delta t > \Delta t_{\text{impulse}}$, reducing measured peak $G$-load by $>15\%$.

---

## 7. Numerical Methods & Sensor Fault Imputation

At $t \in [23.000, 23.398]\,\text{s}$, a hardware sensor dropout caused exactly $200$ consecutive zero-valued samples ($\Delta t_{\text{gap}} = 0.400\,\text{s}$). Four interpolation algorithms were benchmarked:

```
[ Dropout Gap: t0 to t1 ]
       |
       +---> Zero-Order Hold (ZOH): p(t) = y(t0)                      [ C0 Discontinuous ]
       |
       +---> First-Order Hold (FOH): Linear Spline                   [ C0 Continuous ]
       |
       +---> PCHIP: Monotone Cubic Hermite Spline                    [ C1 Continuous ]
       |
       +---> Natural Cubic Spline: S''(t0) = S''(t1) = 0             [ C2 Continuous ]
```

### 7.1 Mathematical Formulations
1. **Zero-Order Hold (ZOH / Piecewise Constant)**:
   $$p_{\text{ZOH}}(t) = y(t_0) \quad \forall t \in [t_0, t_1)$$
2. **First-Order Hold (FOH / Linear Spline)**:
   $$p_{\text{FOH}}(t) = y(t_0) + \frac{y(t_1) - y(t_0)}{t_1 - t_0}(t - t_0)$$
3. **Piecewise Cubic Hermite Interpolating Polynomial (PCHIP - $C^1$ Monotonic)**:
   $$p(u) = h_{00}(u) y_0 + h_{10}(u) \Delta t \, d_0 + h_{01}(u) y_1 + h_{11}(u) \Delta t \, d_1, \quad u = \frac{t - t_0}{\Delta t} \in [0, 1]$$
   Hermite basis functions:
   $$h_{00}(u) = 2u^3 - 3u^2 + 1, \quad h_{10}(u) = u^3 - 2u^2 + u$$
   $$h_{01}(u) = -2u^3 + 3u^2, \quad h_{11}(u) = u^3 - u^2$$
   End-point tangents $d_k$ are constrained by the Brodlie-Fritsch-Carlson harmonic mean to guarantee monotonicity and eliminate overshoot:
   $$d_k = \begin{cases} \frac{2 \Delta_k \Delta_{k-1}}{\Delta_k + \Delta_{k-1}} & \text{if } \text{sgn}(\Delta_k) = \text{sgn}(\Delta_{k-1}) \\ 0 & \text{otherwise} \end{cases}$$
4. **Natural Cubic Spline ($C^2$ Continuous)**:
   Constructs a piecewise cubic polynomial $S_i(t)$ across knot sequence such that $S_i, S_i', S_i''$ are strictly continuous, subject to natural boundary constraints:
   $$S''(t_{\text{start}}) = S''(t_{\text{end}}) = 0$$

### 7.2 In-Flight Fault Imputation Benchmark (Jump 4 Outage)
To assess dynamic imputation accuracy during flight, a synthetic $0.582\,\text{s}$ outage ($N = 292$ samples) was induced during Jump 4 pitch dynamics ($t \in [56.198, 56.780]\,\text{s}$). The ground-truth trajectory was established via aerodynamic parabolic regression ($R^2 = 0.9955$):

$$\omega_{y,\text{baseline}}(t) = \beta_0 + \beta_1 t + \beta_2 t^2$$

| Imputation Scheme | Continuity | Velocity RMSE | Max Dynamic Error | Integrated Pitch Error ($\Delta\theta$) | Engineering Verdict |
| :--- | :---: | :---: | :---: | :---: | :--- |
| **Zero-Order Hold (ZOH)** | $C^{-1}$ | $0.0782\,\text{rad/s}$ | $0.1412\,\text{rad/s}$ | $3.24^\circ$ | **Unusable**: Induces step discontinuity |
| **Linear (FOH)** | $C^0$ | $0.0164\,\text{rad/s}$ | $0.0381\,\text{rad/s}$ | $0.52^\circ$ | **Best Edge Choice**: Zero overshoot, lowest compute |
| **PCHIP** | $C^1$ | **$0.0121\,\text{rad/s}$** | **$0.0245\,\text{rad/s}$** | **$0.21^\circ$** | **Optimal**: Preserves shape, perfectly bounded |
| **Natural Cubic Spline** | $C^2$ | $0.0345\,\text{rad/s}$ | $0.0892\,\text{rad/s}$ | $1.86^\circ$ | **Pathological**: Runge-type overshoot oscillations |

---

## 8. Sensor Fusion & Attitude Estimation

### 8.1 Accelerometer Tilt Inversion & The Free-Fall Singularity
In static or quasi-static conditions, pitch angle $\theta$ is extracted from the gravitational vector:
$$\theta_{\text{accel}} = \arctan\left( \frac{a_x}{\sqrt{a_y^2 + a_z^2}} \right)$$
*Singularity in Flight*: During free-fall jumps, $\|\mathbf{a}\| \to 0$. In this regime:
$$\lim_{\|\mathbf{a}\| \to 0} \arctan\left( \frac{a_x}{\sqrt{a_y^2 + a_z^2}} \right) \to \frac{0}{0} \quad (\text{Mathematically Undefined})$$
Any sensor noise or engine vibration creates violent numerical attitude swings.

### 8.2 Attitude Sensor Fusion: State-Switched Complementary Filter

In motocross telemetry, raw gyroscope integration exhibits unbounded low-frequency drift due to MEMS bias instability, while raw accelerometer tilt inversion suffers from severe high-frequency chassis vibration and kinematic centripetal acceleration. Sensor fusion combines the complementary spectral strengths of both transducers:

```
Gyroscope Rate wy [rad/s] ----> [ High-Pass Filter: s / (s + wc) ] ----+
                                                                       |
                                                                       v
                                                                      (+) ----> Pitch Attitude θ_hat [rad]
                                                                       ^
                                                                       |
Accel Tilt θ_accel [rad] -----> [ Low-Pass Filter:  wc / (s + wc) ] ---+
```

#### Mathematical Formulation & Spectral Splitting
In continuous time, the estimated pitch attitude $\hat{\Theta}(s)$ is formed by spectral all-pass blending:
$$\hat{\Theta}(s) = G_{\text{gyro}}(s) \left( \frac{1}{s} \Omega_y(s) \right) + G_{\text{accel}}(s) \Theta_{\text{accel}}(s)$$
where the complementary filter transfer functions satisfy:
$$G_{\text{gyro}}(s) = \frac{s}{s + \omega_c}, \quad G_{\text{accel}}(s) = \frac{\omega_c}{s + \omega_c} \implies G_{\text{gyro}}(s) + G_{\text{accel}}(s) = \frac{s + \omega_c}{s + \omega_c} \equiv 1.0 \quad (\text{All-Pass})$$
Here $\omega_c = 2\pi f_c = \frac{1}{\tau} \approx 2.0\,\text{rad/s}$ ($f_c \approx 0.32\,\text{Hz}$) defines the crossover blending frequency.

#### Discrete-Time Realization
Discretizing via backward Euler integration:
$$\hat{\theta}[k] = \alpha_k \left( \hat{\theta}[k-1] + \omega_y[k] \Delta t \right) + (1 - \alpha_k) \theta_{\text{accel}}[k]$$
$$\alpha_k = \frac{\tau}{\tau + \Delta t} = \frac{0.50}{0.50 + 0.002} = \mathbf{0.9960} \quad (\text{Nominal Ground Mode})$$

#### State-Switched Airborne Singularity Protection
During ballistic free-fall jumps, $\|\mathbf{a}\| \to 0\,\text{m/s}^2$, rendering $\theta_{\text{accel}} = \arctan(a_x / \sqrt{a_y^2 + a_z^2}) \to \frac{0}{0}$ mathematically undefined (Section 8.1). The filter employs a deterministic state-switched blending gain:
$$\alpha_k = \begin{cases} \frac{\tau}{\tau + \Delta t} \approx 0.996 & \text{if } \|\mathbf{a}[k]\| \ge a_{\text{thresh}} \quad (\text{Ground State: High-pass gyro + Low-pass accel}) \\ 1.0000 & \text{if } \|\mathbf{a}[k]\| < a_{\text{thresh}} \quad (\text{Airborne State: 100\% Pure Gyro Dead-Reckoning}) \end{cases}$$
where $a_{\text{thresh}} = 2.0\,\text{m/s}^2$. In flight, $(1 - \alpha_k) \equiv 0$, completely severing the accelerometer input and freezing pitch estimation to pure kinematic gyroscopic integration.

#### Architectural Limitations of the Complementary Filter:
1. **Zero Gyro Bias Estimation**: Does not model gyroscope bias $b_g(t)$ as an internal dynamic state. Any uncalibrated gyro offset integrates linearly into attitude error: $\Delta \theta = b_g \cdot \Delta t$.
2. **Rigid Static Bandwidth**: Crossover frequency $\omega_c$ cannot adapt dynamically to changing vibration intensities or terrain roughness.

---

### 8.3 Sensor Fusion: Linear & Extended Kalman Filter (Attitude & Bias Estimation)

To overcome the limitations of the complementary filter, attitude estimation is formulated in an optimal stochastic state-space framework that simultaneously tracks true vehicle attitude and estimates dynamic MEMS sensor bias drift online.

#### 8.3.1 State-Space Formulation for Planar Attitude
The state vector augments pitch attitude with the slowly drifting gyroscope rate bias:
$$\mathbf{x}_{\text{att}}[k] = \begin{bmatrix} \theta[k] \\ b_g[k] \end{bmatrix} \in \mathbb{R}^2$$
where $\theta[k]$ is pitch attitude ($\text{rad}$) and $b_g[k]$ is pitch-gyro rate bias ($\text{rad/s}$).

##### Continuous-Time Dynamic Model:
$$\dot{\theta}(t) = \omega_y(t) - b_g(t) + w_\theta(t), \quad w_\theta(t) \sim \mathcal{N}(0, q_\theta)$$
$$\dot{b}_g(t) = w_b(t), \quad w_b(t) \sim \mathcal{N}(0, q_b)$$
where $q_\theta$ is the gyro Angular Random Walk (ARW) power spectral density and $q_b$ is the rate random walk bias instability spectral density.

##### Discrete-Time State Transition Model ($\Delta t = 2.0\,\text{ms}$):
$$\mathbf{x}_{\text{att}}[k+1] = \mathbf{F}_{\text{att}} \mathbf{x}_{\text{att}}[k] + \mathbf{B}_{\text{att}} \omega_y[k] + \mathbf{w}_{\text{att}}[k]$$
$$\mathbf{F}_{\text{att}} = \begin{bmatrix} 1 & -\Delta t \\ 0 & 1 \end{bmatrix}, \quad \mathbf{B}_{\text{att}} = \begin{bmatrix} \Delta t \\ 0 \end{bmatrix}$$
$$\mathbf{Q}_{\text{att}} = \int_0^{\Delta t} \mathbf{F}(\tau) \begin{bmatrix} q_\theta & 0 \\ 0 & q_b \end{bmatrix} \mathbf{F}^T(\tau) \, d\tau = \begin{bmatrix} q_\theta \Delta t + \frac{1}{3} q_b \Delta t^3 & -\frac{1}{2} q_b \Delta t^2 \\ -\frac{1}{2} q_b \Delta t^2 & q_b \Delta t \end{bmatrix}$$

##### Observation Model:
The measurement is provided by accelerometer gravity vector tilt inversion:
$$z_{\text{att}}[k] = \theta_{\text{accel}}[k] = \mathbf{H}_{\text{att}} \mathbf{x}_{\text{att}}[k] + v_{\text{att}}[k], \quad \mathbf{H}_{\text{att}} = \begin{bmatrix} 1 & 0 \end{bmatrix}, \quad v_{\text{att}}[k] \sim \mathcal{N}(0, R_{\text{att}})$$

#### 8.3.2 Steady-State Equivalence to Complementary Filter
Solving the Discrete Algebraic Riccati Equation for $\mathbf{P}_{\text{att}}$ yields the steady-state Kalman gain vector:
$$\mathbf{K}_{\text{att},\infty} = \begin{bmatrix} K_1 \\ K_2 \end{bmatrix} = \begin{bmatrix} \frac{P_{11}}{P_{11} + R_{\text{att}}} \\ \frac{P_{12}}{P_{11} + R_{\text{att}}} \end{bmatrix}$$
The state measurement update equation for pitch is:
$$\hat{\theta}_{k|k} = \hat{\theta}_{k|k-1} + K_1 \left( \theta_{\text{accel}}[k] - \hat{\theta}_{k|k-1} \right) = (1 - K_1) \left( \hat{\theta}_{k-1} + (\omega_y[k] - \hat{b}_g) \Delta t \right) + K_1 \theta_{\text{accel}}[k]$$

##### Mathematical Equivalence Theorem:
If the gyroscope bias state is disabled ($q_b = 0 \implies K_2 = 0$), defining blending parameter $\alpha = 1 - K_1$ reproduces the exact discrete complementary filter:
$$\hat{\theta}_k = \alpha \left( \hat{\theta}_{k-1} + \omega_y[k] \Delta t \right) + (1 - \alpha) \theta_{\text{accel}}[k]$$
**Key Theoretical Insight**: The Complementary Filter is fundamentally a degenerate, steady-state special case of the Kalman Filter where gyro bias dynamics are neglected. The Kalman filter provides a strictly superior formulation by actively tracking bias drift $\hat{b}_g[k]$ via $K_2$, eliminating long-term angle ramps during prolonged operation.

#### 8.3.3 Dynamic Measurement Covariance Scaling (Continuous Free-Fall Protection)
Instead of heuristic binary gain switching, the Kalman filter naturally and continuously handles the free-fall gravity singularity through **specific force residual covariance inflation**:
$$R_{\text{att}}[k] = R_{\text{nominal}} \cdot \left[ 1 + \kappa \cdot \left( \frac{\left\vert \Vert \mathbf{a}[k] \Vert - g \right\vert}{g} \right)^2 \right]$$
- **On Stationary / Smooth Ground** ($\|\mathbf{a}\| \approx g$): $R_{\text{att}}[k] \approx R_{\text{nominal}}$, giving nominal Kalman gain $\mathbf{K} \approx [0.004, 0.0001]^T$.
- **During Motocross Free-Fall Flight** ($\|\mathbf{a}\| \to 0$): The deviation $|\mathbf{a} - g| \to g$ triggers covariance inflation ($R_{\text{att}} \to \infty$). Consequently:
  $$S_k = \mathbf{H} \mathbf{P} \mathbf{H}^T + R_{\text{att}} \to \infty \implies \mathbf{K}_k = \frac{\mathbf{P} \mathbf{H}^T}{S_k} \to \begin{bmatrix} 0 \\ 0 \end{bmatrix}$$
  The Kalman gain smoothly vanishes to zero without requiring hard if/else branch logic, mathematically decoupling attitude from the $\frac{0}{0}$ singularity and freezing bias estimation throughout airborne flight.

#### 8.3.4 Full 3D Attitude Quaternion Extended Kalman Filter (Q-EKF)
For full 3D attitude estimation across roll ($\phi$), pitch ($\theta$), and yaw ($\psi$) without gimbal lock, the state vector is parameterized by unit quaternion $\mathbf{q} = [q_0, q_1, q_2, q_3]^T \in \mathbb{H}$ ($\|\mathbf{q}\| = 1$) and 3-axis gyro bias $\mathbf{b}_g = [b_{gx}, b_{gy}, b_{gz}]^T$:
$$\mathbf{x}_{\text{q}} = \begin{bmatrix} \mathbf{q}^T & \mathbf{b}_g^T \end{bmatrix}^T \in \mathbb{R}^7$$

##### Quaternion Kinematics:
$$\dot{\mathbf{q}}(t) = \frac{1}{2} \mathbf{q}(t) \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega}^b(t) - \mathbf{b}_g(t) \end{bmatrix} = \frac{1}{2} \mathbf{\Xi}(\mathbf{q}) \left( \boldsymbol{\omega}^b - \mathbf{b}_g \right)$$
$$\mathbf{\Xi}(\mathbf{q}) = \begin{bmatrix} -q_1 & -q_2 & -q_3 \\ q_0 & -q_3 & q_2 \\ q_3 & q_0 & -q_1 \\ -q_2 & q_1 & q_0 \end{bmatrix}$$
Observation of Earth gravity vector $\mathbf{g}^n = [0, 0, +g]^T$ transformed to body frame:
$$\mathbf{h}(\mathbf{q}) = \mathbf{R}_n^b(\mathbf{q}) \mathbf{g}^n = \begin{bmatrix} 2(q_1 q_3 - q_0 q_2) g \\ 2(q_2 q_3 + q_0 q_1) g \\ (q_0^2 - q_1^2 - q_2^2 + q_3^2) g \end{bmatrix}$$

---

### 8.4 GPS/INS Multi-Rate Sensor Fusion Extension (15-State Error-State Kalman Filter - ESKF)

While IMU sensors capture high-rate rotational rates and linear accelerations, double-integrating accelerometer measurements to extract position and velocity incurs catastrophic drift:
$$\mathbf{p}(t) = \mathbf{p}_0 + \mathbf{v}_0 t + \iint (\mathbf{a}_{\text{meas}}(t) - \mathbf{b}_a) \, dt^2 = \mathbf{p}_{\text{true}}(t) + \underbrace{\frac{1}{2} \mathbf{b}_a t^2}_{\text{Quadratic Drift}} + \underbrace{\frac{1}{6} \left(\frac{d\mathbf{b}_a}{dt}\right) t^3}_{\text{Cubic Drift}}$$
An accelerometer bias of merely $0.05\,\text{m/s}^2$ accumulates into **$202.5\,\text{m}$ of positional error over a $90\,\text{s}$ motocross session**.

Integrating a Global Positioning System (GPS / GNSS) receiver provides an absolute, drift-free Earth-referenced position and Doppler velocity reference, completely bounding IMU drift.

#### 8.4.1 Kinematic Jump Analytics Unlocked by GPS/INS Integration
Fusing GPS with IMU telemetry unlocks macroscopic jump ballistics that are fundamentally unobservable from a standalone 6-axis IMU:
1. **True Launch Ground Speed**: $\mathbf{v}_{\text{takeoff}}^n = [v_N, v_E, v_D]^T \implies v_{\text{launch}} = \sqrt{v_N^2 + v_E^2}$.
2. **Ballistic Launch Pitch Angle**: $\gamma_{\text{launch}} = \arctan\left( \frac{-v_D}{\sqrt{v_N^2 + v_E^2}} \right)$.
3. **True Ballistic Parabolic Flight Path**:
   $$\mathbf{r}^n(t) = \mathbf{r}^n(t_{\text{to}}) + \mathbf{v}^n(t_{\text{to}}) (t - t_{\text{to}}) + \frac{1}{2} \mathbf{g}^n (t - t_{\text{to}})^2$$
4. **Horizontal Jump Distance**:
   $$d_{\text{jump}} = \int_{t_{\text{to}}}^{t_{\text{td}}} \|\mathbf{v}_{\text{horiz}}^n(t)\| \, dt \approx v_{\text{launch}} \cdot \tau_{\text{air}}$$
5. **Apex Flight Altitude**:
   $$h_{\text{apex}} = z(t_{\text{apex}}) - z(t_{\text{to}}) = \frac{v_{D,\text{takeoff}}^2}{2g}$$
6. **Touchdown Landing Vector & Momentum**: Measures exact velocity vector upon tyre contact to calculate true mechanical energy absorption: $E_{\text{impact}} = \frac{1}{2} m (v_{z,\text{impact}})^2$.

#### 8.4.2 Multi-Rate Asynchronous Architecture
The GPS/INS fusion engine operates across two asynchronous time scales:
- **High-Rate Strapdown Mechanization (IMU @ $500\,\text{Hz}$)**: Propagates nominal kinematic states forward in time at $\Delta t = 2.0\,\text{ms}$.
- **Low-Rate Measurement Updates (GPS @ $5-10\,\text{Hz}$)**: Corrects accumulated error states and sensor biases whenever a GPS PVT (Position, Velocity, Time) packet arrives ($\Delta t_{\text{GPS}} \approx 100-200\,\text{ms}$, with transport latency $\tau_{\text{delay}} \approx 40\,\text{ms}$).

```
[ High-Rate IMU @ 500 Hz ] ----> [ Strapdown Mechanization ] ----> Nominal State x_nom [p, v, q]
                                            ^                                 |
                                            | Error Reset                     v
                                  +-------------------+              +------------------+
                                  | Error-State ESKF  | <----------- | GPS Innovation:  |
                                  | 15-State [δx]     |              | y_GPS - h(x_nom) |
                                  +-------------------+              +------------------+
                                            ^                                 ^
                                            |                                 |
                                  [ Q_cov Propagation ]              [ Low-Rate GPS @ 10 Hz ]
```

#### 8.4.3 15-State Error-State Representation (ESKF / MEKF)
In aerospace and vehicle navigation, the **Error-State Kalman Filter (ESKF)** is strictly preferred over standard EKF because the true errors remain small and linearized around the nominal trajectory, preventing non-linear divergence:
$$\mathbf{x}_{\text{true}} = \mathbf{x}_{\text{nom}} \oplus \delta\mathbf{x}$$
The 15-dimensional error state vector is defined as:
$$\delta\mathbf{x} = \begin{bmatrix} \delta\mathbf{p}^n & \delta\mathbf{v}^n & \delta\boldsymbol{\theta}^n & \delta\mathbf{b}_a^b & \delta\mathbf{b}_g^b \end{bmatrix}^T \in \mathbb{R}^{15}$$
where:
- $\delta\mathbf{p}^n = [\delta p_N, \delta p_E, \delta p_D]^T$: Position error in navigation frame ($[m]$).
- $\delta\mathbf{v}^n = [\delta v_N, \delta v_E, \delta v_D]^T$: Velocity error in navigation frame ($[m/s]$).
- $\delta\boldsymbol{\theta}^n = [\delta\phi, \delta\theta, \delta\psi]^T$: Small angle orientation error vector ($[\text{rad}]$), where $\mathbf{R}_{\text{true}} \approx (\mathbf{I} - [\delta\boldsymbol{\theta}^n]_\times) \hat{\mathbf{R}}$.
- $\delta\mathbf{b}_a^b = [\delta b_{ax}, \delta b_{ay}, \delta b_{az}]^T$: 3-axis accelerometer bias error in body frame ($[m/s^2]$).
- $\delta\mathbf{b}_g^b = [\delta b_{gx}, \delta b_{gy}, \delta b_{gz}]^T$: 3-axis gyroscope bias error in body frame ($[\text{rad/s}]$).

#### 8.4.4 Continuous Error Dynamics Matrix ($\mathbf{F}_{\text{err}}$)
$$\delta\dot{\mathbf{x}}(t) = \mathbf{F}_{\text{err}}(t) \delta\mathbf{x}(t) + \mathbf{w}_{\text{err}}(t)$$
$$\mathbf{F}_{\text{err}} = \begin{bmatrix}
\mathbf{0}_{3\times 3} & \mathbf{I}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} \\
\mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & -[\mathbf{R}_b^n \hat{\mathbf{a}}^b]_\times & \mathbf{R}_b^n & \mathbf{0}_{3\times 3} \\
\mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & -[\hat{\boldsymbol{\omega}}^b]_\times & \mathbf{0}_{3\times 3} & -\mathbf{R}_b^n \\
\mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} \\
\mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3}
\end{bmatrix}$$
where $[\mathbf{u}]_\times$ is the standard skew-symmetric cross-product matrix:
$$[\mathbf{u}]_\times = \begin{bmatrix} 0 & -u_z & u_y \\ u_z & 0 & -u_x \\ -u_y & u_x & 0 \end{bmatrix}$$

#### 8.4.5 Antenna Lever-Arm Vector Correction
On a motocross motorcycle, the GPS antenna is mounted on the rear fender or handlebar, separated from the headstock IMU by a rigid physical lever-arm offset vector $\mathbf{l}^b = [l_x, l_y, l_z]^T$:
$$\mathbf{p}_{\text{ant}}^n = \mathbf{p}_{\text{IMU}}^n + \mathbf{R}_b^n \mathbf{l}^b$$
$$\mathbf{v}_{\text{ant}}^n = \mathbf{v}_{\text{IMU}}^n + \mathbf{R}_b^n \left( \boldsymbol{\omega}^b \times \mathbf{l}^b \right)$$
The GPS measurement vector is:
$$\mathbf{z}_{\text{GPS}} = \begin{bmatrix} \mathbf{p}_{\text{GPS}}^n \\ \mathbf{v}_{\text{GPS}}^n \end{bmatrix} \in \mathbb{R}^6$$
The measurement innovation is:
$$\tilde{\mathbf{y}}_{\text{GPS}} = \mathbf{z}_{\text{GPS}} - \begin{bmatrix} \hat{\mathbf{p}}_{\text{IMU}}^n + \hat{\mathbf{R}}_b^n \mathbf{l}^b \\ \hat{\mathbf{v}}_{\text{IMU}}^n + \hat{\mathbf{R}}_b^n (\hat{\boldsymbol{\omega}}^b \times \mathbf{l}^b) \end{bmatrix} \in \mathbb{R}^6$$
The measurement Jacobian matrix $\mathbf{H}_{\text{GPS}} \in \mathbb{R}^{6 \times 15}$ evaluates to:
$$\mathbf{H}_{\text{GPS}} = \begin{bmatrix}
\mathbf{I}_{3\times 3} & \mathbf{0}_{3\times 3} & -[\hat{\mathbf{R}}_b^n \mathbf{l}^b]_\times & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} \\
\mathbf{0}_{3\times 3} & \mathbf{I}_{3\times 3} & -[\hat{\mathbf{R}}_b^n (\hat{\boldsymbol{\omega}}^b \times \mathbf{l}^b)]_\times & \mathbf{0}_{3\times 3} & -\hat{\mathbf{R}}_b^n [\mathbf{l}^b]_\times
\end{bmatrix}$$

#### 8.4.6 Error Injection & Covariance Reset
Upon computing the Kalman gain $\mathbf{K}_{\text{GPS}} = \mathbf{P} \mathbf{H}^T (\mathbf{H} \mathbf{P} \mathbf{H}^T + \mathbf{R}_{\text{GPS}})^{-1}$ and correcting the error state $\delta\hat{\mathbf{x}} = \mathbf{K}_{\text{GPS}} \tilde{\mathbf{y}}_{\text{GPS}}$:
1. **Nominal State Correction**:
   $$\hat{\mathbf{p}}^n \leftarrow \hat{\mathbf{p}}^n + \delta\hat{\mathbf{p}}^n$$
   $$\hat{\mathbf{v}}^n \leftarrow \hat{\mathbf{v}}^n + \delta\hat{\mathbf{v}}^n$$
   $$\hat{\mathbf{q}} \leftarrow \hat{\mathbf{q}} \otimes \begin{bmatrix} 1 \\ \frac{1}{2} \delta\hat{\boldsymbol{\theta}}^n \end{bmatrix}$$
   $$\hat{\mathbf{b}}_a^b \leftarrow \hat{\mathbf{b}}_a^b + \delta\hat{\mathbf{b}}_a^b, \quad \hat{\mathbf{b}}_g^b \leftarrow \hat{\mathbf{b}}_g^b + \delta\hat{\mathbf{b}}_g^b$$
2. **Error State Reset**: $\delta\hat{\mathbf{x}} \leftarrow \mathbf{0}_{15\times 1}$ (the error vector is cleared because errors have been absorbed into the nominal trajectory).
3. **Covariance Reset**: $\mathbf{P} \leftarrow (\mathbf{I} - \mathbf{K} \mathbf{H}) \mathbf{P} (\mathbf{I} - \mathbf{K} \mathbf{H})^T + \mathbf{K} \mathbf{R}_{\text{GPS}} \mathbf{K}^T$.

---

### 8.5 Control-Theoretic Pole Placement in Sensor Fusion (Attitude & Navigation)

Just as the Harmonic Disturbance Kalman Filter achieves optimal pole-zero cancellation (Section 5.8), the **Sensor Fusion Kalman Filters (Attitude EKF & GPS/INS ESKF)** place their closed-loop estimation poles automatically according to process and measurement noise statistics.

#### 8.5.1 Attitude Observer Closed-Loop Characteristic Polynomial
For the 2-state attitude-bias system (Section 8.3), the closed-loop error system matrix is:
$$\mathbf{A}_{\text{cl}} = \mathbf{A}_{\text{att}} - \mathbf{K} \mathbf{H}_{\text{att}} = \begin{bmatrix} 0 & -1 \\ 0 & 0 \end{bmatrix} - \begin{bmatrix} K_1 \\ K_2 \end{bmatrix} \begin{bmatrix} 1 & 0 \end{bmatrix} = \begin{bmatrix} -K_1 & -1 \\ -K_2 & 0 \end{bmatrix}$$
The continuous closed-loop characteristic polynomial is:
$$\det(s\mathbf{I} - \mathbf{A}_{\text{cl}}) = s(s + K_1) + K_2 = s^2 + K_1 s + K_2 = 0$$
Matching this to a standard second-order system:
$$s^2 + 2\zeta \omega_n s + \omega_n^2 = 0 \implies \omega_n = \sqrt{K_2}, \quad \zeta = \frac{K_1}{2\sqrt{K_2}}$$

#### 8.5.2 Optimal Butterworth Damping Derived from CARE
Solving the Continuous Algebraic Riccati Equation (CARE) analytically for state covariance matrix $\mathbf{P}$:
$$\mathbf{A} \mathbf{P} + \mathbf{P} \mathbf{A}^T - \mathbf{P} \mathbf{H}^T R^{-1} \mathbf{H} \mathbf{P} + \mathbf{Q} = \mathbf{0}$$
with $\mathbf{Q} = \text{diag}([q_\theta, q_b])$ and $R = R_{\text{att}}$:
$$K_1 = \sqrt{2 \frac{\sqrt{q_b R}}{R} + \frac{q_\theta}{R}}, \quad K_2 = \sqrt{\frac{q_b}{R}}$$
For high-quality MEMS gyroscopes where rate random walk dominates over angle noise ($q_\theta \ll \sqrt{q_b R}$):
$$\omega_n = \sqrt[4]{\frac{q_b}{R_{\text{att}}}}, \quad \zeta = \frac{K_1}{2\sqrt{K_2}} = \frac{\sqrt{2 \sqrt{q_b/R}}}{2 \sqrt[4]{q_b/R}} = \frac{\sqrt{2}}{2} = \mathbf{\frac{1}{\sqrt{2}} \approx 0.7071}$$

##### Engineering Significance:
The Kalman filter automatically places the closed-loop attitude estimation poles at a **Butterworth maximally flat damping factor of $\zeta = 1/\sqrt{2}$**. This guarantees:
- Maximum rejection of accelerometer high-frequency noise.
- Zero resonant gain peaking in the frequency response ($M_p = 0\,\text{dB}$).
- Rapid, critically damped convergence of the estimated gyroscope bias without oscillatory overshoot.

#### 8.5.3 Dynamic Root Locus Migration Across Motocross Regimes
The closed-loop pole locations migrate dynamically in the complex $s$-plane as riding conditions fluctuate:

```
                            Im(s)
                              ^
                              |       * Nominal Ground Pole (ζ = 0.707, ωn ≈ 2 rad/s)
                              |      /
                              |     /
                              |    /
       -----------------------+---+---------------------------> Re(s)
                              |  / \
                              | /   * Airborne Flight (R -> ∞):
                              |/      Poles collapse to origin s = 0 (Pure Integrator)
                              +       
                              | \
                              |  \
                              |   *
                              v
```

1. **Nominal Ground Riding**:
   $R_{\text{att}}$ reflects normal accelerometer variance $\implies \omega_n \approx 1.5 - 3.0\,\text{rad/s}$, placing conjugate poles at $s = -1.41 \pm j 1.41$ ($z \approx 0.997 \angle \pm 0.16^\circ$). Bias is estimated steadily within $3-5\,\text{s}$.
2. **Rough Track / Whoops Chatter**:
   Suspension chatter inflates the innovation variance $R_{\text{att}} \uparrow \uparrow \implies \omega_n \to 0$. The closed-loop poles migrate along the root locus toward the open-loop integrator pole at $s = 0$ ($z \to 1.0$). The filter automatically broadens its trust in the clean gyroscope and ignores noisy accelerometer leveling.
3. **Ballistic Airborne Free-Fall**:
   When $\|\mathbf{a}\| \le 2.0\,\text{m/s}^2$, $R_{\text{att}} \to \infty$. Both Kalman gains vanish: $K_1 \to 0, K_2 \to 0$. The closed-loop poles **collapse identically onto the open-loop double integrator at $s = 0$ ($z = 1.0$)**. This completely arrests bias updates and eliminates the gravitational $\frac{0}{0}$ singularity.
4. **GPS/INS Velocity Pole Placement**:
   In navigation fusion, the closed-loop poles for velocity and position errors scale with GPS Dilution of Precision (DOP). Under strong satellite geometry ($\text{HDOP} < 1.2$), the closed-loop velocity poles are placed at $z \approx 0.85-0.90$ (rapid, sub-second correction of IMU velocity drift). During satellite occlusion or jump aerial maneuvers where the rider's body blocks the antenna, $\mathbf{R}_{\text{GPS}} \to \infty$ smoothly pushes the closed-loop poles to $z = 1.0$, seamlessly transitioning into pure inertial dead-reckoning without trajectory discontinuity.

---

### 8.6 Post-Landing MEMS Shock Relaxation Model
Following the severe touchdown impact of Jump 4 ($t = 60.200\,\text{s}$, $a_z = 18.04\,\text{m/s}^2$), yaw angular rate $\omega_z$ exhibited non-kinematic upward drift from $0.00\,\text{rad/s}$ to $+0.44\,\text{rad/s}$ ($+25.2^\circ/\text{s}$), causing $>528^\circ$ of uncorrected heading rotation by $t = 90\,\text{s}$. 

This drift is modeled as a **viscoelastic mechanical stress relaxation** in the MEMS silicon die attach epoxy:
$$\omega_{z,\text{drift}}(t) = \omega_{z,\infty} + (\omega_{z,0} - \omega_{z,\infty}) e^{-(t - t_0)/\tau_{\text{relax}}}$$
Nonlinear least-squares curve fitting verified the model:
- Relaxation Time Constant: $\tau_{\text{relax}} = \mathbf{9.50\,\text{s}}$
- Asymptotic Bias Shift: $\omega_{z,\infty} = \mathbf{+0.442\,\text{rad/s}}$
- Coefficient of Determination: $R^2 = \mathbf{0.9825}$

---

## 9. Embedded Hardware, Energy Modeling & IoT Pareto Optimization

### 9.1 Multi-Component Embedded Energy Dissipation Model
Edge telemetry power consumption is governed by sensor analog front-end (AFE) currents, microcontroller active/sleep duty cycles, and wireless transmission overhead:

$$P_{\text{total}}(f_s) = V_{\text{dd}} \cdot I_{\text{avg}}(f_s)$$
$$I_{\text{avg}}(f_s) = I_{\text{sensor}}(f_s) + I_{\text{MCU\_active}} \left( f_s \cdot T_{\text{proc}} \right) + I_{\text{MCU\_sleep}} \left( 1 - f_s \cdot T_{\text{proc}} \right) + I_{\text{BLE\_burst}}(f_s)$$

Using calibrated hardware benchmarks for an ultra-low-power industrial IMU (e.g., Bosch BMI270 / Sensirion) paired with an ARM Cortex-M4 microcontroller:
- Supply Voltage: $V_{\text{dd}} = 3.3\,\text{V}$
- Nominal LiPo Battery Capacity: $C_{\text{batt}} = 250\,\text{mAh}$ ($E_{\text{batt}} = 925\,\text{mWh}$)
- Microcontroller Active Current: $I_{\text{MCU\_active}} = 4.5\,\text{mA}$, Processing time $T_{\text{proc}} = 120\,\mu\text{s}$ per sample.
- Microcontroller Sleep Current: $I_{\text{MCU\_sleep}} = 3.5\,\mu\text{A}$.
- IMU Operating Modes:
  - 6-Axis Continuous (Gyroscope Drive Active): $I_{\text{sensor}} \approx 3.80\,\text{mA}$ (high electrostatic drive dissipation).
  - Accelerometer-Only Mode: $I_{\text{sensor}}(f_s) \approx 0.015\,\text{mA} + 0.0006 \cdot f_s\,\text{mA}$.

Battery longevity in operational hours and days:
$$T_{\text{hours}}(f_s) = \frac{C_{\text{batt}}}{I_{\text{avg}}(f_s)}, \quad T_{\text{days}}(f_s) = \frac{T_{\text{hours}}(f_s)}{24}$$

### 9.2 Battery Longevity Across the Sampling Spectrum
| Sampling Frequency ($f_s$) | 6-Axis Current ($I_{\text{avg}}$) | 6-Axis Battery Life | Accel-Only Current ($I_{\text{avg}}$) | Accel-Only Battery Life | Power Reduction Factor |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **$500.0\,\text{Hz}$** | $4.07\,\text{mA}$ | **$61.4\,\text{hours}$ ($2.6\,\text{days}$)** | $0.584\,\text{mA}$ | **$428.1\,\text{hours}$ ($17.8\,\text{days}$)** | **$7.0\times$** |
| **$250.0\,\text{Hz}$** | $3.94\,\text{mA}$ | **$63.5\,\text{hours}$ ($2.6\,\text{days}$)** | $0.300\,\text{mA}$ | **$833.3\,\text{hours}$ ($34.7\,\text{days}$)** | **$13.1\times$** |
| **$100.0\,\text{Hz}$** | $3.85\,\text{mA}$ | **$64.9\,\text{hours}$ ($2.7\,\text{days}$)** | $0.129\,\text{mA}$ | **$1{,}938\,\text{hours}$ ($80.7\,\text{days}$)** | **$29.8\times$** |
| **$50.0\,\text{Hz}$** | $3.83\,\text{mA}$ | **$65.3\,\text{hours}$ ($2.7\,\text{days}$)** | $0.072\,\text{mA}$ | **$3{,}472\,\text{hours}$ ($144.7\,\text{days}$)** | **$53.2\times$** |
| **$25.0\,\text{Hz}$** | $3.81\,\text{mA}$ | **$65.6\,\text{hours}$ ($2.7\,\text{days}$)** | $0.044\,\text{mA}$ | **$5{,}681\,\text{hours}$ ($236.7\,\text{days}$)** | **$86.6\times$** |
| **$20.0\,\text{Hz}$** | $3.81\,\text{mA}$ | **$65.6\,\text{hours}$ ($2.7\,\text{days}$)** | $0.038\,\text{mA}$ | **$6{,}578\,\text{hours}$ ($274.1\,\text{days}$)** | **$100.3\times$** |
| **$10.0\,\text{Hz}$** | $3.81\,\text{mA}$ | **$65.6\,\text{hours}$ ($2.7\,\text{days}$)** | $0.026\,\text{mA}$ | **$9{,}615\,\text{hours}$ ($400.6\,\text{days}$)** | **$146.5\times$** |
| **$1.0\,\text{Hz}$** | $3.80\,\text{mA}$ | **$65.8\,\text{hours}$ ($2.7\,\text{days}$)** | $0.016\,\text{mA}$ | **$15{,}625\,\text{hours}$ ($651.0\,\text{days}$)** | **$237.5\times$** |

### 9.3 IoT Pareto Optimization Frontier
In embedded edge architecture, optimizing a device requires balancing **Detection Latency / Accuracy** against **Battery Longevity**:

```
Battery Life (Days)
     ^
 650 |                                        * 1.0 Hz (Max Life, Severe Timing Error)
     |
 274 |                         * 20.0 Hz (PARETO SWEET SPOT)
 236 |                   * 25.0 Hz       (100% Detection, <15 ms Jitter, 83% Shock)
     |
  80 |             * 100.0 Hz
     |
  18 |       * 500 Hz Accel-Only
 2.7 | * 500 Hz 6-Axis (Baseline)
     +------------------------------------------------------------> Detection Fidelity
       Low                                                    High
```

- **The $1.0\,\text{Hz}$ Corner**: Maximum battery life ($651\,\text{days}$), but introduces $\pm 500\,\text{ms}$ timestamp jitter and misses landing shock entirely (smearing ratio $52.6\%$).
- **The $500.0\,\text{Hz}$ 6-Axis Baseline**: Highest fidelity, but drains the battery in $2.6\,\text{days}$ due to continuous gyroscope drive oscillation.
- **The $20.0 - 25.0\,\text{Hz}$ Pareto Optimum**:
  - Employs **Accelerometer-Only Mode** (gyroscope powered down).
  - Preserves **$100.0\%$ jump detection accuracy ($F_1 = 1.0$)**.
  - Maintains timing uncertainty below $\pm 14.4\,\text{ms}$ ($\sigma_t$).
  - Captures $>83.0\%$ of landing impact shock amplitude.
  - Extends battery operational life from **$2.6\,\text{days}$ to $274.1\,\text{days}$ (a $>100\times$ longevity gain)** on a compact $250\,\text{mAh}$ cell.

---

## 10. Summary Matrix of Governing Equations

| Domain / Concept | Governing Mathematical Formula | Core Engineering Significance |
| :--- | :--- | :--- |
| **Kinematic Specific Force** | $\mathbf{a}(t) = \ddot{\mathbf{r}}(t) - \mathbf{g}$ | Proves that free-fall ballistic flight yields zero proper acceleration ($\Vert\mathbf{a}\Vert \to 0$). |
| **Spatial Invariance** | $\Vert\mathbf{a}(t)\Vert = \sqrt{a_x^2 + a_y^2 + a_z^2}$ | Rotation-invariant jump trigger immune to vehicle pitch/roll angles. |
| **Slider-Crank Force** | $F_{\text{recip}}(t) = m_{\text{recip}} r \omega^2 \left(\cos(\omega t) + \frac{r}{l}\cos(2\omega t)\right)$ | Generates primary unbalance at $58.11\,\text{Hz}$ and secondary harmonic at $116.21\,\text{Hz}$. |
| **Vibration Polarization** | $\theta_{\text{incline}} = \arctan\left(\frac{\text{RMS}(a_{z,\text{engine}})}{\text{RMS}(a_{x,\text{engine}})}\right) \approx 56.5^\circ$ | Measures physical engine cylinder bore inclination relative to chassis frame. |
| **Coriolis Acceleration** | $\mathbf{a}_c = 2\,(\mathbf{\Omega} \times \mathbf{v}_{\text{drive}})$ | Underpins gyroscope sensing; shows zero coupling to pure linear frame vibration. |
| **Welch Spectral Density** | $\hat{P}_{xx}(f) = \frac{2}{K f_s S_2}\sum_{m=1}^K \vert\mathcal{F}\{(x_m - \bar{x}_m)w\}\vert^2$ | Unbiased, low-variance spectral density estimation across ground riding segments. |
| **Butterworth Order ($N$)** | $N \ge \frac{\log_{10}((10^{A_s/10}-1)/(10^{A_p/10}-1))}{2\log_{10}(\Omega_s/\Omega_p)}$ | Analytically establishes $N = 4$ as the exact minimum order for $30\,\text{dB}$ engine rejection. |
| **Bilinear Pre-Warping** | $\Omega_c = 2 f_s \tan\left(\frac{\pi f_c}{f_s}\right)$ | Compensates for tangent frequency warping between continuous and discrete domains. |
| **Dual-Notch Transfer** | $H_{\text{notch}}(z) = \prod_{i=1}^2 g_i \frac{1 - 2\cos(\omega_i)z^{-1} + z^{-2}}{1 - 2\rho_i\cos(\omega_i)z^{-1} + \rho_i^2 z^{-2}}$ | Provides $-56.4\,\text{dB}$ engine harmonic cancellation with $<1\,\text{ms}$ passband group delay. |
| **Harmonic Disturbance KF** | $\mathbf{x}_{k+1} = \mathbf{F}\mathbf{x}_k + \mathbf{w}_k, \; \mathbf{K}_\infty = \mathbf{P}\mathbf{H}^T(\mathbf{H}\mathbf{P}\mathbf{H}^T + R)^{-1}$ | Embeds I/Q oscillator states to reject $58.11\,\text{Hz}$ & $116.21\,\text{Hz}$ with $2.5\,\text{ms}$ delay and $100\%$ shock fidelity. |
| **Adaptive Harmonic EKF** | $\mathbf{x}_{k+1} = \mathbf{f}(\mathbf{x}_k) + \mathbf{w}_k, \; \mathbf{F}_k = \left. \frac{\partial \mathbf{f}}{\partial \mathbf{x}} \right\vert_{\hat{\mathbf{x}}}$ | Augments state with $\omega_e$ to blindly track engine RPM ($3,487\,\text{RPM}$) and carve self-tuning notches without tachometer. |
| **Nyquist Aliasing Fold** | $f_{\text{alias}} = \vert f_{\text{source}} - k \cdot f_s \vert \le \frac{f_s}{2}$ | Predicts exact harmonic foldback locations across downsampled telemetry rates. |
| **Discrete Event Limit** | $f_{s,\text{min}} > \frac{1}{\tau_{\text{jump,min}}} = \frac{1}{0.934\,\text{s}} = 1.07\,\text{Hz}$ | Proves why $1.0\,\text{Hz}$ detects all jumps while $0.5\,\text{Hz}$ experiences total failure. |
| **Timestamp Quantization** | $\sigma_t = \frac{\Delta t}{\sqrt{12}} = \frac{1}{f_s \sqrt{12}}$ | Quantifies theoretical event boundary jitter due to discrete sampling intervals. |
| **Impulse Momentum** | $J_z = \int F_z \, dt = m \Delta v_z$ | Models landing touchdown shock and explains high-frequency capture attenuation. |
| **DF2T Zero-Phase Filter** | $\vert H_{\text{zero}}(e^{j\omega}) \vert = \vert H(e^{j\omega}) \vert^2, \; \tau_g(\omega) \equiv 0$ | Non-causal forward-backward filtering eliminating group delay with Gustafsson state initialization. |
| **State-Switched Filter** | $\hat{\theta}_k = \alpha_k (\hat{\theta}_{k-1} + \omega_y \Delta t) + (1-\alpha_k)\theta_{\text{accel}, k}$ | Eliminates $\frac{0}{0}$ gravitational tilt singularity during free-fall flight. |
| **Attitude Kalman Filter** | $\mathbf{x} = [\theta, b_g]^T, \; \mathbf{F} = \begin{bmatrix} 1 & -\Delta t \\ 0 & 1 \end{bmatrix}$ | Blends accelerometer leveling with gyro integration while estimating dynamic gyro bias drift. |
| **GPS/INS Fusion (ESKF)** | $\delta\mathbf{x} \in \mathbb{R}^{15}, \; \tilde{\mathbf{y}}_{\text{GPS}} = \mathbf{z}_{\text{GPS}} - \mathbf{h}(\hat{\mathbf{x}})$ | Fuses $500\,\text{Hz}$ strapdown IMU with $10\,\text{Hz}$ GPS, bounding cubic drift and unlocking jump ballistics. |
| **Kalman Pole Placement** | $\omega_n = \sqrt[4]{\frac{q_b}{R}}, \; \zeta = \frac{1}{\sqrt{2}} \approx 0.707$ | Proves CARE automatically synthesizes optimal Butterworth damping in attitude estimation. |
| **Stress Relaxation** | $\omega_z(t) = \omega_{z,\infty} + (\omega_{z,0} - \omega_{z,\infty})e^{-(t-t_0)/\tau}$ | Models MEMS die attach viscoelastic post-shock strain relaxation ($\tau = 9.50\,\text{s}$). |
| **Battery Longevity** | $T_{\text{batt}} = \frac{C_{\text{batt}}}{I_{\text{avg}}(f_s)} = \frac{C_{\text{batt}}}{I_{\text{sensor}} + I_{\text{MCU}} + I_{\text{RF}}}$ | Formulates the edge energy budget, proving a $>100\times$ lifespan increase at $20\,\text{Hz}$. |

---

*Document compiled and verified by the Motion Algorithm Engineering (MAE) Team. Linked with primary telemetry dataset [`data/motocross_imu_telemetry.csv`](../data/motocross_imu_telemetry.csv), filter implementations in [`src/`](../src/), and extended research notebooks in [`notebooks/`](../notebooks/).*

