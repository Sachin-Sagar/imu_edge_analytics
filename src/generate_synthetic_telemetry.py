#!/usr/bin/env python3
"""
High-Fidelity Synthetic Motocross IMU Telemetry Generator
==========================================================
Module: src.generate_synthetic_telemetry
Author: Sachin Sagar (Edge IMU Motion Analytics & Vibration DSP Engine)

Description:
Generates a realistic 6-axis IMU dataset (500 Hz ODR, 90.0 seconds, 45,000 samples)
simulating a motorcycle telemetry system mounted rigidly near the steering headstock
on a single-cylinder 2-stroke motocross motorcycle.

Physical & Transducer Simulation Models:
1. 2-Stroke Powertrain Kinematics:
   - Primary reciprocating piston unbalance: f1 = 58.11 Hz (3,487 RPM)
   - Secondary connecting-rod harmonic: f2 = 116.21 Hz (2 * f1)
   - 3D Sagittal plane polarization: cylinder stroke tilted at 56.7° forward lean.
   - Gyroscope immunity: MEMS tuning-fork differential Coriolis proof-mass rejection.
2. Track & Chassis Dynamics:
   - Colored terrain roughness: filtered Brownian dirt undulation (fc < 15 Hz).
   - Nominal gravitational reference frame: az = 9.81 m/s² baseline.
3. Ballistic Jump Dynamics:
   - 5 confirmed jump events matching real motocross track flight trajectories:
     * Jump 1: t in [7.000, 9.750] s (Air Time: 2.750 s)
     * Jump 2: t in [18.400, 21.900] s (Air Time: 3.500 s)
     * Jump 3: t in [35.600, 39.300] s (Air Time: 3.700 s)
     * Jump 4: t in [54.600, 60.200] s (Air Time: 5.600 s, large pitch whip)
     * Jump 5: t in [77.000, 79.800] s (Air Time: 2.800 s)
   - Ballistic zero-G free-fall collapse (||a|| < 0.5 - 2.0 m/s²).
   - Landing impact shocks: half-sine compression impulse peaking at 18.0 - 18.7 m/s²
     at 16 ms post-landing, followed by exponential suspension damping (tau ~ 15 ms).
4. Sensor Artifacts & MEMS Packaging:
   - Post-Jump 4 landing impact yaw rate stress relaxation:
     omega_z(t) = 0.445 * (1 - exp(-(t - 60.2) / 9.5)) rad/s for t >= 60.2 s.
   - Serial bus transmission dropout: 200 samples (0.400 s) of NaNs at t in [23.000, 23.398] s.
"""

import os
import argparse
import numpy as np
import pandas as pd


def generate_motocross_telemetry(
    duration: float = 90.0,
    fs: float = 500.0,
    seed: int = 42,
    output_path: str = "data/motocross_imu_telemetry.csv"
) -> pd.DataFrame:
    """
    Synthesize high-fidelity 6-axis IMU telemetry data from first principles.
    """
    np.random.seed(seed)
    
    # 1. Temporal Grid
    dt = 1.0 / fs
    n_samples = int(duration * fs)
    t = np.arange(n_samples) * dt
    
    # Pre-allocate 6-axis arrays
    accel_x = np.zeros(n_samples, dtype=np.float64)
    accel_y = np.zeros(n_samples, dtype=np.float64)
    accel_z = np.zeros(n_samples, dtype=np.float64)
    gyro_x = np.zeros(n_samples, dtype=np.float64)
    gyro_y = np.zeros(n_samples, dtype=np.float64)
    gyro_z = np.zeros(n_samples, dtype=np.float64)
    
    # 2. Jump Intervals Definition (Start, End in seconds)
    jump_intervals = [
        (7.000, 9.748, 17.98),    # Jump 1: ~2.75s, shock 17.98 m/s²
        (18.400, 21.898, 18.35),  # Jump 2: ~3.50s, shock 18.35 m/s²
        (35.600, 39.298, 18.66),  # Jump 3: ~3.70s, shock 18.66 m/s²
        (54.600, 60.198, 18.04),  # Jump 4: ~5.60s, shock 18.04 m/s²
        (77.000, 79.798, 18.19),  # Jump 5: ~2.80s, shock 18.19 m/s²
    ]
    
    # Flight mask
    is_in_flight = np.zeros(n_samples, dtype=bool)
    for t_start, t_end, _ in jump_intervals:
        idx_s = int(round(t_start * fs))
        idx_e = int(round(t_end * fs))
        is_in_flight[idx_s:idx_e] = True
        
    is_on_ground = ~is_in_flight
    
    # 3. Ground Terrain Colored Noise (Low-pass filtered Brownian motion)
    def generate_colored_noise(size, scale, decay=0.985):
        noise = np.random.randn(size) * scale
        colored = np.zeros(size)
        c = 0.0
        for i in range(size):
            c = decay * c + (1.0 - decay) * noise[i]
            colored[i] = c
        return colored
        
    terrain_z = generate_colored_noise(n_samples, scale=0.35, decay=0.980)
    terrain_x = generate_colored_noise(n_samples, scale=0.18, decay=0.980)
    terrain_y = generate_colored_noise(n_samples, scale=0.10, decay=0.980)
    
    # 4. Engine Vibration Synthesis (Ground Segments)
    # Fundamental: 58.11 Hz, 2nd Harmonic: 116.21 Hz
    f1 = 58.11
    f2 = 116.21
    
    # Sagittal polarization along 56.68° cylinder inclination
    # az / ax = tan(56.68°) -> ax = az * cot(56.68°)
    cyl_angle_rad = np.radians(56.68)
    cot_cyl = 1.0 / np.tan(cyl_angle_rad) # ~0.657
    
    amp_z_f1 = 0.110
    amp_z_f2 = 0.058
    amp_x_f1 = amp_z_f1 * cot_cyl
    amp_x_f2 = amp_z_f2 * cot_cyl
    
    # Continuous phase to avoid jumps
    phase_f1 = 2 * np.pi * f1 * t
    phase_f2 = 2 * np.pi * f2 * t
    
    engine_vib_z = amp_z_f1 * np.sin(phase_f1) + amp_z_f2 * np.sin(phase_f2 + 0.3)
    engine_vib_x = amp_x_f1 * np.sin(phase_f1 + 0.05) + amp_x_f2 * np.sin(phase_f2 + 0.35)
    engine_vib_y = 0.008 * np.sin(phase_f1 + 1.2) # Minimal lateral coupling
    
    # 5. Populate Ground Acceleration
    accel_z[is_on_ground] = 9.810 + terrain_z[is_on_ground] + engine_vib_z[is_on_ground]
    accel_x[is_on_ground] = 0.000 + terrain_x[is_on_ground] + engine_vib_x[is_on_ground]
    accel_y[is_on_ground] = 0.000 + terrain_y[is_on_ground] + engine_vib_y[is_on_ground]
    
    # 6. Populate In-Flight Free-Fall Acceleration (Zero-G Plateau)
    # Drivetrain is unloaded during flight -> engine vibration drops by >18 dB
    in_flight_vib_scale = 0.12
    accel_z[is_in_flight] = 0.003 + 0.06 * np.random.randn(np.sum(is_in_flight)) + (engine_vib_z[is_in_flight] * in_flight_vib_scale)
    accel_x[is_in_flight] = 0.001 + 0.05 * np.random.randn(np.sum(is_in_flight)) + (engine_vib_x[is_in_flight] * in_flight_vib_scale)
    accel_y[is_in_flight] = 0.000 + 0.04 * np.random.randn(np.sum(is_in_flight)) + (engine_vib_y[is_in_flight] * in_flight_vib_scale)
    
    # 7. Landing Shock Impulses
    # Peak deceleration occurs exactly 8 samples (16 ms) after landing contact
    for _, t_end, peak_shock in jump_intervals:
        idx_land = int(round(t_end * fs))
        shock_len = 35 # 70 ms
        shock_t = np.arange(shock_len) * dt
        
        # Asymmetric shock pulse: half-sine rise (16 ms) + exponential decay (tau ~ 18 ms)
        t_peak = 0.016
        shock_profile = np.zeros(shock_len)
        rise_mask = shock_t <= t_peak
        shock_profile[rise_mask] = np.sin((np.pi / 2) * (shock_t[rise_mask] / t_peak))
        decay_mask = shock_t > t_peak
        shock_profile[decay_mask] = np.exp(-(shock_t[decay_mask] - t_peak) / 0.018)
        
        # Scale to peak landing deceleration (delta above 9.81)
        shock_mag = peak_shock - 9.81
        end_idx = min(n_samples, idx_land + shock_len)
        actual_len = end_idx - idx_land
        accel_z[idx_land:end_idx] = 9.81 + (shock_mag * shock_profile[:actual_len]) + (np.random.randn(actual_len) * 0.08)
        # Minor longitudinal deceleration
        accel_x[idx_land:end_idx] += -0.15 * shock_profile[:actual_len]
        accel_y[idx_land:end_idx] += -0.05 * shock_profile[:actual_len]

    # 8. Sensor Thermal Noise Floor
    accel_x += np.random.randn(n_samples) * 0.025
    accel_y += np.random.randn(n_samples) * 0.025
    accel_z += np.random.randn(n_samples) * 0.030

    # 9. Gyroscope Telemetry Synthesis
    # Ground: Low angular velocities + white noise floor (zero engine vibration ingress)
    gyro_x = np.random.randn(n_samples) * 0.035
    gyro_y = np.random.randn(n_samples) * 0.022
    gyro_z = np.random.randn(n_samples) * 0.015
    
    # In-Flight Pitch Rotation (omega_y)
    for j_idx, (t_start, t_end, _) in enumerate(jump_intervals):
        idx_s = int(round(t_start * fs))
        idx_e = int(round(t_end * fs))
        dur_samples = idx_e - idx_s
        tau = np.linspace(0, 1, dur_samples)
        
        if j_idx == 3:
            # Jump 4: Aggressive obstacle whip / large pitch excursion (up to 8 rad/s peak)
            whip_profile = 7.8 * np.sin(np.pi * tau)**2 + 0.35
            gyro_y[idx_s:idx_e] += whip_profile
        else:
            # Typical jump flight pitch profile
            pitch_amp = [0.35, 0.45, 0.50, 0.0, 0.28][j_idx]
            gyro_y[idx_s:idx_e] += pitch_amp * np.sin(np.pi * tau)
            
    # Post-Jump 4 MEMS Yaw Rate Stress Relaxation Drift:
    # omega_z(t) = 0.445 * (1 - exp(-(t - 60.2) / 9.5)) for t >= 60.2s
    t_shock4 = 60.200
    idx_shock4 = int(round(t_shock4 * fs))
    t_rel = t[idx_shock4:] - t_shock4
    gyro_z[idx_shock4:] += 0.445 * (1.0 - np.exp(-t_rel / 9.50))
    
    # 10. Serial Bus Communication Dropout
    # 200 samples (0.400s) of NaNs at t in [23.000, 23.398] s (indices 11500 to 11699)
    drop_s = int(round(23.000 * fs))
    drop_e = int(round(23.400 * fs))
    
    accel_x[drop_s:drop_e] = np.nan
    accel_y[drop_s:drop_e] = np.nan
    accel_z[drop_s:drop_e] = np.nan
    gyro_x[drop_s:drop_e] = np.nan
    gyro_y[drop_s:drop_e] = np.nan
    gyro_z[drop_s:drop_e] = np.nan

    # 11. Assemble DataFrame
    df = pd.DataFrame({
        't': np.round(t, 3),
        'accel_x': accel_x,
        'accel_y': accel_y,
        'accel_z': accel_z,
        'gyro_x': gyro_x,
        'gyro_y': gyro_y,
        'gyro_z': gyro_z
    })
    
    # Save output
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    df.to_csv(output_path, index=False)
    print(f"Synthesized {n_samples} telemetry samples ({duration} s @ {fs} Hz) -> {output_path}")
    return df


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Generate synthetic 6-axis IMU motocross telemetry.")
    parser.add_argument('--output', type=str, default="data/motocross_imu_telemetry.csv", help="Output CSV path")
    parser.add_argument('--duration', type=float, default=90.0, help="Duration in seconds")
    parser.add_argument('--fs', type=float, default=500.0, help="Sampling frequency in Hz")
    parser.add_argument('--seed', type=int, default=42, help="Random seed")
    args = parser.parse_args()
    
    generate_motocross_telemetry(
        duration=args.duration,
        fs=args.fs,
        seed=args.seed,
        output_path=args.output
    )

