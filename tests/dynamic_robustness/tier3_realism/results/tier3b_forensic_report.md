# CA-CRC Tier 3B Forensic Mechanism Report: Velocity Uncertainty Audit

## Executive Summary
This report provides a **targeted forensic mechanism audit** of 7 representative cases from the Tier 3B Velocity Uncertainty Sweep. It isolates the physical and predictive causal chains responsible for safety degradation under velocity perception noise.

## Forensic Case Breakdown Table

| Case | $X_{\text{center}}$ | $\sigma_v$ (m/s) | Vel RMSE (m/s) | MPC Feasibility | SF Duration | $t_{\text{sf\_on}}$ | $t_{\text{stop}}$ | Standstill $(x, y)$ | $t_{\text{contact}}$ | Agent | First Contact $C$ | Peak Penetration $C_{\text{peak}}$ |
|:---:|---:|---:|---:|---:|---:|---:|---:|:---:|---:|:---:|---:|---:|
| **Case 1** | 20 m | 0.20 | 0.183 m/s | 99.2% | 0.20 s | 2.30 s | N/A | N/A | N/A | N/A | N/A | +0.97 m |
| **Case 2** | 20 m | 0.30 | 0.252 m/s | 14.4% | 21.60 s | 2.00 s | 5.00 s | (21.05m, 2.68m) | 8.20 s | Goat #10 | -0.02 m | -1.15 m (t=16.90s) |
| **Case 3** | 25 m | 0.05 | 0.050 m/s | 98.8% | 0.30 s | 3.80 s | N/A | N/A | N/A | N/A | N/A | +0.77 m |
| **Case 4** | 25 m | 0.10 | 0.100 m/s | 16.4% | 20.90 s | 3.50 s | 5.70 s | (26.02m, 2.66m) | 8.10 s | Goat #10 | -0.01 m | -1.15 m (t=17.60s) |
| **Case 5** | 25 m | 0.20 | 0.183 m/s | 12.4% | 21.90 s | 2.90 s | 5.10 s | (23.67m, 2.60m) | 7.80 s | Goat #10 | -0.00 m | -1.15 m (t=16.10s) |
| **Case 6** | 30 m | 0.00 | 0.000 m/s | 19.2% | 20.20 s | 4.80 s | 6.70 s | (31.32m, 2.78m) | 8.60 s | Goat #10 | -0.02 m | -1.15 m (t=23.10s) |
| **Case 7** | 40 m | 0.00 | 0.000 m/s | 22.4% | 19.80 s | 5.20 s | 7.00 s | (33.25m, 2.50m) | N/A | N/A | N/A | +1.52 m |

## Causal Mechanism & System Behavior Analysis

### 1. Mechanism 1: Predictive Planning Degradation (MPC Feasibility Collapse)
- Velocity noise (\sigma_v \ge 0.10\text{ m/s}) causes noisy velocity estimates (\hat{v}_x, \hat{v}_y), creating corrupted velocity projections in the predictive free-space map.
- At $X_c = 25\text{ m}$, MPC feasibility collapses from **86.5%** (\sigma_v = 0.00\text{ m/s}) to **32.0%** (\sigma_v = 0.10\text{ m/s}) and **13.0%** (\sigma_v = 0.20\text{ m/s}).
- When MPC feasibility drops, the SafetyFilter is forced to override the planner for extended durations, shifting control from proactive trajectory optimization to reactive emergency deceleration.

### 2. Mechanism 2: Stationary-Ego Vulnerability ($X_c = 30\text{ m}$ Intrinsic Failure Floor)
- At $X_c = 30\text{ m}$, collision probability is **100% across all velocity noise levels** (including \sigma_v = 0.00\text{ m/s}).
- Telemetry confirms that the vehicle brings itself to a complete standstill at $t = 6.70\text{ s}$ ($v = 0.00\text{ m/s}, x = 31.32\text{ m}$), but Goat #9 touches the lateral footprint at $t = 8.60\text{ s}$ ($1.90\text{ s}$ AFTER full stop).
- **Conclusion**: The $X_c = 30\text{ m}$ failure is an intrinsic geometric vulnerability to continued agent lateral motion, independent of perception noise.

### 3. SafetyFilter Duration Telemetry Clarification
- `sf_duration` measures the total simulation time ($N_{\text{active}} \times dt$) during which the SafetyFilter overrides MPC commands.
- For stopped vehicles in blocked corridors, the SafetyFilter remains active to maintain zero-velocity braking bounds while the herd obstructs the forward path.

