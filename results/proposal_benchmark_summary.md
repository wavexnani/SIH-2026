# SIH 2026 PS26037 — Proposal A/B/C Benchmark Summary

**Protocol**: Multi-Scenario Evaluation Matrix comparing:
- **Config A (Baseline)**: Frozen Stage 4/5 QP-MPC & CA-CRC Planner
- **Config B (+ Velocity)**: Curvature-Aware Velocity Profiler Enabled
- **Config C (+ Vel + Steer)**: Curvature Velocity + Delay-Aware Steering Filter Enabled

> [!NOTE]
> All metrics reflect actual numerical measurements. Per the scientific reporting constraint, engineering targets (>=60% chatter reduction, <=3 cm RMSE degradation) are used as design targets rather than pass/fail filters.

> [!IMPORTANT]
> **Scientific Metric Definitions**:
> - **Reference Peak $a_{\text{lat}}$**: $\max(v_{\text{ref}}^2 \cdot |\kappa_{\text{ref}}|)$ of planned speed profile.
> - **Actual Peak $a_{\text{lat}}$**: $\max(|v_{\text{ego}} \cdot \omega_{\text{yaw}}|)$ measured on the simulated vehicle plant.
> - **Matched-Distance RMSE**: Evaluated over identical common longitudinal distance $x \le \min(x_{\text{end},A}, x_{\text{end},B}, x_{\text{end},C})$ for fair side-by-side comparison.
> - **Full-Run RMSE**: Evaluated over full individual trajectory up to completion or safety stop.
> - **Latency Disaggregation**: Separates pure algorithmic computation (Planner QP, Controller logic, Safety Filter) from benchmark simulation/logging overhead. Real-time embedded performance is NOT claimed from total wall-clock benchmark time.

## Scenario: `bottleneck`

| Metric | Config A (Baseline) | Config B (+ Velocity) | Config C (+ Vel + Steer) | Delta (C vs A) |
| :--- | :---: | :---: | :---: | :---: |
| **Completion Status** | YES | YES | YES | - |
| **Max Progress ($x_{\max}$)** | 75.21 m | 75.21 m | 75.20 m | -0.00 m |
| **Matched Comparison Distance** | $x \le 75.20$ m | $x \le 75.20$ m | $x \le 75.20$ m | - |
| **Matched-Distance Tracking RMSE** | **5.61 cm** | **5.61 cm** | **5.66 cm** | **+0.05 cm** |
| **Full-Run Tracking RMSE** | 5.59 cm | 5.59 cm | 5.66 cm | +0.07 cm |
| **Reference Peak $a_{\text{lat}}$ ($v_{\text{ref}}^2 \kappa$)** | 0.000 m/s² | **0.000 m/s²** | **0.000 m/s²** | +0.000 m/s² |
| **Actual Peak $a_{\text{lat}}$ ($|v \omega_{\text{yaw}}|$)** | 0.526 m/s² | 0.526 m/s² | 0.544 m/s² | +0.018 m/s² |
| **Steering Reversals (Chatter)** | 2 | 2 | 11 | **+450.0%** (+9) |
| **Steering Effort ($\sum |\Delta \delta|^2$)** | 0.00382 | 0.00382 | 0.00875 | +0.00493 |
| **Min Clearance ($d_{\min}$)** | Inf m | Inf m | Inf m | NaN m |
| **Total Step Latency** | 7.14 ms | 8.10 ms | 6.86 ms | -0.28 ms |
| └─ *Planner QP Solve* | 4.77 ms | 5.24 ms | 4.24 ms | -0.54 ms |
| └─ *Controller Logic / Filter* | 1.49 ms | 2.00 ms | 1.92 ms | +0.43 ms |
| └─ *Safety Filter* | 0.69 ms | 0.70 ms | 0.56 ms | -0.12 ms |
| └─ *Simulation Overhead* | 0.43 ms | 0.33 ms | 0.28 ms | -0.15 ms |
| **Wall-Clock Duration** | 1.27 s | 1.27 s | 1.07 s | -0.20 s |
| **Safety Filter Overrides** | 0 | 0 | 0 | +0 |

## Scenario: `cattle`

| Metric | Config A (Baseline) | Config B (+ Velocity) | Config C (+ Vel + Steer) | Delta (C vs A) |
| :--- | :---: | :---: | :---: | :---: |
| **Completion Status** | NO | NO | NO | - |
| **Max Progress ($x_{\max}$)** | 30.66 m | 30.66 m | 30.68 m | +0.02 m |
| **Matched Comparison Distance** | $x \le 30.66$ m | $x \le 30.66$ m | $x \le 30.66$ m | - |
| **Matched-Distance Tracking RMSE** | **49.37 cm** | **49.37 cm** | **43.96 cm** | **-5.41 cm** |
| **Full-Run Tracking RMSE** | 49.37 cm | 49.37 cm | 49.06 cm | -0.31 cm |
| **Reference Peak $a_{\text{lat}}$ ($v_{\text{ref}}^2 \kappa$)** | 0.000 m/s² | **0.000 m/s²** | **0.000 m/s²** | +0.000 m/s² |
| **Actual Peak $a_{\text{lat}}$ ($|v \omega_{\text{yaw}}|$)** | 0.761 m/s² | 0.761 m/s² | 0.815 m/s² | +0.054 m/s² |
| **Steering Reversals (Chatter)** | 6 | 6 | 3 | **-50.0%** (-3) |
| **Steering Effort ($\sum |\Delta \delta|^2$)** | 0.05223 | 0.05223 | 0.02998 | -0.02225 |
| **Min Clearance ($d_{\min}$)** | 3.7296 m | 3.7296 m | 3.7039 m | -0.0257 m |
| **Total Step Latency** | 13.93 ms | 8.60 ms | 8.96 ms | -4.96 ms |
| └─ *Planner QP Solve* | 9.96 ms | 6.01 ms | 5.80 ms | -4.16 ms |
| └─ *Controller Logic / Filter* | 2.75 ms | 1.86 ms | 2.36 ms | -0.39 ms |
| └─ *Safety Filter* | 1.06 ms | 0.60 ms | 0.64 ms | -0.41 ms |
| └─ *Simulation Overhead* | 0.64 ms | 0.31 ms | 0.40 ms | -0.24 ms |
| **Wall-Clock Duration** | 0.81 s | 0.46 s | 0.49 s | -0.32 s |
| **Safety Filter Overrides** | 2 | 2 | 2 | +0 |

## Scenario: `chicane`

| Metric | Config A (Baseline) | Config B (+ Velocity) | Config C (+ Vel + Steer) | Delta (C vs A) |
| :--- | :---: | :---: | :---: | :---: |
| **Completion Status** | NO | YES | NO | - |
| **Max Progress ($x_{\max}$)** | 65.27 m | 100.19 m | 65.51 m | +0.24 m |
| **Matched Comparison Distance** | $x \le 65.27$ m | $x \le 65.27$ m | $x \le 65.27$ m | - |
| **Matched-Distance Tracking RMSE** | **289.32 cm** | **53.70 cm** | **280.17 cm** | **-9.15 cm** |
| **Full-Run Tracking RMSE** | 289.32 cm | 147.86 cm | 331.75 cm | +42.43 cm |
| **Reference Peak $a_{\text{lat}}$ ($v_{\text{ref}}^2 \kappa$)** | 5.287 m/s² | **2.500 m/s²** | **2.500 m/s²** | -2.787 m/s² |
| **Actual Peak $a_{\text{lat}}$ ($|v \omega_{\text{yaw}}|$)** | 5.186 m/s² | 4.629 m/s² | 7.030 m/s² | +1.844 m/s² |
| **Steering Reversals (Chatter)** | 6 | 10 | 3 | **-50.0%** (-3) |
| **Steering Effort ($\sum |\Delta \delta|^2$)** | 0.79118 | 0.72861 | 0.10607 | -0.68512 |
| **Min Clearance ($d_{\min}$)** | 1.2000 m | 1.2000 m | 1.2000 m | +0.0000 m |
| **Total Step Latency** | 24.91 ms | 10.69 ms | 21.57 ms | -3.34 ms |
| └─ *Planner QP Solve* | 23.08 ms | 8.63 ms | 19.85 ms | -3.23 ms |
| └─ *Controller Logic / Filter* | 0.45 ms | 0.67 ms | 0.62 ms | +0.17 ms |
| └─ *Safety Filter* | 1.24 ms | 1.26 ms | 0.98 ms | -0.26 ms |
| └─ *Simulation Overhead* | 0.37 ms | 0.36 ms | 0.30 ms | -0.07 ms |
| **Wall-Clock Duration** | 2.59 s | 1.52 s | 2.29 s | -0.30 s |
| **Safety Filter Overrides** | 41 | 27 | 27 | -14 |

