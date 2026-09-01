# CA-CRC Tier 3B Realism Audit: Velocity Uncertainty Sweep

## Executive Summary
This experiment evaluates the sensitivity of the CA-CRC autonomous navigation stack to **velocity perception noise** (\sigma_v \in [0.00, 0.50]\text{ m/s}) across four longitudinal encounter headways ($X_{\text{center}} \in [20, 25, 30, 40]\text{ m}$) in Scenario 01 (60 dynamic goats). Statistical confidence is established via $N = 20$ Monte Carlo trials per cell using 95% Wilson Score Confidence Intervals.

## 2D Sensitivity Matrix: P(Collision) [%] (95% Wilson Score CI)

| Headway X_center | \sigma_v = 0.00m/s | \sigma_v = 0.05m/s | \sigma_v = 0.10m/s | \sigma_v = 0.15m/s | \sigma_v = 0.20m/s | \sigma_v = 0.30m/s | \sigma_v = 0.50m/s |
|---:|---:|---:|---:|---:|---:|---:|---:|
| **20 m** | **0%** <br><sub>[0.0, 16.1]</sub> | **0%** <br><sub>[0.0, 16.1]</sub> | **0%** <br><sub>[0.0, 16.1]</sub> | **0%** <br><sub>[0.0, 16.1]</sub> | **0%** <br><sub>[0.0, 16.1]</sub> | **85%** <br><sub>[64.0, 94.8]</sub> | **100%** <br><sub>[83.9, 100.0]</sub> |
| **25 m** | **0%** <br><sub>[0.0, 16.1]</sub> | **15%** <br><sub>[5.2, 36.0]</sub> | **75%** <br><sub>[53.1, 88.8]</sub> | **90%** <br><sub>[69.9, 97.2]</sub> | **100%** <br><sub>[83.9, 100.0]</sub> | **100%** <br><sub>[83.9, 100.0]</sub> | **100%** <br><sub>[83.9, 100.0]</sub> |
| **30 m** | **100%** <br><sub>[83.9, 100.0]</sub> | **100%** <br><sub>[83.9, 100.0]</sub> | **100%** <br><sub>[83.9, 100.0]</sub> | **100%** <br><sub>[83.9, 100.0]</sub> | **100%** <br><sub>[83.9, 100.0]</sub> | **100%** <br><sub>[83.9, 100.0]</sub> | **100%** <br><sub>[83.9, 100.0]</sub> |
| **40 m** | **15%** <br><sub>[5.2, 36.0]</sub> | **10%** <br><sub>[2.8, 30.1]</sub> | **5%** <br><sub>[0.9, 23.6]</sub> | **5%** <br><sub>[0.9, 23.6]</sub> | **10%** <br><sub>[2.8, 30.1]</sub> | **5%** <br><sub>[0.9, 23.6]</sub> | **0%** <br><sub>[0.0, 16.1]</sub> |

## Telemetry & System Behavior Breakdown

### 1. SafetyFilter Intervention Duration (seconds)

| Headway X_center | \sigma_v = 0.00m/s | \sigma_v = 0.05m/s | \sigma_v = 0.10m/s | \sigma_v = 0.15m/s | \sigma_v = 0.20m/s | \sigma_v = 0.30m/s | \sigma_v = 0.50m/s |
|---:|---:|---:|---:|---:|---:|---:|---:|
| **20 m** | 0.00 s | 0.04 s | 0.57 s | 0.64 s | 5.03 s | 19.96 s | 23.13 s |
| **25 m** | 6.19 s | 9.87 s | 18.79 s | 20.68 s | 21.76 s | 22.00 s | 22.27 s |
| **30 m** | 20.20 s | 20.64 s | 20.81 s | 21.00 s | 21.13 s | 21.20 s | 21.30 s |
| **40 m** | 19.54 s | 19.60 s | 19.65 s | 19.58 s | 19.41 s | 19.47 s | 19.50 s |

### 2. MPC Feasibility Rate (%)

| Headway X_center | \sigma_v = 0.00m/s | \sigma_v = 0.05m/s | \sigma_v = 0.10m/s | \sigma_v = 0.15m/s | \sigma_v = 0.20m/s | \sigma_v = 0.30m/s | \sigma_v = 0.50m/s |
|---:|---:|---:|---:|---:|---:|---:|---:|
| **20 m** | 100.0% | 99.8% | 98.8% | 98.2% | 87.8% | 21.4% | 7.5% |
| **25 m** | 86.5% | 74.4% | 32.0% | 18.5% | 13.0% | 12.0% | 10.9% |
| **30 m** | 19.2% | 17.4% | 16.7% | 16.0% | 15.5% | 15.2% | 14.8% |
| **40 m** | 31.9% | 27.8% | 23.4% | 24.5% | 32.6% | 25.5% | 31.6% |

### 3. Empirical Velocity Measurement RMSE (m/s)

| Headway X_center | \sigma_v = 0.00m/s | \sigma_v = 0.05m/s | \sigma_v = 0.10m/s | \sigma_v = 0.15m/s | \sigma_v = 0.20m/s | \sigma_v = 0.30m/s | \sigma_v = 0.50m/s |
|---:|---:|---:|---:|---:|---:|---:|---:|
| **20 m** | 0.000 m/s | 0.050 m/s | 0.099 m/s | 0.144 m/s | 0.182 m/s | 0.251 m/s | 0.382 m/s |
| **25 m** | 0.000 m/s | 0.050 m/s | 0.099 m/s | 0.143 m/s | 0.182 m/s | 0.251 m/s | 0.384 m/s |
| **30 m** | 0.000 m/s | 0.050 m/s | 0.099 m/s | 0.144 m/s | 0.182 m/s | 0.250 m/s | 0.383 m/s |
| **40 m** | 0.000 m/s | 0.050 m/s | 0.099 m/s | 0.144 m/s | 0.182 m/s | 0.250 m/s | 0.382 m/s |

## Scientific Takeaway & Key Observations
1. **Comparison with Position Uncertainty (Tier 3A)**: Evaluates whether velocity estimation errors degrade safety faster or slower than position estimation noise.
2. **Impact on Trajectory Prediction**: Demonstrates how noisy velocity estimates affect the predictive bounds of the free-space map and MPC horizon.
3. **Deterministic Baseline Alignment**: The zero-noise baseline (\sigma_v = 0.00\text{ m/s}) aligns with the verified Tier 3A.1-F baseline.

