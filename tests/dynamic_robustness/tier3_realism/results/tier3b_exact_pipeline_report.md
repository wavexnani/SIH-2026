# CA-CRC Tier 3B Forensic Pipeline & Multi-Mechanism Audit Report ($X_c = 25\text{ m}$)

## Executive Summary
This report documents the exact **inside-planner telemetry** (`CACRCPlanner.m`) and a fine-grained velocity noise dose-response sweep across $\sigma_v \in [0.00, 0.20]\text{ m/s}$. We establish that planner rejection occurs via **Case A (Pre-QP Geometric Corridor Infeasibility)** when $y_{\text{min}}(k) > y_{\text{max}}(k)$, and demonstrate through an **oracle diagnostic counterfactual intervention** that removing velocity prediction error completely restores MPC feasibility ($16.4\% \to 100.0\%$), drops SafetyFilter takeover ($83.6\% \to 0.0\%$), and eliminates collision.

## 1. Dose-Response Velocity Noise Sweep ($X_c = 25\text{ m}$, 10 Seeds/Cell)

| Velocity Noise \sigma_v | Mean MPC Feasibility | Mean Crossovers (y_{max} < y_{min}) | Mean Min Bound Gap \Delta y_{\text{min}} | SF Override Ratio | Collision Rate |
|---:|---:|---:|---:|---:|---:|
| **0.00 m/s** | **90.1%** | **0.00 / 20 steps** | **1.47 m** | **21.9%** | **0.0%** |
| **0.02 m/s** | **87.6%** | **0.00 / 20 steps** | **1.28 m** | **27.8%** | **0.0%** |
| **0.04 m/s** | **71.2%** | **0.86 / 20 steps** | **0.17 m** | **40.6%** | **10.0%** |
| **0.06 m/s** | **58.9%** | **4.23 / 20 steps** | **-0.02 m** | **52.0%** | **30.0%** |
| **0.08 m/s** | **47.6%** | **7.84 / 20 steps** | **-0.02 m** | **58.9%** | **60.0%** |
| **0.10 m/s** | **30.3%** | **11.27 / 20 steps** | **-0.20 m** | **79.2%** | **80.0%** |
| **0.12 m/s** | **22.5%** | **12.76 / 20 steps** | **-0.20 m** | **81.9%** | **90.0%** |
| **0.15 m/s** | **14.5%** | **14.41 / 20 steps** | **-0.20 m** | **85.5%** | **100.0%** |
| **0.20 m/s** | **13.2%** | **15.31 / 20 steps** | **-0.20 m** | **86.8%** | **100.0%** |

> [!NOTE]
> **Baseline Nonzero Degradation**: At zero velocity noise ($\sigma_v = 0.00\text{ m/s}$), MPC feasibility is $90.1\%$ and SafetyFilter override ratio is $21.9\%$. This proves that velocity prediction error is NOT the sole source of degradation, but an additive failure mechanism that becomes dominant in dense obstacle corridors.
> 
> **Estimated Critical Transition Region**: In the 10-seed sweep, mean minimum corridor gap transitions from positive ($\Delta y_{\text{min}} = +0.17\text{ m}$) at $\sigma_v = 0.04\text{ m/s}$ to negative ($\Delta y_{\text{min}} = -0.02\text{ m}$) at $\sigma_v = 0.06\text{ m/s}$, motivating an estimated critical transition region $\sigma_v^{\text{crit}} \approx 0.06\text{ m/s}$. Initial system degradation begins as early as $\sigma_v = 0.04\text{ m/s}$ (10% collision rate).

## 2. Multi-Mechanism Temporal Breakdown (Condition A: $\sigma_v = 0.10\text{ m/s}$)

Telemetry isolates three distinct failure mechanisms operating in temporal sequence:

1. **Mechanism M0 — Early Safety Filter Degradation** ($t = 3.50\text{ s}$):
   - SafetyFilter activates (`predicted_obstacle_clearance_violation`) while primary MPC is STILL FEASIBLE (`status = 1`).
   - Cause: Perception noise causes MPC to plan trajectories passing within $<0.05\text{ m}$ of dynamic agent projected footprints.

2. **Mechanism M1 — Velocity-Uncertainty-Induced Predictive Corridor Infeasibility** ($t = 4.20\text{ s}$):
   - Dynamic agent velocity errors extrapolate into negative corridor gaps ($\Delta y_{\text{min}} = -0.20\text{ m}$, $y_{\text{min}} > y_{\text{max}}$).
   - `CACRCPlanner.m` (lines 243-246) detects $y_{\text{min}} > y_{\text{max}}$ and hard-gates both left/right topologies as geometrically invalid (`ok_geom = false`), executing **Case A Pre-QP Geometric Rejection** (`status = 0`).

3. **Mechanism M2 — Emergency-Stop-Induced Dynamic Intrusion** ($t = 5.70\text{ s} \to t = 8.10\text{ s}$):
   - SafetyFilter emergency deceleration brings ego to a full stop at $x = 26.02\text{ m}$ ($t_{\text{stop}} = 5.70\text{ s}$).
   - Dynamic Goat #10 continues lateral motion ($v_y = 0.25\text{ m/s}$) and intrudes into stationary ego footprint $2.40\text{ s}$ after stopping ($t_{\text{collision}} = 8.10\text{ s}$, clearance $C = -1.15\text{ m}$).

## 3. Oracle Diagnostic Counterfactual Intervention Results

> [!IMPORTANT]
> **Oracle Counterfactual Framing**: Condition B (Counterfactual Predictor Correction) is an **oracle diagnostic intervention** used solely for causal diagnosis; it is not proposed as an operational perception architecture.

| Intervention Condition | Horizon Pred Error e_y(2s) | Mean QP Crossovers | Min Bound Gap \Delta y_{\text{min}} | MPC Feasibility | SF Override Ratio R_{SF} | Emergency Stop t_{stop} | Physical Outcome |
|:---|---:|---:|---:|---:|---:|---:|:---:|
| **Condition A (Noisy Velocity)** | 0.159 m | **14.79 / 20 steps** | **-0.20 m** | **16.4%** | **83.6%** | 5.70 s | **COLLISION (C=-1.15m)** |
| **Condition B (Oracle Corrected)** | **0.000 m** | **0.00 / 20 steps** | **+1.70 m** | **100.0%** | **0.0%** | **N/A (No Stop)** | **SAFE (C=+0.76m)** |

## 4. Defensible Causal Conclusion
1. **Interventional Evidence**: The experiments provide strong interventional evidence that velocity prediction error is a primary causal contributor to the observed Tier 3B failure at $X_c = 25\text{ m}$. Inside-planner telemetry confirms that sufficiently large prediction errors produce negative corridor bound gaps, causing `CACRCPlanner` to reject both geometric topologies before QP invocation.
2. **Diagnostic Restoration**: A controlled oracle intervention that removes only the predictor velocity error eliminates negative corridor gaps, restores planner feasibility ($16.4\% \to 100.0\%$), eliminates SafetyFilter takeover ($83.6\% \to 0.0\%$), and converts the collision into a safe outcome ($C = -1.15\text{ m} \to +0.76\text{ m}$).
3. **Non-Exclusivity**: The results do not imply that velocity uncertainty is the sole source of Tier 3B degradation, since nonzero SafetyFilter activity ($21.9\%$) and reduced MPC feasibility ($90.1\%$) remain at $\sigma_v = 0.00\text{ m/s}$.
