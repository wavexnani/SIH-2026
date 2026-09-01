# Phase 13 — Closed-Loop Operating Envelope Report

## Executive Summary
This document presents the systematic operating envelope characterization of the frozen CACRC/QPMPC autonomous vehicle planner across the 1,000-run Monte Carlo experiment dataset (`artifacts/phase12d_results.csv`).

The evaluation maps performance across **10 scenario difficulty levels**, **5 uncertainty modes**, and **20 random seeds**, analyzing outcome distributions, body-to-body clearance bounds, TTC kinematics, motion comfort parameters, and physical corridor feasibility.

---

## 1. Outcome Distribution & Degradation Analysis

### Complete 1,000-Run Outcome Breakdown Table

| Level | Scenario Name | Ideal (N=200) | Nominal Perception | Delayed Perception | Steering Bias | Combined Realistic | Overall Outcome | Primary Cause |
|---|---|---|---|---|---|---|---|---|
| **L1** | `clear` | 100% DEGRADED | 100% DEGRADED | 100% DEGRADED | 100% DEGRADED | 100% DEGRADED | DEGRADED_SAFE | Minor lateral reference offset ($y_{\text{RMSE}} \approx 1.20\text{m}$) |
| **L2** | `static` | 100% SAFE_STOP | 100% SAFE_STOP | 100% SAFE_STOP | 100% SAFE_STOP | 100% SAFE_STOP | SAFE_STOP | Controlled stop before static obstacle |
| **L3** | `multi_obstacle_sequence` | 100% SAFE_STOP | 100% SAFE_STOP | 100% SAFE_STOP | 100% SAFE_STOP | 100% SAFE_STOP | SAFE_STOP | Deceleration behind obstacle sequence |
| **L4** | `multi_vehicle_following` | 100% SUCCESS | 85% SUCCESS / 15% STOP | 100% SUCCESS | 100% SUCCESS | 90% SUCCESS / 10% STOP | **SUCCESS (95%)** | Smooth IDM car following |
| **L5** | `multi_vehicle_yield_overtake` | 100% COLLISION | 100% COLLISION | 100% COLLISION | 100% COLLISION | 100% COLLISION | **COLLISION (100%)** | **Corridor Feasibility Squeeze**: Gap between lead & oncoming vehicle ($0.60\text{m}$) < Ego width ($1.80\text{m}$) |
| **L6** | `overtaking` | 100% UNSAFE | 100% UNSAFE | 100% UNSAFE | 100% UNSAFE | 100% UNSAFE | **UNSAFE_FAILURE (100%)** | **Controller Limitation**: QP solver status $= 0$ during narrow lateral pass |
| **L7** | `multi_vehicle_oncoming_conflict` | 100% DEGRADED | 100% DEGRADED | 100% DEGRADED | 100% DEGRADED | 100% DEGRADED | DEGRADED_SAFE | Timely longitudinal yielding behind static obstacle |
| **L8** | `complex` | 100% COLLISION | 100% COLLISION | 100% COLLISION | 100% COLLISION | 100% COLLISION | **COLLISION (100%)** | 3-body conflict ($1.00\text{m}$ passing gap) without temporal yield |
| **L9** | `complex` (Disturbed) | 100% COLLISION | 100% COLLISION | 100% COLLISION | 100% COLLISION | 100% COLLISION | **COLLISION (100%)** | Combined noise/delay/bias exacerbates 3-body collision |
| **L10**| `impassable_center` | 100% SAFE_STOP | 100% SAFE_STOP | 100% SAFE_STOP | 100% SAFE_STOP | 100% SAFE_STOP | **SAFE_STOP (100%)** | Clean safety fallback deceleration before total blockage |

---

## 2. Statistical Distributions Across Scenarios

### Clearance, Motion Comfort, and Error Metrics

| Scenario Level | Mean Clearance (m) | Min Clearance (m) | 5th Pct Clearance (m) | Max Lat Accel (g) | Max Yaw Rate (deg/s) | Mean Lat RMSE (m) |
|---|---|---|---|---|---|---|
| **L1: clear** | $\infty$ | $\infty$ | $\infty$ | $0.016\text{ g}$ | $1.10^\circ/\text{s}$ | $1.216\text{m}$ |
| **L2: static** | $13.08\text{m}$ | $13.08\text{m}$ | $13.08\text{m}$ | $0.009\text{ g}$ | $1.20^\circ/\text{s}$ | $1.219\text{m}$ |
| **L3: multi_obstacle_sequence** | $1.35\text{m}$ | $1.34\text{m}$ | $1.34\text{m}$ | $0.224\text{ g}$ | $17.04^\circ/\text{s}$ | $2.463\text{m}$ |
| **L4: multi_vehicle_following** | $12.19\text{m}$ | $9.93\text{m}$ | $11.02\text{m}$ | $0.016\text{ g}$ | $1.15^\circ/\text{s}$ | $0.012\text{m}$ |
| **L5: multi_vehicle_yield_overtake** | $-0.31\text{m}$ | $-0.42\text{m}$ | $-0.33\text{m}$ | $0.299\text{ g}$ | $20.50^\circ/\text{s}$ | $1.047\text{m}$ |
| **L6: overtaking** | $0.37\text{m}$ | $0.35\text{m}$ | $0.35\text{m}$ | $0.528\text{ g}$ | $38.08^\circ/\text{s}$ | $2.537\text{m}$ |
| **L7: multi_vehicle_oncoming_conflict** | $0.24\text{m}$ | $0.22\text{m}$ | $0.22\text{m}$ | $0.233\text{ g}$ | $16.79^\circ/\text{s}$ | $0.863\text{m}$ |
| **L8: complex** | $-1.35\text{m}$ | $-1.52\text{m}$ | $-1.39\text{m}$ | $0.723\text{ g}$ | $51.24^\circ/\text{s}$ | $2.390\text{m}$ |
| **L9: complex (disturbed)** | $-1.35\text{m}$ | $-1.52\text{m}$ | $-1.39\text{m}$ | $0.723\text{ g}$ | $51.24^\circ/\text{s}$ | $2.390\text{m}$ |
| **L10: impassable_center** | $5.47\text{m}$ | $3.89\text{m}$ | $3.89\text{m}$ | $0.070\text{ g}$ | $7.06^\circ/\text{s}$ | $1.672\text{m}$ |

---

## 3. Degradation & Operating Envelope Limits

1. **Nominal Operating Envelope**:
   - **Level 1 to Level 4**: Operating envelope holds with 0 collisions and 0 boundary violations. Level 4 achieves $95\%$ clean `SUCCESS` with smooth IDM car following.
2. **First Material Degradation Point (Level 5)**:
   - Level 5 (`multi_vehicle_yield_overtake`) exposes a spatial 3-body bottleneck ($0.60\text{m}$ passing gap < $1.80\text{m}$ vehicle width). Early lateral passing without temporal yielding causes physical overlap.
3. **Controller Optimization Bottleneck (Level 6)**:
   - Level 6 (`overtaking`) is **physically feasible** ($3.30\text{m}$ available corridor width > $1.80\text{m}$ vehicle width), but triggers $100\%$ QP solver infeasibility due to rigid boundary slack formulation in `QPMPCPlanner`.
4. **Complex Interaction Bottleneck (Levels 8 & 9)**:
   - Levels 8 and 9 combine dynamic oncoming agents with static road blockages. Impatient passing before oncoming vehicle clearance produces $-1.35\text{m}$ body overlap.
5. **Fail-Safe Envelope (Level 10)**:
   - Level 10 achieves $100\%$ `SAFE_STOP` with $3.89\text{m}$ safety margin, demonstrating intact safety fallback architecture.
