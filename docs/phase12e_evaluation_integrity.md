# Phase 12E — Evaluation Integrity & Scenario Realism Audit Report

## Baseline & Protection Compliance
- **Git Baseline Commit**: `8cfc240`
- **Hard Freeze Protection**: The core planning stack (`CACRCPlanner.m`, `QPMPCPlanner.m`, `Stage5CoordinationController.m`, `SafetyFilter.m`, `FreeSpaceMap.m`, `FreeSpaceBoundProvider.m`, `AbstractBoundProvider.m`, `MultiVehicleDetector.m`) remained strictly untouched. No QP cost matrices or planner parameters were tuned.

---

## 1. Monte Carlo Accounting & Run Verification
- **Requested Runs**: 1,000 (10 Scenario Levels $\times$ 5 Uncertainty Modes $\times$ 20 Random Seeds)
- **Completed Runs**: 1,000
- **Failed / Aborted Runs**: 0
- **Accounting Verification**:
  - `artifacts/phase12d_results.csv` contains exactly 1,000 rows plus header.
  - Explicit columns: `Requested`, `Completed`, `Failed`, `Level`, `Mode`, `Seed`, `TrajectoryHash`, `Outcome`, `LatRMSE`, `MinClr`, `MinTTC`, `MaxLatAccel`, `MaxYawRate`, `EmergencyCount`, `QPInfeasCount`.

---

## 2. Seed Independence & Determinism Audit
- **Identical Seed Reproducibility**: Executing the identical stochastic setup with `seed = 42` twice produced bit-exact trajectories (`TrajectoryHash: 74dd8856`, Difference = $0.00\times 10^0$).
- **Seed Variation**: Executing with `seed = 42` vs `seed = 99` produced distinct stochastic realizations (`Hash1: 74dd8856` vs `Hash3: fa2cea29`).
- **Initialization Invariance**: Changing the seed does not alter deterministic scenario initial conditions (ego position $(10, 1.80)$, obstacle coordinates, dynamic agent velocity).

---

## 3. Scenario Ladder Distinctness (10 Levels)
1. **Level 1 (`clear`)**: Nominal straight open road.
2. **Level 2 (`static`)**: Static obstacle avoidance.
3. **Level 3 (`multi_obstacle_sequence`)**: Road geometry & static obstacle sequence.
4. **Level 4 (`multi_vehicle_following`)**: Slow lead vehicle car following with IDM dynamics.
5. **Level 5 (`multi_vehicle_yield_overtake`)**: Conflicting dynamic vehicle crossing ego lane.
6. **Level 6 (`overtaking`)**: Slow lead vehicle requiring lateral pass in narrow corridor.
7. **Level 7 (`multi_vehicle_oncoming_conflict`)**: High-speed opposing dynamic agent conflict.
8. **Level 8 (`complex`)**: Combined narrowing, static obstacle, and dynamic vehicle.
9. **Level 9 (`complex` disturbed)**: Level 8 combined environment under 100ms delay, noise, and +0.8° steering bias.
10. **Level 10 (`impassable_center`)**: Deliberately infeasible total free-space collapse.

---

## 4. TTC Vector Kinematics & Oncoming Traffic Audit
- **Discovered Issue**: Previous TTC calculation used scalar speed difference $v_{\text{ego}} - \|\mathbf{v}_{\text{agent}}\|$, which yielded negative closing speed for head-on oncoming traffic ($v_{\text{agent}, x} < 0$), evaluating TTC to $\infty$.
- **Fix Implemented**: Corrected vector TTC formula considering velocity heading:
  $$v_{\text{closing}} = v_{\text{ego}, x} - v_{\text{agent}, x}$$
  For oncoming traffic heading towards ego ($v_{\text{agent}, x} < 0$), $v_{\text{closing}} = v_{\text{ego}} + |v_{\text{agent}, x}| > 0$.
- **Physical Verification**: Oncoming head-on vehicles now evaluate to finite physical TTC (e.g. $2.7154\text{s}$ at $13.0\text{m/s}$ closing rate). Vehicles in parallel adjacent lanes evaluate to $\infty$ unless a lateral trajectory overlap occurs.

---

## 5. Minimum-Clearance Semantics
- **Definition**: Exact ground-truth body-to-body footprint clearance in meters (distance between ego polygon boundary and obstacle polygon boundary).
- **Semantics**:
  - Clearance $> 0.0\text{m}$: Safe physical gap.
  - Clearance $= 0.0\text{m}$: Touch / contact.
  - Clearance $< 0.0\text{m}$: Physical footprint overlap.
- **Safety Margin Exclusion**: Body-to-body clearance excludes artificial planning safety margins.

---

## 6. Outcome Classification Precedence Truth Table
Rules evaluated in strict mutually exclusive order:
1. `COLLISION`: Physical footprint collision (`is_collision` > 0).
2. `UNSAFE_FAILURE`: Drivable road boundary violation (`inside_bounds` false).
3. `PLANNER_INFEASIBLE`: QP solver status $= 0$ for $> 5$ steps and vehicle did NOT bring itself to safe standstill ($v_{\text{final}} \ge 0.10\text{m/s}$).
4. `SAFE_STOP`: Vehicle brought to controlled stop ($v_{\text{final}} < 0.10\text{m/s}$) before blockage with 0 collisions.
5. `DEGRADED_SAFE`: 0 collisions, 0 boundary violations, but $y_{\text{RMSE}} > 0.50\text{m}$ or emergency braking / filter active steps $> 0$.
6. `SUCCESS`: 0 collisions, 0 boundary violations, $y_{\text{RMSE}} \le 0.50\text{m}$, 0 emergency interventions.

---

## 7. Safe-Stop Detection Validation (Level 10)
- **Road Closure**: Level 10 (`impassable_center`) places a $1.80\text{m} \times 1.80\text{m}$ barrier at $x=30\text{m}$, blocking both left and right corridor passing spaces.
- **Stopping Behavior**: Vehicle decelerates smoothly under CACRC safety fallback to $v < 0.10\text{m/s}$ at $x \approx 23.5\text{m}$, maintaining a safe distance of $\approx 3.89\text{m}$ from the barrier.
- **Classification**: 100% classified as `SAFE_STOP` with 0 collisions.

---

## 8. Uncertainty-Mode Separation & Telemetry Verification
- **Observation Noise Telemetry**: Confirmed Gaussian position noise ($\sigma=0.15\text{m}$), velocity noise ($\sigma=0.20\text{m/s}$), and heading noise ($\sigma=0.02\text{rad}$).
- **100ms Perception Delay**: Delay buffer preserves exact state from $k-1$ ($0.10\text{s}$ age).
- **Steering Bias Telemetry**: Actuator model consistently offsets command $\delta_{\text{actual}} = \delta_{\text{cmd}} + 0.8^\circ$.

---

## 9. Flagship Demo State Transition Authenticity
- `run_phase12d_combined_demo.m` tracks closed-loop ego state transitions (`DETECT` $\to$ `FOLLOW` $\to$ `YIELD/OVERTAKE` $\to$ `MANEUVER` $\to$ `CLEAR` $\to$ `RECENTER`) derived directly from closed-loop macro-intent selections and distance triggers.

---

## Summary Table of Issues Discovered & Fixed
| # | Issue Description | Severity | Fix Applied | Status |
|---|---|---|---|---|
| 1 | Scalar TTC calculation evaluated oncoming traffic ($v_x < 0$) as negative closing speed $\to$ reported $\text{TTC} = \text{Inf}$ | HIGH | Vector TTC kinematics implemented ($v_{\text{closing}} = v_{\text{ego}, x} - v_{\text{agent}, x}$) | **FIXED** |
| 2 | Monte Carlo CSV missing explicit run accounting (`requested`, `completed`, `failed`) and trajectory signatures | MEDIUM | Added explicit accounting fields and MD5 trajectory signature hashing | **FIXED** |
| 3 | Outcome classification ambiguity between degraded safe and safe stop | MEDIUM | Established strict mutually exclusive 6-rule precedence truth table | **FIXED** |
| 4 | Off-road dummy agents ($x = -100$) evaluated in TTC loop | LOW | Added `ag.x < -50` filter check to ignore off-road dummy agents | **FIXED** |

Remaining Limitations:
- The 6.0m narrow two-lane corridor creates physical spatio-temporal boundary squeezes during high-speed overtaking maneuvers (Levels 5, 6, 8, 9), which represents physical space limitations rather than planner code instability.
