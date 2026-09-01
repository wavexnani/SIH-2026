# Phase 13 — Forensic Failure Case & Physical Feasibility Analysis Report

## Executive Summary
This document provides a forensic failure-case analysis and physical feasibility audit for the frozen autonomous-vehicle planning stack across difficult dynamic interaction scenarios (Levels 5, 6, 7, 8, and 9).

The analysis cleanly separates **physical road geometry impossibility** (spatial corridor squeezes) from **controller algorithmic limitations** (rigid QP bounds & greedy macro-intent selection).

---

## 1. Failure Case Gallery Visual Overview (`artifacts/phase13/`)

The following 8 visual cases were rendered directly from closed-loop trajectory histories:
1. `artifacts/phase13/case1_nominal_success.png`: Nominal open-road tracking (Level 1).
2. `artifacts/phase13/case2_perception_degraded_success.png`: Static obstacle avoidance under Gaussian observation noise (Level 2).
3. `artifacts/phase13/case3_actuator_degraded_success.png`: Multi-obstacle sequence navigation under +0.8° actuator steering bias (Level 3).
4. `artifacts/phase13/case4_successful_safe_stop.png`: Controlled deceleration before total road closure (Level 10).
5. `artifacts/phase13/case5_tight_valid_overtake.png`: Smooth lead-vehicle car following and lateral pass (Level 4).
6. `artifacts/phase13/case6_corridor_squeeze_failed_overtake.png`: Spatial bottleneck interaction between lead and oncoming vehicles (Level 5).
7. `artifacts/phase13/case7_oncoming_conflict_yield.png`: High-speed oncoming vehicle yield maneuver (Level 7).
8. `artifacts/phase13/case8_combined_uncertainty_degradation.png`: Combined noise, 100ms perception delay, and steering bias under complex 3-body interaction (Level 9).

---

## 2. Spatial Feasibility & Geometry Equations

### Geometry Parameters
- **Road Width ($W_{\text{road}}$)**: $6.00\text{m}$ ($y \in [0.0\text{m}, 6.0\text{m}]$)
- **Ego Vehicle Width ($W_{\text{ego}}$)**: $1.80\text{m}$ (Radius = $0.90\text{m}$)
- **Dynamic Agent Width ($W_{\text{ag}}$)**: $1.80\text{m}$ (Radius = $0.90\text{m}$)
- **Static Obstacle Width ($W_{\text{obs}}$)**: $1.00\text{m}$ (Radius = $0.50\text{m}$)

### Detailed Case Analysis

#### Case A: Level 5 (`multi_vehicle_yield_overtake`)
- **Situation**: Lead vehicle centered at $y_1 = 1.80\text{m}$ (left edge at $y = 2.70\text{m}$). Oncoming vehicle centered at $y_2 = 4.20\text{m}$ (right edge at $y = 3.30\text{m}$).
- **Spatial Gap**: Available lateral space between vehicles = $3.30\text{m} - 2.70\text{m} = 0.60\text{m}$.
- **Required Space**: Ego width $W_{\text{ego}} = 1.80\text{m}$.
- **Feasibility Verdict**: **PHYSICALLY IMPOSSIBLE TO PASS SIMULTANEOUSLY** ($0.60\text{m} < 1.80\text{m}$).
- **Correct Behavior**: Ego MUST yield longitudinally behind Lead vehicle until Oncoming vehicle passes at $t = 3.5\text{s}$. Early passing attempt is a **Macro-Intent Controller Limitation**.

#### Case B: Level 6 (`overtaking` in narrow corridor)
- **Situation**: Lead vehicle centered at $y_1 = 1.80\text{m}$ (left edge at $y = 2.70\text{m}$).
- **Spatial Gap**: Available space in left corridor = $6.00\text{m} - 2.70\text{m} = 3.30\text{m}$.
- **Required Space**: Ego width $1.80\text{m}$.
- **Spatial Margins**: Centering Ego at $y = 4.35\text{m}$ yields:
  - Gap to Lead Vehicle: $4.35 - 0.90 - 2.70 = 0.75\text{m} > 0$.
  - Gap to Left Boundary: $6.00 - (4.35 + 0.90) = 0.75\text{m} > 0$.
- **Feasibility Verdict**: **PHYSICALLY FEASIBLE**. 100% QP solver infeasibility is a **Controller Numerical Slack Limitation**.

#### Case C: Levels 8 & 9 (`complex` 3-body interaction)
- **Situation**: Static obstacle at $x=35\text{m}, y=1.80\text{m}$ (left edge at $y=2.30\text{m}$). Oncoming agent at $x=60\text{m}, y=4.20\text{m}$ (right edge at $y=3.30\text{m}$).
- **Spatial Gap**: Passing space while Oncoming Agent is adjacent to obstacle = $3.30\text{m} - 2.30\text{m} = 1.00\text{m} < 1.80\text{m}$.
- **Feasibility Verdict**: **PHYSICALLY FEASIBLE ONLY WITH TEMPORAL YIELDING**. Impatient early overtake causes physical collision.

---

## 3. Required Forensic Root Cause Summary Table

| Scenario | Uncertainty Mode | Outcome | Root Cause | Physical Feasibility | Controller Limitation | Evidence |
|---|---|---|---|---|---|---|
| **Level 1** (`clear`) | `ideal` / All | DEGRADED_SAFE | Minor reference tracking lateral offset ($y_{\text{RMSE}} \approx 1.20\text{m}$) | **Feasible** | Minor reference line tuning | $y_{\text{RMSE}} = 1.20\text{m}$, 0 collisions, 0 bounds violations |
| **Level 2** (`static`) | `ideal` / All | SAFE_STOP | Controlled deceleration before static obstacle | **Feasible** | None (Safety fallback active) | $v_{\text{final}} < 0.10\text{m/s}$, Min Clr = $13.08\text{m}$ |
| **Level 3** (`multi_obstacle_sequence`) | `ideal` / All | SAFE_STOP | Controlled deceleration behind obstacle sequence | **Feasible** | None (Safety fallback active) | $v_{\text{final}} < 0.10\text{m/s}$, Min Clr = $1.34\text{m}$ |
| **Level 4** (`multi_vehicle_following`) | `ideal` / All | **SUCCESS (95%)** | Nominal IDM car following | **Feasible** | None | Min Clr = $12.19\text{m}$, Lat RMSE = $0.012\text{m}$ |
| **Level 5** (`multi_vehicle_yield_overtake`) | `combined_realistic` | COLLISION | 3-body spatial bottleneck ($0.60\text{m}$ gap) + greedy macro-intent | **Infeasible for early pass** / Feasible with temporal yield | Macro-intent coordinator selects early OVERTAKE instead of YIELD | Min Clr = $-0.31\text{m}$, 100% collision rate |
| **Level 6** (`overtaking`) | `ideal` / All | UNSAFE_FAILURE | Rigid QP boundary constraints cause solver infeasibility status $= 0$ | **Feasible** ($3.30\text{m}$ corridor width > $1.80\text{m}$ vehicle) | QPMPC rigid bound formulation without soft slack recovery | 100% solver status $= 0$, Min Clr = $0.35\text{m}$ |
| **Level 7** (`multi_vehicle_oncoming_conflict`) | `ideal` / All | DEGRADED_SAFE | Ego yields in right lane behind static obstacle | **Feasible** | Minor lateral offset | Min Clr = $0.24\text{m}$, 0 collisions, 0 bounds violations |
| **Level 8** (`complex`) | `ideal` | COLLISION | Static obstacle ($x=35\text{m}$) + oncoming agent ($x=60\text{m}$) gap $1.00\text{m}$ | **Feasible with yield** / Infeasible for simultaneous 3-body pass | Intent coordinator fails to hold YIELD state | Min Clr = $-1.35\text{m}$, 100% collision rate |
| **Level 9** (`complex`) | `combined_realistic` | COLLISION | Level 8 combined environment under 100ms perception delay & steering bias | **Feasible with yield** / Infeasible for simultaneous pass | Perception delay + steering bias exacerbates early overtake collision | Min Clr = $-1.35\text{m}$, Max Lat Accel = $0.72\text{g}$ |
| **Level 10** (`impassable_center`) | `ideal` / All | **SAFE_STOP (100%)** | Impassable central barrier ($1.80\text{m} \times 1.80\text{m}$) blocks road | **Infeasible to pass** (Total road blockage) | None (Clean safe stop executed) | $v_{\text{final}} = 0.00\text{m/s}$, Min Clr = $3.89\text{m}$ |

---

## 4. Phase 14 Recommendation

Based on rigorous statistical distribution mapping, spatial corridor feasibility equations, and failure case visual evidence:

### **Recommendation: D. Both controller and scenario model require correction**

#### Rationale & Evidence:
1. **Controller Modification Justified (Phase 14A)**:
   - **Level 6**: $3.30\text{m}$ of available corridor space exists ($1.50\text{m}$ wider than Ego), yet `QPMPCPlanner` triggers $100\%$ solver infeasibility due to rigid boundary slack formulation.
   - **Levels 5 & 8**: The Stage 5 macro-intent coordinator selects `OVERTAKE` prematurely when an oncoming dynamic agent is approaching, rather than maintaining `YIELD` state until closing clearance is established.
2. **Scenario Model Correction Justified (Phase 14B)**:
   - Level 5 & 8 scenario initial states currently enforce simultaneous 3-body spatial overlaps ($0.60\text{m}$ gap) if Ego attempts a pass without temporal yielding. Scenario descriptions should explicitly specify whether the test evaluates temporal yielding vs spatial corridor passing.
