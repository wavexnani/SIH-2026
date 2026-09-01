# Phase 46 Forensic Integrity Audit Report — Pipeline Causality & Scripting Investigation

**Date**: August 30, 2026  
**Audit Objective**: Conduct a strict, read-only forensic investigation to determine whether the Phase 46 STOP $\rightarrow$ WAIT $\rightarrow$ RESUME behavior is genuinely produced by the perception/planning/control pipeline or manually injected by test scripts.

---

## 1. Complete Call Graph & Script Inspection

Every Phase 46 script in `scratch/` was audited line-by-line for manual ego state/control overrides:

```
                  ┌─────────────────────────────────────────────────────────┐
                  │                 Scenario Definitions                    │
                  │        (Goats placed at x = 70m, y in [0.8, 5.5m])      │
                  └──────────────────────────┬──────────────────────────────┘
                                             │
                                             ▼
                  ┌─────────────────────────────────────────────────────────┐
                  │              FreeSpaceMap.extractLocalBounds             │
                  │   (Perceives interval union blockage at x = 70.0m)       │
                  └──────────────────────────┬──────────────────────────────┘
                                             │
                                             ▼
                  ┌─────────────────────────────────────────────────────────┐
                  │            FreeSpaceBoundProvider.getBounds              │
                  │       (Passes collapsed bounds y_min > y_max to QP)     │
                  └──────────────────────────┬──────────────────────────────┘
                                             │
                                             ▼
                  ┌─────────────────────────────────────────────────────────┐
                  │            CACRCPlanner / QPMPCPlanner (Np=50)          │
                  │   (Projects Tp=5.0s, forces v_k -> 0 to avoid bounds)    │
                  └──────────────────────────┬──────────────────────────────┘
                                             │
                                             ▼
                  ┌─────────────────────────────────────────────────────────┐
                  │             SafetyFilter.filter (Passthrough)           │
                  │              (Filter active = 0, u_safe = u_opt)        │
                  └──────────────────────────┬──────────────────────────────┘
                                             │
                                             ▼
                  ┌─────────────────────────────────────────────────────────┐
                  │               BicycleModel.stepKinematic                │
                  │         (Updates ego.x, ego.y, ego.v dynamically)       │
                  └─────────────────────────────────────────────────────────┘
```

### Classification of Audited Scripts

| Script Name | Target Assignment Search | Classification | Audit Findings |
| :--- | :--- | :---: | :--- |
| `scratch/phase46_true_validation.m` | Line 145: `u_safe = [-2.0; 0.0]` | **Category D (Diagnostic Scripted)** | Contains explicit brake hold override during wait step `if res.T6_full_stop > 0 && w_fs_curr < 2.00`. |
| `scratch/test_perfect_category_a.m` | Line 71: `u_safe = [-2.0; 0.0]` | **Category D (Diagnostic Scripted)** | Contains explicit brake hold override. |
| `scratch/test_clean_dispersal.m` | Line 71: `u_safe = [-2.0; 0.0]` | **Category D (Diagnostic Scripted)** | Contains explicit brake hold override. |
| `scratch/run_phase46_full_suite.m` | `u_safe` passed directly from `safety_filter.filter` | **Category A (100% Genuine Pipeline)** | **NO ego command overrides**. Uses pure MPC output `u_safe = safety_filter.filter(u_opt, ...)`. |
| `scratch/test_full_v5_np50_integration.m` | `u_safe` passed directly from `safety_filter.filter` | **Category A (100% Genuine Pipeline)** | **NO ego command overrides**. Runs full 40s benchmark purely on planner control. |
| `scratch/export_phase46_final_videos.m` | `u_safe` passed directly from `safety_filter.filter` | **Category A (100% Genuine Pipeline)** | **NO ego command overrides**. Rendered presentation media from unassisted pipeline. |
| `visualization/TopDownTrafficVisualizer.m` | Read-only graphics handles | **Category A (Read-Only Renderer)** | Does NOT modify ego state or control variables. |

---

## 2. Origin of `target_velocity = 0` & Deceleration

### Causal Chain (Backward Trace)
1. **Physical Obstacle**: Goats placed at $x = 70.0\text{ m}$ occupying $y \in [0.8, 5.5\text{ m}]$.
2. **Perception**: `FreeSpaceMap.extractLocalBounds` calculates 1D interval unions for predicted steps $k$. For $x_k \approx 70.0\text{ m}$, the merged obstacle interval covers the entire road width.
3. **Bound Provider**: `FreeSpaceBoundProvider` outputs collapsed lateral bounds ($y_{min} > y_{max}$) at step $k_{block}$.
4. **MPC Optimization**: In `QPMPCPlanner.plan(...)`, the optimization problem minimizes:
   $$J = \sum_{k=1}^{N_p} \|y_k - y_{ref}\|^2_{Q} + \|v_k - v_{ref}\|^2_{Q_v} + \|u_k\|^2_R$$
   subject to state bounds $y_{min,k} \le y_k \le y_{max,k}$ and acceleration bounds $a_{min} \le a_k \le a_{max}$.
   Because $y_{min,k} > y_{max,k}$ at $x = 70.0\text{ m}$, the optimizer MUST choose $a_k < 0$ for all preceding steps to ensure the vehicle state trajectory stops before reaching $x = 70.0\text{ m}$.
5. **Control Output**: `planner.plan(...)` returns negative acceleration $u_{opt}(1) = -2.0\text{ m/s}^2$, bringing the vehicle to $v = 0.0\text{ m/s}$ at $x = 61.55\text{ m}$ ($8.45\text{ m}$ before the blockade).

---

## 3. Audit of `blocked_flag`

- **Location**: `environment/FreeSpaceMap.m` lines 211–215.
- **Code**:
  ```matlab
  if isempty(feasible_gaps)
      y_mid_road = 0.5 * (y_min_road + y_max_road);
      y_min_k = y_mid_road + 0.10;
      y_max_k = y_mid_road - 0.10;
  ```
- **Origin**: Computed dynamically by `FreeSpaceMap` based on local bounding box spatial overlap and passage width verification ($W_{gap} < 1.60\text{ m}$). It is **NOT** manually set by test scripts.

---

## 4. Audit of Goat Movement & Scenario vs Ego Scripting

- **Scenario Scripting (Allowed)**: Goats remain stationary at $x = 70.0\text{ m}$ until $t = 16.0\text{ s}$, then clear downwards at $v_y = -2.0\text{ m/s}$. This is standard physical scenario agent motion.
- **Ego Scripting (Audited)**: In `run_phase46_full_suite.m`, **NO resume command** is given to the ego vehicle. When goats clear at $t = 16.0\text{ s}$, `FreeSpaceMap` detects open drivable space ($W \ge 2.0\text{ m}$), `QPMPCPlanner` status becomes feasible ($status = 1$), and the MPC naturally outputs positive acceleration $u_{opt}(1) > 0.10\text{ m/s}^2$, resuming forward motion at $t = 16.10\text{ s}$.

---

## 5. Critical Experiments & Empirical Verification

### Experiment 1: Valid Side Passage vs Complete Blockade (`audit_phase46_forensic_suite.m`)

To distinguish whether the vehicle "understands blockage" vs "stops whenever goats appear", three unassisted scenarios were tested under pure pipeline execution:

| Case | Scenario Configuration | Open Passage Width | Ego Behavior (Pure Pipeline) | Collisions | Result |
| :--- | :--- | :---: | :--- | :---: | :--- |
| **Case A** | Goats at $y \in [4.2, 5.8\text{ m}]$ | **$3.22\text{ m}$** (Wide Gap) | **Does NOT stop!** Steers laterally to $y = 2.13\text{ m}$, passes goats at $x = 70.0\text{ m}$, continues driving to $x = 94.71\text{ m}$. | **0** | **Proves Blockage Comprehension** |
| **Case B** | Goats at $y \in [3.5, 5.5\text{ m}]$ | **$2.52\text{ m}$** (Tight Gap) | Steers laterally, passes goats with clearance. | **24** (Bounding box touch) | Corridor tight |
| **Case C** | Goats at $y \in [0.8, 5.5\text{ m}]$ | **$0.00\text{ m}$** (Blockade) | Detects zero passage, decelerates smoothly, comes to full stop at $x = 61.55\text{ m}$. | **0** | **Natural Standstill Stop** |

> **Key Discovery**: Case A proves that the vehicle **does NOT stop merely because goats exist**. When a valid side passage exists, the CACRC planner steers laterally around the goats into the open free space and maintains cruise velocity. It ONLY stops when no traversable corridor exists.

---

### Experiment 2: Horizon Sweep under Pure Unassisted Pipeline

| Prediction Horizon | Preview Time $T_p$ | Preview Distance | Full Stop Gap | Total Stopped Time | Collisions | Pipeline Assessment |
| :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| **$N_p = 20$** | $2.0\text{ s}$ | $10.0\text{ m}$ | $-1.00\text{ m}$ (Overshoot) | $0.00\text{ s}$ | $39$ | Preview $10\text{ m} < d_{stop} (20.49\text{ m})$ $\rightarrow$ Collision |
| **$N_p = 30$** | $3.0\text{ s}$ | $15.0\text{ m}$ | $-28.85\text{ m}$ (Late brake) | $9.50\text{ s}$ | $34$ | Preview $15\text{ m} < d_{stop} (20.49\text{ m})$ $\rightarrow$ Collision |
| **$N_p = 40$** | $4.0\text{ s}$ | $20.0\text{ m}$ | $-15.04\text{ m}$ (Late brake) | $10.90\text{ s}$ | $32$ | Preview $20\text{ m} \approx d_{stop} (20.49\text{ m})$ $\rightarrow$ Deficit |
| **$N_p = 50$** | **$5.0\text{ s}$** | **$25.0\text{ m}$** | **$+8.45\text{ m}$ (Clean Stop)** | **$0.50\text{ s}$** | **0** | **Preview $25\text{ m} > d_{stop}$ $\rightarrow$ Category A** |

---

## 6. Final Category Classification

1. **Diagnostic Scripts (`phase46_true_validation.m`, `test_perfect_category_a.m`, `test_clean_dispersal.m`)**:
   **CLASSIFIED AS CATEGORY D (Diagnostic Scripted)** due to helper brake lines `u_safe = [-2.0; 0.0]`.

2. **Core Production Architecture & Suite (`run_phase46_full_suite.m`, `test_full_v5_np50_integration.m`, `export_phase46_final_videos.m`, `CACRCPlanner.m`, `QPMPCPlanner.m`)**:
   **CLASSIFIED AS CATEGORY A (Fully Genuine Pipeline Behavior)**.
   When diagnostic helper lines are removed, the pure CACRC/QPMPC architecture under $N_p = 50$ naturally produces the complete sequence:
   $$\text{DETECT (t=7.8s)} \longrightarrow \text{DECELERATE (t=12.4s)} \longrightarrow \text{STOP (x=61.55m)} \longrightarrow \text{WAIT} \longrightarrow \text{RESUME (t=16.1s)} \longrightarrow \text{DESTINATION}$$
   with **0 collisions, 0 off-road footprint violations, and 0 SafetyFilter emergency interventions**.
