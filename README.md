# Perception-Derived Free-Space Autonomous Navigation for Unstructured Indian Roads

> **Scientific Conclusion**:  
> **HYPOTHESIS SUPPORTED BY CONTROLLED SIMULATION**  
> *On the evaluated unstructured-road benchmark, perception-derived free-space navigation successfully identified and traversed a narrow physically feasible corridor that the fixed-lane representation could not exploit. The fixed-lane controller exhibited structural geometric infeasibility and premature stopping, while the free-space controller completed the bottleneck with zero collisions, zero SafetyFilter interventions, and positive OBB clearance.*

---

## 1. Problem Statement

Autonomous vehicles relying on hardcoded fixed-lane centerlines fail on unstructured Indian roads due to:
- Absence of lane markings.
- Irregular, undulating road boundaries.
- Unstructured road users (e.g., parked autorickshaws, encroaching stalls, free-roaming cattle).

---

## 2. Proposed Solution

Instead of requiring fixed lane centerlines, the vehicle dynamically extracts local drivable free-space bounds $[y_{\min}(x), y_{\max}(x)]$ from real-time perception observations. These admissible spatial bounds are directly supplied to the existing **CACRC / QP-MPC** optimal planning and **SafetyFilter** architecture.

---

## 3. Key Architectural Innovation

The planner does not require a predefined lane centerline to determine a safe lateral reference. By computing local admissible intervals from perceived obstacles and road boundaries, `FreeSpaceMap` and `FreeSpaceBoundProvider` derive optimal, collision-free lateral corridors dynamically on the fly.

---

## 4. Benchmark Performance

### Static Bottleneck Benchmark

$$\begin{aligned}
\text{Fixed Lane Baseline} &\longrightarrow \mathbf{\text{FAIL}} \quad (\text{Stopped at } x = 33.42\text{ m due to anticipatory avoidance lockup}) \\
\text{Free Space Proposed} &\longrightarrow \mathbf{\text{PASS}} \quad (\text{Completed full traversal } x = 177.23\text{ m with 0 interventions})
\end{aligned}$$

### Dynamic Cattle Coordination Benchmark

$$\begin{aligned}
\text{Cattle Detected at } x = 65\text{ m} &\longrightarrow \text{YIELD Intent Triggered at } k = 1 \\
&\longrightarrow \text{Controlled Deceleration Stop at } x = 28.75\text{ m} \\
&\longrightarrow \mathbf{0\text{ Collisions}}, \quad +5.85\text{ m Minimum Clearance}
\end{aligned}$$

---

## 5. Critical Physical Numerical Summary

| Metric | Value |
| :--- | :--- |
| **Physical Bottleneck Gap** | **2.00 m** (Obstacle 1 at $y=1.00\text{m}$, Obstacle 2 at $y=4.50\text{m}$) |
| **Vehicle Width / Half-Width** | **1.80 m / 0.90 m** |
| **Collision-Free Center Corridor** | **[2.65, 2.85] m** |
| **Free-Space Safety Corridor Target** | **$y_{ref} = 2.75\text{ m}$** |
| **Minimum True OBB Obstacle Clearance** | **+0.0998 m** ($+10.0\text{ cm}$) |
| **Proposed Physical Collisions** | **0** |
| **Proposed SafetyFilter Interventions** | **0** |
| **Baseline Maximum $x$ Position** | **33.42 m** |
| **Proposed Maximum $x$ Position** | **177.23 m** |

---

## 6. Project Architecture & Reproduction Instructions

### Run Final Sanity & Regression Suite:
```matlab
addpath('planning', 'config', 'vehicle', 'core', 'environment', 'stages', 'tests', 'scratch');
run_phase31_sanity_suite();
```

### Run Static Bottleneck A/B Benchmark:
```matlab
run_bottleneck_ab_experiment();
```

### Run Dynamic Cattle Yield Benchmark:
```matlab
run_cattle_coordination_experiment();
```

---

## 7. License & System Status

- **Production Core**: `QPMPCPlanner.m`, `CACRCPlanner.m`, `SafetyFilter.m`, `BicycleModel.m`, `Stage5CoordinationController.m`, `CoordinationDecisionLayer.m` are **FROZEN**.
- **Hackathon Status**: **READY FOR DEMONSTRATION**.
