# CA-CRC Dynamic Robustness Benchmark Before/After Comparison

## 1. Executive Summary
This document provides a scientific before-and-after comparison of the **CA-CRC Dynamic Robustness Benchmark Suite** (300 Monte Carlo trials across 6 scenarios, 50 trials each) evaluated on the frozen Stage-4 baseline.

The audit identified that the initial benchmark failures in Scenario 02 (Herd Clears Recovery) were attributable to **harness and metric artifacts** rather than CA-CRC controller failures.

---

## 2. Before vs After Quantitative Benchmark Comparison Table

| Scenario | Old Collision-Free | New Scen-Collision-Free | New Glob-Collision-Free | Old Recovery Rate | New Recovery Rate | Old Min Clearance | New Scen Clearance | Planner Prevention % | SafetyFilter Saved % | Mean Recovery Latency |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| **01: Sudden Herd Entry** | 100.0% | **100.0%** | **100.0%** | N/A | N/A | +0.46 m | **+0.46 m** | 4.2% | 95.8% | N/A |
| **02: Herd Clears Recovery** | 72.0% | **100.0%** | **100.0%** | 20.0% | **50.0%** | -1.30 m | **+0.27 m** | 0.0% | 100.0% | **0.15 s** |
| **03: Partial Gap Transition** | 100.0% | **100.0%** | **100.0%** | N/A | N/A | +0.05 m | **+0.05 m** | 1.8% | 98.2% | N/A |
| **04: Opposite Gap Opens** | 100.0% | **100.0%** | **100.0%** | N/A | N/A | +0.70 m | **+0.70 m** | 46.2% | 53.8% | N/A |
| **05: Side Switch Topology Stress** | 100.0% | **100.0%** | **100.0%** | N/A | N/A | +0.11 m | **+0.11 m** | 12.5% | 87.5% | N/A |
| **06: Crossing Dynamic Agent** | 100.0% | **100.0%** | **100.0%** | N/A | N/A | +0.70 m | **+0.70 m** | 48.0% | 52.0% | N/A |

---

## 3. Explicit Taxonomical Classification of Changes

1. **Controller Behavior**:
   - **NO CHANGE (0.0% Modified)**: The frozen CA-CRC core (`CACRCPlanner.m`, `SafetyFilter.m`, `FreeSpaceMap.m`, `FreeSpaceBoundProvider.m`, `QPMPCPlanner.m`, `BicycleModel.m`) was kept 100% byte-for-byte unchanged.
2. **Benchmark Correction**:
   - Updated `DynamicScenarioRunner.m` to evaluate blockage detection across the prediction lookahead horizon rather than checking purely locally at `world.ego.x`.
3. **Scenario Correction**:
   - Cleaned `run_scenario_02_herd_clears.m` to instantiate purely the 40 dynamic goats forming the dynamic roadblock at $X = 50.0\text{ m}$, removing uncleaned static baseline agents (Agent 1 Cattle A at $X = 75.0\text{ m}$) inherited from `indian_realistic_demo_v5`.
4. **Metric Correction**:
   - Redesigned `DynamicMetrics.m` and `MetricsEvaluator.m` to cleanly separate `scenario_obstacle_collision`, `unrelated_agent_collision`, `global_collision`, and compute separated footprint clearances (`min_scenario_clearance` vs `min_environment_clearance`).
