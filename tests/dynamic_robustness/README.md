# CA-CRC Dynamic Robustness Benchmark Suite

This directory contains the isolated, multi-scenario Monte Carlo test framework for evaluating the frozen **CA-CRC Stage-4 Navigation Stack** against dynamic agents and road blockage scenarios.

## Architectural Isolation
- **Frozen Core Baseline**: Planning modules (`CACRCPlanner.m`, `QPMPCPlanner.m`, `SafetyFilter.m`, `FreeSpaceMap.m`, `FreeSpaceBoundProvider.m`) remain byte-for-byte read-only.
- **Scenario World Decoupling**: All obstacle variations, agent dynamics, and road perturbations are injected strictly at the `WorldState` / `ObservationModel` layer.

## Included Scenarios (300 Total Monte Carlo Trials)
1. **Scenario 01 (Sudden Herd Entry)**: Dense 60-goat crossing forcing dynamic corridor collapse ($y_{\min} > y_{\max}$) and emergency stopping.
2. **Scenario 02 (Herd Clears Recovery)**: Tests recovery latency $T_{\text{recovery}} = T_{\text{resume}} - T_{\text{gap-open}}$ as herd moves off-road.
3. **Scenario 03 (Partial Gap Transition)**: Smooth gap expansion ($0.8\text{m} \to 2.2\text{m}$) testing feasibility transition around $W_{\text{req}} = 1.60\text{m}$.
4. **Scenario 04 (Opposite Gap Opens)**: Asymmetric blockage where left corridor is blocked and right corridor re-opens.
5. **Scenario 05 (Side Switch Topology Stress)**: Dynamic swap where open left corridor closes while right corridor opens, testing topological memory (`locked_side`).
6. **Scenario 06 (Crossing Dynamic Agent)**: High-speed perpendicular dynamic agent crossing ego trajectory to test predictive collision avoidance.

## Execution
Run all 6 scenarios (50 trials each) in MATLAB:
```matlab
addpath('tests/dynamic_robustness');
run_all_dynamic_tests();
```
Results will be stored in `tests/dynamic_robustness/results/`.
