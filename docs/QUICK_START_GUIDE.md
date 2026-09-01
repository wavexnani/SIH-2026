# Quick Start Guide - SIH Internal Hackathon Stages 0-5

## Overview

This guide explains how to use the implementation plan and start coding the SIH solution.

**Total Implementation Time:** 4-6 weeks for Stages 0-5
**Target Date:** September 1, 2026

---

## File Structure (After Implementation)

```
sih_new_2026/
├── README.md
├── DEEP_IMPLEMENTATION_PLAN_STAGES_0-5.md    [Reference]
├── QUICK_START_GUIDE.md                      [This file]
├── problem_statment.md                       [SIH requirements]
│
├── main_sih_simulation.m                     [Entry point - run all stages]
│
├── config/
│   └── SimulationConfig.m                    [Global parameters]
│
├── core/
│   ├── AgentClass.m                          [Agent data structure]
│   ├── EgoState.m                            [Vehicle state structure]
│   └── WorldModel.m                          [World representation]
│
├── vehicle/
│   ├── BicycleModel.m                        [Kinematic/dynamic model]
│   └── VehicleController.m                   [PID + Stanley controller]
│
├── environment/
│   ├── DrivableSpace.m                       [Road representation]
│   └── Scenario.m                            [Scenario definitions]
│
├── planning/
│   ├── stage2/
│   │   └── BaselinePlannerSimple.m           [Stage 2: Simple planner]
│   ├── stage3/
│   │   └── QPMPC_Planner.m                   [Stage 3: QP-MPC baseline]
│   └── stage4/
│       ├── CompositeRiskCost.m               [CA-CRC: Risk calculator]
│       ├── HardSafetyFilter.m                [CA-CRC: Safety filter]
│       └── CARCPlanner.m                     [Stage 4: Full planner]
│
├── metrics/
│   ├── PerformanceMetrics.m                  [Metric calculations]
│   └── stage5_metrics_comparison.m           [Stage 5: Comparison]
│
├── stages/
│   ├── stage0_roadrunner_setup.m             [Simulation setup]
│   ├── stage1_closed_loop_vehicle.m          [Vehicle control validation]
│   ├── stage2_baseline_planning.m            [Baseline planner test]
│   ├── stage3_qpmpc_baseline.m               [MPC baseline test]
│   ├── stage4_carc_planner.m                 [CA-CRC planner test]
│   └── stage5_metrics_comparison.m           [Comprehensive metrics]
│
└── results/
    ├── sih_results_YYYY-MM-DD_HH-mm-ss.mat   [Saved workspace]
    ├── sih_metrics_YYYY-MM-DD_HH-mm-ss.csv   [Exported metrics]
    └── figures/                              [Plots and visualizations]
```

---

## Key Implementation Decisions

### 1. Ground Truth → Perception (Phased)
- **Stages 0-4:** Use perfect world knowledge from scenario definition
- **Stage 2 (Future):** Replace with simulated camera/LiDAR
- **Stage 3 (Future):** Replace with ML detection

### 2. Constant Velocity → ML Prediction (Phased)
- **Stages 0-4:** Use simple parametric prediction
- **Stage 2 (Future):** Replace with multimodal ML prediction

### 3. Candidate Scoring (Computational)
- Generate 7-15 candidate trajectories
- Score each independently (parallelizable)
- Select best = fast replanning

### 4. Hard Safety Filter (Critical)
- Soft cost (CA-CRC) vs Hard constraint (Safety filter)
- Separation of concerns: preference vs feasibility
- Emergency braking fallback

---

## Implementation Roadmap

### Week 1-2: Foundations (Stages 0-1)

**Objectives:**
- ✅ RoadRunner scenario setup or MATLAB fallback
- ✅ World model data structures
- ✅ Vehicle dynamics (bicycle model)
- ✅ Closed-loop control (PID + Stanley)

**Deliverable:** Vehicle tracks reference path safely

**Code:**
```matlab
% Test Stage 0 & 1
main_sih_simulation('stage', '1', 'verbose', true)
```

### Week 3-4: Baselines (Stages 2-3)

**Objectives:**
- ✅ Simple planner (7 candidates, basic scoring)
- ✅ QP-MPC formulation and solver integration

**Deliverable:** Both baselines complete, metrics collected

**Code:**
```matlab
% Test both baselines
main_sih_simulation('stage', '2', 'verbose', true)
main_sih_simulation('stage', '3', 'verbose', true)
```

### Week 5: Proposed Method (Stage 4)

**Objectives:**
- ✅ Composite Risk Cost (7 components)
- ✅ Context-adaptive weighting
- ✅ Hard safety filter

**Deliverable:** CA-CRC planner working, safety filter validated

**Code:**
```matlab
% Test Stage 4
main_sih_simulation('stage', '4', 'verbose', true)
```

### Week 6: Evaluation (Stage 5)

**Objectives:**
- ✅ Comprehensive metrics (17+ metrics)
- ✅ Comparative analysis
- ✅ Visualizations and CSV export
- ✅ Statistical significance

**Deliverable:** Complete comparative report with visualizations

**Code:**
```matlab
% Run full comparison
main_sih_simulation('stage', 'all', 'verbose', true, ...
                    'save_results', true, 'export_csv', true)
```

---

## Variable Naming Conventions

### Geometry
- `x, y` - Position in global frame (meters)
- `theta` - Heading (radians, -π to π)
- `L, W` - Length, width (meters)
- `x_corners, y_corners` - Bounding box corners

### Dynamics
- `v` - Velocity magnitude (m/s)
- `a` - Acceleration (m/s²)
- `delta` - Steering angle (rad)
- `omega` - Yaw rate (rad/s)

### Agents
- `type` - 'car', 'pedestrian', 'auto', 'animal', 'bicycle', 'pothole'
- `confidence` - Detection confidence [0-1]
- `severity` - Anomaly severity [0-1]
- `pred_trajectories` - Multimodal futures {trajectory, probability}

### Costs/Risks
- `J_crc` - Composite Risk Cost (total)
- `J_static` - Static obstacle risk
- `J_dynamic` - Dynamic agent risk
- `J_uncertainty` - Uncertainty risk
- `J_anomaly` - Road anomaly risk
- `cost_feasibility` - Vehicle feasibility cost
- `cost_comfort` - Jerk/smoothness cost
- `cost_efficiency` - Lateral deviation cost

### Metrics
- `collision_rate` - Percentage of time steps with collision
- `min_clearance` - Closest approach distance (m)
- `completion_rate` - Percentage of scenario completed
- `avg_speed` - Average velocity (m/s)
- `smoothness` - Average heading change (rad)
- `replan_time` - Computation latency (seconds)

---

## How to Debug Each Stage

### Stage 0 Issues
```matlab
% Check if world model is populated
disp(world_model.ego)
disp(world_model.agents)
disp(world_model.static)

% Check RoadRunner connectivity
rr_apps = driving.roadrunner.RoadRunnerApp.getOpenInstances()
```

### Stage 1 Issues
```matlab
% Plot trajectory
figure;
plot(log.x, log.y); grid;
title('Vehicle Path');
xlabel('X (m)'); ylabel('Y (m)');

% Check if vehicle is following reference path
hold on;
plot(reference_path(:,1), reference_path(:,2), 'r--', 'LineWidth', 2);
legend('Actual', 'Reference');
```

### Stage 2-4 Issues
```matlab
% Check candidate trajectory quality
figure;
for i = 1:length(candidates)
    plot(candidates{i}(:,1), candidates{i}(:,2));
    hold on;
end
legend('Candidates');
title('Generated Trajectories');

% Check scoring
disp(scores);
[~, best_idx] = min(scores);
fprintf('Best candidate: %d with score %.3f\n', best_idx, scores(best_idx));
```

### Stage 5 Issues
```matlab
% Export logs for analysis
metrics = PerformanceMetrics.collision_rate(log.collision);
fprintf('Collision rate: %.2f%%\n', metrics * 100);

% Check individual metric calculation
lat_error = PerformanceMetrics.lateral_error_from_path(...
    log.x, log.y, reference_path);
fprintf('Lateral error: mean=%.2f, max=%.2f\n', ...
    lat_error.mean, lat_error.max);
```

---

## Performance Expectations

### Stage 1 (Closed-Loop)
- Lateral error < 1.0 m
- No collisions
- Replan latency: N/A (open-loop)

### Stage 2 (Simple Baseline)
- Collision rate: 5-15%
- Min clearance: 0.5-2.0 m
- Completion: 80-95%
- Replan latency: 10-50 ms

### Stage 3 (QP-MPC)
- Collision rate: 2-10%
- Min clearance: 1.0-2.5 m
- Completion: 85-98%
- Replan latency: 50-150 ms

### Stage 4 (CA-CRC) - Expected Improvement
- Collision rate: 0-5% (↓ by 50-75%)
- Min clearance: 1.5-3.0 m (↑ by 20-30%)
- Completion: 90-99% (≈ same)
- Replan latency: 30-120 ms (≈ same or faster)
- Safety filter activations: 5-10

---

## Testing Checklist

### Unit Tests
- [ ] Agent bounding box calculation
- [ ] Collision detection (AABB, polygon)
- [ ] Drivable space membership testing
- [ ] Risk map calculations

### Integration Tests
- [ ] World model update pipeline
- [ ] Prediction generation
- [ ] Trajectory scoring
- [ ] Controller command propagation

### System Tests
- [ ] Full Stage 0-1 pipeline
- [ ] Full Stage 2-3 pipeline
- [ ] Full Stage 4 pipeline
- [ ] Comparative Stage 5 analysis

### Safety Tests
- [ ] Hard safety filter enforcement
- [ ] Emergency braking activation
- [ ] Collision detection boundary cases
- [ ] Numerical stability (NaN/Inf checks)

---

## Common Pitfalls to Avoid

❌ **Don't:**
1. Use lane-only planning (road is unstructured)
2. Assume perfect perception (Stage 2+ add uncertainty)
3. Ignore vehicle dynamics (use bicycle model)
4. Replan too frequently (0.5 s is good)
5. Create a single monolithic planner (separate concerns)

✅ **Do:**
1. Use drivable space representation
2. Plan trajectories (not just paths)
3. Implement hard safety layer
4. Log all metrics carefully
5. Validate on all five SIH scenarios (eventually)

---

## Getting Help

### For RoadRunner Issues
- Check MathWorks documentation: `doc roadrunnerScenario`
- Verify RoadRunner version compatibility
- Use MATLAB-only fallback if issues arise

### For MATLAB Issues
- Enable "Use Parallel Computing" for candidate scoring
- Monitor memory usage (world model can grow large)
- Use profiler to identify bottlenecks: `profile viewer`

### For Algorithm Issues
- Start with very simple scenarios (1 agent, 1 obstacle)
- Gradually increase complexity
- Visualize trajectories and risk maps
- Check intermediate computations

---

## Milestones for September 1

### Must-Have (Minimum)
- ✅ Stage 0-4 implemented and running
- ✅ One complete scenario tested
- ✅ Baseline vs CA-CRC comparison metrics
- ✅ Technical report draft

### Nice-to-Have (If time permits)
- ✅ Two RoadRunner scenarios
- ✅ ML-based prediction integration
- ✅ Demo video
- ✅ Ablation study (Stage 5)

### For Final SIH Submission (After Sept 1)
- ✅ All five SIH scenarios
- ✅ Real perception integration
- ✅ Multimodal prediction
- ✅ Complete technical documentation

---

## Next Steps

1. **Read and understand** the `DEEP_IMPLEMENTATION_PLAN_STAGES_0-5.md` file
2. **Verify RoadRunner** installation or plan MATLAB fallback
3. **Start with Stage 0:** Implement `SimulationConfig.m`, `AgentClass.m`, `WorldModel.m`
4. **Implement Stage 1:** Add `BicycleModel.m`, `VehicleController.m`
5. **Test early and often:** Run `main_sih_simulation('stage', '1')` after each module

---

**Good luck! The team is ready to build something great.** 🚗✨
