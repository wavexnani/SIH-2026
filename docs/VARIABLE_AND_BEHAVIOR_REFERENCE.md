# Variable & Behavior Reference Guide

**Purpose:** Quick lookup during implementation for variable meanings, types, ranges, and usage

---

## Table of Contents

1. [Global Configuration Variables](#global-configuration)
2. [Vehicle State Variables](#vehicle-state)
3. [Agent Variables](#agent-variables)
4. [World Model Variables](#world-model)
5. [Planning & Control Variables](#planning-control)
6. [Risk/Cost Variables](#risk-cost)
7. [Metric Variables](#metric-variables)
8. [Behavioral Specifications](#behavioral-specifications)

---

## Global Configuration

### Time & Simulation

| Variable | Type | Value | Unit | Notes |
|----------|------|-------|------|-------|
| `dt` | double | 0.1 | s | Simulation time step (100 ms) |
| `T_horizon` | double | 10 | s | Total simulation duration |
| `T_replan` | double | 0.5 | s | Replanning interval |
| `frames_per_step` | int | 5 | - | = `T_replan / dt` |

### Road Parameters

| Variable | Type | Value/Range | Unit | Notes |
|----------|------|-------------|------|-------|
| `road.width` | double | 6.0 | m | Road width (3m each side of center) |
| `road.length` | double | 100.0 | m | Road length for scenario |
| `road.boundary_type` | string | "unclear" | - | "clear" \| "unclear" \| "mixed" |
| `road.center_x` | double | 0.0 | m | Road center line x-coordinate |
| `road.marking_type` | string | "none" | - | "full" \| "dashed" \| "none" |
| `road.left_bound` | double[N×2] | - | m | Left boundary coordinates [x, y] |
| `road.right_bound` | double[N×2] | - | m | Right boundary coordinates [x, y] |

### Vehicle Parameters

| Variable | Type | Value | Unit | Notes |
|----------|------|-------|------|-------|
| `vehicle.L` | double | 4.7 | m | Vehicle length |
| `vehicle.W` | double | 1.8 | m | Vehicle width |
| `vehicle.wheelbase` | double | 2.7 | m | Distance between front/rear axles |
| `vehicle.max_speed` | double | 20.0 | m/s | Maximum velocity (~72 km/h) |
| `vehicle.min_speed` | double | 0.0 | m/s | Minimum velocity (stopped) |
| `vehicle.max_accel` | double | 3.0 | m/s² | Maximum acceleration |
| `vehicle.max_decel` | double | -6.0 | m/s² | Maximum deceleration (braking) |
| `vehicle.max_steering_angle` | double | 35° (rad) | rad | Maximum steering angle |
| `vehicle.max_steering_rate` | double | 60° (rad/s) | rad/s | Maximum steering rate |

---

## Vehicle State

### Kinematics (Position & Heading)

| Variable | Type | Range | Unit | Notes |
|----------|------|-------|------|-------|
| `ego.x` | double | [0, 100] | m | Global X position |
| `ego.y` | double | [-3, 3] | m | Global Y position (road-relative) |
| `ego.theta` | double | [-π, π] | rad | Heading angle |
| `ego.v` | double | [0, 20] | m/s | Velocity magnitude |
| `ego.omega` | double | [-2, 2] | rad/s | Yaw rate |

### Control Inputs

| Variable | Type | Range | Unit | Notes |
|----------|------|-------|------|-------|
| `ego.delta` | double | [-0.61, 0.61] | rad | Steering angle (±35°) |
| `ego.delta_cmd` | double | [-0.61, 0.61] | rad | Commanded steering |
| `ego.a` | double | [-6, 3] | m/s² | Acceleration |
| `ego.a_cmd` | double | [-6, 3] | m/s² | Commanded acceleration |

### Trajectory Data

| Variable | Type | Dimension | Unit | Notes |
|----------|------|-----------|------|-------|
| `ego.reference_path` | double | [N×3] | - | [x, y, theta] reference trajectory |
| `ego.planned_traj` | double | [M×3] | - | [x, y, v] planned trajectory |
| `ego.trajectory_history` | struct | - | - | `.x`, `.y`, `.v`, `.t` arrays |

### Safety Status

| Variable | Type | Value | Unit | Notes |
|----------|------|-------|------|-------|
| `ego.is_collision` | logical | {true, false} | - | Collision flag |
| `ego.min_clearance` | double | [0, ∞) | m | Minimum distance to obstacles |
| `ego.is_emergency` | logical | {true, false} | - | Emergency braking active |

---

## Agent Variables

### Agent Identity

| Variable | Type | Value | Unit | Notes |
|----------|------|-------|------|-------|
| `agent.id` | int32 | [1, N] | - | Unique agent identifier |
| `agent.type` | string | See below | - | Agent classification |

**Agent Types:**
- `"car"` - Passenger vehicle
- `"pedestrian"` - Walking person
- `"auto"` - Auto-rickshaw (common in India)
- `"bicycle"` - Bicycle/two-wheeler
- `"animal"` - Cow, dog, etc.
- `"cart"` - Pushcart
- `"truck"` - Commercial vehicle
- `"pothole"` - Road anomaly (static)

### Agent State

| Variable | Type | Range | Unit | Notes |
|----------|------|-------|------|-------|
| `agent.x` | double | [0, 100] | m | Global X position |
| `agent.y` | double | [-5, 5] | m | Global Y position |
| `agent.theta` | double | [-π, π] | rad | Heading (direction facing) |
| `agent.v` | double | [0, 15] | m/s | Velocity magnitude |
| `agent.vx` | double | [-15, 15] | m/s | Longitudinal velocity |
| `agent.vy` | double | [-15, 15] | m/s | Lateral velocity |
| `agent.omega` | double | [-2, 2] | rad/s | Yaw rate |

### Agent Geometry

| Variable | Type | Value | Unit | Notes |
|----------|------|-------|------|-------|
| `agent.length` | double | Type-dependent | m | Bounding box length |
| `agent.width` | double | Type-dependent | m | Bounding box width |

**Typical Dimensions:**
- Car: 4.7m × 1.8m
- Auto-rickshaw: 3.5m × 1.5m
- Pedestrian: 0.5m × 0.4m
- Bicycle: 1.8m × 0.6m
- Pothole: 0.5m × 0.5m

### Agent Perception

| Variable | Type | Range | Unit | Notes |
|----------|------|-------|------|-------|
| `agent.confidence` | double | [0, 1] | - | Detection confidence (0=uncertain, 1=certain) |
| `agent.uncertainty_radius` | double | [0, 1] | m | Positional uncertainty (Gaussian σ) |

### Agent Behavior

| Variable | Type | Value | Unit | Notes |
|----------|------|-------|------|-------|
| `agent.behavior_pattern` | string | See below | - | Inferred behavior model |
| `agent.threat_level` | double | [0, 1] | - | Immediate threat assessment |

**Behavior Patterns:**
- `"lane_following"` - Staying in lane
- `"crossing"` - Moving across road
- `"stopped"` - Stationary or very slow
- `"irregular"` - Unpredictable movement
- `"merging"` - Entering main road

### Prediction Data

| Variable | Type | Dimension | Unit | Notes |
|----------|------|-----------|------|-------|
| `agent.pred_horizon` | double | - | s | Prediction time horizon (default: 5s) |
| `agent.pred_trajectories` | struct[] | - | - | Array of multimodal predictions |

**Prediction Structure:**
```matlab
agent.pred_trajectories(i).trajectory = [x1, y1, theta1; ...  % [N×3]
                                         x2, y2, theta2; ...
                                         ...]
agent.pred_trajectories(i).probability = 0.6  % P(future i)
agent.pred_trajectories(i).confidence = 0.8   % Prediction confidence
```

### Anomaly-Specific Variables (Pothole/Debris)

| Variable | Type | Range | Unit | Notes |
|----------|------|-------|------|-------|
| `anomaly.severity` | double | [0, 1] | - | 0=negligible, 1=severe damage |
| `anomaly.depth` | double | [0, 1] | m | Pothole depth below road surface |
| `anomaly.width_impact` | double | [0, vehicle.W] | m | How much road width is blocked |

---

## World Model

### Structure Overview

```matlab
world_model.ego_state           % EgoState object
world_model.dynamic_agents      % Agent[] - moving entities
world_model.static_obs          % Agent[] - parked/fixed
world_model.road_anomalies      % Agent[] - potholes, debris
world_model.drivable_space      % polyshape - valid driving area
world_model.road_left_bound     % double[N×2] - left edge
world_model.road_right_bound    % double[N×2] - right edge
world_model.occupancy_grid      % logical[M×N] - grid bitmap
world_model.timestamp           % double - current time
```

### Risk Maps (Stage 4+)

| Variable | Type | Dimension | Unit | Notes |
|----------|------|-----------|------|-------|
| `world_model.static_risk_map` | double | [M×N] | [0, 100] | Risk from static obstacles |
| `world_model.dynamic_risk_map` | double | [M×N] | [0, 100] | Risk from moving agents |
| `world_model.uncertainty_map` | double | [M×N] | [0, 100] | Risk from uncertainty |
| `world_model.anomaly_risk_map` | double | [M×N] | [0, 100] | Risk from road anomalies |
| `world_model.composite_risk_map` | double | [M×N] | [0, 100] | Combined risk field |

**Risk Map Ranges:**
- 0-20: Safe region
- 20-50: Caution required
- 50-80: High risk
- 80-100: Critical/collision imminent

---

## Planning & Control

### Controller Parameters

| Variable | Type | Value | Unit | Notes |
|----------|------|-------|------|-------|
| `controller.speed_kp` | double | 1.0 | - | Speed P gain |
| `controller.speed_ki` | double | 0.1 | - | Speed I gain |
| `controller.speed_kd` | double | 0.2 | - | Speed D gain |
| `controller.stanley_k` | double | 0.5 | - | Stanley method gain |
| `controller.stanley_kp` | double | 2.0 | - | Stanley heading gain |

### Planning Parameters

| Variable | Type | Value | Unit | Notes |
|----------|------|-------|------|-------|
| `planner.horizon_distance` | double | 20-30 | m | Look-ahead distance |
| `planner.horizon_time` | double | 5.0 | s | Look-ahead time |
| `planner.n_candidates` | int | 7-15 | - | Number of trajectories to generate |
| `planner.step_size` | double | 0.5 | m | Trajectory discretization |
| `planner.dt` | double | 0.1 | s | Time step for trajectory |

---

## Risk & Cost Variables

### CRC Components (Stage 4)

| Variable | Type | Range | Unit | Notes |
|----------|------|-------|------|-------|
| `crc.J_static` | double | [0, ∞) | cost | Static obstacle risk |
| `crc.J_dynamic` | double | [0, ∞) | cost | Moving agent risk |
| `crc.J_uncertainty` | double | [0, ∞) | cost | Prediction uncertainty risk |
| `crc.J_anomaly` | double | [0, ∞) | cost | Road anomaly risk |
| `crc.cost_feasibility` | double | [0, ∞) | cost | Vehicle feasibility cost |
| `crc.cost_comfort` | double | [0, ∞) | cost | Jerk/smoothness cost |
| `crc.cost_efficiency` | double | [0, ∞) | cost | Lateral deviation cost |
| `crc.J_crc` | double | [0, ∞) | cost | Total composite risk cost |

### CRC Weights

| Variable | Type | Default | Unit | Notes |
|----------|------|---------|------|-------|
| `crc.w_static` | double | 5.0 | - | Static risk weight |
| `crc.w_dynamic` | double | 8.0 | - | Dynamic risk weight |
| `crc.w_uncertainty` | double | 3.0 | - | Uncertainty weight |
| `crc.w_anomaly` | double | 6.0 | - | Anomaly weight |
| `crc.w_feasibility` | double | 2.0 | - | Feasibility weight |
| `crc.w_comfort` | double | 1.0 | - | Comfort weight |
| `crc.w_efficiency` | double | 0.5 | - | Efficiency weight |

### Context-Adaptive Multipliers

| Variable | Type | Range | Unit | Notes |
|----------|------|-------|------|-------|
| `crc.context_traffic_density` | double | [0, 1] | - | 0=empty, 1=congested |
| `crc.context_uncertainty` | double | [0, 1] | - | 0=certain, 1=highly uncertain |
| `crc.context_road_anomalies` | double | [0, 1] | - | 0=none, 1=many severe |

**Weight Adaptation:**
```
if traffic_density > 0.5:
    w_dynamic *= (1 + traffic_density)
if uncertainty > 0.3:
    w_uncertainty *= (1 + uncertainty)
if anomalies > 0.5:
    w_anomaly *= (1 + anomalies)
```

### Safety Filter

| Variable | Type | Value | Unit | Notes |
|----------|------|-------|------|-------|
| `safety_filter.min_clearance_threshold` | double | 0.5 | m | Minimum required distance |
| `safety_filter.max_decel_emergency` | double | -8.0 | m/s² | Max emergency braking |

---

## Metric Variables

### Safety Metrics

| Variable | Type | Calculation | Unit | Notes |
|----------|------|-------------|------|-------|
| `metrics.collision_rate` | double | sum(collisions)/total_steps | % | Percentage of collision steps |
| `metrics.min_clearance` | double | min(clearance_log) | m | Closest approach to obstacle |
| `metrics.collision_free_dist` | double | Distance before first collision | m | How far before hitting something |
| `metrics.num_collisions` | int | sum(collision_flags) | - | Total collision count |

### Efficiency Metrics

| Variable | Type | Calculation | Unit | Notes |
|----------|------|-------------|------|-------|
| `metrics.completion_rate` | double | (x_end - x_start)/(x_goal - x_start) | % | Scenario completion percentage |
| `metrics.avg_speed` | double | mean(v[v > 0.1]) | m/s | Average velocity |
| `metrics.distance_traveled` | double | sum(segment_distances) | m | Total path length |
| `metrics.time_to_goal` | double | t when x reaches goal | s | Time to complete scenario |

### Comfort Metrics

| Variable | Type | Calculation | Unit | Notes |
|----------|------|-------------|------|-------|
| `metrics.path_smoothness` | double | mean(\|Δheading\|) | rad | Average heading change rate |
| `metrics.avg_acceleration` | double | mean(\|Δv\|/dt) | m/s² | Average acceleration magnitude |
| `metrics.jerk` | double | mean(\|Δa\|/dt) | m/s³ | Rate of change of acceleration |

### Replanning Metrics

| Variable | Type | Calculation | Unit | Notes |
|----------|------|-------------|------|-------|
| `metrics.avg_replan_time` | double | mean(replan_times) | ms | Average computation time |
| `metrics.max_replan_time` | double | max(replan_times) | ms | Worst-case latency |
| `metrics.min_replan_time` | double | min(replan_times) | ms | Best-case latency |
| `metrics.replan_frequency` | double | num_replans/total_time | Hz | Replanning rate |

### Trajectory Quality Metrics

| Variable | Type | Calculation | Unit | Notes |
|----------|------|-------------|------|-------|
| `metrics.lateral_error_mean` | double | mean(distances to ref path) | m | Average lateral deviation |
| `metrics.lateral_error_max` | double | max(distances to ref path) | m | Maximum lateral deviation |
| `metrics.lateral_error_std` | double | std(distances to ref path) | m | Deviation variability |

---

## Behavioral Specifications

### Vehicle Control Behavior

```matlab
% Speed Control (PID)
v_error = v_desired - v_current
integral_error += v_error * dt
derivative_error = (v_error - prev_error) / dt
accel_cmd = kp*v_error + ki*integral_error + kd*derivative_error
accel_cmd = clamp(accel_cmd, max_decel, max_accel)

% Lateral Control (Stanley Method)
crosstrack_error = perpendicular distance to path
heading_error = θ_vehicle - θ_path
steering_cmd = heading_error + atan2(k*crosstrack_error, v+0.5)
steering_cmd = clamp(steering_cmd, -max_steering, max_steering)
```

### Planning Behavior

```matlab
% Candidate Generation
For each lateral offset in [-3, -2, -1, 0, 1, 2, 3] m:
    Generate trajectory offset from reference path
    Consider trajectory feasibility

% Trajectory Scoring
For each candidate:
    Calculate CRC = w_static*J_static + w_dynamic*J_dynamic + ...
    Store (candidate, score) pair

% Best Selection
[~, best_idx] = min(all_scores)
best_trajectory = candidates[best_idx]
If not safe: best_trajectory = emergency_braking()

% Execution
Follow best_trajectory with controller
Every 0.5s: replan with new world state
```

### Risk Calculation Behavior

```matlab
% Static Risk
For each static obstacle:
    dist = ||ego_pos - obs_pos||
    if dist < margin: risk += 1000 (collision)
    else: risk += exp(-(dist-margin)²/(2*decay²))

% Dynamic Risk (with prediction)
For each moving agent:
    % Current position risk
    dist = ||ego_pos - agent_pos||
    risk += exponential decay function
    
    % Predicted future risk
    For each predicted trajectory:
        For each time step:
            dist = ||ego_pos - pred_pos(t)||
            risk += probability * decay function

% Uncertainty Risk
For each agent:
    if confidence < 0.8:
        risk += (1 - confidence) * penalty
    risk += agent.uncertainty_radius * scale_factor

% Anomaly Risk
For each pothole/anomaly:
    dist = ||ego_pos - anomaly_pos||
    if dist < 2m:
        risk += (1 - dist/2) * severity * scale_factor
```

### Hard Safety Filter Behavior

```matlab
% Check Trajectory Safety
For each waypoint on trajectory:
    For each static obstacle:
        if distance < (vehicle_width + obs_width)/2 + min_clearance:
            return UNSAFE
    For each dynamic agent:
        if distance < (vehicle_width + agent_width)/2 + min_clearance:
            return UNSAFE
return SAFE

% If UNSAFE:
    Generate emergency braking trajectory
    v_new = v_current - max_decel_emergency * dt
    x_new = x_current + v_new * cos(θ) * dt
    Follow braking trajectory until safe
```

---

## Quick Reference Lookup

### "How do I access...?"

| Question | Answer |
|----------|--------|
| Current vehicle speed? | `world_model.ego_state.v` |
| Nearest obstacle distance? | `world_model.ego_state.min_clearance` |
| Agent position? | `world_model.dynamic_agents(i).{x, y}` |
| Agent prediction? | `world_model.dynamic_agents(i).pred_trajectories(j).trajectory` |
| Road boundaries? | `world_model.road_left_bound`, `world_model.road_right_bound` |
| Current risk at point (x,y)? | `world_model.composite_risk_map` at grid location |
| Replanning latency? | `log.replan_time(k)` or `metrics.avg_replan_time` |
| Collision detected? | `world_model.ego_state.is_collision` |
| Best trajectory score? | `min(scores)` |

### "What's the normal range for...?"

| Variable | Typical Range | Notes |
|----------|---------------|-------|
| Collision rate (good) | 0-5% | <10% is acceptable |
| Min clearance (good) | >1.0 m | <0.5 m is dangerous |
| Replan latency (good) | 30-150 ms | <200 ms required for real-time |
| Completion rate (good) | >90% | Full scenario completion |
| Smoothness (good) | <0.3 rad | Low heading changes = smooth |
| Avg speed (good) | 3-5 m/s | Matches scenario speeds |

---

**End of Variable & Behavior Reference Guide**
