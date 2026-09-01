# Simulation & Visualization Architecture Audit (Phase 1)

## 1. Current Simulation Architecture
- **Engine**: Closed-loop discrete-time simulation driven by `BicycleModel` (kinematic bicycle model) with step size $dt=0.1\text{ s}$.
- **State Model**: `WorldState.m` tracks ego vehicle $(x, y, \theta, v)$, dynamic agents array, static obstacle matrix, and road bounds.
- **Controller/Planner Layer**: `Stage5CoordinationController` integrates `CACRCPlanner` (topology selection + QP formulation), `QPMPCPlanner` (Hildreth dual active-set solver), and `SafetyFilter` (Layer 2 emergency supervisor).
- **Bound Providers**: Abstracted via `AbstractBoundProvider`, with `CorridorBoundProvider` (fixed $[0, 6.0\text{ m}]$ corridor) and `FreeSpaceBoundProvider` (2D `FreeSpaceMap` query).

## 2. Current Visualization Capabilities
- **Status**: Limited to offline static PNG generation (`scripts/generate_stage51_figures.m`) and basic line plots.
- **Ego Vehicle**: Rendered only as a 1D center point trajectory line `plot(x, y)`. Footprint orientation and corner projections are not shown.
- **Obstacles & Agents**: Rendered as static 2D rectangles without orientation vectors or trajectory previews.
- **Road Boundaries**: Rendered as fixed horizontal dashed lines (`yline(0.0)`, `yline(4.8)`). Variable `FreeSpaceMap` boundaries are **not** currently rendered.
- **Prediction Preview**: MPC prediction horizon $X_{pred}$ and candidate topology trajectories are not displayed dynamically.

## 3. Existing Scenarios
- **Stage 4 Baseline**: `passable_moderate`, `passable_marginal`, `impassable_center`, `multi_obstacle_sequence`.
- **Stage 5 Baseline**: `multi_vehicle_following`, `multi_vehicle_yield_overtake`, `multi_vehicle_oncoming_conflict`.
- **Stage 5.1 Free-Space**: `unstructured_road`, `cattle_crossing`.

## 4. Missing Realism & Visual Features
- **Ego Vehicle Footprint**: Oriented Bounding Box (OBB) rectangle rendering showing true vehicle dimensions ($L=4.7\text{ m}, W=1.8\text{ m}$), heading angle $\theta$, and velocity vector.
- **True 2D Map Rendering**: Rendering of continuous variable road boundaries $[y_{min}(x), y_{max}(x)]$, narrowing sections, curves, and drivable free space polygon overlays from `FreeSpaceMap`.
- **Dynamic Agents & Obstacles**: Rendered with vehicle bodies, headings, and velocity vectors.
- **Live Visualizer Tool**: Real-time graphics renderer displaying ego footprint, prediction horizon, active topology (`hard_left`, `soft_left`, etc.), and solver status overlay.
- **Scenario Sweep Generator**: Structured scenario generator covering Level 1 (Nominal) to Level 4 (Infeasible) difficulty tiers.
- **Automated Failure Categorizer**: Diagnostic classification for solver infeasibility vs. physical impossibility vs. safety filter overrides.

## 5. File Modification & Freeze Matrix

| Subsystem / File | Classification | Action |
|---|---|---|
| `planning/CACRCPlanner.m` | **FROZEN** | Do NOT modify mathematical formulation or parameters. |
| `planning/QPMPCPlanner.m` | **FROZEN** | Do NOT modify Hildreth solver or KKT residual gates. |
| `planning/SafetyFilter.m` | **FROZEN** | Do NOT modify safety threshold or emergency logic. |
| `planning/Stage5CoordinationController.m` | **FROZEN** | Do NOT modify intent state machine or reference targets. |
| `planning/MultiVehicleDetector.m` | **FROZEN** | Do NOT modify lane classification thresholds. |
| `visualization/SceneVisualizer.m` | **NEW** | Build modular 2D scene renderer for ego footprint, bounds, and horizon. |
| `environment/ScenarioGenerator.m` | **NEW** | Build multi-tier scenario generator (Level 1–4 difficulty). |
| `tests/run_controlled_ab_experiment.m` | **VERIFIED** | Preserve as baseline controlled ablation script. |
| `tests/run_monte_carlo_sweep.m` | **NEW** | Build repeatable Monte Carlo evaluation and failure categorizer. |
| `tests/run_demonstration_suite.m` | **NEW** | Build demonstration output script for Demos 1–6. |
