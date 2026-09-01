# Stage 5.1 — Validation Plan and Execution Record

## Scope and freeze boundary

Stage 5 supplies perception, interaction classification, state-based
kinematic prediction, and macro-intent coordination to frozen Stage 4. The
following files are immutable for this checkpoint:

- `planning/CACRCPlanner.m`
- `planning/SafetyFilter.m`
- `planning/QPMPCPlanner.m`

Their SHA-256 values at the end of this execution are respectively:

```text
0fec17c3b4ce82dfd8293e6673e98e0115292789a013b0b1276449e555726a09
87ceef3c896c35c051886c19fa058a57537d13d5e30afdf1d2500c13ba3e4045
efd9d9ec7bb03fbec1d7aa4e801598b60a1144cd3064b9840d7ad29591457506
```

## Acceptance criteria

For each 150-step scenario, require all of:

1. `collision_steps == 0`
2. `bounds_steps == 150`
3. `min_clr > 0.0`
4. `emergency_braking_count == 0` for nominal execution

The implementation now applies these criteria both in the direct scenario
runner and in `stage5_multivehicle_validation`.

## Stage 5.1 implementation additions

- Oncoming interactions are primary only while ahead and within a finite,
  six-second TTC window; the interaction releases after the agent passes.
- The decision layer now yields even when the oncoming conflict has no
  same-lane lead vehicle.
- Dynamic agents are converted once into state-based swept envelopes. Static
  obstacles are not duplicated, and the frozen planner and filter receive the
  same augmented world.
- The decision suite now has five deterministic checks, including an
  oncoming-only YIELD case.

## Required execution order

```matlab
addpath('tests','stages','planning','config','vehicle','core','environment');
test_stage5_perception();
test_stage5_decision();
stage5_multivehicle_coordination('scenario','multi_vehicle_following');
stage5_multivehicle_coordination('scenario','multi_vehicle_yield_overtake');
stage5_multivehicle_coordination('scenario','multi_vehicle_oncoming_conflict');
stage5_multivehicle_validation();
```

Then run the Stage 4 regression scenarios `passable_moderate`,
`passable_marginal`, `impassable_center`, and `multi_obstacle_sequence`.

## Execution result on 2026-08-29

MATLAB was run locally. The perception/risk suite passed 3/3 and the
decision suite passed 5/5. The initial three-scenario audit failed only the
closed-loop requirements: all scenarios were collision-free, but emergency
braking occurred in every scenario and the yield/overtake scenario achieved
only 118/150 bound-compliant steps. No Stage 5.1 pass is claimed.

The follow scenario is now explicitly FOLLOW-only, and the runner retains
planner failure reasons, Safety Filter reasons, and selected topologies so the
next test run isolates the remaining Stage 5-to-Stage 4 interface failure.
