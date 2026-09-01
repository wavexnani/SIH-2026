# Stage 5.1 — Multi-Vehicle Validation Status

## Objective

Validate the Stage 5 interaction layer above the frozen Stage 4 CA-CRC
planner and Safety Filter. Required acceptance criteria are zero collisions,
150/150 actual road-bound steps, strictly positive minimum clearance, and no
Layer 2 emergency-braking step during nominal execution.

## Work completed

- Confirmed the Stage 4 freeze files were not edited:
  `planning/CACRCPlanner.m`, `planning/SafetyFilter.m`, and
  `planning/QPMPCPlanner.m`.
- Corrected the Stage 5 scenario runner so `max_steps` is honored (default
  150 rather than an unconditional internal overwrite).
- Corrected its direct acceptance gate to require zero collisions, 150/150
  bounds, positive clearance, and zero emergency-braking steps.
- Made the validation harness explicitly request 150 steps per scenario.
- Fixed oncoming-threat clearance after the agent passes, avoiding a permanent
  YIELD state.
- Added an oncoming-only YIELD unit case and strengthened Stage 5 dynamic-agent
  envelopes so both frozen Stage 4 components evaluate the same prediction.

## Validation status — not passed

On 2026-08-29, the following commands were attempted:

- `test_stage5_perception()`
- `test_stage5_decision()`
- `stage5_multivehicle_validation()`
- Stage 4 regression scenarios: `passable_moderate`, `passable_marginal`,
  `impassable_center`, and `multi_obstacle_sequence`

The unit tests have now passed locally: perception/risk **3/3** and
decision/intent **5/5**. The initial closed-loop validation remains a fail:

- `multi_vehicle_following`: 0 collisions, 150/150 bounds, 73 emergency steps.
- `multi_vehicle_yield_overtake`: 0 collisions, 118/150 bounds, 22 emergency steps.
- `multi_vehicle_oncoming_conflict`: 0 collisions, 150/150 bounds, 98 emergency steps.

Stage 5.1 is **not accepted**. The runner now records failure reasons and
selected topologies to make the next measurement diagnostic.

## Next action

Run `run_stage51_validation()` in a MATLAB-capable local terminal and provide
the updated scenario summaries, including the new failure-reason lines.
