# Stage 5.1 Walkthrough

Stage 5.1 is an interaction-aware wrapper over frozen Stage 4 planning:

```text
Perception and kinematic prediction
  -> interaction classification and macro intent
  -> frozen CA-CRC planner
  -> frozen Safety Filter
  -> vehicle dynamics
```

The scenario runner now uses the caller-provided simulation length (150 steps
by default) and enforces the full scientific gate: zero collision steps,
150/150 actual road-bound steps, positive minimum clearance, and no nominal
emergency braking. This prevents a partial road-bound score or an emergency
stop from being labeled a Stage 5.1 pass.

The controller also releases an oncoming threat after it passes, yields for an
oncoming-only conflict, and supplies the frozen planner and filter with the
same state-based swept envelopes for dynamic agents. Static obstacles are not
duplicated in those envelopes.

## Current result

The perception/risk tests passed 3/3 and decision tests passed 5/5. The
three-scenario audit is still a **FAIL**: all scenarios had zero collisions,
but emergency braking occurred (73, 22, and 98 steps respectively), and the
yield/overtake scenario was in bounds for only 118/150 steps. The Stage 5.1
acceptance gate remains unsatisfied.

The current code makes the following scenario a FOLLOW-only test and logs
planner failure reason, filter reason, and selected topology per timestep.
Re-run the validator to identify the remaining interface defect before making
any further changes.

The three frozen Stage 4 source files were hash-checked after the Stage 5-only
edits and remain unchanged.
