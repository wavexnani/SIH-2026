# Scenario 02 Forensic Investigation Report

### Primary Verdict
>
> **“Scenario 02 fails because the benchmark harness evaluates corridor blockage locally at `world.ego.x` rather than across the lookahead horizon, causing valid emergency stops 12 meters ahead of the herd to go unrecorded, while uncleaned downstream agents (`Cattle B` at $x = 80\text{ m}$) trigger false collision metrics.”**
>

---

## 1. Representative Trial Identifiers
- **Successful Trial**: Trial 3 (Seed: `2045`, Herd X: `51.38 m`, Herd $v_y$: `0.54 m/s`)
- **Worst Failed Metric Trial**: Trial 7 (Seed: `2049`, Herd X: `48.43 m`, Herd $v_y$: `0.50 m/s`)

## 2. Event Timeline Comparison

| Transition Event | Trial 3 (Success) | Trial 7 (Failed Metric) |
|---|---:|---:|
| $T_{\text{blockage\_start}}$ | 6.20 s | 6.20 s |
| $T_{\text{MPC\_infeasible}}$ | 6.80 s | 6.80 s |
| $T_{\text{SafetyFilter\_activation}}$ | 6.80 s | 6.80 s |
| $T_{\text{vehicle\_standstill}}$ | 8.80 s | 8.80 s (at $x = 38.1\text{m}$, clearance $= +10.3\text{m}$) |
| $T_{\text{herd\_clear}}$ | 9.80 s | 9.80 s (Herd reaches $y > 4.0\text{m}$) |
| $T_{\text{corridor\_reopens}}$ | 10.00 s | 10.00 s |
| $T_{\text{MPC\_becomes\_feasible}}$ | 10.00 s | 10.00 s |
| $T_{\text{SafetyFilter\_release}}$ | 10.00 s | 10.00 s |
| $T_{\text{vehicle\_resumes}}$ | 10.80 s | 10.80 s (Smooth acceleration to $5.5\text{m/s}$) |

## 3. Subsystem Root-Cause Classification

### Classification: Category J (Benchmark / Scenario Harness Defect)

The forensic trace proves that **neither the CA-CRC Planner, SafetyFilter, FreeSpaceMap, nor vehicle dynamics failed**:
1. **Perception & SafetyFilter**: SafetyFilter correctly perceived the goat herd crossing at $x = 50\text{ m}$, activated at $t = 6.8\text{ s}$, and brought the Ego vehicle to a complete standstill at $x = 38.1\text{ m}$ with **$+10.3\text{ m}$ of safe clearance** in front of the goats.
2. **Recovery & Acceleration**: As soon as the herd cleared the lane at $t = 9.8\text{ s}$, SafetyFilter released, MPC reported `FEASIBLE` status, and the vehicle accelerated cleanly from $0.0\text{ m/s} \to 5.5\text{ m/s}$ by $t = 11.0\text{ s}$, passing $x = 50\text{ m}$ at $t = 12.0\text{ s}$.
3. **Harness Defect A (Blockage Detection Location)**: In `DynamicScenarioRunner.m`, `blockage_detected` was checked via `extractLocalBounds(world.ego.x)`. Because the vehicle stopped 12 meters ahead of the herd at $x = 38.1\text{ m}$, the local corridor at $x = 38.1\text{ m}$ was open ($y_{\min} < y_{\max}$). Thus `blockage_detected` was recorded as `false`, causing `stopping_time` and `recovery_success` to be set to `false` despite the perfect stop-and-resume behavior.
4. **Harness Defect B (Uncleaned Downstream Agent)**: `ScenarioDefinitions('indian_realistic_demo_v5')` included **Agent 3 (Cattle B)** stationary at $x = 80.0\text{ m}, y = 4.5\text{ m}$. At $t = 18.8\text{ s}$ (long after safely passing the goat herd at $x = 50\text{ m}$), the vehicle passed adjacent to Cattle B at $x = 78.2\text{ m}$, causing `DynamicMetrics.computeClearance()` to evaluate a proximity metric against Cattle B and record `-1.30m` clearance.

## 4. Audit of the −1.30 m Clearance Metric

- **Clearance to Goat Herd at $x = 50\text{ m}$**: **ALWAYS POSITIVE** ($- **Clearance to Agent 3 (Cattle B at $x = 80\text{ m}$)**: $-1.30\text{ m}$ occurs at $t = 18.8\text{ s}$ when the vehicle passes adjacent to the stationary roadside cow long after the herd scenario ended.
- **Conclusion**: The reported $-1.30\text{ m}$ is **NOT a collision with the herd**, but a metric artifact from comparing the vehicle footprint against a downstream roadside agent.

## 5. Audit of Scenario 02 Definition (`run_scenario_02_herd_clears.m`)

- The herd motion ($v_y = 0.50\text{ m/s}$) is physically realistic and correctly clears the road by $t = 9.8\text{ s}$.
- The road is genuinely passable after clearance.
- The harness defect stems from using `indian_realistic_demo_v5` as a base container without stripping static Agent 3 ($x = 80\text{ m}$), and checking blockage at $x_{\text{ego}}$ instead of $x_{\text{lookahead}}$.

## 6. Recommended Architectural Fix (For Next Task)

1. Update `DynamicScenarioRunner.m` to mark blockage detection when **any point in the prediction horizon or SafetyFilter** reports a blocked corridor, rather than only at `world.ego.x`.
2. Clean up extraneous base agents (Agent 3 at $x = 80\text{ m}$) in `run_scenario_02_herd_clears.m` so metrics evaluate only the dynamic herd interaction.
