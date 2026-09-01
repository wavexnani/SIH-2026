# CA-CRC Dynamic Robustness Benchmark Report (V3)

Evaluated across 6 dynamic scenarios with 50 Monte Carlo trials per scenario (300 total trials) against the frozen Stage-4 baseline.

## 1. Overall System Benchmark Suite (300 Trials)

| Scenario | Trials | Scenario Success Rate | Scenario Collision-Free Rate | Global Collision-Free Rate | Min Scenario Clearance | Planner Prevention % | SafetyFilter Intervention % |
|---|---:|---:|---:|---:|---:|---:|---:|
| Sudden Herd Entry | 50 | 100.0% | 100.0% | 100.0% | 0.46 m | 38.4% | 61.6% |
| Herd Clears Recovery | 50 | 100.0% | 100.0% | 100.0% | 0.27 m | 89.3% | 10.7% |
| Partial Gap Transition | 50 | 100.0% | 100.0% | 100.0% | 0.05 m | 34.0% | 66.0% |
| Opposite Gap Opens | 50 | 100.0% | 100.0% | 100.0% | 0.70 m | 34.3% | 65.7% |
| Side Switch Topology Stress | 50 | 100.0% | 100.0% | 100.0% | 0.11 m | 90.5% | 9.5% |
| Crossing Dynamic Agent | 50 | 100.0% | 100.0% | 100.0% | 0.70 m | 100.0% | 0.0% |

## 2. Scenario 02 (Herd Clears Recovery) Behavior & Response Breakdown

- **Total Monte Carlo Trials**: 50
- **Scenario Collision-Free Rate**: **100.0%** (50/50 trials)
- **Full-Stop Recovery Rate**: **50.0%** (25/50 trials)
- **Safe Dynamic-Yield Recovery Rate**: **50.0%** (25/50 trials)
- **Overall Safe Recovery Rate**: **100.0%** (50/50 trials)

### Scenario 02 Recovery Latency Breakdown (Separated Metrics)

#### Full-Stop Recovery Latency (Standstill v <= 0.05 m/s to Resume v >= 1.0 m/s):
- **Count**: 25 trials
- **Mean**: 0.15 s (0.1480 s)
- **Median**: 0.00 s
- **Min / Max**: 0.00 s / 1.10 s
- **Std**: 0.3537 s

#### Safe Dynamic-Yield Recovery Latency (Corridor Reopen to Cruising Speed v >= 4.0 m/s):
- **Count**: 25 trials
- **Mean**: 0.31 s (0.3120 s)
- **Median**: 0.00 s
- **Min / Max**: 0.00 s / 1.60 s
- **Std**: 0.5812 s

*Note: Full-stop latency measures time from corridor reopening until vehicle accelerates back to 1.0 m/s after complete standstill. Dynamic-yield latency measures time from corridor reopening until vehicle resumes cruising speed (>= 4.0 m/s) without ever reaching standstill.*

## 3. Final Scientific Conclusion

```text
CORE CONTROLLER STATUS:
PASS

BENCHMARK INFRASTRUCTURE STATUS:
PASS

SCENARIO 02 SAFETY:
PASS

SCENARIO 02 RECOVERY:
PASS

FULL-STOP RECOVERY:
25/50

DYNAMIC-YIELD RECOVERY:
25/50

OVERALL SCENARIO-02 SAFE RECOVERY:
50/50

CORE CONTROLLER CHANGES REQUIRED:
NONE

NEXT INVESTIGATION:
Proceed with publication figure suite generation and final safety validation on the verified V3 benchmark baseline.
```
