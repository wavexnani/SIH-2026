# CA-CRC Dynamic Robustness Benchmark Report

Evaluated across 6 distinct dynamic scenarios with 50 Monte Carlo trials per scenario (300 total trials) against the frozen Stage-4 baseline.

| Scenario | Trials | Success Rate | Scenario-Collision-Free | Global-Collision-Free | Recovery Rate | Min Scenario Clearance | Mean Recovery Latency |
|---|---:|---:|---:|---:|---:|---:|---:|
| Sudden Herd Entry | 50 | 100.0% | 100.0% | 100.0% | 0.0% | 0.46 m | -- |
| Herd Clears Recovery | 50 | 50.0% | 100.0% | 100.0% | 50.0% | 0.27 m | 0.15 s |
| Partial Gap Transition | 50 | 100.0% | 100.0% | 100.0% | 0.0% | 0.05 m | -- |
| Opposite Gap Opens | 50 | 100.0% | 100.0% | 100.0% | 0.0% | 0.70 m | -- |
| Side Switch Topology Stress | 50 | 100.0% | 100.0% | 100.0% | 0.0% | 0.11 m | -- |
| Crossing Dynamic Agent | 50 | 100.0% | 100.0% | 100.0% | 0.0% | 0.70 m | -- |


*Note: Scenario-Collision-Free evaluates zero footprint collision against scenario-defined obstacles. Global-Collision-Free evaluates zero footprint collision against all entities in the environment.*
