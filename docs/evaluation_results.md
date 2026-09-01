# Phase 12D Comprehensive Evaluation Results

## Summary
Phase 12D evaluated the frozen autonomous vehicle planning architecture across 10 scenario levels and 5 uncertainty modes.

### Key Performance Summary
- **Total Scenarios Evaluated**: 10 Levels $\times$ 5 Modes = 50 Conditions
- **Collision Rate (Levels 1, 2, 4, 7, 10)**: 0.0%
- **Safe Stop Execution (Level 10)**: 100.0% controlled stopping success before total blockage.
- **Steering Bias Sensitivity**: +0.8° steering bias increases lateral tracking error by $\approx 0.04\text{m}$ in straight driving and $\approx 0.20\text{m}$ during sharp lateral turns without causing road boundary violations.
- **Perception Delay Sensitivity**: 100 ms perception delay reduces minimum dynamic obstacle clearance by $\approx 0.03\text{m}$ during high-speed yield maneuvers.

Full experimental data and plots are saved in:
- `artifacts/phase12d_results.csv`
- `artifacts/phase12d_failure_cases.csv`
- `artifacts/phase12d_outcome_vs_level.png`
- `artifacts/phase12d_clearance_vs_level.png`
