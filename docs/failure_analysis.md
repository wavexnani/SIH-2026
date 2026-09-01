# Phase 12D Failure Analysis & Characterization

## 1. Failure Mode Taxonomy
Experimental failures during Phase 12D were categorized according to five structural outcome modes:
1. `COLLISION`: Physical vehicle footprint intersection with static or dynamic obstacle.
2. `UNSAFE_FAILURE`: Violation of drivable free-space boundary limits.
3. `PLANNER_INFEASIBLE`: QP solver failure (> 5 steps) where safety filter took over.
4. `SAFE_STOP`: Controlled emergency braking to standstill before impassable boundary.
5. `DEGRADED_SAFE`: Safe path completion with elevated tracking RMSE ($y_{\text{RMSE}} > 0.50\text{m}$).

---

## 2. Identified Failure Modes & Root Cause Mapping

### Failure Mode A: Free-Space Corridor Squeeze (Level 5 & 8)
- **Symptom**: Clearance drops below zero during tight multi-vehicle overtaking/yield in narrow 6m corridor.
- **Root Cause**: Spatio-temporal corridor overlap when opposing vehicle speed exceeds $6\text{m/s}$ in a 6m wide road with 1.8m vehicle widths.
- **Classification**: Experimental structural boundary squeeze, not a planner software crash.

### Failure Mode B: Total Blockage Controlled Stop (Level 10)
- **Symptom**: Vehicle stops at $x \approx 65\text{m}$ before total road closure.
- **Root Cause**: CACRC safety layer detects non-drivable terminal free space and commands smooth deceleration to zero velocity ($v < 0.1\text{m/s}$).
- **Classification**: `SAFE_STOP` (Desired safe fallback behavior).
