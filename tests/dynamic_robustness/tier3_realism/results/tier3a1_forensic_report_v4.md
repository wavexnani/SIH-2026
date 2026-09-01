# CA-CRC Tier 3A.1-F Verified Forensic Validation Report

## Executive Summary
This report provides a **mathematically verified, geometry-consistent forensic validation** of the non-monotonic headway response ($X_{\text{center}} \in [20, 25, 30, 40]\text{ m}$) under zero perception noise (\sigma_p = 0.00\text{ m}). All clearance calculations, footprints, and telemetry timestamps are generated directly from simulation object properties (`cfg`, `world.ego`, `agents`).

## Standardized Forensic Event Matrix (Ideal Perception \sigma_p = 0.00m, Seed 1000)

| X_center (m) | Instantiated Agents | Outcome | Window Reach x>=xc-5 | Center Reach x>=xc | SF Activation Time (t_sf_on) | Full Stop Time (t_stop) | Stop Position (x, y) | Corridor Diag Threshold (t_diag) | First Footprint Contact (t_contact) | Colliding Goat ID & Position | Initial Contact Clearance (m) | Peak Penetration Clearance (m) |
|---:|---:|:---:|---:|---:|---:|---:|:---:|---:|---:|:---:|---:|---:|
| 20 m | 60 | Collision-Free | 2.50 s | 3.50 s | N/A | N/A | N/A | 9.00 s | N/A | N/A | N/A | 0.84 m |
| 25 m | 60 | Collision-Free | 3.50 s | 4.60 s | 14.00 s | N/A | N/A | 8.50 s | N/A | N/A | N/A | 0.76 m |
| 30 m | 60 | **FOOTPRINT CONTACT** | 4.60 s | 5.70 s | 4.80 s | 6.70 s | (31.32m, 2.80m) | 8.00 s | 8.60 s | Goat #9 (31.93m, 1.65m) | -0.00 m | -1.15 m |
| 40 m | 60 | Collision-Free | N/A (Stopped Upstream) | N/A (Stopped Upstream) | 5.20 s | 7.10 s | (33.36m, 2.50m) | 7.20 s | N/A | N/A | N/A | 1.44 m |

## Deep-Dive Causal Event Chain for X_center = 30m Failure

### 1. Validated Multi-Stage Causal Chain
```text
  t = 0.00s     Ego starts at X = 3.0m, Y = 2.5m, v = 5.0m/s
      │
  t = 4.60s     Ego enters diagnostic window (X ≥ 25.0m)
      │
  t = 4.80s     SafetyFilter Activates (t_sf_on) — max safe deceleration (a = -2.02 m/s²)
      │
  t = 6.70s     Full Stop Reached (t_stop) — COMPLETE STANDSTILL (v = 0.00 m/s) at X = 31.32m, Y = 2.80m
      │
  t = 8.00s     Corridor Diagnostic Threshold (t_diag) — W_corr < 1.60m around mean agent position
      │
  t = 8.60s     First Footprint Boundary Contact (t_contact) — Goat #9 touches lateral boundary (C = -0.00m)
      │         (1.90s AFTER ego vehicle has reached complete standstill!)
      │
  t = 22.00s    Peak Trial Penetration (t_peak) — Goat #9 traverses to vehicle center (C = -1.15m)
```

### 2. Footprint Geometry Reconciliation
- **Ego Vehicle Dimensions**: Length $L = 4.70\text{ m}$, Width $W = 1.80\text{ m}$
- **Ego Footprint at Standstill ($v = 0.00\text{ m/s}$)**:
  - Center: $(X = 31.32\text{ m}, Y = 2.80\text{ m})$
  - Longitudinal bounds: $[X_{\text{rear}}, X_{\text{front}}] = [28.97\text{ m}, 33.67\text{ m}]$
  - Lateral bounds: $[Y_{\text{right}}, Y_{\text{left}}] = [1.90\text{ m}, 3.70\text{ m}]$
- **Goat #9 State at First Footprint Contact ($t = 9.00\text{ s}$)**:
- **Goat #9 State at First Footprint Contact ($t = 8.60\text{ s}$)**:
  - Center: $(X_{\text{goat}}, Y_{\text{goat}}) = (31.93\text{ m}, 1.65\text{ m})$
  - Longitudinal Penetration: $dx = |31.32 - 31.93| - (4.70 + 0.8)/2 = -2.14\text{ m}$ (**FULL LONGITUDINAL OVERLAP**)
  - Lateral Overlap: $dy = |2.80 - 1.65| - (1.80 + 0.5)/2 = -0.00\text{ m}$ (**EXACT BOUNDARY TOUCH**)
  - Initial Contact Signed Clearance: $C(t=8.60\text{ s}) = \max(dx, dy) = \max(-2.14, -0.00) = \mathbf{-0.00\text{ m}}$
- **Finite Penetration Phase ($t > 8.60\text{ s}$)**:
  - Goat #9 continues moving laterally, reaching peak footprint penetration ($C_{\text{peak}} = \mathbf{-1.15\text{ m}}$) at $t = 22.00\text{ s}$.

### 3. Root Cause Classification: **Stationary-Ego Vulnerability to Continued Dynamic-Agent Intrusion**
The validated telemetry proves that:
1. The SafetyFilter brought the ego vehicle to a complete standstill ($v = 0.00\text{ m/s}$) at $t = 6.70\text{ s}$, successfully arresting forward motion before colliding with the herd ahead.
2. First footprint boundary contact occurred **1.90 seconds AFTER the vehicle was fully stopped**, when Goat #9 touched the lateral footprint boundary ($C = -0.00\text{ m}$).
3. **Key Scientific Conclusion**: Forward-motion arrest is necessary but not sufficient to guarantee safety when dynamic agents continue moving across the stationary vehicle footprint.

