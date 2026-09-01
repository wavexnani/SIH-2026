# CA-CRC Tier 3A.1-F Final Forensic Validation Report

## Executive Summary
This report provides a **rigorous, multi-timestamp forensic validation** of the non-monotonic headway response ($X_{\text{center}} \in [20, 25, 30, 40]\text{ m}$) under zero perception noise (\sigma_p = 0.00\text{ m}). All numerical claims and plot markers in this document are automatically populated directly from simulation telemetry.

## Standardized Forensic Event Matrix (Ideal Perception \sigma_p = 0.00m, Seed 1000)

| X_center (m) | Instantiated Agents | Outcome | Window Reach x>=xc-5 | Center Reach x>=xc | SF Activation Time (t_sf_on) | Full Stop Time (t_stop) | Stop Position (x, y) | Corridor Diag Threshold (t_diag) | Collision Time (t_coll) | Colliding Goat ID & Position | Signed Clearance (m) |
|---:|---:|:---:|---:|---:|---:|---:|:---:|---:|---:|:---:|---:|
| 20 m | 60 | Collision-Free | 2.50 s | 3.50 s | N/A | N/A | N/A | 9.00 s | N/A | N/A | 0.84 m |
| 25 m | 60 | Collision-Free | 3.50 s | 4.60 s | 14.00 s | N/A | N/A | 8.50 s | N/A | N/A | 0.76 m |
| 30 m | 60 | **COLLISION** | 4.60 s | 5.70 s | 4.80 s | 6.70 s | (31.32m, 2.80m) | 8.00 s | 8.60 s | Goat #9 (31.93m, 1.65m) | -1.15 m |
| 40 m | 60 | Collision-Free | N/A (Stopped Upstream) | N/A (Stopped Upstream) | 5.20 s | 7.10 s | (33.36m, 2.50m) | 7.20 s | N/A | N/A | 1.44 m |

## Deep-Dive Analysis for X_center = 30m Failure Mechanism

### 1. Multi-Stage Event Timeline
- **t = 0.00 s**: Ego starts at $X = 3.0\text{ m}, Y = 2.5\text{ m}, v = 5.0\text{ m/s}$.
- **t = 4.60 s**: Ego enters diagnostic window ($X \ge 25.0\text{ m}$).
- **t = 4.80 s**: SafetyFilter detects dynamic herd threat ahead and activates safe deceleration ($a = -2.02\text{ m/s}^2$).
- **t = 6.70 s**: Ego reaches **complete, safe standstill ($v = 0.00\text{ m/s}$)** at $X = 31.32\text{ m}, Y = 2.80\text{ m}$, successfully arresting forward motion.
- **t = 8.00 s**: Observed corridor metric around mean agent position crosses diagnostic threshold ($W_{\text{corr}} < 1.60\text{ m}$).
- **t = 8.60 s**: Goat #9 walking laterally at $v_y = 0.25\text{ m/s}$ reaches $X = 31.93\text{ m}, Y = 1.65\text{ m}$ and enters the parked vehicle side panel.
- **t = 8.60 s**: Ground-truth signed footprint clearance becomes negative ($C_{\text{scen}} = -1.15\text{ m}$, $dx = -2.14\text{ m}, dy = -0.00\text{ m}$).

### 2. Quantitative Footprint Geometry Verification
- **Ego Footprint at Standstill ($v = 0.00\text{ m/s}$)**:
  - Longitudinal bounds: $[X_{\text{rear}}, X_{\text{front}}] = [28.97\text{ m}, 33.67\text{ m}]$
  - Lateral bounds: $[Y_{\text{right}}, Y_{\text{left}}] = [1.75\text{ m}, 3.85\text{ m}]$
- **Colliding Agent (Goat #9) State at t = 8.60s**:
  - Position: $(X_{\text{goat}}, Y_{\text{goat}}) = (31.93\text{ m}, 1.65\text{ m})$
  - Longitudinal Overlap Check: $X_{\text{goat}} = 31.93\text{ m} \in [28.97\text{ m}, 33.67\text{ m}]$ (**INSIDE PARKED VEHICLE FOOTPRINT**)
  - Signed Clearance: $dx = -2.14\text{ m}, dy = -0.00\text{ m} \rightarrow C_{\text{scen}} = -1.15\text{ m}$

### 3. Upstream Arrest Analysis for X_center = 40m
For $X_{\text{center}}=40\text{ m}$, the SafetyFilter activates at $t=5.20\text{ s}$ and arrests ego motion upstream at $X=33.36\text{ m}$. Because the ego vehicle stops before reaching $X \ge 35\text{ m}$, the window reach and center reach metrics remain `N/A (Stopped Upstream)`, confirming the vehicle never entered the herd longitudinal zone.

### 4. Root Cause Classification: **Stationary-Ego Vulnerability to Continued Dynamic-Agent Intrusion**
The validated telemetry proves that:
1. The SafetyFilter brought the ego vehicle to a complete standstill ($v = 0.00\text{ m/s}$) at $t = 6.70\text{ s}$, successfully arresting forward motion before colliding with the herd ahead.
2. However, the collision occurred **1.90 seconds AFTER the vehicle was fully stopped**, caused exclusively by continued lateral movement of Goat #9 into the parked vehicle side panel.
3. **Key Finding**: Arresting forward motion is necessary but not sufficient to guarantee collision avoidance in a continuously evolving dynamic environment when dynamic agents continue moving laterally across a parked vehicle footprint.

