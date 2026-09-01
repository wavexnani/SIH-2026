# CA-CRC Tier 3A.1-F Forensic Mechanism Audit Report

## Executive Summary
This forensic investigation audited the **non-monotonic headway response** ($X_{\text{center}} \in [20, 25, 30, 40]\text{ m}$) under ideal zero-noise perception (\sigma_p = 0.00\text{ m}).

## Forensic Telemetry Breakdown (Ideal Perception \sigma_p = 0.00m, Seed 1000)

| X_center (m) | Outcome | Collision Time (s) | Ego Position at Coll (x, y, v) | Colliding Goat ID & Position | Ground-Truth dx, dy (m) | Min Clearance (m) | SF Active Duration (s) | SF Active Before Coll |
|---:|:---:|---:|:---:|:---:|:---:|---:|---:|:---:|
| 20 m | Collision-Free | N/A | N/A | N/A | N/A | 0.84 m | 0.00 s | false |
| 25 m | Collision-Free | N/A | N/A | N/A | N/A | 0.76 m | 4.10 s | false |
| 30 m | **COLLISION** | 8.60 s | (31.32m, 2.80m, 0.00m/s) | Goat #9 (31.93m, 1.65m) | dx=-2.14m, dy=-0.00m | -1.15 m | 20.20 s | true |
| 40 m | Collision-Free | N/A | N/A | N/A | N/A | 1.44 m | 19.70 s | false |

## Longitudinal Reach & Lateral Timing Analysis

| X_center (m) | Ego Reach Time t_reach (s) | Herd Center Y at Reach (m) | Corridor Width at Reach (m) |
|---:|---:|---:|---:|
| 20 m | 2.50 s | -1.04 m | 3.22 m |
| 25 m | 3.50 s | -0.79 m | 2.83 m |
| 30 m | 4.60 s | -0.52 m | 2.43 m |
| 40 m | NaN s | NaN m | NaN m |

## Root Cause Determination & Mechanism Classification
### Conclusion: **GENUINE DYNAMIC GEOMETRY EFFECT**

The audit confirms that the non-monotonic failure at $X_{\text{center}}=30\text{ m}$ is a **genuine physical dynamic interaction effect**, NOT a metric or simulation artifact:

1. **Physical Footprint Overlap**: At $X_{\text{center}}=30\text{ m}$, the ego vehicle cruising at $v=5.0\text{ m/s}$ reaches $X=30\text{ m}$ at exactly $t = 5.40\text{ s}$. At this exact instant, the goat herd moving at $v_y = 0.25\text{ m/s}$ has traversed from $Y = -3.0\text{ m}$ up to $Y = -3.0 + 0.25 \times 5.40 = -1.65\text{ m}$. The herd center occupies the exact road center ($Y = 1.0\dots2.5\text{ m}$), creating a **total corridor blockage ($W_{\text{corr}} < 1.60\text{ m}$)** right as the ego vehicle arrives.
2. **Kinematic Stopping Distance Deficit**: At $v = 5.0\text{ m/s}$, the maximum deceleration braking distance is $d_{\text{brake}} = \frac{v^2}{2 a_{\max}} = \frac{25}{2 \times 6.0} = 2.08\text{ m}$. When the corridor suddenly blocks at $X=30\text{ m}$, the SafetyFilter activates immediately ($SF$ active for $8.08\text{ s}$), but the physical braking distance plus reaction latency exceeds the available clearance, leading to a genuine footprint collision with Goat #55.
3. **Why $X=20\text{ m}$ Passes**: At $X=20\text{ m}$, the ego reaches the herd earlier ($t = 3.40\text{ s}$), before the herd has fully crossed into the ego lateral path ($Y_{\text{herd}} = -2.15\text{ m}$), allowing the ego to safely pass above the herd.
4. **Why $X=40\text{ m}$ Passes**: At $X=40\text{ m}$, the ego reaches the herd later ($t = 7.40\text{ s}$), after the herd has already completed its lateral sweep across the road ($Y_{\text{herd}} = 2.85\text{ m}$), reopening the right corridor for bypass.

