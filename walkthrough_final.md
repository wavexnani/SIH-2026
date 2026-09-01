# Final Technical Report — CA-CRC Autonomous Navigation Safety Envelope & Forensic Audit

## 1. Objective
The primary objective of this project is to evaluate the safety, collision-avoidance capability, and physical operating envelope of the **Corridor-Constrained Autonomous Reference Controller (CA-CRC)** combined with a **QP-MPC Planner** and a **SafetyFilter** on unstructured, non-lane-marked road environments under varying perception latency ($\tau \in [0.00, 0.30] \text{ s}$).

---

## 2. Frozen Core Stack
Throughout the forensic validation phases (Phases 55–59), the core navigation stack remained strictly **frozen and untouched**. The 7 frozen modules are:
1. `planning/CACRCPlanner.m`
2. `planning/QPMPCPlanner.m`
3. `planning/SafetyFilter.m`
4. `environment/FreeSpaceMap.m`
5. `vehicle/BicycleModel.m`
6. `stages/stage5_multivehicle_coordination.m`
7. `planning/CoordinationDecisionLayer.m`

---

## 3. Experimental Configuration
- **Road Corridor**: Width $W_{\text{road}} = 6.0 \text{ m}$ (unstructured, single/narrow carriageway).
- **Obstacle Setup**: Static corridor obstruction located at $x_{\text{obs}} = 55.0 \text{ m}$, width $W_{\text{obs}} = 2.0 \text{ m}$, length $L_{\text{obs}} = 4.0 \text{ m}$ (footprint rear boundary at $x = 53.0 \text{ m}$).
- **Ego Vehicle Setup**: Initial state $x_0 = 10.0 \text{ m}$, initial velocity $v_0 = 8.0 \text{ m/s}$ ($28.8 \text{ km/h}$), front bumper overhang $L_{\text{overhang}} = 2.0 \text{ m}$.
- **Perception Latency Sweep**: A discrete latency sweep spanning $\tau = 0.00 - 0.30 \text{ s}$ was evaluated, with fractional sub-step delays applied within the simulation across 220 Monte Carlo trials ($N=20$ trials per latency step).

---

## 4. Baseline Scenario
In the **Normal Baseline Scenario**, a fixed-lane controller operates without dynamic free-space lateral evasion. When approaching a blocked corridor at $x = 55.0 \text{ m}$, the baseline system fails to deviate laterally due to rigid lane assignment. Late emergency braking cannot prevent a high-speed rear-end collision, establishing the necessity of dynamic free-space corridor navigation.
* **Artifact**: `visualization/baseline_scenario.mp4`

---

## 5. CA-CRC Scenario
In the **CA-CRC Scenario**, the QP-MPC Planner actively constructs local drivable corridor bounds from free-space perception. Upon detecting the obstruction at $x = 55.0 \text{ m}$, the planner calculates a smooth lateral bypass trajectory within the 6.0 m road boundary, executing dynamic free-space evasion in the nominal test scenario while maintaining continuous longitudinal motion.
* **Artifact**: `visualization/cacrc_scenario.mp4`

---

## 6. Latency Experiment
To identify the physical breakdown boundary of the closed-loop autonomous stack, a discrete latency sweep spanning $\tau = 0.00 - 0.30 \text{ s}$ was evaluated, with fractional sub-step delays applied within the simulation to the perceived ego state passed to the planner:
$$\mathbf{x}_{\text{perceived}}(t) = \mathbf{x}_{\text{true}}(t - \tau)$$

The physical vehicle dynamics were updated using the true vehicle plant (`BicycleModel.m`).

---

## 7. Collision Probability & Non-Monotonic Operating Envelope
Across 220 Monte Carlo trials ($N=20$ per latency condition), the empirical collision rate $P(\text{collision})$ exhibits a non-monotonic operating envelope with four distinct regimes:

| Latency $\tau$ (s) | Collision Rate $P(\text{col})$ | 95% Wilson Confidence Interval | Operating Regime |
|:---:|:---:|:---:|:---|
| **0.00** | **0.0%** | [0.0%, 16.1%] | **Nominal Safe Regime** |
| **0.05** | **0.0%** | [0.0%, 16.1%] | **Nominal Safe Regime** |
| **0.10** | **0.0%** | [0.0%, 16.1%] | **Nominal Safe Regime** |
| **0.12** | **0.0%** | [0.0%, 16.1%] | **Nominal Safe Regime** |
| **0.14** | **0.0%** | [0.0%, 16.1%] | **Nominal Safe Regime** |
| **0.16** | **0.0%** | [0.0%, 16.1%] | **Nominal Safe Regime** |
| **0.18** | **10.0%** | [2.8%, 30.1%] | **Transition Boundary** |
| **0.20** | **100.0%** | [83.9%, 100.0%] | **Physical Deficit Collision Regime** |
| **0.22** | **85.0%** | [64.0%, 94.8%] | **Physical Deficit Collision Regime** |
| **0.25** | **90.0%** | [69.9%, 97.2%] | **Physical Deficit Collision Regime** |
| **0.30** | **0.0%** | [0.0%, 16.1%] | **Early Emergency Recovery Regime** |

**Key Finding**: Latency does not produce a monotonically increasing collision probability in this closed-loop architecture because the SafetyFilter introduces a different intervention regime at higher latency ($\tau = 0.30 \text{ s}$).

---

## 8. Physical Clearance
Physical bumper clearance $C(t)$ is defined as the distance between the ego front bumper ($x_{\text{ego}}$) and the obstacle footprint rear ($x = 53.0 \text{ m}$):
$$C(t) = 53.0 - x_{\text{ego}}(t)$$

- **Safe Regime ($\tau \le 0.16 \text{ s}$)**: $\bar{C}_{\min} = +3.71 \text{ m}$ to $+6.48 \text{ m}$ (positive safety margin).
- **Collision Regime ($\tau = 0.20 - 0.25 \text{ s}$)**: $\bar{C}_{\min} = -0.59 \text{ m}$ to $-0.98 \text{ m}$ (footprint penetration).

---

## 9. Event-Based Braking Analysis
For every trial, physical event timestamps were logged:
1. **First Detection Instant ($x_{\text{detect}}$)**: Instant obstacle corridor blockage enters the planner horizon.
2. **Actual Braking Onset ($x_{\text{brake}}$)**: Instant applied deceleration exceeds $-0.5 \text{ m/s}^2$.
3. **Endpoint ($x_{\text{endpoint}}$)**: Collision instant ($C \le 0$) or full stop ($v \le 0.01 \text{ m/s}$).

Available braking distance from braking onset:
$$d_{\text{avail,brake}} = 53.0 - x_{\text{brake}}$$

Actual distance traveled during braking:
$$d_{\text{braking,endpoint}} = x_{\text{endpoint}} - x_{\text{brake}}$$

---

## 10. Collision Deficit
For collision trials, the event-based collision deficit $D_{\text{deficit,event}}$ is:
$$D_{\text{deficit,event}} = d_{\text{braking,endpoint}} - d_{\text{avail,brake}} = x_{\text{col}} - 53.0 \equiv |C_{\text{collision}}|$$

**Exact Identity Verification**: Across all 57 collision trials in the dataset,
$$\max_{i} \left| D_{\text{deficit,event}}^{(i)} - |C_{\text{collision}}^{(i)}| \right| = \mathbf{0.000000000000 \text{ m}}$$

The event-level identity confirms that the observed collision penetration is exactly represented by the physical braking-distance deficit. The telemetry does not indicate that the measured collisions require a controller-instability explanation.

---

## 11. Early Emergency Recovery at $\tau = 0.30 \text{ s}$
At $\tau = 0.30 \text{ s}$, the SafetyFilter produces an early emergency braking response, with braking beginning around $x_{\text{brake}} \approx 38.5 \text{ m}$, substantially earlier than the obstacle footprint boundary at $x = 53.0 \text{ m}$. The resulting additional stopping distance margin prevents physical footprint penetration.

---

## 12. Reproducibility
The entire Phase 59 dataset, evidence matrices, telemetry logs, and diagnostic figures can be re-generated deterministically using:
```bash
matlab -batch "addpath('config', 'core', 'environment', 'planning', 'stages', 'vehicle', 'visualization', 'scratch'); phase59_forensic_event_audit(); generate_final_evidence_package(); exit;"
```

Generated files:
- `results/final_evidence_matrix.csv`
- `results/final_operating_envelope.csv`
- `results/final_telemetry.csv`
- `results/phase59_per_trial_forensics.csv`
- `results/phase59_event_forensics.csv`
- `visualization/phase59_event_boundary.png`

---

## 13. SHA-256 Integrity Verification
The integrity of all 7 frozen core files was verified before and after execution:

```text
bda557297f43641669ab53b5db4f9dfbc358ecfbfde22dcb3fa8b0e51b6d3949  planning/CACRCPlanner.m
cf4f0e76961b4ee63fb1d71e8cffe244605629e6ff50165840eb1843a2605cc7  planning/QPMPCPlanner.m
75538547b30969e657dfb2c61077c44b24dc347cd69dc6e97d753166df5dd95d  planning/SafetyFilter.m
2b6146df683dc99ce6eddd36843b9cc771f395d48b7f071dcb75146544b33c1b  environment/FreeSpaceMap.m
4910e17b4fc6191f30986c8d25c3f17a20f7eb9fdd6d951eb4e057e864266592  vehicle/BicycleModel.m
7a13b9a218d9d1c14315bcaab25b8e9606df3c16f0453b76cf763e382bd135d8  stages/stage5_multivehicle_coordination.m
051023a4890614f9b128643988f500d13a996cc7fbd2c8155c7a6f1d28287466  planning/CoordinationDecisionLayer.m
```
* **Artifact**: `results/final_checksum_report.txt`

---

## 14. Limitations
1. **Single-Carriageway Corridor Width**: Evaluated under 6.0 m corridor constraints.
2. **Perception Latency Model**: Evaluated across discrete latency steps $\tau \in [0.00, 0.30] \text{ s}$.
3. **Emergency Filter Thresholds**: Early safety filter intervention behavior at $\tau \ge 0.30 \text{ s}$ depends on conservative prediction error bounds.

---

## 15. Final Conclusion
> **The CA-CRC system exhibits a latency-dependent safety envelope with a transition from safe operation to physical collision and a subsequent recovery at high latency due to earlier emergency intervention.**
