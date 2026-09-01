# CA-CRC Benchmark Scenario Realism Scorecard (V1)

**Baseline Reference**: Verified V3 Benchmark Baseline (`v3-verified-baseline`, Commit `d73154f`)  
**Scope**: Scientific assessment of scenario fidelity, perception realism, and failure exposure across the 6 dynamic benchmark scenarios.

---

## 1. Qualitative Evaluation Criteria Definitions

To avoid arbitrary numerical scores, all evaluations use qualitative classifications defined as follows:

* **Dynamic Complexity**:
  * `LOW`: Single static obstacle or single constant-velocity agent with no interaction.
  * `MODERATE`: Multi-agent herd or scripted multi-stage lateral corridor changes.
  * `HIGH`: Dense multi-agent dynamic interaction with non-linear turning and acceleration.
* **Perception Realism**:
  * `LOW`: Perfect ground-truth state vector ($0.0\text{ m}$ noise, $0.0\text{ s}$ latency, zero occlusion).
  * `MODERATE`: Synthetic Gaussian noise ($\sigma \le 0.1\text{ m}$) with fixed bounding box uncertainty.
  * `HIGH`: Raw sensor simulation (LiDAR/Camera point-cloud/tracking jitter, occlusions, false positives).
* **Agent Realism**:
  * `LOW`: Linear constant-velocity model with zero acceleration or direction changes.
  * `MODERATE`: Scripted velocity triggers and constant lateral drift.
  * `HIGH`: Stochastic non-holonomic agent behavior (social force / bio-inspired livestock dynamics).
* **Geometry Realism**:
  * `MODERATE`: Straight 5.0m wide village road corridor with static/dynamic boundary constraints.
  * `HIGH`: Variable width, curved road topology with roadside obstacles and ditches.
* **Disturbance Diversity**:
  * `LOW`: Single deterministic seed or small parameter variation.
  * `MODERATE`: 50-trial Monte Carlo randomizing obstacle position and velocity vectors.
  * `HIGH`: Full multi-parameter randomization including friction, noise, latency, and agent counts.
* **Failure Exposure**:
  * `LOW`: Generous advance distance ($\ge 40\text{ m}$) ensuring kinematic stopping is easy.
  * `MODERATE`: Constricted corridor forcing deceleration/standstill without unavoidable collision.
  * `HIGH`: Near-boundary adversarial timing testing maximum kinematic braking limit ($a_{\text{min}} = -3.0\text{ m/s}^2$).

---

## 2. Scenario Realism Scorecard Table

| Scenario | Dynamic Complexity | Perception Realism | Agent Realism | Geometry Realism | Disturbance Diversity | Failure Exposure | Overall Rating | Primary Realism Limitation |
|---|---|---|---|---|---|---|---|---|
| **01 — Sudden Herd Entry** | MODERATE | LOW | LOW | MODERATE | MODERATE | LOW | **MODERATE** | Generous headway ($65\text{m}$) allows easy early braking |
| **02 — Herd Clears Recovery** | MODERATE | LOW | LOW | MODERATE | MODERATE | MODERATE | **MODERATE** | Goats cross in straight lines without panicking |
| **03 — Partial Gap Transition** | MODERATE | LOW | MODERATE | MODERATE | MODERATE | MODERATE | **MODERATE** | Gap opens deterministically at $t=8.0\text{s}$ |
| **04 — Opposite Gap Opens** | MODERATE | LOW | MODERATE | MODERATE | MODERATE | LOW | **MODERATE** | Gap opens when ego is still $25\text{m}$ away |
| **05 — Side Switch Topology** | MODERATE | LOW | MODERATE | MODERATE | MODERATE | MODERATE | **MODERATE** | Sudden boundary shift uses step velocity |
| **06 — Crossing Dynamic Agent**| LOW | LOW | LOW | MODERATE | MODERATE | LOW | **LOW-MODERATE** | Single agent moves at perfect constant velocity |

---

## 3. Key Audit Findings & Realism Justifications

1. **Perception Fidelity Baseline**: All 6 scenarios are currently classified as `LOW` in Perception Realism because the benchmark feeds ground-truth agent states from `WorldState` directly to `FreeSpaceMap` without sensor noise, latency, or occlusions.
2. **Kinematic Buffer Exposure**: Nominal Monte Carlo seeds place initial dynamic hazards at longitudinal distances of $45\text{ m}$ to $65\text{ m}$. At ego velocity $v = 5.0\text{ m/s}$, the controller has $8.0\text{ s}$ to $12.0\text{ s}$ of advance travel time, resulting in `LOW` to `MODERATE` failure exposure.
3. **Controller Soundness vs Field Readiness**: The scorecard confirms that while the frozen CA-CRC controller is **architecturally 100% safe and mathematically verified** for the nominal V3 benchmark suite, field deployment validation requires advancing Perception and Agent Realism from `LOW` to `MODERATE/HIGH`.
