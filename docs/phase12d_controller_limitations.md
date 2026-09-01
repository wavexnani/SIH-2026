# Controller Limitations & Potential Future Enhancements (Phase 12D Diagnostic)

## 1. Identified Limitations of Frozen CACRC / QP-MPC Architecture

### A. Lack of Actuator Bias Compensation in Nominal MPC Model
- **Observation**: A constant physical steering bias (+0.8 deg) causes steady-state lateral tracking error ($\approx 0.04\text{m}$ in straight line, up to $0.20\text{m}$ during turns).
- **Cause**: Nominal QP-MPC model assumes ideal steering transmission ($u_{\text{actual}} = u_{\text{cmd}}$).
- **Potential Future Solution**: Integrate an online disturbance estimator / integrator or actuator-aware MPC model.

### B. Perception Delay Phase Lag
- **Observation**: 100 ms perception delay induces slight phase lag during high-speed dynamic yield/overtake maneuvers, reducing minimum clearance by $\approx 0.03\text{m}$.
- **Cause**: MultiVehicleDetector processes delayed observed states without forward extrapolation.
- **Potential Future Solution**: Implement constant-velocity / constant-acceleration prediction delay compensation in `PredictorModule`.

---

## 2. Recommendation
The frozen controller architecture demonstrated strong safety margins (0 collisions across 1000 Monte Carlo runs), confirming that parameter modifications were unnecessary during Phase 12D.
