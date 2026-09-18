# How to Run & View the SIH26037 Autonomous Pipeline HTML Viewer & Presentation Slides

This guide provides clear, step-by-step instructions for executing MATLAB simulation exports and launching the interactive **HTML Pipeline Viewer** and **Presentation Slides** in your web browser or MATLAB environment.

---

## 1. Quick Launch Summary

| Component | Target File | Quick Launch Command / URL |
| :--- | :--- | :--- |
| **Interactive Pipeline Viewer** | `web_viewer/index.html` | `http://localhost:8000/web_viewer/` |
| **Slide 2: Architecture Blueprint** | `artifacts/SIH26037_Slide2_Architecture.html` | `http://localhost:8000/artifacts/SIH26037_Slide2_Architecture.html` |
| **Slide 3: Feasibility & Viability** | `feasibility_viability_slide.html` | `http://localhost:8000/feasibility_viability_slide.html` |

---

## 2. Step 1: Export Simulation Telemetry in MATLAB

Before launching the interactive viewer, generate fresh frame-by-frame telemetry from the closed-loop Stage 5 coordination pipeline.

### In MATLAB Command Window:

```matlab
% 1. Navigate to project root directory
cd('/home/yeswanth/projects/sih_new_2026');

% 2. Execute the Hero Scenario Export Script
run('scripts/run_hero_export.m');
```

**What this does**:
- Executes `stage5_multivehicle_coordination.m` for the flagship `multi_vehicle_yield_overtake` scenario (Seed 42, 150 timesteps).
- Runs perception, interaction classification, risk prediction, dynamic free-space mapping, and QP-MPC controller logic.
- Writes the complete telemetry file to:  
  `web_viewer/data/hero_scenario.json`

---

## 3. Step 2: Open and View in Web Browser

### Option A: Local Python Web Server (Recommended)

Running a local HTTP server avoids browser file-access security restrictions (CORS) when loading JSON data.

1. Open your terminal in the project directory:
   ```bash
   cd /home/yeswanth/projects/sih_new_2026
   python3 -m http.server 8000
   ```

2. Open your web browser (Chrome, Firefox, Edge, Safari) and navigate to:
   - **Pipeline Viewer**: [http://localhost:8000/web_viewer/](http://localhost:8000/web_viewer/)
   - **Architecture Slide**: [http://localhost:8000/artifacts/SIH26037_Slide2_Architecture.html](http://localhost:8000/artifacts/SIH26037_Slide2_Architecture.html)
   - **Feasibility Slide**: [http://localhost:8000/feasibility_viability_slide.html](http://localhost:8000/feasibility_viability_slide.html)

---

### Option B: Open Directly via File Explorer (Double-Click)

You can open the HTML files directly from your operating system file manager:

1. Open File Manager and navigate to `/home/yeswanth/projects/sih_new_2026/web_viewer/`.
2. Double-click `index.html`.
3. The interactive viewer will open in your default browser.

---

### Option C: Launch Directly from MATLAB

You can open the web viewer directly from inside MATLAB using the `web` command:

```matlab
% Open in your System Default Web Browser:
web(fullfile(pwd, 'web_viewer', 'index.html'), '-browser');

% OR Open in MATLAB's Embedded Web Browser Window:
web(fullfile(pwd, 'web_viewer', 'index.html'));
```

---

## 4. Pipeline Viewer Features & Controls

Once `web_viewer/index.html` is open, you can interact with the scenario simulation:

- **Playback Controls (Bottom Bar)**:
  - `Play / Pause` toggle simulation animation.
  - `Step Forward / Backward` inspect frame-by-frame state.
  - `Speed Selector`: $0.5\times$, $1\times$, $2\times$, $5\times$.
  - `Timeline Scrub Bar`: Click any timeline node to jump to a specific timestep.
- **Top Panel**: Displays active Macro-Intent state (`MAINTAIN`, `FOLLOW`, `YIELD`, `OVERTAKE`) and verbatim decision reasoning text from `CoordinationDecisionLayer.m`.
- **Canvas Rendering (Center)**: Visualizes real-time vehicle footprints, free-space bounds, relative obstacle positions, trajectory predictions, and clearance.
- **Telemetry Inspector (Right Panel)**: Displays perception targets, time-to-collision (TTC) values, track IDs, and topology passability flags (`hard_left`, `in_lane`, etc.).

---

## 5. Troubleshooting & Verification

| Issue | Cause | Solution |
| :--- | :--- | :--- |
| **Viewer shows black screen / "Loading..."** | JSON telemetry missing | Run `scripts/run_hero_export.m` in MATLAB to generate `hero_scenario.json`. |
| **Browser CORS Error (`Fetch failed`)** | Local `file://` security policy | Start `python3 -m http.server 8000` and open via `http://localhost:8000/web_viewer/`. |
| **MATLAB cannot find path** | Wrong current directory | Run `cd('/home/yeswanth/projects/sih_new_2026');` before executing MATLAB scripts. |
