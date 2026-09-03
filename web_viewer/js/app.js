/* ====================================================================
   SIH26037 Autonomous Vehicle Pipeline Viewer — Main Controller
   Pure read-only visualization layer. Zero simulation logic.
   All displayed values are parsed from exported MATLAB simulationLog JSON.
   ==================================================================== */

document.addEventListener('DOMContentLoaded', () => {
  const App = {
    data: null,
    currentStep: 0,
    isPlaying: false,
    playInterval: null,
    playbackSpeed: 1.0,
    events: [],
    
    // Canvas & Viewport
    canvas: document.getElementById('scene-canvas'),
    ctx: null,
    scale: 12, // pixels per meter
    offsetX: 0,
    offsetY: 0,

    init() {
      this.ctx = this.canvas.getContext('2d');
      this.resizeCanvas();
      window.addEventListener('resize', () => this.resizeCanvas());

      this.bindEvents();
      this.loadData();
    },

    resizeCanvas() {
      const container = document.getElementById('main-view');
      this.canvas.width = container.clientWidth;
      this.canvas.height = container.clientHeight;
      this.renderFrame();
    },

    async loadData() {
      try {
        const res = await fetch('data/hero_scenario.json');
        if (!res.ok) throw new Error(`HTTP error! status: ${res.status}`);
        this.data = await res.json();
        
        // Hide loading overlay
        document.getElementById('loading-overlay').style.display = 'none';

        this.setupHeader();
        this.detectEvents();
        this.setupTimeline();
        this.setStep(0);
      } catch (err) {
        console.error('Failed to load simulation data:', err);
        document.getElementById('loading-overlay').innerHTML = `
          <h2 style="color:var(--accent-red)">Failed to load data</h2>
          <p style="margin-top:8px;color:var(--text-secondary)">Please ensure hero_scenario.json exists in web_viewer/data/</p>
          <pre style="margin-top:12px;font-size:11px;color:var(--text-muted)">${err.message}</pre>
        `;
      }
    },

    setupHeader() {
      const meta = this.data.metadata || {};
      document.getElementById('scenario-name').textContent = meta.scenario_name || 'Hero Scenario';
      document.getElementById('hdr-seed').textContent = meta.seed !== undefined ? meta.seed : '42';
      document.getElementById('hdr-steps').textContent = (this.data.timesteps || []).length;
      
      const outcomeElem = document.getElementById('hdr-outcome');
      outcomeElem.textContent = meta.outcome || (meta.passed ? 'PASSED' : 'FAILED');
      outcomeElem.style.color = meta.passed ? 'var(--accent-green)' : 'var(--accent-yellow)';
    },

    detectEvents() {
      this.events = [];
      const steps = this.data.timesteps || [];
      
      let prevIntent = null;
      let prevTopo = null;

      steps.forEach((st, idx) => {
        const intent = st.decision ? st.decision.macro_intent : st.decision_intent;
        const topo = st.topology ? st.topology.selected : null;
        const minTtc = st.risk ? st.risk.min_ttc : null;
        const filterActive = st.safety ? st.safety.filter_active : false;

        // Intent change
        if (prevIntent && intent && intent !== prevIntent) {
          this.events.push({
            step: idx,
            type: 'intent-change',
            title: `Intent Change: ${prevIntent} ➔ ${intent}`,
            desc: st.decision ? st.decision.reason : ''
          });
        }
        prevIntent = intent;

        // Topology change
        if (prevTopo && topo && topo !== prevTopo) {
          this.events.push({
            step: idx,
            type: 'topology-change',
            title: `Topology Change: ${prevTopo} ➔ ${topo}`,
            desc: `Selected route topology changed to ${topo}`
          });
        }
        prevTopo = topo;

        // Critical TTC
        if (minTtc !== null && minTtc < 3.0 && minTtc > 0) {
          this.events.push({
            step: idx,
            type: 'ttc-critical',
            title: `Critical TTC: ${minTtc.toFixed(2)}s`,
            desc: `TTC dropped below safety threshold`
          });
        }

        // Safety intervention
        if (filterActive) {
          this.events.push({
            step: idx,
            type: 'safety-event',
            title: `Safety Intervention`,
            desc: st.safety ? st.safety.filter_reason : 'Layer 2 Safety Filter active'
          });
        }
      });
    },

    setupTimeline() {
      const steps = this.data.timesteps || [];
      const slider = document.getElementById('timeline-slider');
      slider.max = Math.max(0, steps.length - 1);
      slider.value = 0;

      slider.addEventListener('input', (e) => {
        this.pause();
        this.setStep(parseInt(e.target.value, 10));
      });

      // Render event markers on timeline track
      const track = document.getElementById('timeline-track');
      // remove old markers
      track.querySelectorAll('.event-marker').forEach(m => m.remove());

      const maxStep = steps.length - 1;
      if (maxStep <= 0) return;

      this.events.forEach(evt => {
        const marker = document.createElement('div');
        marker.className = `event-marker ${evt.type}`;
        marker.style.left = `${(evt.step / maxStep) * 100}%`;
        
        marker.addEventListener('mouseenter', (e) => this.showTooltip(e, evt));
        marker.addEventListener('mouseleave', () => this.hideTooltip());
        marker.addEventListener('click', () => {
          this.pause();
          this.setStep(evt.step);
        });

        track.appendChild(marker);
      });
    },

    bindEvents() {
      document.getElementById('btn-play').addEventListener('click', () => this.togglePlay());
      document.getElementById('btn-restart').addEventListener('click', () => {
        this.pause();
        this.setStep(0);
      });
      document.getElementById('btn-step-back').addEventListener('click', () => {
        this.pause();
        this.setStep(Math.max(0, this.currentStep - 1));
      });
      document.getElementById('btn-step-fwd').addEventListener('click', () => {
        this.pause();
        this.setStep(Math.min((this.data.timesteps.length - 1), this.currentStep + 1));
      });

      document.getElementById('speed-select').addEventListener('change', (e) => {
        this.playbackSpeed = parseFloat(e.target.value);
        if (this.isPlaying) {
          this.pause();
          this.play();
        }
      });

      // Keyboard shortcuts
      window.addEventListener('keydown', (e) => {
        if (e.key === ' ') {
          e.preventDefault();
          this.togglePlay();
        } else if (e.key === 'ArrowLeft') {
          this.pause();
          this.setStep(Math.max(0, this.currentStep - 1));
        } else if (e.key === 'ArrowRight') {
          this.pause();
          this.setStep(Math.min((this.data.timesteps.length - 1), this.currentStep + 1));
        }
      });
    },

    togglePlay() {
      if (this.isPlaying) this.pause();
      else this.play();
    },

    play() {
      if (!this.data || !this.data.timesteps) return;
      this.isPlaying = true;
      document.getElementById('btn-play').textContent = '⏸';
      document.getElementById('btn-play').classList.add('active');

      const dt = (this.data.metadata ? this.data.metadata.dt : 0.1) * 1000;
      const interval = dt / this.playbackSpeed;

      this.playInterval = setInterval(() => {
        if (this.currentStep >= this.data.timesteps.length - 1) {
          this.pause();
          return;
        }
        this.setStep(this.currentStep + 1);
      }, interval);
    },

    pause() {
      this.isPlaying = false;
      document.getElementById('btn-play').textContent = '▶';
      document.getElementById('btn-play').classList.remove('active');
      if (this.playInterval) {
        clearInterval(this.playInterval);
        this.playInterval = null;
      }
    },

    setStep(stepIdx) {
      if (!this.data || !this.data.timesteps || stepIdx < 0 || stepIdx >= this.data.timesteps.length) return;
      this.currentStep = stepIdx;
      document.getElementById('timeline-slider').value = stepIdx;

      const st = this.data.timesteps[stepIdx];
      const totalSteps = this.data.timesteps.length;
      document.getElementById('time-display').textContent = `t = ${(st.time || stepIdx*0.1).toFixed(2)}s | Step ${stepIdx + 1}/${totalSteps}`;

      this.renderFrame();
      this.renderPanels();
    },

    showTooltip(e, evt) {
      const tooltip = document.getElementById('event-tooltip');
      tooltip.innerHTML = `<strong>${evt.title}</strong><br><span style="color:var(--text-secondary)">${evt.desc || ''}</span>`;
      tooltip.style.left = `${e.clientX + 10}px`;
      tooltip.style.top = `${e.clientY - 30}px`;
      tooltip.style.display = 'block';
    },

    hideTooltip() {
      document.getElementById('event-tooltip').style.display = 'none';
    },

    /* ====================================================================
       CANVAS RENDERING ENGINE
       ==================================================================== */
    renderFrame() {
      if (!this.ctx || !this.data || !this.data.timesteps) return;
      const step = this.data.timesteps[this.currentStep];
      if (!step) return;

      const ctx = this.ctx;
      const width = this.canvas.width;
      const height = this.canvas.height;

      // Clear canvas
      ctx.fillStyle = '#0d1117';
      ctx.fillRect(0, 0, width, height);

      // Camera centering around Ego vehicle
      const egoX = step.ego ? step.ego.x : (step.groundTruth ? step.groundTruth.ego.x : 10);
      const roadLength = this.data.metadata ? this.data.metadata.road_length : 150;
      const roadWidth = this.data.metadata ? this.data.metadata.road_width : 6;
      
      // Auto-scale to fit road vertically with margins
      this.scale = height / (roadWidth + 8);
      const centerY = height / 2;

      // Camera offset horizontally: keep Ego near 25% from left
      this.offsetX = width * 0.25 - egoX * this.scale;
      this.offsetY = centerY;

      ctx.save();
      ctx.translate(this.offsetX, this.offsetY);
      // Flip Y axis so +Y is up
      ctx.scale(1, -1);

      // 1. Draw Road Surface & Markings
      this.drawRoad(roadLength, roadWidth);

      // 2. Draw Static Obstacles
      if (this.data.metadata && this.data.metadata.static_obstacles) {
        this.drawStaticObstacles(this.data.metadata.static_obstacles);
      }

      // 3. Draw Free Space Corridor Bounds
      if (step.freeSpace && step.freeSpace.y_min_vec && step.freeSpace.y_max_vec) {
        this.drawFreeSpaceBounds(step.freeSpace, egoX);
      }

      // 4. Draw Agent Predictions
      if (step.prediction && step.prediction.agents) {
        this.drawAgentPredictions(step.prediction.agents);
      }

      // 5. Draw Ego MPC Planned Trajectory
      if (step.mpc && step.mpc.pred_ego_traj) {
        this.drawEgoPlan(step.mpc.pred_ego_traj);
      }

      // 6. Draw Observed Agents (Ghost outline)
      if (step.observation && step.observation.agents) {
        this.drawObservedAgents(step.observation.agents);
      }

      // 7. Draw Ground Truth Dynamic Agents
      if (step.groundTruth && step.groundTruth.agents) {
        this.drawGroundTruthAgents(step.groundTruth.agents);
      }

      // 8. Draw Ego Vehicle
      if (step.ego) {
        this.drawEgoVehicle(step.ego);
      }

      ctx.restore();

      // Render HUD Overlay text on canvas
      this.drawCanvasHUD(step);
    },

    drawRoad(length, width) {
      const ctx = this.ctx;
      const s = this.scale;

      // Road asphalt surface
      ctx.fillStyle = '#161b22';
      ctx.fillRect(-20 * s, 0, (length + 40) * s, width * s);

      // Outer Road Boundaries
      ctx.strokeStyle = '#484f58';
      ctx.lineWidth = 3;
      
      // Bottom Boundary (y=0)
      ctx.beginPath();
      ctx.moveTo(-20 * s, 0);
      ctx.lineTo((length + 40) * s, 0);
      ctx.stroke();

      // Top Boundary (y=width)
      ctx.beginPath();
      ctx.moveTo(-20 * s, width * s);
      ctx.lineTo((length + 40) * s, width * s);
      ctx.stroke();

      // Center Lane Divider Line (Dashed)
      ctx.strokeStyle = '#30363d';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([10, 10]);
      ctx.beginPath();
      ctx.moveTo(-20 * s, (width / 2) * s);
      ctx.lineTo((length + 40) * s, (width / 2) * s);
      ctx.stroke();
      ctx.setLineDash([]);
    },

    drawStaticObstacles(obsList) {
      const ctx = this.ctx;
      const s = this.scale;

      obsList.forEach(obs => {
        if (obs.x < -50) return;
        ctx.fillStyle = 'rgba(248,81,73,0.3)';
        ctx.strokeStyle = '#f85149';
        ctx.lineWidth = 1.5;

        ctx.fillRect((obs.x - obs.L/2) * s, (obs.y - obs.W/2) * s, obs.L * s, obs.W * s);
        ctx.strokeRect((obs.x - obs.L/2) * s, (obs.y - obs.W/2) * s, obs.L * s, obs.W * s);
      });
    },

    drawFreeSpaceBounds(freeSpace, egoX) {
      const ctx = this.ctx;
      const s = this.scale;
      const minVec = freeSpace.y_min_vec || [];
      const maxVec = freeSpace.y_max_vec || [];
      if (!minVec.length) return;

      const N = minVec.length;
      const dx = 0.5; // step increment along horizon

      ctx.fillStyle = 'rgba(63,185,80,0.08)';
      ctx.strokeStyle = 'rgba(63,185,80,0.4)';
      ctx.lineWidth = 1;

      ctx.beginPath();
      for (let k = 0; k < N; k++) {
        const px = egoX + k * dx;
        const py = minVec[k];
        if (k === 0) ctx.moveTo(px * s, py * s);
        else ctx.lineTo(px * s, py * s);
      }
      for (let k = N - 1; k >= 0; k--) {
        const px = egoX + k * dx;
        const py = maxVec[k];
        ctx.lineTo(px * s, py * s);
      }
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
    },

    drawAgentPredictions(preds) {
      const ctx = this.ctx;
      const s = this.scale;

      preds.forEach(p => {
        if (!p.x_traj || !p.x_traj.length) return;
        ctx.strokeStyle = 'rgba(188,140,255,0.7)';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([4, 4]);

        ctx.beginPath();
        for (let k = 0; k < p.x_traj.length; k++) {
          const px = p.x_traj[k];
          const py = p.y_traj[k];
          if (k === 0) ctx.moveTo(px * s, py * s);
          else ctx.lineTo(px * s, py * s);
        }
        ctx.stroke();
        ctx.setLineDash([]);
      });
    },

    drawEgoPlan(traj) {
      const ctx = this.ctx;
      const s = this.scale;
      if (!traj || !traj.length) return;

      ctx.strokeStyle = '#58a6ff';
      ctx.lineWidth = 2.5;

      ctx.beginPath();
      // Handle both matrix form [N_p x 4] or struct array
      for (let k = 0; k < traj.length; k++) {
        let px, py;
        if (Array.isArray(traj[k])) {
          px = traj[k][0]; py = traj[k][1];
        } else {
          px = traj[k].x; py = traj[k].y;
        }
        if (k === 0) ctx.moveTo(px * s, py * s);
        else ctx.lineTo(px * s, py * s);
      }
      ctx.stroke();

      // Planned horizon dots
      ctx.fillStyle = '#58a6ff';
      for (let k = 0; k < traj.length; k += 2) {
        let px, py;
        if (Array.isArray(traj[k])) { px = traj[k][0]; py = traj[k][1]; }
        else { px = traj[k].x; py = traj[k].y; }
        ctx.beginPath();
        ctx.arc(px * s, py * s, 2.5, 0, 2 * Math.PI);
        ctx.fill();
      }
    },

    drawObservedAgents(obsAgents) {
      const ctx = this.ctx;
      const s = this.scale;

      obsAgents.forEach(ag => {
        ctx.strokeStyle = '#d29922';
        ctx.lineWidth = 1;
        ctx.setLineDash([3, 3]);

        ctx.save();
        ctx.translate(ag.x * s, ag.y * s);
        ctx.rotate(ag.heading || 0);

        const L = 4.7, W = 1.8;
        ctx.strokeRect(-L/2 * s, -W/2 * s, L * s, W * s);
        ctx.restore();
        ctx.setLineDash([]);
      });
    },

    drawGroundTruthAgents(agents) {
      const ctx = this.ctx;
      const s = this.scale;

      agents.forEach(ag => {
        if (ag.x < -50) return;
        const L = ag.length || 4.7;
        const W = ag.width || 1.8;
        const heading = Math.atan2(ag.vy || 0, (ag.vx || 0) + 1e-6);

        ctx.save();
        ctx.translate(ag.x * s, ag.y * s);
        ctx.rotate(heading);

        // Vehicle body
        ctx.fillStyle = '#f0883e';
        ctx.strokeStyle = '#d29922';
        ctx.lineWidth = 1.5;
        ctx.fillRect(-L/2 * s, -W/2 * s, L * s, W * s);
        ctx.strokeRect(-L/2 * s, -W/2 * s, L * s, W * s);

        // Direction Indicator
        ctx.fillStyle = '#ffffff';
        ctx.beginPath();
        ctx.arc(L/3 * s, 0, 2, 0, 2*Math.PI);
        ctx.fill();

        ctx.restore();

        // Label (unrotated text)
        ctx.save();
        ctx.scale(1, -1);
        ctx.fillStyle = '#ffffff';
        ctx.font = '10px "JetBrains Mono", monospace';
        ctx.fillText(`A${ag.id}`, ag.x * s - 6, -ag.y * s - W/2 * s - 4);
        ctx.restore();
      });
    },

    drawEgoVehicle(ego) {
      const ctx = this.ctx;
      const s = this.scale;
      const L = (this.data.metadata ? this.data.metadata.vehicle_length : 4.7);
      const W = (this.data.metadata ? this.data.metadata.vehicle_width : 1.8);

      ctx.save();
      ctx.translate(ego.x * s, ego.y * s);
      ctx.rotate(ego.theta || 0);

      // Ego Body
      ctx.fillStyle = '#58a6ff';
      ctx.strokeStyle = '#388bfd';
      ctx.lineWidth = 2;
      ctx.fillRect(-L/2 * s, -W/2 * s, L * s, W * s);
      ctx.strokeRect(-L/2 * s, -W/2 * s, L * s, W * s);

      // Heading Vector Line
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo((L/2 + 1.2) * s, 0);
      ctx.stroke();

      ctx.restore();

      // Ego Label
      ctx.save();
      ctx.scale(1, -1);
      ctx.fillStyle = '#58a6ff';
      ctx.font = 'bold 11px "JetBrains Mono", monospace';
      ctx.fillText(`EGO (${ego.v ? ego.v.toFixed(1) : '0.0'} m/s)`, ego.x * s - 25, -ego.y * s - W/2 * s - 6);
      ctx.restore();
    },

    drawCanvasHUD(step) {
      const ctx = this.ctx;
      ctx.save();
      ctx.font = '12px "JetBrains Mono", monospace';
      ctx.fillStyle = '#8b949e';

      const intent = step.decision ? step.decision.macro_intent : (step.decision_intent || 'UNKNOWN');
      const minTtc = step.risk && step.risk.min_ttc !== undefined ? step.risk.min_ttc.toFixed(2) : '∞';
      const egoV = step.ego ? step.ego.v.toFixed(1) : '0.0';

      ctx.fillText(`Macro-Intent: ${intent}`, 20, this.canvas.height - 40);
      ctx.fillText(`Ego Speed: ${egoV} m/s | Min TTC: ${minTtc}s`, 20, this.canvas.height - 20);
      ctx.restore();
    },

    /* ====================================================================
       PIPELINE INSPECTION PANELS (RIGHT SIDE ACCORDION)
       ==================================================================== */
    renderPanels() {
      const container = document.getElementById('panel-container');
      const st = this.data.timesteps[this.currentStep];
      if (!st) return;

      const prevSt = this.currentStep > 0 ? this.data.timesteps[this.currentStep - 1] : null;

      let html = '';

      // 1. "WHY DID IT DECIDE THAT?" PANEL (HIGHLIGHT PANEL)
      html += this.renderDecisionExplanationPanel(st, prevSt);

      // 2. PERCEPTION & DETECTIONS PANEL
      html += this.renderPerceptionPanel(st);

      // 3. PREDICTION & RISK PANEL
      html += this.renderPredictionRiskPanel(st);

      // 4. FREE SPACE & TOPOLOGY PANEL
      html += this.renderFreeSpaceTopologyPanel(st);

      // 5. MPC DIAGNOSTICS PANEL
      html += this.renderMPCPanel(st);

      // 6. LAYER 2 SAFETY FILTER PANEL
      html += this.renderSafetyPanel(st);

      // 7. ACTUATION & PLANT DYNAMICS PANEL
      html += this.renderActuationPanel(st);

      // 8. GROUND TRUTH & VEHICLE STATE PANEL
      html += this.renderGroundTruthPanel(st);

      container.innerHTML = html;
    },

    renderDecisionExplanationPanel(st, prevSt) {
      const dec = st.decision || {};
      const intent = dec.macro_intent || 'UNKNOWN';
      const reason = dec.reason || st.coord_reason || 'No reasoning log available';
      const targetV = dec.target_v !== undefined ? dec.target_v.toFixed(1) : '—';
      const prevIntent = prevSt && prevSt.decision ? prevSt.decision.macro_intent : null;
      const intentChanged = prevIntent && prevIntent !== intent;

      let alertCard = '';
      if (intentChanged) {
        alertCard = `
          <div class="decision-change-card">
            <div class="change-title">⚡ Intent Transition Detected</div>
            <div class="change-arrow">
              <span class="intent-tag intent-${prevIntent}">${prevIntent}</span>
              ➔
              <span class="intent-tag intent-${intent}">${intent}</span>
            </div>
            <div style="font-size:11px;color:var(--text-secondary);margin-top:4px;">${reason}</div>
          </div>
        `;
      }

      return `
        <div class="panel-section">
          <div class="panel-header">
            <h2>🧠 Why Did It Decide That?</h2>
            <span class="intent-tag intent-${intent}">${intent}</span>
          </div>
          <div class="panel-body">
            ${alertCard}
            <div class="panel-row">
              <span class="panel-key">Macro-Intent</span>
              <span class="panel-val"><span class="intent-tag intent-${intent}">${intent}</span></span>
            </div>
            <div class="panel-row">
              <span class="panel-key">Target Speed</span>
              <span class="panel-val">${targetV} m/s</span>
            </div>
            <div style="margin-top:8px;padding:8px;background:var(--bg-card);border-radius:6px;border:1px solid var(--border);">
              <div style="font-size:10px;text-transform:uppercase;color:var(--text-secondary);margin-bottom:4px;">Decision Reasoning (Verbatim Pipeline Output)</div>
              <div style="font-family:var(--font-mono);font-size:11px;color:var(--accent-blue);line-height:1.4;">"${reason}"</div>
            </div>
          </div>
        </div>
      `;
    },

    renderPerceptionPanel(st) {
      const obs = st.observation || {};
      const dets = st.detections || [];
      
      let detHtml = '';
      if (dets.length === 0) {
        detHtml = '<div style="font-size:11px;color:var(--text-muted);padding:4px 0;">No active detections in field of view</div>';
      } else {
        dets.forEach(d => {
          detHtml += `
            <div style="background:var(--bg-card);padding:6px;border-radius:4px;margin-top:4px;font-size:11px;">
              <div style="display:flex;justify-content:space-between;color:var(--accent-orange);font-weight:600;">
                <span>Target Agent #${d.id}</span>
                <span>${d.is_oncoming ? 'ONCOMING' : 'AHEAD'}</span>
              </div>
              <div style="display:grid;grid-template-columns:1fr 1fr;gap:4px;margin-top:4px;color:var(--text-secondary);font-family:var(--font-mono);">
                <div>Rel dx: <span style="color:var(--text-primary)">${d.dx.toFixed(1)}m</span></div>
                <div>Rel dy: <span style="color:var(--text-primary)">${d.dy.toFixed(1)}m</span></div>
                <div>Closing dvx: <span style="color:var(--text-primary)">${d.dvx ? d.dvx.toFixed(1) : '0.0'} m/s</span></div>
                <div>Same Lane: <span style="color:var(--text-primary)">${d.is_same_lane ? 'Yes' : 'No'}</span></div>
              </div>
            </div>
          `;
        });
      }

      return `
        <div class="panel-section">
          <div class="panel-header">
            <h2>👀 Perception & Detections</h2>
            <span class="panel-badge">${dets.length} Targets</span>
          </div>
          <div class="panel-body">
            <div class="panel-row">
              <span class="panel-key">Perception Mode</span>
              <span class="panel-val">${obs.mode || 'ideal'}</span>
            </div>
            ${detHtml}
          </div>
        </div>
      `;
    },

    renderPredictionRiskPanel(st) {
      const risk = st.risk || {};
      const ints = st.interaction || [];
      const minTtc = risk.min_ttc !== undefined ? risk.min_ttc : null;
      const ttcStr = (minTtc !== null && minTtc < 99) ? `${minTtc.toFixed(2)}s` : '∞ (Clear)';

      let intHtml = '';
      ints.forEach(ix => {
        intHtml += `
          <div class="panel-row">
            <span class="panel-key">Agent #${ix.id} Class</span>
            <span class="panel-val" style="color:var(--accent-yellow)">${ix.class_name || 'UNKNOWN'}</span>
          </div>
        `;
      });

      return `
        <div class="panel-section">
          <div class="panel-header">
            <h2>⚠️ Risk & Trajectory Prediction</h2>
            <span class="panel-badge" style="color:${minTtc < 3 ? 'var(--accent-red)' : 'var(--text-primary)'}">TTC ${ttcStr}</span>
          </div>
          <div class="panel-body">
            <div class="panel-row">
              <span class="panel-key">Minimum TTC</span>
              <span class="panel-val" style="color:${minTtc < 3 ? 'var(--accent-red)' : 'var(--accent-green)'}">${ttcStr}</span>
            </div>
            ${intHtml}
          </div>
        </div>
      `;
    },

    renderFreeSpaceTopologyPanel(st) {
      const topo = st.topology || {};
      const fs = st.freeSpace || {};

      return `
        <div class="panel-section">
          <div class="panel-header">
            <h2>🛣️ Free-Space & Topology Selection</h2>
            <span class="panel-badge" style="color:var(--accent-purple)">${topo.selected || 'center'}</span>
          </div>
          <div class="panel-body">
            <div class="panel-row">
              <span class="panel-key">Selected Route Topology</span>
              <span class="panel-val" style="color:var(--accent-purple);font-weight:600">${topo.selected || 'center'}</span>
            </div>
            <div class="panel-row">
              <span class="panel-key">Latch State (Locked Side)</span>
              <span class="panel-val">${topo.locked_side || 'none'}</span>
            </div>
            <div class="panel-row">
              <span class="panel-key">Left Pass Geometric OK?</span>
              <span class="panel-val ${topo.ok_geom_left ? 'feasible-tag' : 'infeasible-tag'}">${topo.ok_geom_left ? 'YES' : 'NO'}</span>
            </div>
            <div class="panel-row">
              <span class="panel-key">Right Pass Geometric OK?</span>
              <span class="panel-val ${topo.ok_geom_right ? 'feasible-tag' : 'infeasible-tag'}">${topo.ok_geom_right ? 'YES' : 'NO'}</span>
            </div>
            <div class="panel-row">
              <span class="panel-key">Left Topology Cost J_left</span>
              <span class="panel-val">${topo.J_left ? topo.J_left.toFixed(2) : '—'}</span>
            </div>
            <div class="panel-row">
              <span class="panel-key">Right Topology Cost J_right</span>
              <span class="panel-val">${topo.J_right ? topo.J_right.toFixed(2) : '—'}</span>
            </div>
            <div class="panel-row">
              <span class="panel-key">Min Corridor Width</span>
              <span class="panel-val">${fs.min_corridor_width ? fs.min_corridor_width.toFixed(2) + 'm' : '—'}</span>
            </div>
          </div>
        </div>
      `;
    },

    renderMPCPanel(st) {
      const mpc = st.mpc || {};
      const statusStr = mpc.status === 0 ? 'SUCCESS (0)' : `SOFT FALLBACK (${mpc.status})`;
      const cmd = mpc.u_cmd || [0, 0];

      return `
        <div class="panel-section">
          <div class="panel-header">
            <h2>🎯 QP-MPC Trajectory Solver</h2>
            <span class="panel-badge ${mpc.status === 0 ? 'safe-tag' : 'override-tag'}">${mpc.status === 0 ? 'Hard QP' : 'Soft QP'}</span>
          </div>
          <div class="panel-body">
            <div class="panel-row">
              <span class="panel-key">Hildreth Solver Status</span>
              <span class="panel-val ${mpc.status === 0 ? 'safe-tag' : 'override-tag'}">${statusStr}</span>
            </div>
            <div class="panel-row">
              <span class="panel-key">Solve Execution Time</span>
              <span class="panel-val">${mpc.solve_time_ms ? mpc.solve_time_ms.toFixed(2) + ' ms' : '—'}</span>
            </div>
            <div class="panel-row">
              <span class="panel-key">Is Soft Fallback?</span>
              <span class="panel-val">${mpc.is_soft ? 'Yes (Relaxed)' : 'No (Strict)'}</span>
            </div>
            <div class="panel-row">
              <span class="panel-key">MPC Command [delta, a]</span>
              <span class="panel-val">${(cmd[0]*180/Math.PI).toFixed(1)}° | ${cmd[1].toFixed(2)} m/s²</span>
            </div>
          </div>
        </div>
      `;
    },

    renderSafetyPanel(st) {
      const sft = st.safety || {};
      const active = sft.filter_active || false;
      const reason = sft.filter_reason || 'Layer 2 inactive (MPC command safe)';

      return `
        <div class="panel-section">
          <div class="panel-header">
            <h2>🛡️ Layer 2 Safety Filter</h2>
            <span class="panel-badge ${active ? 'override-tag' : 'safe-tag'}">${active ? 'INTERVENTION' : 'NOMINAL'}</span>
          </div>
          <div class="panel-body">
            <div class="panel-row">
              <span class="panel-key">Filter Status</span>
              <span class="panel-val ${active ? 'override-tag' : 'safe-tag'}">${active ? 'ACTIVE OVERRIDE' : 'INACTIVE (PASSTHROUGH)'}</span>
            </div>
            <div style="margin-top:6px;padding:6px;background:var(--bg-card);border-radius:4px;font-size:11px;color:var(--text-secondary);">
              Filter Note: <span style="color:var(--text-primary)">${reason}</span>
            </div>
          </div>
        </div>
      `;
    },

    renderActuationPanel(st) {
      const act = st.actuation || {};

      return `
        <div class="panel-section">
          <div class="panel-header">
            <h2>⚙️ Actuator & Plant Dynamics</h2>
            <span class="panel-badge">Physical Plant</span>
          </div>
          <div class="panel-body">
            <div class="panel-row">
              <span class="panel-key">Steering Cmd vs Plant</span>
              <span class="panel-val">${((act.delta_cmd||0)*180/Math.PI).toFixed(1)}° ➔ ${((act.delta_plant||0)*180/Math.PI).toFixed(1)}°</span>
            </div>
            <div class="panel-row">
              <span class="panel-key">Accel Cmd vs Plant</span>
              <span class="panel-val">${(act.a_cmd||0).toFixed(2)} ➔ ${(act.a_plant||0).toFixed(2)} m/s²</span>
            </div>
            <div class="panel-row">
              <span class="panel-key">Steering Error</span>
              <span class="panel-val">${((act.steering_error||0)*180/Math.PI).toFixed(2)}°</span>
            </div>
          </div>
        </div>
      `;
    },

    renderGroundTruthPanel(st) {
      const gt = st.groundTruth || {};
      const ego = st.ego || (gt.ego || {});
      const clr = gt.min_clearance !== undefined ? gt.min_clearance.toFixed(2) + 'm' : '—';

      return `
        <div class="panel-section">
          <div class="panel-header">
            <h2>🌐 Ground Truth State</h2>
            <span class="panel-badge">Telemetry</span>
          </div>
          <div class="panel-body">
            <div class="panel-row">
              <span class="panel-key">Ego Position (x, y)</span>
              <span class="panel-val">(${ego.x ? ego.x.toFixed(2) : 0}, ${ego.y ? ego.y.toFixed(2) : 0})</span>
            </div>
            <div class="panel-row">
              <span class="panel-key">Ego Heading theta</span>
              <span class="panel-val">${ego.theta ? (ego.theta * 180/Math.PI).toFixed(1) + '°' : '0°'}</span>
            </div>
            <div class="panel-row">
              <span class="panel-key">Speed v</span>
              <span class="panel-val">${ego.v ? ego.v.toFixed(2) + ' m/s' : '0.00 m/s'}</span>
            </div>
            <div class="panel-row">
              <span class="panel-key">OBB SAT Min Clearance</span>
              <span class="panel-val" style="color:var(--accent-green)">${clr}</span>
            </div>
            <div class="panel-row">
              <span class="panel-key">In Road Bounds?</span>
              <span class="panel-val safe-tag">${gt.in_bounds !== false ? 'YES' : 'NO'}</span>
            </div>
          </div>
        </div>
      `;
    }
  };

  App.init();
});
