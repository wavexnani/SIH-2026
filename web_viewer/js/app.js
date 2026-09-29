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
      const scenSelect = document.getElementById('scenario-select');
      const initialUrl = scenSelect ? scenSelect.value : 'data/handdrawn_village_corrected.json';
      this.loadData(initialUrl);
    },

    resizeCanvas() {
      const container = document.getElementById('main-view');
      this.canvas.width = container.clientWidth;
      this.canvas.height = container.clientHeight;
      this.renderFrame();
    },

    async loadData(jsonUrl = 'data/handdrawn_village_corrected.json') {
      try {
        const overlay = document.getElementById('loading-overlay');
        if (overlay) overlay.style.display = 'flex';
        const res = await fetch(jsonUrl);
        if (!res.ok) throw new Error(`HTTP error! status: ${res.status}`);
        this.data = await res.json();
        
        // Hide loading overlay
        if (overlay) overlay.style.display = 'none';

        this.setupHeader();
        this.detectEvents();
        this.setupTimeline();
        this.setStep(0);
      } catch (err) {
        console.error('Failed to load simulation data:', err);
        const overlay = document.getElementById('loading-overlay');
        if (overlay) {
          overlay.style.display = 'flex';
          overlay.innerHTML = `
            <h2 style="color:var(--accent-red)">Failed to load data</h2>
            <p style="margin-top:8px;color:var(--text-secondary)">Please ensure ${jsonUrl} exists in web_viewer/data/</p>
            <pre style="margin-top:12px;font-size:11px;color:var(--text-muted)">${err.message}</pre>
          `;
        }
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

        // Speed Breaker Bump Traversal Event
        if (st.ego && st.ego.x >= 86.5 && st.ego.x <= 89.5 && (!this.events.some(e => e.type === 'speed-breaker'))) {
          this.events.push({
            step: idx,
            type: 'speed-breaker',
            title: `Speed Breaker: ${(st.ego.v || 2.0).toFixed(1)} m/s Crawl`,
            desc: `Suspension regulated speed for 10cm IRC road bump traversal`
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

      const scenSelect = document.getElementById('scenario-select');
      if (scenSelect) {
        scenSelect.addEventListener('change', (e) => {
          this.pause();
          this.loadData(e.target.value);
        });
      }

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
      const egoX = step.ego ? step.ego.x : (step.groundTruth && step.groundTruth.ego ? step.groundTruth.ego.x : 10);
      const roadLength = this.data.metadata ? this.data.metadata.road_length : 150;
      const roadWidth = this.data.metadata ? this.data.metadata.road_width : 6;
      
      // Auto-scale to fit road vertically with margins
      this.scale = height / (roadWidth + 8);

      // Camera offset horizontally & vertically: center road (y=roadWidth/2) on screen
      this.offsetX = width * 0.25 - egoX * this.scale;
      this.offsetY = height / 2 + (roadWidth / 2) * this.scale;

      ctx.save();
      ctx.translate(this.offsetX, this.offsetY);
      // Flip Y axis so +Y is up
      ctx.scale(1, -1);

      // 1. Draw Road Surface & Markings
      this.drawRoad(roadLength, roadWidth);

      // 1b. Draw Canal Trench & Culvert Bridge (if defined in metadata)
      if (this.data.metadata && this.data.metadata.bridge_canal) {
        this.drawCanalBridge(this.data.metadata.bridge_canal, roadLength, roadWidth);
      }

      // 1c. Draw Physical Speed Breakers (if defined in metadata)
      if (this.data.metadata && this.data.metadata.speed_breakers) {
        this.drawSpeedBreakers(this.data.metadata.speed_breakers);
      }

      // 2. Draw Static Obstacles
      if (this.data.metadata && this.data.metadata.static_obstacles) {
        this.drawStaticObstacles(this.data.metadata.static_obstacles);
      }

      // 2b. Draw Potholes & Road Defects
      if (this.data.metadata && this.data.metadata.potholes) {
        this.drawPotholes(this.data.metadata.potholes);
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
      const meta = this.data.metadata || {};
      const yCenterBase = (meta.road_bounds ? (meta.road_bounds[2] + meta.road_bounds[3]) / 2 : width / 2);
      const halfW = width / 2;
      const noiseAmp = meta.boundary_noise_amp || 0.12;
      const curveAmp = meta.curve_amp || 2.80;

      // Mathematical centerline function strictly adhering to RoadGeometry.m
      const getCenter = (x) => {
        if (x <= 60.0) {
          return yCenterBase + 0.30 * Math.sin(2 * Math.PI * x / 80.0);
        } else if (x <= 85.0) {
          const ym = 0.30 * Math.sin(2 * Math.PI * x / 80.0);
          const sIn = (x - 60.0) / 25.0;
          const wBr = 3.0 * sIn * sIn - 2.0 * sIn * sIn * sIn;
          return yCenterBase + (1.0 - wBr) * ym;
        } else if (x <= 106.0) {
          return yCenterBase;
        } else {
          if (meta.blind_bend_active) {
            const xStart = meta.blind_bend_x_start || 135.0;
            const bAmp = meta.blind_bend_amp || 20.0;
            const bLen = meta.blind_bend_length || 38.0;
            if (x < xStart) {
              return yCenterBase;
            } else if (x < xStart + bLen) {
              const sNorm = (x - xStart) / bLen;
              const S = 10.0 * Math.pow(sNorm, 3) - 15.0 * Math.pow(sNorm, 4) + 6.0 * Math.pow(sNorm, 5);
              return yCenterBase + bAmp * S;
            } else {
              return yCenterBase + bAmp;
            }
          } else {
            const dx = x - 106.0;
            const LTurn = 48.0;
            if (dx < LTurn) {
              const sNorm = dx / LTurn;
              const S = 10.0 * Math.pow(sNorm, 3) - 15.0 * Math.pow(sNorm, 4) + 6.0 * Math.pow(sNorm, 5);
              return yCenterBase + curveAmp * S;
            } else {
              return yCenterBase + curveAmp;
            }
          }
        }
      };

      const xMin = -25;
      const xMax = length + 25;
      const stepX = 1.0;

      // 1. Off-Road Dirt Shoulder & Vegetation Background
      ctx.save();
      // Lower dirt shoulder
      ctx.fillStyle = '#423124';
      ctx.beginPath();
      for (let x = xMin; x <= xMax; x += stepX) {
        const yc = getCenter(x);
        const yDirtBot = yc - halfW - 3.5;
        if (x === xMin) ctx.moveTo(x * s, yDirtBot * s);
        else ctx.lineTo(x * s, yDirtBot * s);
      }
      for (let x = xMax; x >= xMin; x -= stepX) {
        const yc = getCenter(x);
        const yBot = yc - halfW;
        ctx.lineTo(x * s, yBot * s);
      }
      ctx.closePath();
      ctx.fill();

      // Upper dirt shoulder
      ctx.beginPath();
      for (let x = xMin; x <= xMax; x += stepX) {
        const yc = getCenter(x);
        const yTop = yc + halfW;
        if (x === xMin) ctx.moveTo(x * s, yTop * s);
        else ctx.lineTo(x * s, yTop * s);
      }
      for (let x = xMax; x >= xMin; x -= stepX) {
        const yc = getCenter(x);
        const yDirtTop = yc + halfW + 3.5;
        ctx.lineTo(x * s, yDirtTop * s);
      }
      ctx.closePath();
      ctx.fill();

      // Roadside grass tufts along dirt shoulder
      for (let x = xMin; x <= xMax; x += 6.0) {
        const yc = getCenter(x);
        const yGrassBot = yc - halfW - 2.2 + 0.4 * Math.sin(x * 1.7);
        const yGrassTop = yc + halfW + 2.2 + 0.4 * Math.cos(x * 1.9);
        
        ctx.fillStyle = '#2d3b25';
        ctx.beginPath();
        ctx.ellipse(x * s, yGrassBot * s, 1.2 * s, 0.6 * s, 0.2, 0, 2 * Math.PI);
        ctx.fill();

        ctx.fillStyle = '#35452b';
        ctx.beginPath();
        ctx.ellipse((x + 2.0) * s, yGrassTop * s, 1.4 * s, 0.7 * s, -0.2, 0, 2 * Math.PI);
        ctx.fill();
      }
      ctx.restore();

      // 2. Realistic Asphalt Surface with Organic Eroded Edges
      ctx.save();
      ctx.fillStyle = '#1c2128';
      ctx.beginPath();
      for (let x = xMin; x <= xMax; x += stepX) {
        const yc = getCenter(x);
        const jitter = (noiseAmp > 0 ? noiseAmp * (0.6 * Math.cos(x / 7.8 + 0.8) + 0.4 * Math.sin(x / 2.9)) : 0);
        const yBot = yc - halfW + jitter;
        if (x === xMin) ctx.moveTo(x * s, yBot * s);
        else ctx.lineTo(x * s, yBot * s);
      }
      for (let x = xMax; x >= xMin; x -= stepX) {
        const yc = getCenter(x);
        const jitter = (noiseAmp > 0 ? noiseAmp * (0.6 * Math.sin(x / 6.5 + 0.3) + 0.4 * Math.cos(x / 3.2)) : 0);
        const yTop = yc + halfW + jitter;
        ctx.lineTo(x * s, yTop * s);
      }
      ctx.closePath();
      ctx.fill();

      // Dark Tire Compaction Wheel Ruts (Traffic Tracks)
      ctx.strokeStyle = '#12161c';
      ctx.lineWidth = 1.3 * s;
      ctx.lineCap = 'round';
      // Lower lane tire tracks (y_c - 1.20m)
      ctx.beginPath();
      for (let x = xMin; x <= xMax; x += stepX * 2) {
        const yc = getCenter(x);
        const yRut = yc - 1.20;
        if (x === xMin) ctx.moveTo(x * s, yRut * s);
        else ctx.lineTo(x * s, yRut * s);
      }
      ctx.stroke();
      // Upper lane tire tracks (y_c + 1.20m)
      ctx.beginPath();
      for (let x = xMin; x <= xMax; x += stepX * 2) {
        const yc = getCenter(x);
        const yRut = yc + 1.20;
        if (x === xMin) ctx.moveTo(x * s, yRut * s);
        else ctx.lineTo(x * s, yRut * s);
      }
      ctx.stroke();

      // Crumbling Eroded Shoulder Outline
      ctx.strokeStyle = '#574332';
      ctx.lineWidth = 2.0;
      ctx.beginPath();
      for (let x = xMin; x <= xMax; x += stepX) {
        const yc = getCenter(x);
        const jitter = (noiseAmp > 0 ? noiseAmp * (0.6 * Math.cos(x / 7.8 + 0.8) + 0.4 * Math.sin(x / 2.9)) : 0);
        const yBot = yc - halfW + jitter;
        if (x === xMin) ctx.moveTo(x * s, yBot * s);
        else ctx.lineTo(x * s, yBot * s);
      }
      ctx.stroke();

      ctx.beginPath();
      for (let x = xMin; x <= xMax; x += stepX) {
        const yc = getCenter(x);
        const jitter = (noiseAmp > 0 ? noiseAmp * (0.6 * Math.sin(x / 6.5 + 0.3) + 0.4 * Math.cos(x / 3.2)) : 0);
        const yTop = yc + halfW + jitter;
        if (x === xMin) ctx.moveTo(x * s, yTop * s);
        else ctx.lineTo(x * s, yTop * s);
      }
      ctx.stroke();

      // Faded, Weathered Road Centerline (Sun-bleached dashes with irregular wear)
      ctx.strokeStyle = 'rgba(210, 180, 140, 0.45)';
      ctx.lineWidth = 1.2;
      ctx.setLineDash([8, 12]);
      ctx.beginPath();
      for (let x = xMin; x <= xMax; x += stepX) {
        const yc = getCenter(x);
        if (x === xMin) ctx.moveTo(x * s, yc * s);
        else ctx.lineTo(x * s, yc * s);
      }
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.restore();
    },

    drawPotholes(potholes) {
      if (!potholes || !potholes.length) return;
      const list = Array.isArray(potholes) ? potholes : [potholes];
      const ctx = this.ctx;
      const s = this.scale;

      list.forEach(p => {
        if (!p || p.x === undefined) return;
        ctx.save();
        ctx.translate(p.x * s, p.y * s);

        const rx = (p.length / 2) * s;
        const ry = (p.width / 2) * s;

        // Broken asphalt fractured rim
        ctx.fillStyle = '#11151a';
        ctx.strokeStyle = '#6e7681';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        const nPoints = 14;
        for (let i = 0; i < nPoints; i++) {
          const angle = (i / nPoints) * 2 * Math.PI;
          const rVar = 0.85 + 0.30 * Math.sin(angle * 3.5 + p.id);
          const px = Math.cos(angle) * rx * rVar;
          const py = Math.sin(angle) * ry * rVar;
          if (i === 0) ctx.moveTo(px, py);
          else ctx.lineTo(px, py);
        }
        ctx.closePath();
        ctx.fill();
        ctx.stroke();

        // Inner murky water puddle reflection
        ctx.fillStyle = '#0a1017';
        ctx.beginPath();
        for (let i = 0; i < nPoints; i++) {
          const angle = (i / nPoints) * 2 * Math.PI;
          const rVar = 0.65 + 0.20 * Math.cos(angle * 2.5 + p.id);
          const px = Math.cos(angle) * rx * rVar;
          const py = Math.sin(angle) * ry * rVar;
          if (i === 0) ctx.moveTo(px, py);
          else ctx.lineTo(px, py);
        }
        ctx.closePath();
        ctx.fill();

        // Water specular highlight
        ctx.strokeStyle = 'rgba(56, 189, 248, 0.35)';
        ctx.lineWidth = 1.0;
        ctx.beginPath();
        ctx.arc(-rx * 0.25, -ry * 0.2, rx * 0.35, 0.2, Math.PI * 0.8);
        ctx.stroke();

        // Scattered gravel pieces around rim
        ctx.fillStyle = '#8b949e';
        for (let g = 0; g < 6; g++) {
          const gAngle = (g / 6) * 2 * Math.PI + 0.4;
          const gx = Math.cos(gAngle) * (rx + 2.5);
          const gy = Math.sin(gAngle) * (ry + 2.0);
          ctx.fillRect(gx, gy, 1.8, 1.8);
        }

        ctx.restore();

        // Label
        ctx.save();
        ctx.scale(1, -1);
        ctx.fillStyle = '#f85149';
        ctx.font = 'bold 9px "JetBrains Mono", monospace';
        ctx.fillText(`POTHOLE #${p.id} (${p.severity || 'severe'})`, p.x * s - 18, -p.y * s - ry - 4);
        ctx.restore();
      });
    },

    drawCanalBridge(bridgeCanal, roadLength, roadWidth) {
      if (!bridgeCanal || (!bridgeCanal.active && !bridgeCanal.is_active)) return;
      const ctx = this.ctx;
      const s = this.scale;
      const xStart = bridgeCanal.x_start || 92.0;
      const xEnd = bridgeCanal.x_end || 106.0;
      const bridgeW = bridgeCanal.width || 5.2;
      const yCenter = roadWidth / 2;

      // 1. Canal Water Trench (Flowing across road under bridge)
      const canalMargin = 12.0; // Extend water beyond road boundaries
      ctx.save();
      // Flowing water gradient
      const waterGrad = ctx.createLinearGradient(xStart * s, 0, xEnd * s, 0);
      waterGrad.addColorStop(0, '#0c4a6e');
      waterGrad.addColorStop(0.5, '#0284c7');
      waterGrad.addColorStop(1, '#0c4a6e');
      ctx.fillStyle = waterGrad;
      ctx.fillRect(xStart * s, -canalMargin * s, (xEnd - xStart) * s, (roadWidth + 2 * canalMargin) * s);

      // Water Ripple Waves
      ctx.strokeStyle = 'rgba(56, 189, 248, 0.4)';
      ctx.lineWidth = 1.5;
      for (let yW = -canalMargin; yW <= roadWidth + canalMargin; yW += 1.8) {
        ctx.beginPath();
        ctx.moveTo(xStart * s, yW * s);
        ctx.bezierCurveTo(
          (xStart + 4) * s, (yW + 0.3) * s,
          (xEnd - 4) * s, (yW - 0.3) * s,
          xEnd * s, yW * s
        );
        ctx.stroke();
      }

      // Canal Earth/Stone Banks
      ctx.fillStyle = '#78350f';
      ctx.fillRect((xStart - 1.2) * s, -canalMargin * s, 1.2 * s, (roadWidth + 2 * canalMargin) * s);
      ctx.fillRect(xEnd * s, -canalMargin * s, 1.2 * s, (roadWidth + 2 * canalMargin) * s);

      // 2. Concrete Culvert Bridge Deck (Asphalt road over canal)
      const yBridgeLo = yCenter - bridgeW / 2;
      const yBridgeHi = yCenter + bridgeW / 2;

      ctx.fillStyle = '#1e293b';
      ctx.fillRect(xStart * s, yBridgeLo * s, (xEnd - xStart) * s, bridgeW * s);

      // 3. Concrete Parapet / Safety Railings (Constrains drivable road width to 5.2m)
      const drawParapet = (yPos) => {
        // Concrete base barrier
        ctx.fillStyle = '#64748b';
        ctx.fillRect(xStart * s, (yPos - 0.25) * s, (xEnd - xStart) * s, 0.5 * s);
        ctx.strokeStyle = '#334155';
        ctx.lineWidth = 1;
        ctx.strokeRect(xStart * s, (yPos - 0.25) * s, (xEnd - xStart) * s, 0.5 * s);

        // Hazard striped reflectors along barrier
        for (let xp = xStart; xp < xEnd - 1.0; xp += 2.0) {
          ctx.fillStyle = '#facc15';
          ctx.fillRect(xp * s, (yPos - 0.15) * s, 1.0 * s, 0.3 * s);
          ctx.fillStyle = '#0f172a';
          ctx.fillRect((xp + 1.0) * s, (yPos - 0.15) * s, 1.0 * s, 0.3 * s);
        }
      };

      drawParapet(yBridgeLo);
      drawParapet(yBridgeHi);

      // Centerline across bridge
      ctx.strokeStyle = '#e2e8f0';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([5, 5]);
      ctx.beginPath();
      ctx.moveTo(xStart * s, yCenter * s);
      ctx.lineTo(xEnd * s, yCenter * s);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.restore();

      // Bridge & Canal Label
      ctx.save();
      ctx.scale(1, -1);
      ctx.fillStyle = '#38bdf8';
      ctx.font = 'bold 10px "JetBrains Mono", monospace';
      ctx.fillText('CULVERT BRIDGE (CANAL CROSSING)', (xStart + 0.5) * s, -(yBridgeHi + 0.6) * s);
      ctx.fillStyle = '#0284c7';
      ctx.fillText('WATER CANAL', (xStart + 3.0) * s, -(yBridgeLo - 1.2) * s);
      ctx.restore();
    },

    drawSpeedBreakers(speedBreakers) {
      if (!speedBreakers || !speedBreakers.length) return;
      const list = Array.isArray(speedBreakers) ? speedBreakers : [speedBreakers];
      const ctx = this.ctx;
      const s = this.scale;
      const meta = this.data.metadata || {};
      const roadW = meta.road_width || 6.0;

      list.forEach(sb => {
        if (!sb || sb.x === undefined) return;
        const L = sb.length || 0.8;
        const yMin = 0.0;
        const yMax = roadW;

        ctx.save();
        // 3D Mound shadow and raised asphalt gradient
        const moundGrad = ctx.createLinearGradient((sb.x - L/2) * s, 0, (sb.x + L/2) * s, 0);
        moundGrad.addColorStop(0, '#1f2937');
        moundGrad.addColorStop(0.5, '#4b5563');
        moundGrad.addColorStop(1, '#111827');
        ctx.fillStyle = moundGrad;
        ctx.fillRect((sb.x - L/2) * s, yMin * s, L * s, roadW * s);

        // Indian IRC Standard High-Contrast Yellow/Black Chevron Arrows Pattern
        const chevronH = 0.6;
        for (let y = yMin; y < yMax; y += chevronH) {
          const isYellow = (Math.floor(y / chevronH) % 2 === 0);
          ctx.fillStyle = isYellow ? '#facc15' : '#0f172a';
          ctx.beginPath();
          ctx.moveTo((sb.x - L/2) * s, y * s);
          ctx.lineTo((sb.x + L/2 * 0.4) * s, (y + chevronH/2) * s);
          ctx.lineTo((sb.x - L/2) * s, (y + chevronH) * s);
          ctx.lineTo((sb.x - L/2 + 0.3) * s, (y + chevronH) * s);
          ctx.lineTo((sb.x + L/2) * s, (y + chevronH/2) * s);
          ctx.lineTo((sb.x - L/2 + 0.3) * s, y * s);
          ctx.closePath();
          ctx.fill();
        }

        // White approach rumble warning stripes
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.7)';
        ctx.lineWidth = 1.5;
        for (let dX of [-3.5, -2.5, -1.5, 1.5, 2.5, 3.5]) {
          ctx.beginPath();
          ctx.moveTo((sb.x + dX) * s, (yMin + 0.2) * s);
          ctx.lineTo((sb.x + dX) * s, (yMax - 0.2) * s);
          ctx.stroke();
        }
        ctx.restore();

        // Speed Breaker Sign Label
        ctx.save();
        ctx.scale(1, -1);
        ctx.fillStyle = '#facc15';
        ctx.font = 'bold 10px "JetBrains Mono", monospace';
        ctx.fillText(`IRC SPEED BUMP (${sb.x}m)`, (sb.x - 5.0) * s, -(yMax + 0.6) * s);
        ctx.restore();
      });
    },

    drawStaticObstacles(obsList) {
      if (!obsList) return;
      const list = Array.isArray(obsList) ? obsList : [obsList];
      const ctx = this.ctx;
      const s = this.scale;

      list.forEach(obs => {
        if (!obs || obs.x === undefined || obs.x < -50) return;
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
      const dx = 0.5;

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
      const list = Array.isArray(preds) ? preds : [preds];

      list.forEach(p => {
        if (!p || !p.x_traj || !p.x_traj.length) return;
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
      const list = Array.isArray(obsAgents) ? obsAgents : [obsAgents];

      list.forEach(ag => {
        if (!ag || ag.x === undefined || ag.x < -50) return;
        const L = ag.length || 4.7;
        const W = ag.width || 1.8;
        const heading = (ag.heading !== undefined) ? ag.heading : Math.atan2(ag.vy || 0, (ag.vx || 0) + 1e-6);

        ctx.strokeStyle = '#d29922';
        ctx.lineWidth = 1;
        ctx.setLineDash([3, 3]);

        ctx.save();
        ctx.translate(ag.x * s, ag.y * s);
        ctx.rotate(heading);

        const type = (ag.type || 'car').toLowerCase();
        if (type === 'pedestrian' || type === 'ped') {
          ctx.beginPath();
          ctx.arc(0, 0, (Math.max(L, W) / 2) * s, 0, 2 * Math.PI);
          ctx.stroke();
        } else if (type === 'cattle' || type === 'sheep') {
          ctx.beginPath();
          ctx.ellipse(0, 0, (L / 2) * s, (W / 2) * s, 0, 0, 2 * Math.PI);
          ctx.stroke();
        } else {
          ctx.strokeRect(-L/2 * s, -W/2 * s, L * s, W * s);
        }
        ctx.restore();
        ctx.setLineDash([]);
      });
    },

    drawAutoRickshawSprite(ctx, s, L, W) {
      const halfL = (L / 2) * s;
      const halfW = (W / 2) * s;

      // 1. Lower Body Panels (Forest Green)
      ctx.fillStyle = '#15803d';
      ctx.strokeStyle = '#14532d';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(halfL, 0); // Front nose
      ctx.lineTo(halfL * 0.35, halfW); // Front-to-side taper
      ctx.lineTo(-halfL, halfW); // Rear left
      ctx.lineTo(-halfL, -halfW); // Rear right
      ctx.lineTo(halfL * 0.35, -halfW); // Side-to-front taper
      ctx.closePath();
      ctx.fill();
      ctx.stroke();

      // 2. Yellow Canvas Roof Canopy
      ctx.fillStyle = '#eab308';
      ctx.strokeStyle = '#ca8a04';
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.moveTo(halfL * 0.5, 0);
      ctx.lineTo(halfL * 0.15, halfW * 0.88);
      ctx.lineTo(-halfL * 0.85, halfW * 0.88);
      ctx.lineTo(-halfL * 0.85, -halfW * 0.88);
      ctx.lineTo(halfL * 0.15, -halfW * 0.88);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();

      // Roof canvas ribs
      ctx.strokeStyle = '#a16207';
      ctx.lineWidth = 1.0;
      for (let rx of [-halfL * 0.6, -halfL * 0.2, halfL * 0.1]) {
        ctx.beginPath();
        ctx.moveTo(rx, -halfW * 0.85);
        ctx.lineTo(rx, halfW * 0.85);
        ctx.stroke();
      }

      // 3. Curved Windshield with Wiper
      ctx.fillStyle = 'rgba(56, 189, 248, 0.45)';
      ctx.strokeStyle = '#0f172a';
      ctx.lineWidth = 1.0;
      ctx.beginPath();
      ctx.moveTo(halfL * 0.45, -halfW * 0.55);
      ctx.lineTo(halfL * 0.65, 0);
      ctx.lineTo(halfL * 0.45, halfW * 0.55);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();

      // Wiper
      ctx.strokeStyle = '#0f172a';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(halfL * 0.52, 0);
      ctx.lineTo(halfL * 0.48, halfW * 0.35);
      ctx.stroke();

      // 4. Wheels & Mudguards
      ctx.fillStyle = '#0f172a';
      ctx.fillRect((halfL - 2), -halfW * 0.2, 4, halfW * 0.4);
      ctx.fillRect((-halfL + 2), halfW - 2, halfL * 0.45, 3.5);
      ctx.fillRect((-halfL + 2), -halfW - 1.5, halfL * 0.45, 3.5);

      // Rear spare tire mount
      ctx.beginPath();
      ctx.arc(-halfL, 0, halfW * 0.35, 0, 2 * Math.PI);
      ctx.fillStyle = '#1e293b';
      ctx.fill();
      ctx.strokeStyle = '#0f172a';
      ctx.stroke();

      // Front Headlamp
      ctx.fillStyle = '#fef08a';
      ctx.beginPath();
      ctx.arc(halfL + 1, 0, 2.5, 0, 2 * Math.PI);
      ctx.fill();
    },

    drawCattleSprite(ctx, s, L, W) {
      const halfL = (L / 2) * s;
      const halfW = (W / 2) * s;

      // 1. Torso & Flanks (Earthy mottled hide)
      ctx.fillStyle = '#a05a2c';
      ctx.strokeStyle = '#78350f';
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.ellipse(0, 0, halfL * 0.75, halfW * 0.75, 0, 0, 2 * Math.PI);
      ctx.fill();
      ctx.stroke();

      // Mottled hide patch
      ctx.fillStyle = '#f5ebe0';
      ctx.beginPath();
      ctx.ellipse(-halfL * 0.2, halfW * 0.2, halfL * 0.3, halfW * 0.35, 0.4, 0, 2 * Math.PI);
      ctx.fill();

      // 2. Thoracic Shoulder Hump (Zebu hump)
      ctx.fillStyle = '#854d0e';
      ctx.beginPath();
      ctx.arc(halfL * 0.25, 0, halfW * 0.55, 0, 2 * Math.PI);
      ctx.fill();

      // 3. Bovine Head & Snout
      ctx.fillStyle = '#b45309';
      ctx.beginPath();
      ctx.ellipse(halfL * 0.8, 0, halfL * 0.32, halfW * 0.45, 0, 0, 2 * Math.PI);
      ctx.fill();
      ctx.stroke();

      // Muzzle & Nostrils
      ctx.fillStyle = '#d97706';
      ctx.beginPath();
      ctx.ellipse(halfL * 1.05, 0, halfL * 0.14, halfW * 0.32, 0, 0, 2 * Math.PI);
      ctx.fill();
      ctx.fillStyle = '#1c1917';
      ctx.fillRect(halfL * 1.08, -halfW * 0.15, 1.8, 1.8);
      ctx.fillRect(halfL * 1.08, halfW * 0.15 - 1.8, 1.8, 1.8);

      // Drooping Ears
      ctx.fillStyle = '#9a3412';
      ctx.beginPath();
      ctx.ellipse(halfL * 0.65, halfW * 0.58, halfL * 0.18, halfW * 0.22, 0.6, 0, 2 * Math.PI);
      ctx.fill();
      ctx.beginPath();
      ctx.ellipse(halfL * 0.65, -halfW * 0.58, halfL * 0.18, halfW * 0.22, -0.6, 0, 2 * Math.PI);
      ctx.fill();

      // 4. Curved Ivory Horns with Dark Tips
      ctx.strokeStyle = '#f8fafc';
      ctx.lineWidth = 2.2;
      ctx.beginPath();
      ctx.moveTo(halfL * 0.65, halfW * 0.35);
      ctx.quadraticCurveTo(halfL * 0.95, halfW * 0.85, halfL * 0.8, halfW * 1.15);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(halfL * 0.65, -halfW * 0.35);
      ctx.quadraticCurveTo(halfL * 0.95, -halfW * 0.85, halfL * 0.8, -halfW * 1.15);
      ctx.stroke();

      ctx.fillStyle = '#1e293b';
      ctx.beginPath();
      ctx.arc(halfL * 0.8, halfW * 1.15, 1.6, 0, 2 * Math.PI);
      ctx.arc(halfL * 0.8, -halfW * 1.15, 1.6, 0, 2 * Math.PI);
      ctx.fill();

      // 5. Tail
      ctx.strokeStyle = '#78350f';
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.moveTo(-halfL * 0.75, 0);
      ctx.quadraticCurveTo(-halfL * 1.05, halfW * 0.35, -halfL * 1.25, halfW * 0.2);
      ctx.stroke();
      ctx.fillStyle = '#1e293b';
      ctx.beginPath();
      ctx.arc(-halfL * 1.25, halfW * 0.2, 2.2, 0, 2 * Math.PI);
      ctx.fill();
    },

    drawSheepSprite(ctx, s, L, W, isLaying = false) {
      const halfL = (L / 2) * s;
      const halfW = (W / 2) * s;

      // 1. Tiny dark hooves extending slightly (only when standing/grazing)
      if (!isLaying) {
        ctx.fillStyle = '#1e293b';
        // Front hooves
        ctx.fillRect(halfL * 0.35, halfW * 0.65, halfL * 0.18, halfW * 0.28);
        ctx.fillRect(halfL * 0.35, -halfW * 0.93, halfL * 0.18, halfW * 0.28);
        // Rear hooves
        ctx.fillRect(-halfL * 0.55, halfW * 0.65, halfL * 0.18, halfW * 0.28);
        ctx.fillRect(-halfL * 0.55, -halfW * 0.93, halfL * 0.18, halfW * 0.28);
      }

      // 2. Fluffy Woolly Body (Plump fleece with cloud-like scalloped wool tufts)
      // Wool base shadow
      ctx.fillStyle = '#cbd5e1';
      ctx.beginPath();
      const bodyWScale = isLaying ? 0.98 : 0.88;
      ctx.ellipse(-halfL * 0.05, 0, halfL * 0.82, halfW * bodyWScale, 0, 0, 2 * Math.PI);
      ctx.fill();

      // Fluffy wool puffs (overlapping ivory/cream clouds)
      ctx.fillStyle = '#f8fafc';
      ctx.strokeStyle = '#e2e8f0';
      ctx.lineWidth = 1.0;

      const woolTufts = [
        { x: -halfL * 0.55, y: -halfW * 0.45, r: halfW * 0.45 },
        { x: -halfL * 0.55, y: halfW * 0.45,  r: halfW * 0.45 },
        { x: -halfL * 0.65, y: 0,              r: halfW * 0.48 },
        { x: -halfL * 0.15, y: -halfW * 0.55, r: halfW * 0.46 },
        { x: -halfL * 0.15, y: halfW * 0.55,  r: halfW * 0.46 },
        { x: -halfL * 0.10, y: 0,              r: halfW * 0.55 },
        { x: halfL * 0.30,  y: -halfW * 0.45, r: halfW * 0.44 },
        { x: halfL * 0.30,  y: halfW * 0.45,  r: halfW * 0.44 },
        { x: halfL * 0.35,  y: 0,              r: halfW * 0.50 }
      ];

      woolTufts.forEach(tuft => {
        ctx.beginPath();
        ctx.arc(tuft.x, tuft.y, tuft.r, 0, 2 * Math.PI);
        ctx.fill();
        ctx.stroke();
      });

      // Fluffy wool tail
      ctx.fillStyle = '#f1f5f9';
      ctx.beginPath();
      ctx.arc(-halfL * 0.85, 0, halfW * 0.30, 0, 2 * Math.PI);
      ctx.fill();

      // 3. Black/Charcoal Sheep Head & Snout (classic Suffolk / Indian indigenous sheep)
      const headOffsetY = isLaying ? halfW * 0.10 : 0;
      ctx.fillStyle = '#1e293b';
      ctx.beginPath();
      ctx.ellipse(halfL * 0.72, headOffsetY, halfL * 0.35, halfW * 0.42, 0, 0, 2 * Math.PI);
      ctx.fill();

      // Muzzle / pinkish gray nose
      ctx.fillStyle = '#475569';
      ctx.beginPath();
      ctx.ellipse(halfL * 0.98, headOffsetY, halfL * 0.12, halfW * 0.24, 0, 0, 2 * Math.PI);
      ctx.fill();

      // Nostrils
      ctx.fillStyle = '#0f172a';
      ctx.fillRect(halfL * 1.02, headOffsetY - halfW * 0.10, 1.2, 1.2);
      ctx.fillRect(halfL * 1.02, headOffsetY + halfW * 0.05, 1.2, 1.2);

      // Floppy cute ears
      ctx.fillStyle = '#0f172a';
      ctx.beginPath();
      ctx.ellipse(halfL * 0.60, headOffsetY + halfW * 0.60, halfL * 0.18, halfW * 0.22, 0.7, 0, 2 * Math.PI);
      ctx.fill();
      ctx.beginPath();
      ctx.ellipse(halfL * 0.60, headOffsetY - halfW * 0.60, halfL * 0.18, halfW * 0.22, -0.7, 0, 2 * Math.PI);
      ctx.fill();

      // Head wool crown (forelock puff)
      ctx.fillStyle = '#f8fafc';
      ctx.beginPath();
      ctx.arc(halfL * 0.55, headOffsetY, halfW * 0.30, 0, 2 * Math.PI);
      ctx.fill();
    },

    drawMotorcycleSprite(ctx, s, L, W) {
      const halfL = (L / 2) * s;
      const halfW = (W / 2) * s;

      // Spoked Tires
      ctx.fillStyle = '#0f172a';
      ctx.fillRect(halfL * 0.55, -halfW * 0.22, halfL * 0.45, halfW * 0.44);
      ctx.fillRect(-halfL * 0.95, -halfW * 0.22, halfL * 0.45, halfW * 0.44);

      // Chassis & Fuel Tank
      ctx.fillStyle = '#4f46e5';
      ctx.beginPath();
      ctx.ellipse(halfL * 0.15, 0, halfL * 0.35, halfW * 0.45, 0, 0, 2 * Math.PI);
      ctx.fill();

      // Chrome exhaust
      ctx.strokeStyle = '#cbd5e1';
      ctx.lineWidth = 2.0;
      ctx.beginPath();
      ctx.moveTo(halfL * 0.1, -halfW * 0.55);
      ctx.lineTo(-halfL * 0.9, -halfW * 0.55);
      ctx.stroke();

      // Handlebars & Mirrors
      ctx.strokeStyle = '#1e293b';
      ctx.lineWidth = 2.2;
      ctx.beginPath();
      ctx.moveTo(halfL * 0.45, -halfW * 0.85);
      ctx.lineTo(halfL * 0.45, halfW * 0.85);
      ctx.stroke();

      ctx.fillStyle = '#94a3b8';
      ctx.beginPath();
      ctx.arc(halfL * 0.52, -halfW * 0.95, 1.8, 0, 2 * Math.PI);
      ctx.arc(halfL * 0.52, halfW * 0.95, 1.8, 0, 2 * Math.PI);
      ctx.fill();

      // Rider Torso
      ctx.fillStyle = '#1e293b';
      ctx.beginPath();
      ctx.ellipse(-halfL * 0.1, 0, halfL * 0.35, halfW * 0.65, 0, 0, 2 * Math.PI);
      ctx.fill();

      // Safety Helmet
      ctx.fillStyle = '#38bdf8';
      ctx.strokeStyle = '#0284c7';
      ctx.lineWidth = 1.0;
      ctx.beginPath();
      ctx.arc(0, 0, halfW * 0.45, 0, 2 * Math.PI);
      ctx.fill();
      ctx.stroke();

      // Visor
      ctx.fillStyle = '#0f172a';
      ctx.beginPath();
      ctx.arc(0, 0, halfW * 0.45, -Math.PI * 0.25, Math.PI * 0.25);
      ctx.fill();

      // Headlamp
      ctx.fillStyle = '#fef08a';
      ctx.beginPath();
      ctx.arc(halfL, 0, 2.2, 0, 2 * Math.PI);
      ctx.fill();
    },

    drawPedestrianSprite(ctx, s, L, W, ag) {
      const r = (Math.max(L, W) / 2) * s;
      const isCrossing = (ag.behavior_state || '').includes('CROSS');

      ctx.fillStyle = isCrossing ? '#ef4444' : '#10b981';
      ctx.beginPath();
      ctx.ellipse(0, 0, r * 0.9, r * 1.35, 0, 0, 2 * Math.PI);
      ctx.fill();

      ctx.fillStyle = '#1e293b';
      ctx.beginPath();
      ctx.arc(0, 0, r * 0.72, 0, 2 * Math.PI);
      ctx.fill();

      ctx.strokeStyle = '#fed7aa';
      ctx.lineWidth = 2.0;
      ctx.beginPath();
      ctx.moveTo(-r * 0.2, -r * 1.2);
      ctx.lineTo(r * 0.6, -r * 0.9);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(-r * 0.2, r * 1.2);
      ctx.lineTo(-r * 0.6, r * 0.9);
      ctx.stroke();
    },

    drawCarSprite(ctx, s, L, W, isLead, isOncoming) {
      const halfL = (L / 2) * s;
      const halfW = (W / 2) * s;

      const carCol = isLead ? '#3b82f6' : (isOncoming ? '#f97316' : '#64748b');
      ctx.fillStyle = carCol;
      ctx.strokeStyle = '#1e293b';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.roundRect(-halfL, -halfW, L * s, W * s, 4);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = 'rgba(15, 23, 42, 0.85)';
      ctx.beginPath();
      ctx.roundRect(-halfL * 0.55, -halfW * 0.8, halfL * 1.1, halfW * 1.6, 3);
      ctx.fill();

      ctx.fillStyle = carCol;
      ctx.fillRect(-halfL * 0.25, -halfW * 0.7, halfL * 0.7, halfW * 1.4);

      ctx.fillStyle = 'rgba(56, 189, 248, 0.45)';
      ctx.beginPath();
      ctx.moveTo(halfL * 0.5, -halfW * 0.75);
      ctx.lineTo(halfL * 0.22, -halfW * 0.7);
      ctx.lineTo(halfL * 0.22, halfW * 0.7);
      ctx.lineTo(halfL * 0.5, halfW * 0.75);
      ctx.closePath();
      ctx.fill();

      ctx.fillStyle = '#fef08a';
      ctx.fillRect(halfL - 2, halfW * 0.55, 2.5, halfW * 0.35);
      ctx.fillRect(halfL - 2, -halfW * 0.9, 2.5, halfW * 0.35);

      ctx.fillStyle = '#ef4444';
      ctx.fillRect(-halfL, halfW * 0.6, 2, halfW * 0.3);
      ctx.fillRect(-halfL, -halfW * 0.9, 2, halfW * 0.3);
    },

    drawTractorSprite(ctx, s, L, W) {
      const halfL = (L / 2) * s;
      const halfW = (W / 2) * s;

      // Heavy Indian Agricultural Tractor (Mahindra Red / Sonalika Blue)
      // 1. Rear Huge Mudguards & Tires
      ctx.fillStyle = '#0f172a';
      ctx.fillRect(-halfL * 0.4, -halfW, halfL * 0.7, halfW * 0.38);
      ctx.fillRect(-halfL * 0.4, halfW * 0.62, halfL * 0.7, halfW * 0.38);

      // Deep tread ribs on rear tires
      ctx.fillStyle = '#334155';
      for (let tx = -halfL * 0.35; tx <= halfL * 0.25; tx += 4) {
        ctx.fillRect(tx, -halfW, 2, halfW * 0.38);
        ctx.fillRect(tx, halfW * 0.62, 2, halfW * 0.38);
      }

      // 2. Front Smaller Steer Tires
      ctx.fillStyle = '#0f172a';
      ctx.fillRect(halfL * 0.5, -halfW * 0.85, halfL * 0.35, halfW * 0.28);
      ctx.fillRect(halfL * 0.5, halfW * 0.57, halfL * 0.35, halfW * 0.28);

      // 3. Engine Hood & Body (Vibrant Indian Agricultural Crimson Red)
      ctx.fillStyle = '#dc2626';
      ctx.strokeStyle = '#991b1b';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.roundRect(-halfL * 0.1, -halfW * 0.55, halfL * 0.95, halfW * 1.1, 4);
      ctx.fill();
      ctx.stroke();

      // Front Grill
      ctx.fillStyle = '#475569';
      ctx.fillRect(halfL * 0.8, -halfW * 0.45, halfL * 0.05, halfW * 0.9);

      // Headlamps
      ctx.fillStyle = '#fef08a';
      ctx.beginPath();
      ctx.arc(halfL * 0.82, -halfW * 0.35, 2.5, 0, 2 * Math.PI);
      ctx.arc(halfL * 0.82, halfW * 0.35, 2.5, 0, 2 * Math.PI);
      ctx.fill();

      // 4. Driver Open Platform & Seat
      ctx.fillStyle = '#1e293b';
      ctx.fillRect(-halfL * 0.6, -halfW * 0.5, halfL * 0.5, halfW * 1.0);

      ctx.fillStyle = '#0f172a';
      ctx.fillRect(-halfL * 0.5, -halfW * 0.25, halfL * 0.25, halfW * 0.5);

      // Steering Wheel
      ctx.strokeStyle = '#cbd5e1';
      ctx.lineWidth = 1.8;
      ctx.beginPath();
      ctx.arc(-halfL * 0.15, 0, halfW * 0.22, 0, 2 * Math.PI);
      ctx.stroke();

      // Vertical Exhaust Smokestack
      ctx.fillStyle = '#0f172a';
      ctx.beginPath();
      ctx.arc(halfL * 0.4, -halfW * 0.42, 3, 0, 2 * Math.PI);
      ctx.fill();

      // 5. Heavy Hitch / Trailer (if length > 5m)
      if (L > 5.0) {
        ctx.fillStyle = '#78350f';
        ctx.strokeStyle = '#451a03';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.roundRect(-halfL * 0.98, -halfW * 0.9, halfL * 0.36, halfW * 1.8, 2);
        ctx.fill();
        ctx.stroke();

        ctx.strokeStyle = '#64748b';
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(-halfL * 0.62, 0);
        ctx.lineTo(-halfL * 0.4, 0);
        ctx.stroke();
      }
    },

    drawGroundTruthAgents(agents) {
      const ctx = this.ctx;
      const s = this.scale;
      const list = Array.isArray(agents) ? agents : [agents];

      list.forEach(ag => {
        if (!ag || ag.x === undefined || ag.x < -50) return;
        const L = ag.length || 4.7;
        const W = ag.width || 1.8;
        const heading = (ag.heading !== undefined) ? ag.heading : Math.atan2(ag.vy || 0, (ag.vx || 0) + 1e-6);
        const type = (ag.type || 'car').toLowerCase();

        ctx.save();
        ctx.translate(ag.x * s, ag.y * s);
        ctx.rotate(heading);

        if (type === 'auto' || type === 'autorickshaw') {
          this.drawAutoRickshawSprite(ctx, s, L, W);
        } else if (type === 'sheep' || (ag.id_str && ag.id_str.includes('SHEEP'))) {
          const isLaying = (ag.id % 2 === 0);
          this.drawSheepSprite(ctx, s, L, W, isLaying);
        } else if (type === 'cattle' || type === 'cow') {
          this.drawCattleSprite(ctx, s, L, W);
        } else if (type === 'bike' || type === 'motorcycle') {
          this.drawMotorcycleSprite(ctx, s, L, W);
        } else if (type === 'pedestrian' || type === 'ped') {
          this.drawPedestrianSprite(ctx, s, L, W, ag);
        } else if (type === 'truck' || (ag.id_str && ag.id_str.includes('TRACTOR'))) {
          this.drawTractorSprite(ctx, s, L, W);
        } else {
          const isLead = (ag.id_str === 'CAR_LEAD');
          const isOncoming = (ag.id_str && ag.id_str.includes('ONCOMING'));
          this.drawCarSprite(ctx, s, L, W, isLead, isOncoming);
        }

        ctx.restore();

        // Agent State Label
        ctx.save();
        ctx.scale(1, -1);
        ctx.fillStyle = '#e2e8f0';
        ctx.font = 'bold 10px "JetBrains Mono", monospace';
        const labelText = ag.id_str || `A${ag.id}`;
        ctx.fillText(labelText, ag.x * s - 12, -ag.y * s - (W / 2) * s - 5);
        ctx.restore();
      });
    },

    drawEgoVehicle(ego) {
      const ctx = this.ctx;
      const s = this.scale;
      const L = (this.data.metadata ? this.data.metadata.vehicle_length : 4.7);
      const W = (this.data.metadata ? this.data.metadata.vehicle_width : 1.8);

      // Check if traversing speed breaker (x in [86.2, 90.2m])
      const isOverBump = (ego.x >= 86.2 && ego.x <= 90.2);
      let bumpPitch = 0;
      if (isOverBump) {
        const dxBump = ego.x - 88.0;
        const frontImpulse = Math.max(0, 1 - Math.abs(dxBump + 1.2) / 0.85);
        const rearImpulse = Math.max(0, 1 - Math.abs(dxBump - 1.2) / 0.85);
        bumpPitch = (frontImpulse - rearImpulse) * 0.08; // dynamic pitch tilt over bump
      }

      // Ego Local Perception Sensing Window (Forward 50m, Rear 15m, Lateral 6m)
      ctx.save();
      ctx.translate(ego.x * s, ego.y * s);
      ctx.rotate((ego.theta || 0) + bumpPitch);
      const fwdRange = 50.0;
      const rearRange = 15.0;
      const latRange = 6.0;
      ctx.fillStyle = 'rgba(56, 189, 248, 0.04)';
      ctx.strokeStyle = 'rgba(56, 189, 248, 0.28)';
      ctx.lineWidth = 1;
      ctx.setLineDash([4, 4]);
      ctx.fillRect(-rearRange * s, -latRange * s, (fwdRange + rearRange) * s, 2 * latRange * s);
      ctx.strokeRect(-rearRange * s, -latRange * s, (fwdRange + rearRange) * s, 2 * latRange * s);
      ctx.setLineDash([]);

      const halfL = (L / 2) * s;
      const halfW = (W / 2) * s;

      // Realistic Autonomous Vehicle Body (Metallic Blue)
      ctx.fillStyle = '#0284c7';
      ctx.strokeStyle = '#0369a1';
      ctx.lineWidth = 1.8;
      ctx.beginPath();
      ctx.roundRect(-halfL, -halfW, L * s, W * s, 5);
      ctx.fill();
      ctx.stroke();

      // Dark Glass Roof & Cabin Greenhouse
      ctx.fillStyle = '#0f172a';
      ctx.beginPath();
      ctx.roundRect(-halfL * 0.5, -halfW * 0.8, halfL * 1.05, halfW * 1.6, 3);
      ctx.fill();

      // Autonomous Roof Sensor Pod (LiDAR Puck)
      ctx.fillStyle = '#38bdf8';
      ctx.strokeStyle = '#0284c7';
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.arc(0, 0, halfW * 0.32, 0, 2 * Math.PI);
      ctx.fill();
      ctx.stroke();

      // Forward Headlamps Beam
      ctx.fillStyle = '#fef08a';
      ctx.fillRect(halfL - 2, halfW * 0.6, 3, halfW * 0.3);
      ctx.fillRect(halfL - 2, -halfW * 0.9, 3, halfW * 0.3);

      // Red Tail Lamps
      ctx.fillStyle = '#ef4444';
      ctx.fillRect(-halfL, halfW * 0.65, 2.5, halfW * 0.25);
      ctx.fillRect(-halfL, -halfW * 0.9, 2.5, halfW * 0.25);

      // Heading Vector Line
      ctx.strokeStyle = '#38bdf8';
      ctx.lineWidth = 2.2;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo((halfL + 18), 0);
      ctx.stroke();

      ctx.restore();

      // Ego Speed & Position Label
      ctx.save();
      ctx.scale(1, -1);
      ctx.fillStyle = '#38bdf8';
      ctx.font = 'bold 11px "JetBrains Mono", monospace';
      ctx.fillText(`EGO (${ego.v ? ego.v.toFixed(1) : '0.0'} m/s)`, ego.x * s - 25, -ego.y * s - W/2 * s - 7);

      if (isOverBump) {
        ctx.fillStyle = '#facc15';
        ctx.font = 'bold 10px "JetBrains Mono", monospace';
        ctx.fillText(`⚡ SPEED BUMP CRAWL (${ego.v ? ego.v.toFixed(1) : '2.0'} m/s)`, ego.x * s - 48, -ego.y * s - W/2 * s - 20);
      }
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
      const clr = step.groundTruth && step.groundTruth.min_clearance !== undefined ? step.groundTruth.min_clearance.toFixed(2) : '—';
      const nObs = (step.observation && step.observation.agents) ? step.observation.agents.length : 0;
      const nWorld = (step.groundTruth && step.groundTruth.agents) ? step.groundTruth.agents.length : 0;

      ctx.fillText(`Macro-Intent: ${intent} | Clearance: ${clr}m`, 20, this.canvas.height - 45);
      ctx.fillText(`Ego Speed: ${egoV} m/s | Min TTC: ${minTtc}s`, 20, this.canvas.height - 28);
      ctx.fillText(`Observed Agents: ${nObs} in Frame (World Active: ${nWorld})`, 20, this.canvas.height - 11);
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
      const isHardQP = !mpc.is_soft;
      const statusStr = isHardQP ? `SUCCESS (Status ${mpc.status !== undefined ? mpc.status : 1})` : `SOFT FALLBACK (${mpc.status})`;
      const cmd = mpc.u_cmd || [0, 0];

      return `
        <div class="panel-section">
          <div class="panel-header">
            <h2>🎯 QP-MPC Trajectory Solver</h2>
            <span class="panel-badge ${isHardQP ? 'safe-tag' : 'override-tag'}">${isHardQP ? 'Hard QP' : 'Soft QP'}</span>
          </div>
          <div class="panel-body">
            <div class="panel-row">
              <span class="panel-key">Hildreth Solver Status</span>
              <span class="panel-val ${isHardQP ? 'safe-tag' : 'override-tag'}">${statusStr}</span>
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
