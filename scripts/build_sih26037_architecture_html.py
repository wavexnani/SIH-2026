import os

def build_html():
    html = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>SIH26037 — Autonomous Vehicle Supervisory Architecture</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700;800;900&family=JetBrains+Mono:wght@400;500;600;700&display=swap" rel="stylesheet">
<style>
  :root {
    --bg-dark: #070a12;
    --bg-card: #0f172a;
    --bg-card-hover: #1e293b;
    --border-dim: #1e293b;
    --border-bright: #334155;
    --text-main: #f8fafc;
    --text-sub: #94a3b8;
    --text-muted: #64748b;
    
    --green-bright: #10b981;
    --green-bg: rgba(16, 185, 129, 0.12);
    --green-border: rgba(16, 185, 129, 0.4);
    
    --amber-bright: #f59e0b;
    --amber-bg: rgba(245, 158, 11, 0.12);
    --amber-border: rgba(245, 158, 11, 0.4);
    
    --cyan-bright: #38bdf8;
    --cyan-bg: rgba(56, 189, 248, 0.12);
    --cyan-border: rgba(56, 189, 248, 0.4);
    
    --rose-bright: #f43f5e;
    --rose-bg: rgba(244, 63, 94, 0.12);
    --rose-border: rgba(244, 63, 94, 0.4);
    
    --purple-bright: #a855f7;
    --purple-bg: rgba(168, 85, 247, 0.12);
    --purple-border: rgba(168, 85, 247, 0.4);
  }

  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background-color: var(--bg-dark);
    color: var(--text-main);
    font-family: 'Inter', system-ui, -apple-system, sans-serif;
    line-height: 1.6;
    padding: 32px 24px;
    background-image: 
      radial-gradient(circle at 15% 15%, rgba(56, 189, 248, 0.05) 0%, transparent 40%),
      radial-gradient(circle at 85% 85%, rgba(168, 85, 247, 0.05) 0%, transparent 40%);
  }
  
  .container { max-width: 1680px; margin: 0 auto; }
  
  /* Sticky Section Nav Bar */
  .nav-bar {
    position: sticky;
    top: 16px;
    z-index: 100;
    background: rgba(15, 23, 42, 0.85);
    backdrop-filter: blur(12px);
    border: 1px solid var(--border-bright);
    border-radius: 50px;
    padding: 8px 16px;
    display: flex;
    gap: 8px;
    overflow-x: auto;
    margin-bottom: 32px;
    box-shadow: 0 10px 30px rgba(0,0,0,0.5);
  }
  .nav-item {
    color: var(--text-sub);
    text-decoration: none;
    font-size: 12px;
    font-weight: 600;
    padding: 6px 14px;
    border-radius: 20px;
    white-space: nowrap;
    transition: all 0.2s ease;
  }
  .nav-item:hover {
    color: var(--text-main);
    background: rgba(255,255,255,0.08);
  }
  .nav-item.active {
    background: var(--cyan-bright);
    color: var(--bg-dark);
    font-weight: 700;
  }

  /* Header */
  header.hero-header {
    background: linear-gradient(135deg, #0f172a 0%, #1e1b4b 100%);
    border: 1px solid var(--border-bright);
    border-radius: 20px;
    padding: 40px;
    margin-bottom: 32px;
    position: relative;
    overflow: hidden;
    box-shadow: 0 20px 40px rgba(0,0,0,0.4);
  }
  header.hero-header::before {
    content: '';
    position: absolute;
    top: 0; right: 0; width: 400px; height: 100%;
    background: radial-gradient(circle, rgba(56, 189, 248, 0.15) 0%, transparent 70%);
    pointer-events: none;
  }
  .hero-tag {
    display: inline-flex;
    align-items: center;
    gap: 8px;
    background: var(--cyan-bg);
    border: 1px solid var(--cyan-border);
    color: var(--cyan-bright);
    font-size: 12px;
    font-weight: 700;
    padding: 4px 12px;
    border-radius: 20px;
    letter-spacing: 0.5px;
    text-transform: uppercase;
    margin-bottom: 16px;
  }
  .hero-title {
    font-size: 36px;
    font-weight: 900;
    letter-spacing: -1px;
    background: linear-gradient(180deg, #ffffff 0%, #cbd5e1 100%);
    -webkit-background-clip: text;
    -webkit-text-fill-color: transparent;
    margin-bottom: 8px;
  }
  .hero-subtitle {
    font-size: 18px;
    font-weight: 500;
    color: var(--cyan-bright);
    margin-bottom: 20px;
  }
  .hero-meta {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(240px, 1fr));
    gap: 16px;
    border-top: 1px solid rgba(255,255,255,0.1);
    padding-top: 20px;
  }
  .meta-box { font-size: 13px; color: var(--text-sub); }
  .meta-box strong { color: var(--text-main); font-family: 'JetBrains Mono', monospace; font-size: 12px; }

  /* Legend */
  .legend-bar {
    background: var(--bg-card);
    border: 1px solid var(--border-dim);
    border-radius: 16px;
    padding: 20px 28px;
    margin-bottom: 32px;
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    justify-content: space-between;
    gap: 16px;
  }
  .legend-title { font-size: 13px; font-weight: 800; text-transform: uppercase; letter-spacing: 1px; color: var(--text-sub); }
  .legend-items { display: flex; flex-wrap: wrap; gap: 16px; }
  .legend-pill {
    display: flex;
    align-items: center;
    gap: 8px;
    font-size: 12px;
    font-weight: 600;
    padding: 6px 14px;
    border-radius: 30px;
  }
  .pill-green { background: var(--green-bg); border: 1px solid var(--green-border); color: var(--green-bright); }
  .pill-amber { background: var(--amber-bg); border: 1px solid var(--amber-border); color: var(--amber-bright); }
  .pill-cyan { background: var(--cyan-bg); border: 1px dashed var(--cyan-border); color: var(--cyan-bright); }
  .pill-rose { background: var(--rose-bg); border: 1px solid var(--rose-border); color: var(--rose-bright); }
  .pill-purple { background: var(--purple-bg); border: 1px solid var(--purple-border); color: var(--purple-bright); }

  /* Card Containers */
  .section-card {
    background: var(--bg-card);
    border: 1px solid var(--border-dim);
    border-radius: 20px;
    padding: 32px;
    margin-bottom: 32px;
    transition: border-color 0.3s ease, box-shadow 0.3s ease;
  }
  .section-card:hover {
    border-color: var(--border-bright);
    box-shadow: 0 10px 30px rgba(0,0,0,0.3);
  }
  .section-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    margin-bottom: 24px;
    padding-bottom: 16px;
    border-bottom: 1px solid var(--border-dim);
  }
  .section-title {
    font-size: 20px;
    font-weight: 800;
    display: flex;
    align-items: center;
    gap: 12px;
  }
  .section-num {
    background: var(--cyan-bg);
    color: var(--cyan-bright);
    border: 1px solid var(--cyan-border);
    font-family: 'JetBrains Mono', monospace;
    font-size: 13px;
    font-weight: 700;
    padding: 2px 10px;
    border-radius: 6px;
  }

  /* Grid Layouts */
  .grid-2 { display: grid; grid-template-columns: repeat(auto-fit, minmax(600px, 1fr)); gap: 24px; }
  .grid-3 { display: grid; grid-template-columns: repeat(auto-fit, minmax(420px, 1fr)); gap: 20px; }
  .grid-4 { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 16px; }

  /* Sub Cards */
  .box-card {
    background: rgba(30, 41, 59, 0.4);
    border: 1px solid var(--border-dim);
    border-radius: 14px;
    padding: 20px;
    position: relative;
  }
  .box-card.green { border-left: 4px solid var(--green-bright); }
  .box-card.amber { border-left: 4px solid var(--amber-bright); }
  .box-card.cyan { border-left: 4px solid var(--cyan-bright); }
  .box-card.rose { border-left: 4px solid var(--rose-bright); }
  .box-card.purple { border-left: 4px solid var(--purple-bright); }

  .box-title {
    font-size: 15px;
    font-weight: 700;
    margin-bottom: 10px;
    display: flex;
    align-items: center;
    justify-content: space-between;
  }

  /* Code & Mono Text */
  code, pre {
    font-family: 'JetBrains Mono', monospace;
    font-size: 12px;
  }
  code {
    background: rgba(15, 23, 42, 0.8);
    color: var(--cyan-bright);
    padding: 2px 6px;
    border-radius: 4px;
    border: 1px solid rgba(56, 189, 248, 0.2);
  }
  pre {
    background: #090d16;
    border: 1px solid var(--border-dim);
    padding: 16px;
    border-radius: 10px;
    color: #e2e8f0;
    overflow-x: auto;
    line-height: 1.5;
  }

  /* Tables */
  .table-responsive { width: 100%; overflow-x: auto; border-radius: 12px; border: 1px solid var(--border-dim); }
  table.dark-table {
    width: 100%;
    border-collapse: collapse;
    font-size: 13px;
    text-align: left;
  }
  table.dark-table th {
    background: #1e293b;
    color: var(--text-main);
    font-weight: 700;
    padding: 14px 18px;
    border-bottom: 2px solid var(--border-bright);
    white-space: nowrap;
  }
  table.dark-table td {
    padding: 14px 18px;
    border-bottom: 1px solid var(--border-dim);
    color: var(--text-sub);
    vertical-align: top;
  }
  table.dark-table tr:hover td {
    background: rgba(255,255,255,0.02);
  }

  /* Badge Pills */
  .badge {
    display: inline-block;
    padding: 3px 8px;
    border-radius: 6px;
    font-size: 10px;
    font-weight: 700;
    text-transform: uppercase;
    letter-spacing: 0.5px;
    font-family: 'JetBrains Mono', monospace;
  }
  .b-green { background: var(--green-bg); color: var(--green-bright); border: 1px solid var(--green-border); }
  .b-amber { background: var(--amber-bg); color: var(--amber-bright); border: 1px solid var(--amber-border); }
  .b-cyan { background: var(--cyan-bg); color: var(--cyan-bright); border: 1px dashed var(--cyan-border); }
  .b-rose { background: var(--rose-bg); color: var(--rose-bright); border: 1px solid var(--rose-border); }

  /* Callout Alerts */
  .alert-card {
    padding: 18px 22px;
    border-radius: 12px;
    margin-bottom: 20px;
    font-size: 13.5px;
    display: flex;
    gap: 14px;
    align-items: flex-start;
  }
  .alert-warning { background: var(--amber-bg); border: 1px solid var(--amber-border); color: #fde68a; }
  .alert-info { background: var(--cyan-bg); border: 1px solid var(--cyan-border); color: #e0f2fe; }
  .alert-danger { background: var(--rose-bg); border: 1px solid var(--rose-border); color: #fecdd3; }

  /* SVG Wrapper */
  .svg-container {
    background: #090d16;
    border: 1px solid var(--border-dim);
    border-radius: 14px;
    padding: 16px;
    overflow-x: auto;
  }
</style>
</head>
<body>
<div class="container">

  <!-- STICKY NAV -->
  <nav class="nav-bar">
    <a href="#section-svg" class="nav-item active">1. System Architecture SVG</a>
    <a href="#section-hsma" class="nav-item">2. Supervisory Tree (HSMA)</a>
    <a href="#section-init" class="nav-item">3. Initialization</a>
    <a href="#section-states" class="nav-item">4. Active States Detail</a>
    <a href="#section-monitors" class="nav-item">5. Parallel Monitors</a>
    <a href="#section-guards" class="nav-item">6. Guarded Transitions</a>
    <a href="#section-dataflow" class="nav-item">7. Data-Flow Map</a>
    <a href="#section-gap" class="nav-item">8. Gap Map (Current vs Proposed)</a>
    <a href="#section-unstructured" class="nav-item">9. Road-User & Unstructured Model</a>
    <a href="#section-rationale" class="nav-item">10. Design Rationales</a>
    <a href="#section-validation" class="nav-item">11. Validation Plan</a>
    <a href="#section-ppt" class="nav-item">12. Slide 2 PPT Blueprint</a>
    <a href="#section-status" class="nav-item">13. Engineering Audit Checklist</a>
    <a href="#section-summary" class="nav-item">14. Final Summary</a>
  </nav>

  <!-- HERO HEADER -->
  <header class="hero-header">
    <div class="hero-tag">SIH26037 Master Technical Specification</div>
    <h1 class="hero-title">Hierarchical Stateflow / Supervisory Architecture</h1>
    <div class="hero-subtitle">Adaptive Closed-Loop Path Planning & Collision Avoidance for Unstructured Indian Roads</div>
    <div class="hero-meta">
      <div class="meta-box">Executable Controller: <strong>planning/Stage5CoordinationController.m</strong></div>
      <div class="meta-box">Supervisory Chart: <strong>SIH26037_SupervisoryArchitecture.slx</strong></div>
      <div class="meta-box">Optimization Engine: <strong>CACRCPlanner.m / QPMPCPlanner.m</strong></div>
    </div>
  </header>

  <!-- LEGEND BAR -->
  <div class="legend-bar">
    <div class="legend-title">System Status Taxonomy</div>
    <div class="legend-items">
      <div class="legend-pill pill-green">🟢 CURRENTLY IMPLEMENTED & EXECUTED</div>
      <div class="legend-pill pill-amber">🟡 PARTIALLY IMPLEMENTED / DISCARDED</div>
      <div class="legend-pill pill-cyan">🔵 PROPOSED EXTENSION (SIH COMPLETE)</div>
      <div class="legend-pill pill-rose">🔴 FAILURE / SAFETY OVERRIDE</div>
      <div class="legend-pill pill-purple">🟣 ENVIRONMENT / SIMULATION</div>
    </div>
  </div>

  <!-- SECTION 1: FULL ARCHITECTURE SVG DIAGRAM -->
  <section class="section-card" id="section-svg">
    <div class="section-header">
      <div class="section-title"><span class="section-num">01</span> Full Closed-Loop System Architecture</div>
      <span class="badge b-cyan">SVG Interactive Map</span>
    </div>
    <div class="svg-container">
      <svg viewBox="0 0 1500 840" style="width:100%; height:auto;">
        <defs>
          <linearGradient id="grad-env" x1="0%" y1="0%" x2="0%" y2="100%">
            <stop offset="0%" stop-color="#1e1b4b" stop-opacity="0.6"/>
            <stop offset="100%" stop-color="#0f172a" stop-opacity="0.8"/>
          </linearGradient>
          <linearGradient id="grad-percept" x1="0%" y1="0%" x2="0%" y2="100%">
            <stop offset="0%" stop-color="#064e3b" stop-opacity="0.4"/>
            <stop offset="100%" stop-color="#0f172a" stop-opacity="0.8"/>
          </linearGradient>
          <linearGradient id="grad-planner" x1="0%" y1="0%" x2="0%" y2="100%">
            <stop offset="0%" stop-color="#0c4a6e" stop-opacity="0.5"/>
            <stop offset="100%" stop-color="#0f172a" stop-opacity="0.8"/>
          </linearGradient>
          <linearGradient id="grad-safety" x1="0%" y1="0%" x2="0%" y2="100%">
            <stop offset="0%" stop-color="#4c0519" stop-opacity="0.5"/>
            <stop offset="100%" stop-color="#0f172a" stop-opacity="0.8"/>
          </linearGradient>

          <marker id="arr-green" markerWidth="8" markerHeight="8" refX="6" refY="4" orient="auto">
            <path d="M0,0 L8,4 L0,8 Z" fill="#10b981"/>
          </marker>
          <marker id="arr-cyan" markerWidth="8" markerHeight="8" refX="6" refY="4" orient="auto">
            <path d="M0,0 L8,4 L0,8 Z" fill="#38bdf8"/>
          </marker>
          <marker id="arr-rose" markerWidth="8" markerHeight="8" refX="6" refY="4" orient="auto">
            <path d="M0,0 L8,4 L0,8 Z" fill="#f43f5e"/>
          </marker>
          <marker id="arr-purple" markerWidth="8" markerHeight="8" refX="6" refY="4" orient="auto">
            <path d="M0,0 L8,4 L0,8 Z" fill="#a855f7"/>
          </marker>
        </defs>

        <!-- Background Columns -->
        <!-- Environment Zone -->
        <rect x="20" y="20" width="220" height="800" rx="12" fill="url(#grad-env)" stroke="#4c1d95" stroke-width="1.5"/>
        <text x="130" y="50" fill="#c084fc" font-size="12" font-weight="800" text-anchor="middle" letter-spacing="1">ENVIRONMENT & WORLD</text>

        <!-- Perception Zone -->
        <rect x="260" y="20" width="280" height="800" rx="12" fill="url(#grad-percept)" stroke="#059669" stroke-width="1.5"/>
        <text x="400" y="50" fill="#34d399" font-size="12" font-weight="800" text-anchor="middle" letter-spacing="1">PERCEIVE, TRACK & PREDICT</text>

        <!-- Supervisory & Planner Zone -->
        <rect x="560" y="20" width="400" height="800" rx="12" fill="url(#grad-planner)" stroke="#0284c7" stroke-width="2"/>
        <text x="760" y="50" fill="#38bdf8" font-size="13" font-weight="900" text-anchor="middle" letter-spacing="1">SUPERVISORY & ADAPTIVE PLANNER</text>

        <!-- Safety & Actuator Zone -->
        <rect x="980" y="20" width="260" height="800" rx="12" fill="url(#grad-safety)" stroke="#e11d48" stroke-width="1.5"/>
        <text x="1110" y="50" fill="#fb7185" font-size="12" font-weight="800" text-anchor="middle" letter-spacing="1">SAFETY FILTER & ACTUATION</text>

        <!-- Vehicle Plant Zone -->
        <rect x="1260" y="20" width="220" height="800" rx="12" fill="url(#grad-env)" stroke="#475569" stroke-width="1.5"/>
        <text x="1370" y="50" fill="#94a3b8" font-size="12" font-weight="800" text-anchor="middle" letter-spacing="1">PLANT & CLOSED LOOP</text>

        <!-- ENVIRONMENT MODULES -->
        <g transform="translate(35, 80)">
          <rect width="190" height="70" rx="8" fill="#0f172a" stroke="#a855f7" stroke-width="1.5"/>
          <text x="95" y="28" fill="#f8fafc" font-size="12" font-weight="800" text-anchor="middle">ScenarioDefinitions</text>
          <text x="95" y="48" fill="#94a3b8" font-size="10" text-anchor="middle">Unstructured Road & Agents</text>
        </g>

        <g transform="translate(35, 175)">
          <rect width="190" height="70" rx="8" fill="#0f172a" stroke="#a855f7" stroke-width="1.5"/>
          <text x="95" y="28" fill="#f8fafc" font-size="12" font-weight="800" text-anchor="middle">WorldState</text>
          <text x="95" y="48" fill="#94a3b8" font-size="10" text-anchor="middle">Ground-Truth Telemetry</text>
        </g>

        <g transform="translate(35, 700)">
          <rect width="190" height="90" rx="8" fill="#0f172a" stroke="#38bdf8" stroke-width="1.5" stroke-dasharray="5,3"/>
          <text x="95" y="28" fill="#38bdf8" font-size="12" font-weight="800" text-anchor="middle">RoadRunner Platform</text>
          <text x="95" y="48" fill="#f59e0b" font-size="10" font-weight="700" text-anchor="middle">[Proposed Extension]</text>
          <text x="95" y="66" fill="#64748b" font-size="9" text-anchor="middle">3D Closed-Loop Co-Simulation</text>
        </g>

        <!-- PERCEPTION MODULES -->
        <g transform="translate(275, 80)">
          <rect width="250" height="80" rx="8" fill="#0f172a" stroke="#10b981" stroke-width="2"/>
          <text x="125" y="28" fill="#10b981" font-size="12" font-weight="900" text-anchor="middle">ObservationModel.m</text>
          <text x="125" y="48" fill="#e2e8f0" font-size="10" text-anchor="middle">Noise (&sigma;_pos, &sigma;_vel) & Delay (100ms)</text>
          <text x="125" y="64" fill="#94a3b8" font-size="9" text-anchor="middle">Output: obs_world, obs_structs</text>
        </g>

        <g transform="translate(275, 185)">
          <rect width="250" height="80" rx="8" fill="#0f172a" stroke="#10b981" stroke-width="2"/>
          <text x="125" y="28" fill="#10b981" font-size="12" font-weight="900" text-anchor="middle">MultiVehicleDetector.m</text>
          <text x="125" y="48" fill="#e2e8f0" font-size="10" text-anchor="middle">Relative Metrics (dx, dy, dvx, dvy)</text>
          <text x="125" y="64" fill="#94a3b8" font-size="9" text-anchor="middle">Static obs mapped to det (id &ge; 1000)</text>
        </g>

        <g transform="translate(275, 290)">
          <rect width="250" height="85" rx="8" fill="#0f172a" stroke="#10b981" stroke-width="2"/>
          <text x="125" y="28" fill="#10b981" font-size="12" font-weight="900" text-anchor="middle">RiskPredictor.m (TTC)</text>
          <text x="125" y="48" fill="#e2e8f0" font-size="10" text-anchor="middle">predictTTC(detections, L_ego)</text>
          <text x="125" y="66" fill="#94a3b8" font-size="9" text-anchor="middle">Output: ttc_vector, min_ttc</text>
        </g>

        <!-- DISCARDED CALLOUT -->
        <g transform="translate(275, 400)">
          <rect width="250" height="95" rx="8" fill="rgba(245, 158, 11, 0.1)" stroke="#f59e0b" stroke-width="2"/>
          <text x="125" y="24" fill="#f59e0b" font-size="11" font-weight="900" text-anchor="middle">⚠ RiskPredictor.predictTrajectories</text>
          <text x="125" y="42" fill="#cbd5e1" font-size="9.5" text-anchor="middle">Multi-step linear prediction preds</text>
          <rect x="15" y="52" width="220" height="30" rx="4" fill="rgba(244, 63, 94, 0.2)" stroke="#f43f5e" stroke-width="1"/>
          <text x="125" y="71" fill="#f43f5e" font-size="10" font-weight="900" text-anchor="middle">DISCARDED / UNUSED IN MPC LOOP</text>
        </g>

        <g transform="translate(275, 520)">
          <rect width="250" height="80" rx="8" fill="#0f172a" stroke="#10b981" stroke-width="2"/>
          <text x="125" y="28" fill="#10b981" font-size="12" font-weight="900" text-anchor="middle">UncertaintyPredictor.m</text>
          <text x="125" y="48" fill="#e2e8f0" font-size="10" text-anchor="middle">&Delta;y_unc = &alpha; &middot; k_&sigma; &middot; (&sigma;_ag + &sigma;_v &middot; &tau;)</text>
          <text x="125" y="64" fill="#94a3b8" font-size="9" text-anchor="middle">Output: y_blo_lo, y_blo_hi</text>
        </g>

        <g transform="translate(275, 625)">
          <rect width="250" height="80" rx="8" fill="#0f172a" stroke="#38bdf8" stroke-width="1.5" stroke-dasharray="5,3"/>
          <text x="125" y="28" fill="#38bdf8" font-size="12" font-weight="800" text-anchor="middle">Sensor Fusion (EKF / MOT)</text>
          <text x="125" y="48" fill="#f59e0b" font-size="10" font-weight="700" text-anchor="middle">[Proposed Extension]</text>
          <text x="125" y="64" fill="#64748b" font-size="9" text-anchor="middle">Multi-Object Tracking & Covariance</text>
        </g>

        <!-- SUPERVISORY & PLANNER MODULES -->
        <g transform="translate(585, 80)">
          <rect width="350" height="80" rx="8" fill="#0f172a" stroke="#10b981" stroke-width="2"/>
          <text x="175" y="28" fill="#10b981" font-size="12" font-weight="900" text-anchor="middle">InteractionClassifier.m</text>
          <text x="175" y="48" fill="#e2e8f0" font-size="10" text-anchor="middle">6 Threat Classes (Following, Closing, Oncoming, etc.)</text>
          <text x="175" y="64" fill="#94a3b8" font-size="9" text-anchor="middle">Output: interactions (class_name, risk_level, ttc)</text>
        </g>

        <g transform="translate(585, 185)">
          <rect width="350" height="110" rx="8" fill="#0f172a" stroke="#10b981" stroke-width="2"/>
          <text x="175" y="26" fill="#10b981" font-size="13" font-weight="900" text-anchor="middle">CoordinationDecisionLayer.m</text>
          <text x="175" y="46" fill="#38bdf8" font-size="10" font-weight="700" text-anchor="middle">MAINTAIN, FOLLOW, YIELD, OVERTAKE</text>
          <text x="175" y="64" fill="#e2e8f0" font-size="9.5" text-anchor="middle">Spatial Feasibility Gate: W_avail &ge; 4.10m, TTC > 6s, v_ego &ge; 1.5m/s</text>
          <text x="175" y="80" fill="#e2e8f0" font-size="9.5" text-anchor="middle">Latched Overtake: dx_lead &ge; 7.50m (5 steps)</text>
          <text x="175" y="96" fill="#94a3b8" font-size="9" text-anchor="middle">Output: decision (macro_intent, target_v, allow_overtake)</text>
        </g>

        <g transform="translate(585, 320)">
          <rect width="350" height="90" rx="8" fill="#0f172a" stroke="#10b981" stroke-width="2"/>
          <text x="175" y="28" fill="#10b981" font-size="12" font-weight="900" text-anchor="middle">FreeSpaceMap.m & BoundProvider</text>
          <text x="175" y="48" fill="#e2e8f0" font-size="10" text-anchor="middle">2D Dynamic Drivable Space Extraction</text>
          <text x="175" y="66" fill="#e2e8f0" font-size="9.5" text-anchor="middle">31-pt Moving Avg Corridor Center Smoothing</text>
          <text x="175" y="80" fill="#94a3b8" font-size="9" text-anchor="middle">Output: y_min_vec, y_max_vec, ref_path_coord</text>
        </g>

        <g transform="translate(585, 435)">
          <rect width="350" height="150" rx="8" fill="#0f172a" stroke="#10b981" stroke-width="2"/>
          <text x="175" y="28" fill="#10b981" font-size="13" font-weight="900" text-anchor="middle">CACRCPlanner.m & QPMPCPlanner.m</text>
          <text x="175" y="48" fill="#38bdf8" font-size="10" font-weight="700" text-anchor="middle">Dual Topology Evaluation (J_left vs J_right)</text>
          <text x="175" y="66" fill="#e2e8f0" font-size="9.5" text-anchor="middle">4D Kinematic Bicycle Model: [x, y, &theta;, v]^T</text>
          <text x="175" y="84" fill="#e2e8f0" font-size="9.5" text-anchor="middle">Hildreth Dual Coordinate Descent Solver (Np=20, dt=0.1s)</text>
          <text x="175" y="102" fill="#f59e0b" font-size="9.5" text-anchor="middle">Layer 1 Slack-Softened QP Fallback (W_slack = 1000)</text>
          <text x="175" y="120" fill="#f43f5e" font-size="9.5" text-anchor="middle">Emergency Road-Bound Recovery Fallback</text>
          <text x="175" y="138" fill="#94a3b8" font-size="9" text-anchor="middle">Output: u_mpc [delta, a]^T, pred_states, status (1/0)</text>
        </g>

        <g transform="translate(585, 610)">
          <rect width="350" height="80" rx="8" fill="#0f172a" stroke="#38bdf8" stroke-width="1.5" stroke-dasharray="5,3"/>
          <text x="175" y="28" fill="#38bdf8" font-size="12" font-weight="800" text-anchor="middle">Intersection & Merge Topology Layer</text>
          <text x="175" y="48" fill="#f59e0b" font-size="10" font-weight="700" text-anchor="middle">[Proposed Extension]</text>
          <text x="175" y="66" fill="#64748b" font-size="9" text-anchor="middle">CROSS, MERGE, YIELD, PASS_LEFT, PASS_RIGHT</text>
        </g>

        <!-- SAFETY SUPERVISOR & ACTUATOR -->
        <g transform="translate(995, 80)">
          <rect width="230" height="230" rx="8" fill="#0f172a" stroke="#f43f5e" stroke-width="2"/>
          <text x="115" y="28" fill="#f43f5e" font-size="13" font-weight="900" text-anchor="middle">SafetyFilter.m</text>
          <text x="115" y="46" fill="#fb7185" font-size="10" font-weight="800" text-anchor="middle">INDEPENDENT SAFETY LAYER</text>
          <text x="115" y="70" fill="#e2e8f0" font-size="9.5" text-anchor="middle">1. Current road bound check</text>
          <text x="115" y="88" fill="#e2e8f0" font-size="9.5" text-anchor="middle">2. Primary MPC status check</text>
          <text x="115" y="106" fill="#e2e8f0" font-size="9.5" text-anchor="middle">3. Predicted road bound check</text>
          <text x="115" y="124" fill="#e2e8f0" font-size="9.5" text-anchor="middle">4. 2D Footprint OBB Clearance</text>
          <rect x="10" y="138" width="210" height="42" rx="6" fill="rgba(244, 63, 94, 0.2)" stroke="#f43f5e" stroke-width="1.5"/>
          <text x="115" y="156" fill="#f43f5e" font-size="10" font-weight="900" text-anchor="middle">LAYER 2 EMERGENCY BRAKING</text>
          <text x="115" y="172" fill="#fecdd3" font-size="9" text-anchor="middle">a = -3.0 m/s&sup2;, PD Steer Re-center</text>
          <text x="115" y="210" fill="#94a3b8" font-size="9" text-anchor="middle">Output: u_safe [delta_cmd, a_cmd]^T</text>
        </g>

        <g transform="translate(995, 335)">
          <rect width="230" height="90" rx="8" fill="#0f172a" stroke="#10b981" stroke-width="2"/>
          <text x="115" y="28" fill="#10b981" font-size="12" font-weight="900" text-anchor="middle">ActuatorUncertaintyModel.m</text>
          <text x="115" y="48" fill="#e2e8f0" font-size="10" text-anchor="middle">Steering Bias (+0.8 deg in nominal)</text>
          <text x="115" y="64" fill="#e2e8f0" font-size="9.5" text-anchor="middle">Physical Saturation Limits</text>
          <text x="115" y="80" fill="#94a3b8" font-size="9" text-anchor="middle">Output: u_actual [delta_act, a_act]^T</text>
        </g>

        <g transform="translate(995, 450)">
          <rect width="230" height="80" rx="8" fill="#0f172a" stroke="#38bdf8" stroke-width="1.5" stroke-dasharray="5,3"/>
          <text x="115" y="28" fill="#38bdf8" font-size="12" font-weight="800" text-anchor="middle">Control Barrier Functions</text>
          <text x="115" y="48" fill="#f59e0b" font-size="10" font-weight="700" text-anchor="middle">[Proposed Extension]</text>
          <text x="115" y="66" fill="#64748b" font-size="9" text-anchor="middle">Formal Invariant Safety Guarantees</text>
        </g>

        <!-- PLANT & CLOSED LOOP -->
        <g transform="translate(1275, 80)">
          <rect width="190" height="110" rx="8" fill="#0f172a" stroke="#10b981" stroke-width="2"/>
          <text x="95" y="28" fill="#10b981" font-size="12" font-weight="900" text-anchor="middle">BicycleModel.m</text>
          <text x="95" y="46" fill="#38bdf8" font-size="10" font-weight="700" text-anchor="middle">Vehicle Plant Kinematics</text>
          <text x="95" y="64" fill="#e2e8f0" font-size="9" text-anchor="middle">Constant-Curvature Midpoint</text>
          <text x="95" y="80" fill="#e2e8f0" font-size="9" text-anchor="middle">Steer Rate 60&deg;/s, Lag 0.15s</text>
          <text x="95" y="98" fill="#94a3b8" font-size="9" text-anchor="middle">State: [x, y, &theta;, v, &delta;]</text>
        </g>

        <!-- FLOW CONNECTOR LINES -->
        <!-- World to Obs -->
        <path d="M 225 115 L 275 115" stroke="#a855f7" stroke-width="2" marker-end="url(#arr-purple)"/>
        <!-- Obs to Det -->
        <path d="M 400 160 L 400 185" stroke="#10b981" stroke-width="2" marker-end="url(#arr-green)"/>
        <!-- Det to TTC -->
        <path d="M 400 265 L 400 290" stroke="#10b981" stroke-width="2" marker-end="url(#arr-green)"/>
        <!-- Det to Classifier -->
        <path d="M 525 225 L 585 120" stroke="#10b981" stroke-width="2" marker-end="url(#arr-green)"/>
        <!-- TTC to Classifier -->
        <path d="M 525 330 L 585 140" stroke="#10b981" stroke-width="2" marker-end="url(#arr-green)"/>
        <!-- Classifier to Decision -->
        <path d="M 760 160 L 760 185" stroke="#10b981" stroke-width="2" marker-end="url(#arr-green)"/>
        <!-- Decision to FreeSpace -->
        <path d="M 760 295 L 760 320" stroke="#10b981" stroke-width="2" marker-end="url(#arr-green)"/>
        <!-- FreeSpace to CACRC -->
        <path d="M 760 410 L 760 435" stroke="#10b981" stroke-width="2" marker-end="url(#arr-green)"/>
        <!-- CACRC to SafetyFilter -->
        <path d="M 935 510 L 995 195" stroke="#10b981" stroke-width="2" marker-end="url(#arr-green)"/>
        <!-- SafetyFilter to Actuator -->
        <path d="M 1110 310 L 1110 335" stroke="#f43f5e" stroke-width="2" marker-end="url(#arr-rose)"/>
        <!-- Actuator to Bicycle -->
        <path d="M 1225 380 L 1370 380 L 1370 190" stroke="#10b981" stroke-width="2" marker-end="url(#arr-green)"/>
        <!-- Bicycle to World Feedback -->
        <path d="M 1370 80 L 1370 35 L 130 35 L 130 80" stroke="#38bdf8" stroke-width="2.5" stroke-dasharray="6,4" marker-end="url(#arr-cyan)"/>
        
        <rect x="650" y="22" width="220" height="26" rx="13" fill="#0f172a" stroke="#38bdf8" stroke-width="1.5"/>
        <text x="760" y="39" fill="#38bdf8" font-size="10" font-weight="900" text-anchor="middle">CLOSED-LOOP REPLANNING (dt = 0.10s)</text>
      </svg>
    </div>
  </section>

  <!-- SECTION 2: HIERARCHICAL STATE MACHINE ARCHITECTURE (HSMA) -->
  <section class="section-card" id="section-hsma">
    <div class="section-header">
      <div class="section-title"><span class="section-num">02</span> Hierarchical State Machine Tree</div>
      <span class="badge b-green">Stateflow Hierarchy</span>
    </div>
    
    <div class="alert-card alert-info">
      <div>
        <strong>Supervisory Tree Hierarchy:</strong> The intelligence machine decouples discrete behavioral state selection (topologies & intents) from continuous optimal trajectory optimization (QP-MPC).
      </div>
    </div>

    <div class="grid-2">
      <div class="box-card cyan">
        <div class="box-title">SUPERVISORY STATE TREE</div>
        <pre>
SYSTEM
│
├── 1. INITIALIZATION [INIT]
│     ├── Load Scenario Geometry & Vehicle Parameters
│     ├── Instantiate Observation & Actuator Uncertainty Layers
│     └── Initialize FreeSpaceMap & Bound Providers
│
├── 2. PERCEIVE & ESTIMATE WORLD STATE
│     ├── Compute Relative Metrics (dx, dy, dvx, dvy)
│     ├── Estimate Time-To-Conflict (min_ttc)
│     └── Classify 6 Interaction Threat Categories
│
├── 3. NORMAL NAVIGATION [AUTONOMOUS_NAVIGATION]
│     ├── MAINTAIN (Nominal Cruising at 8.0 m/s)
│     ├── FOLLOW (Adaptive PD Distance-Based Car Following)
│     ├── YIELD (Controlled Speed Reduction for Threat)
│     └── OVERTAKE (Latched Spatial Corridor Passing)
│
├── 4. CONFLICT MANAGEMENT
│     ├── STATIC OBSTACLE CONFLICT (Corridor Squeeze)
│     ├── ONCOMING VEHICLE CONFLICT (Lane Blockage)
│     └── MULTI-AGENT BOTTLENECK CONFLICT (Corridor Collapse)
│
├── 5. ADAPTIVE PLANNING & TOPOLOGY OPTIMIZATION
│     ├── EXTRACT DYNAMIC FREE SPACE (y_min_vec, y_max_vec)
│     ├── EVALUATE DUAL TOPOLOGIES (J_left vs J_right)
│     └── QP-MPC OPTIMIZATION (Hildreth Dual Solver)
│
├── 6. SAFETY SUPERVISION & SAFEGUARDS
│     ├── SAFE (Execute Planned u_mpc)
│     ├── WARNING / REPLAN (Soft QP Fallback)
│     └── OVERRIDE (Layer 2 Emergency Brake a = -3.0 m/s²)
│
├── 7. RECOVERY [RECOVER]
│     └── Active Proportional Steering Re-centering
│
└── 8. TERMINATION (SUCCESS / TIMEOUT / FAILURE)</pre>
      </div>

      <div>
        <div class="box-card green" style="margin-bottom: 16px;">
          <div class="box-title">STATE DECOMPOSITION & LATCHING</div>
          <p style="font-size:13px; color:var(--text-sub); margin-bottom:8px;">
            <strong>Super-State Invariant:</strong> <code>AUTONOMOUS_NAVIGATION</code> uses EXCLUSIVE_OR decomposition. Exactly one sub-state (<code>MAINTAIN</code>, <code>FOLLOW</code>, <code>YIELD</code>, <code>OVERTAKE</code>, <code>RECOVER</code>) is active at any timestep.
          </p>
          <p style="font-size:13px; color:var(--text-sub);">
            <strong>Overtake Latching Guard:</strong> Once <code>OVERTAKE</code> triggers, it remains locked until ego front bumper is $\ge 7.50$m ahead of lead vehicle for 5 consecutive timesteps (0.5s debounce), preventing lateral oscillation/chatter.
          </p>
        </div>

        <div class="box-card amber">
          <div class="box-title">SLX VS MATLAB MASTER IMPLEMENTATION</div>
          <p style="font-size:13px; color:var(--text-sub);">
            <strong>Simulink Master:</strong> <code>SIH26037_SupervisoryArchitecture.slx</code> defines the formal graphical Stateflow chart.
          </p>
          <p style="font-size:13px; color:var(--text-sub); margin-top:6px;">
            <strong>Executable Runtime:</strong> <code>CoordinationDecisionLayer.m</code> implements exact production logic in pure MATLAB for automated batch testing.
          </p>
        </div>
      </div>
    </div>
  </section>

  <!-- SECTION 3: INITIAL STATE & CONFIGURATION -->
  <section class="section-card" id="section-init">
    <div class="section-header">
      <div class="section-title"><span class="section-num">03</span> Initial State & Initialization Setup</div>
      <span class="badge b-purple">Stage 0 Execution</span>
    </div>

    <div class="grid-2">
      <div class="box-card purple">
        <div class="box-title">INITIALIZATION SEQUENCE (INIT STATE)</div>
        <ol style="font-size:13px; color:var(--text-sub); padding-left:20px; line-height:1.8;">
          <li><strong>Load Scenario Geometry:</strong> <code>ScenarioDefinitions</code> instantiates road boundaries, ego starting pose ($x_0=10, y_0=1.8, \theta_0=0, v_0=5$), and static/dynamic obstacles.</li>
          <li><strong>Initialize Observation Model:</strong> <code>ObservationModel(mode, seed)</code> sets position noise ($\sigma_{pos}=0.15$m), velocity noise ($\sigma_{vel}=0.20$m/s), and delay ($\tau_{delay}=100$ms).</li>
          <li><strong>Initialize Controller & Safety Layers:</strong> <code>Stage5CoordinationController</code> instantiates <code>CACRCPlanner</code>, <code>QPMPCPlanner</code>, and <code>SafetyFilter</code>.</li>
          <li><strong>Validate Initial Feasibility:</strong> Confirms initial state is outside obstacle collision boundaries and within drivable bounds.</li>
          <li><strong>Enter Closed-Loop Perception:</strong> Transitions state machine to active execution.</li>
        </ol>
      </div>

      <div class="box-card cyan">
        <div class="box-title">INITIAL STATE SPECIFICATION TABLE</div>
        <div style="font-size:13px; color:var(--text-sub); line-height:1.8;">
          <p><strong>Ego State Vector:</strong> <code>x_0 = [10.0, 1.80, 0.0, 5.0, 0.0]^T</code></p>
          <p><strong>Vehicle Dimensions:</strong> Length $L_{total}=4.70$m, Width $W_{ego}=1.80$m, Wheelbase $L=2.70$m.</p>
          <p><strong>MPC Horizon:</strong> Prediction steps $N_p = 20$, sample time $dt = 0.10$s ($Lookahead = 2.0$s).</p>
          <p><strong>MPC Weighting:</strong> $Q = \text{diag}([1, 20, 80, 50])$, $R = \text{diag}([50, 0.1])$, Slack Weight $W_{slack} = 1000$.</p>
          <p><strong>Physical Saturation:</strong> Steering $\delta \in \pm 35^\circ$, Accel $a \in [-6.0, +3.0]$ m/s$^2$.</p>
        </div>
      </div>
    </div>
  </section>

  <!-- SECTION 4: ACTIVE STATES SPECIFICATION -->
  <section class="section-card" id="section-states">
    <div class="section-header">
      <div class="section-title"><span class="section-num">04</span> Operational States Detailed Breakdown</div>
      <span class="badge b-green">Active Logic</span>
    </div>

    <div class="grid-3">
      <!-- MAINTAIN -->
      <div class="box-card green">
        <div class="box-title">MAINTAIN <span class="badge b-green">Implemented</span></div>
        <p style="font-size:12.5px; color:var(--text-sub); margin-bottom:8px;">
          <strong>Purpose:</strong> Nominal cruising along drivable corridor when road ahead is clear.
        </p>
        <p style="font-size:12.5px; color:var(--text-sub);">
          <strong>Active Output:</strong> <code>target_v = 8.0 m/s</code>, <code>allow_overtake = true</code>.
        </p>
      </div>

      <!-- FOLLOW -->
      <div class="box-card green">
        <div class="box-title">FOLLOW <span class="badge b-green">Implemented</span></div>
        <p style="font-size:12.5px; color:var(--text-sub); margin-bottom:8px;">
          <strong>Purpose:</strong> Car-following behind lead vehicle at safe headway.
        </p>
        <p style="font-size:12.5px; color:var(--text-sub);">
          <strong>Control Law:</strong> $v_{pd} = v_{lead} + 0.40(dx - 15) + 0.60(dvx)$.
        </p>
      </div>

      <!-- OVERTAKE -->
      <div class="box-card green">
        <div class="box-title">OVERTAKE <span class="badge b-green">Implemented</span></div>
        <p style="font-size:12.5px; color:var(--text-sub); margin-bottom:8px;">
          <strong>Purpose:</strong> Latched passing maneuver around slower lead vehicle.
        </p>
        <p style="font-size:12.5px; color:var(--text-sub);">
          <strong>Gate:</strong> $W_{avail} \ge 4.10$m, $TTC > 6$s, $6 \le dx \le 28$m, $v \ge 1.5$m/s.
        </p>
      </div>

      <!-- YIELD -->
      <div class="box-card green">
        <div class="box-title">YIELD <span class="badge b-green">Implemented</span></div>
        <p style="font-size:12.5px; color:var(--text-sub); margin-bottom:8px;">
          <strong>Purpose:</strong> Active speed reduction for oncoming threats or corridor restriction.
        </p>
        <p style="font-size:12.5px; color:var(--text-sub);">
          <strong>Control Law:</strong> $v_{yield} = \max(0.0, 0.50(dx_{lead} - 15.0))$.
        </p>
      </div>

      <!-- EMERGENCY -->
      <div class="box-card rose">
        <div class="box-title">EMERGENCY <span class="badge b-rose">Safety Override</span></div>
        <p style="font-size:12.5px; color:var(--text-sub); margin-bottom:8px;">
          <strong>Purpose:</strong> Layer 2 Controlled Emergency Deceleration safeguard.
        </p>
        <p style="font-size:12.5px; color:var(--text-sub);">
          <strong>Action:</strong> Mandates $a = -3.0$ m/s$^2$, PD steer re-centering.
        </p>
      </div>

      <!-- RECOVER -->
      <div class="box-card green">
        <div class="box-title">RECOVER <span class="badge b-green">Implemented</span></div>
        <p style="font-size:12.5px; color:var(--text-sub); margin-bottom:8px;">
          <strong>Purpose:</strong> Smooth re-centering back to nominal right lane centerline ($y=1.8$m).
        </p>
        <p style="font-size:12.5px; color:var(--text-sub);">
          <strong>Trigger:</strong> Overtake physically complete ($dx_{lead} \ge 7.50$m).
        </p>
      </div>
    </div>
  </section>

  <!-- SECTION 5: PARALLEL MONITORS -->
  <section class="section-card" id="section-monitors">
    <div class="section-header">
      <div class="section-title"><span class="section-num">05</span> Logical Parallel Monitors</div>
      <span class="badge b-cyan">Concurrent Auditing</span>
    </div>

    <div class="alert-card alert-warning">
      <div>
        <strong>Execution Note:</strong> In the MATLAB prototype, these processes execute sequentially within each 100ms timestep (<code>step()</code> loop). Conceptually, they represent logical concurrent supervisory processes operating continuously across the stack.
      </div>
    </div>

    <div class="grid-3">
      <div class="box-card cyan">
        <div class="box-title">A. PERCEPTION MONITOR</div>
        <p style="font-size:12.5px; color:var(--text-sub);">Transforms raw observations, applies noise/delay, extracts relative metrics, and maps static obstacles.</p>
      </div>

      <div class="box-card cyan">
        <div class="box-title">B. PREDICTION MONITOR</div>
        <p style="font-size:12.5px; color:var(--text-sub);">Computes Time-To-Conflict ($TTC$) and velocity uncertainty lateral expansion ($\Delta y_{unc}$) across horizon.</p>
      </div>

      <div class="box-card rose">
        <div class="box-title">C. SAFETY MONITOR</div>
        <p style="font-size:12.5px; color:var(--text-sub);">Background verification comparing planned trajectory against road bounds and 2D footprint OBB clearance.</p>
      </div>

      <div class="box-card purple">
        <div class="box-title">D. VEHICLE STATE MONITOR</div>
        <p style="font-size:12.5px; color:var(--text-sub);">Tracks ego speed $v$, heading $\theta$, steering angle $\delta$, acceleration $a$, and actuator saturation limits.</p>
      </div>

      <div class="box-card amber">
        <div class="box-title">E. PLANNER HEALTH MONITOR</div>
        <p style="font-size:12.5px; color:var(--text-sub);">Audits Hildreth QP iterations, 4-condition KKT residuals ($r_{primal}, r_{dual}, r_{comp}, r_{stat} < 1e-3$), and solve time.</p>
      </div>
    </div>
  </section>

  <!-- SECTION 6: GUARDED TRANSITIONS TABLE -->
  <section class="section-card" id="section-guards">
    <div class="section-header">
      <div class="section-title"><span class="section-num">06</span> Guarded State Transition Matrix</div>
      <span class="badge b-green">Stateflow Guard Rules</span>
    </div>

    <div class="table-responsive">
      <table class="dark-table">
        <thead>
          <tr>
            <th>Current State</th>
            <th>Transition Guard Condition</th>
            <th>Next State</th>
            <th>Supervisory Action</th>
            <th>Output Command</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td><code>INIT</code></td>
            <td>Scenario loaded & initial state validated</td>
            <td><code>MAINTAIN</code></td>
            <td>Set nominal cruise velocity</td>
            <td><code>v_target = 8.0 m/s</code></td>
          </tr>
          <tr>
            <td><code>MAINTAIN</code></td>
            <td>Lead vehicle detected ahead ($dx \le 40.0$m)</td>
            <td><code>FOLLOW</code></td>
            <td>Engage PD car-following control</td>
            <td><code>v_target = v_pd_clamped</code></td>
          </tr>
          <tr>
            <td><code>FOLLOW</code></td>
            <td>Passing Gate Passed ($W_{avail} \ge 4.10$m, $TTC > 6$s, $v \ge 1.5$m/s)</td>
            <td><code>OVERTAKE</code></td>
            <td>Latch overtake & lock left topology</td>
            <td><code>macro_intent = 'OVERTAKE'</code></td>
          </tr>
          <tr>
            <td><code>OVERTAKE</code></td>
            <td>Lead cleared ($dx_{lead} \ge 7.50$m for 5 steps)</td>
            <td><code>RECOVER</code></td>
            <td>Initiate smooth lane re-centering</td>
            <td><code>x_recenter = x_ego</code></td>
          </tr>
          <tr>
            <td><code>OVERTAKE</code></td>
            <td>Oncoming threat ($TTC \le 6.0$s or $W_{avail} < 4.10$m)</td>
            <td><code>YIELD</code></td>
            <td>Abort overtake & decelerate</td>
            <td><code>macro_intent = 'YIELD'</code></td>
          </tr>
          <tr>
            <td><code>ANY STATE</code></td>
            <td>MPC infeasible (status=0) OR safety violation</td>
            <td><code>EMERGENCY</code></td>
            <td>Trigger Layer 2 Emergency Brake</td>
            <td><code>a_cmd = -3.0 m/s²</code></td>
          </tr>
          <tr>
            <td><code>EMERGENCY</code></td>
            <td>Ego velocity drops below 0.10 m/s</td>
            <td><code>SAFE_STOP</code></td>
            <td>Hold standstill brake</td>
            <td><code>a_cmd = 0.0, v = 0.0</code></td>
          </tr>
          <tr>
            <td><code>RECOVER</code></td>
            <td>Ego returned to right lane ($|y - 1.80| \le 0.15$m)</td>
            <td><code>MAINTAIN</code></td>
            <td>Restore nominal cruising</td>
            <td><code>macro_intent = 'MAINTAIN'</code></td>
          </tr>
        </tbody>
      </table>
    </div>
  </section>

  <!-- SECTION 7: DATA-FLOW MAP -->
  <section class="section-card" id="section-dataflow">
    <div class="section-header">
      <div class="section-title"><span class="section-num">07</span> Forensic Data-Flow Map</div>
      <span class="badge b-amber">Data Pipeline Audit</span>
    </div>

    <div class="alert-card alert-warning">
      <div>
        <strong>Forensic Disconnected Call Audit:</strong> <code>RiskPredictor.predictTrajectories()</code> generates multi-step trajectory prediction structs. However, this data is <strong>DISCARDED/UNUSED</strong> in <code>CACRCPlanner</code>, <code>QPMPCPlanner</code>, <code>FreeSpaceMap</code>, and <code>SafetyFilter</code>, and is only preserved in <code>info.predictions</code> for logging purposes.
      </div>
    </div>

    <div class="table-responsive">
      <table class="dark-table">
        <thead>
          <tr>
            <th>Module Name</th>
            <th>File Path</th>
            <th>Input Structure</th>
            <th>Processing Description</th>
            <th>Output Structure</th>
            <th>Consumer Module</th>
            <th>Status</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td><strong>ObservationModel</strong></td>
            <td><code>environment/ObservationModel.m</code></td>
            <td><code>world</code>, <code>dt</code></td>
            <td>Applies Gaussian noise ($\sigma_{pos}, \sigma_{vel}$) & 100ms delay</td>
            <td><code>obs_world</code>, <code>obs_structs</code></td>
            <td>MultiVehicleDetector, FreeSpaceMap</td>
            <td><span class="badge b-green">Implemented</span></td>
          </tr>
          <tr>
            <td><strong>MultiVehicleDetector</strong></td>
            <td><code>planning/MultiVehicleDetector.m</code></td>
            <td><code>obs_world</code>, <code>ego</code></td>
            <td>Extracts relative metrics ($dx, dy, dvx, dvy$) & maps static obs</td>
            <td><code>detections</code> struct array</td>
            <td>RiskPredictor, InteractionClassifier</td>
            <td><span class="badge b-green">Implemented</span></td>
          </tr>
          <tr>
            <td><strong>RiskPredictor (TTC)</strong></td>
            <td><code>planning/RiskPredictor.m</code></td>
            <td><code>detections</code>, <code>L_ego</code></td>
            <td>Calculates linear time-to-conflict along longitudinal/oncoming axes</td>
            <td><code>ttc_vector</code>, <code>min_ttc</code></td>
            <td>InteractionClassifier, CoordinationDecisionLayer</td>
            <td><span class="badge b-green">Implemented</span></td>
          </tr>
          <tr>
            <td><strong>RiskPredictor (Traj)</strong></td>
            <td><code>planning/RiskPredictor.m</code></td>
            <td><code>detections</code>, $N_p$, $dt$</td>
            <td>Multi-step linear propagation over horizon $N_p$</td>
            <td><code>preds</code> struct array</td>
            <td><code>info.predictions</code> (Logging Only)</td>
            <td><span class="badge b-amber">Generated but DISCARDED</span></td>
          </tr>
          <tr>
            <td><strong>InteractionClassifier</strong></td>
            <td><code>planning/InteractionClassifier.m</code></td>
            <td><code>detections</code>, <code>ego</code>, <code>ttc_vector</code></td>
            <td>Categorizes surrounding agents into 6 threat classes</td>
            <td><code>interactions</code> struct array</td>
            <td>CoordinationDecisionLayer</td>
            <td><span class="badge b-green">Implemented</span></td>
          </tr>
          <tr>
            <td><strong>CoordinationDecisionLayer</strong></td>
            <td><code>planning/CoordinationDecisionLayer.m</code></td>
            <td><code>detections</code>, <code>interactions</code>, <code>ego</code></td>
            <td>Evaluates spatial gate, latches overtake, sets target speed</td>
            <td><code>decision</code> (intent, target_v, allow_overtake)</td>
            <td>CACRCPlanner, Stage5Controller</td>
            <td><span class="badge b-green">Implemented</span></td>
          </tr>
          <tr>
            <td><strong>FreeSpaceMap</strong></td>
            <td><code>environment/FreeSpaceMap.m</code></td>
            <td><code>x_vec</code>, <code>obs_world</code>, <code>half_W</code></td>
            <td>Extracts drivable lateral bounds considering static & dynamic obstacles</td>
            <td><code>y_min_vec</code>, <code>y_max_vec</code></td>
            <td>CACRCPlanner, SafetyFilter</td>
            <td><span class="badge b-green">Implemented</span></td>
          </tr>
          <tr>
            <td><strong>CACRCPlanner</strong></td>
            <td><code>planning/CACRCPlanner.m</code></td>
            <td><code>world</code>, <code>ref_path</code>, <code>target_v</code></td>
            <td>Dual topology cost evaluation ($J_{left}$ vs $J_{right}$) & Hildreth QP solve</td>
            <td><code>u_mpc</code> [$\delta, a$]$^T$, <code>pred_states</code>, <code>status</code></td>
            <td>SafetyFilter</td>
            <td><span class="badge b-green">Implemented</span></td>
          </tr>
          <tr>
            <td><strong>SafetyFilter</strong></td>
            <td><code>planning/SafetyFilter.m</code></td>
            <td><code>u_mpc</code>, <code>status</code>, <code>world</code>, <code>pred_states</code></td>
            <td>Independent 2D OBB footprint clearance & road bound check</td>
            <td><code>u_safe</code> [$\delta_{cmd}, a_{cmd}$]$^T$, <code>filter_active</code></td>
            <td>ActuatorUncertaintyModel</td>
            <td><span class="badge b-green">Implemented</span></td>
          </tr>
          <tr>
            <td><strong>ActuatorUncertaintyModel</strong></td>
            <td><code>environment/ActuatorUncertaintyModel.m</code></td>
            <td><code>u_safe</code>, <code>config</code></td>
            <td>Applies +0.8 deg steering bias & physical saturation limits</td>
            <td><code>u_actual</code> [$\delta_{act}, a_{act}$]$^T$</td>
            <td>BicycleModel (Ego Plant)</td>
            <td><span class="badge b-green">Implemented</span></td>
          </tr>
          <tr>
            <td><strong>BicycleModel</strong></td>
            <td><code>vehicle/BicycleModel.m</code></td>
            <td><code>ego_state</code>, <code>u_actual</code>, <code>dt</code></td>
            <td>Kinematic bicycle step via constant-curvature midpoint method</td>
            <td><code>state_next</code> (EgoState object)</td>
            <td>WorldState (Next Iteration)</td>
            <td><span class="badge b-green">Implemented</span></td>
          </tr>
        </tbody>
      </table>
    </div>
  </section>

  <!-- SECTION 8: GAP MAP (CURRENT VS PROPOSED) -->
  <section class="section-card" id="section-gap">
    <div class="section-header">
      <div class="section-title"><span class="section-num">08</span> Current Prototype vs Proposed Gap Map</div>
      <span class="badge b-cyan">SIH Extension Roadmap</span>
    </div>

    <div class="table-responsive">
      <table class="dark-table">
        <thead>
          <tr>
            <th>Current Prototype Component</th>
            <th>Current Limitation / Abstraction</th>
            <th>Proposed SIH Extension</th>
            <th>SIH Requirement Satisfied</th>
            <th>Validation Method</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td><strong>ObservationModel</strong></td>
            <td>Analytical Gaussian noise & delay on ground truth</td>
            <td>True Camera / LiDAR / Radar point cloud perception pipeline</td>
            <td>Multi-sensor perception & unstructured road sensing</td>
            <td>Synthetic sensor dataset in RoadRunner</td>
          </tr>
          <tr>
            <td><strong>MultiVehicleDetector</strong></td>
            <td>Direct extraction from simulation objects</td>
            <td>EKF / UKF Multi-Object Tracking (MOT) & object association</td>
            <td>Diverse road users & tracking under occlusion</td>
            <td>MOTA / MOTP tracking metrics</td>
          </tr>
          <tr>
            <td><strong>RiskPredictor Trajectories</strong></td>
            <td>Linear constant velocity predictions DISCARDED</td>
            <td>Uncertainty-aware downstream trajectory prediction consumed in MPC</td>
            <td>Irregular movement & sudden pedestrian/cattle movement</td>
            <td>ADE / FDE prediction error & clearance</td>
          </tr>
          <tr>
            <td><strong>FreeSpaceMap</strong></td>
            <td>Hand-authored sinusoidal boundary equations</td>
            <td>2D Dynamic Occupancy Grid & perception-derived drivable space</td>
            <td>Unmarked roads & dynamic obstacles</td>
            <td>Drivable space segmentation accuracy</td>
          </tr>
          <tr>
            <td><strong>CoordinationDecisionLayer</strong></td>
            <td>Single-lane passing / following macro-intents</td>
            <td>Unsignalized intersection & informal highway merge behavior trees</td>
            <td>Informal merging & complex intersections</td>
            <td>Intersection scenario completion rate</td>
          </tr>
          <tr>
            <td><strong>SafetyFilter</strong></td>
            <td>Ad-hoc 2D OBB clearance & Layer 2 emergency brake</td>
            <td>Control Barrier Functions (CBF) with formal safety guarantees</td>
            <td>Collision avoidance & safety verification</td>
            <td>Zero collision proof & invariance metrics</td>
          </tr>
          <tr>
            <td><strong>RoadRunner Integration</strong></td>
            <td>MATLAB-only fallback (check_roadrunner_available)</td>
            <td>Full closed-loop RoadRunner co-simulation API coupling</td>
            <td>RoadRunner closed-loop simulation requirement</td>
            <td>Repeatable co-simulation benchmarks</td>
          </tr>
        </tbody>
      </table>
    </div>
  </section>

  <!-- SECTION 9: ROAD-USER & UNSTRUCTURED ROAD MODEL -->
  <section class="section-card" id="section-unstructured">
    <div class="section-header">
      <div class="section-title"><span class="section-num">09</span> Road-User Heterogeneity & Unstructured Road Model</div>
      <span class="badge b-purple">Indian Road Context</span>
    </div>

    <div class="grid-2">
      <div class="box-card purple">
        <div class="box-title">ROAD-USER CLASS HETEROGENEITY PIPELINE</div>
        <p style="font-size:13px; color:var(--text-sub); margin-bottom:10px;">
          The system handles diverse Indian road users: <strong>cars, trucks, buses, auto-rickshaws, motorcycles, bicycles, pedestrians, cattle, pushcarts, and roadside stalls.</strong>
        </p>
        <p style="font-size:13px; color:var(--text-sub);">
          <strong>Pipeline Integration:</strong> Detection $\rightarrow$ Classification $\rightarrow$ Class-Specific Motion Model $\rightarrow$ Interaction Model $\rightarrow$ Risk Assessment $\rightarrow$ Behavior Decision.
        </p>
      </div>

      <div class="box-card cyan">
        <div class="box-title">UNSTRUCTURED ROAD GEOMETRY MODEL</div>
        <p style="font-size:13px; color:var(--text-sub); margin-bottom:10px;">
          The architecture does <strong>NOT</strong> rely on lane markings. Drivable space is modeled geometrically:
        </p>
        <div style="background:#090d16; border:1px solid var(--border-bright); padding:12px; border-radius:8px; font-family:'JetBrains Mono', monospace; font-size:11px; text-align:center; color:var(--cyan-bright);">
          ROAD BOUNDS + STATIC OBS + DYNAMIC OCCUPANCY + SAFETY MARGIN<br>= <strong>AVAILABLE DRIVABLE CORRIDOR (y_min_vec, y_max_vec)</strong>
        </div>
      </div>
    </div>
  </section>

  <!-- SECTION 10: DESIGN RATIONALE -->
  <section class="section-card" id="section-rationale">
    <div class="section-header">
      <div class="section-title"><span class="section-num">10</span> Design-Decision Technical Rationales</div>
      <span class="badge b-cyan">Engineering Rationale</span>
    </div>

    <div class="grid-3">
      <div class="box-card cyan">
        <div class="box-title">WHY DYNAMIC FREE SPACE?</div>
        <p style="font-size:12.5px; color:var(--text-sub);">Lane markings cannot be assumed on Indian unstructured roads. Dynamic free-space derives local drivable boundaries directly from road geometry and obstacle boundaries.</p>
      </div>

      <div class="box-card cyan">
        <div class="box-title">WHY PREDICTION?</div>
        <p style="font-size:12.5px; color:var(--text-sub);">Current obstacle position is insufficient for moving or crossing agents. Trajectory prediction enables proactive collision avoidance.</p>
      </div>

      <div class="box-card cyan">
        <div class="box-title">WHY ADAPTIVE TOPOLOGY?</div>
        <p style="font-size:12.5px; color:var(--text-sub);">Obstacle avoidance requires selecting distinct qualitative routes (Pass Left vs Pass Right). Discrete topology selection prevents local minima in MPC.</p>
      </div>

      <div class="box-card cyan">
        <div class="box-title">WHY QP-MPC?</div>
        <p style="font-size:12.5px; color:var(--text-sub);">QP-MPC enables fast, real-time optimal trajectory synthesis while strictly enforcing physical vehicle dynamics, actuator bounds, and road limits.</p>
      </div>

      <div class="box-card rose">
        <div class="box-title">WHY SAFETY SUPERVISOR?</div>
        <p style="font-size:12.5px; color:var(--text-sub);">Planning and safety verification are distinct responsibilities. An independent safety supervisor guarantees Layer 2 emergency braking if the optimizer fails.</p>
      </div>

      <div class="box-card purple">
        <div class="box-title">WHY EDGE EXECUTION?</div>
        <p style="font-size:12.5px; color:var(--text-sub);">Safety-critical autonomous control cannot rely on cloud latency or network availability. Full closed-loop perception and control must execute locally on edge hardware.</p>
      </div>
    </div>
  </section>

  <!-- SECTION 11: VALIDATION PLAN -->
  <section class="section-card" id="section-validation">
    <div class="section-header">
      <div class="section-title"><span class="section-num">11</span> SIH Traceability & Validation Plan</div>
      <span class="badge b-green">Verification Matrix</span>
    </div>

    <div class="table-responsive">
      <table class="dark-table">
        <thead>
          <tr>
            <th>SIH26037 Problem Requirement</th>
            <th>Architecture Component</th>
            <th>Validation Experiment / Benchmark</th>
            <th>Quantitative Metric</th>
            <th>Demonstrated Evidence</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td>Unmarked Indian Roads</td>
            <td>FreeSpaceMap & BoundProvider</td>
            <td>Village road scenario with non-uniform sinusoidal bounds</td>
            <td>Road-bound compliance rate & min clearance</td>
            <td>100% road containment across tested steps</td>
          </tr>
          <tr>
            <td>Diverse Road Users</td>
            <td>MultiVehicleDetector & InteractionClassifier</td>
            <td>Multi-agent traffic scenario (cars, auto-rickshaws, bikes)</td>
            <td>Detection accuracy & interaction classification</td>
            <td>Correct classification across 6 threat categories</td>
          </tr>
          <tr>
            <td>Irregular Movement & Intrusions</td>
            <td>RiskPredictor & UncertaintyPredictor</td>
            <td>Roadside Livestock / Cattle crossing scenario</td>
            <td>Clearance $C_{geom} > 0.0$m & zero collisions</td>
            <td>0 collisions across dynamic cattle intrusion tests</td>
          </tr>
          <tr>
            <td>Informal Merging & Overtaking</td>
            <td>CoordinationDecisionLayer & Topology Layer</td>
            <td>Multi-vehicle yield & overtake benchmark scenario</td>
            <td>Maneuver success rate & latch stability</td>
            <td>Successful latched overtake with zero chatter</td>
          </tr>
          <tr>
            <td>Real-Time Replanning</td>
            <td>Receding-Horizon CACRC / QP-MPC</td>
            <td>Closed-loop simulation at $dt=0.10$s</td>
            <td>End-to-end solve latency (ms)</td>
            <td>Hildreth QP solve time $\approx 4.0 - 6.4$ms</td>
          </tr>
          <tr>
            <td>RoadRunner Co-Simulation</td>
            <td>RoadRunner API Interface</td>
            <td>RoadRunner scenario export & co-simulation test</td>
            <td>Co-simulation frame rate & API stability</td>
            <td>Fallback interface implemented in Stage 0/5</td>
          </tr>
        </tbody>
      </table>
    </div>
  </section>

  <!-- SECTION 12: PPT SLIDE 2 BLUEPRINT -->
  <section class="section-card" id="section-ppt" style="border: 2px solid var(--cyan-bright);">
    <div class="section-header">
      <div class="section-title" style="color:var(--cyan-bright);"><span class="section-num">12</span> What to Show on PPT Slide 2</div>
      <span class="badge b-cyan">Presentation Blueprint</span>
    </div>

    <div class="alert-card alert-info">
      <div>
        <strong>Slide 2 PPT Extraction Advice:</strong> Keep the slide clean. Present these 6 key boxes in a horizontal/vertical flow chart:
      </div>
    </div>

    <div class="grid-3">
      <div class="box-card cyan">
        <div class="box-title">1. PERCEIVE & TRACK</div>
        <p style="font-size:12.5px; color:var(--text-sub);">Relative perception metrics, noise/delay modeling, multi-vehicle detection.</p>
      </div>

      <div class="box-card cyan">
        <div class="box-title">2. PREDICT & ASSESS RISK</div>
        <p style="font-size:12.5px; color:var(--text-sub);">Time-To-Conflict ($TTC$) calculation, 6-class interaction threat classification.</p>
      </div>

      <div class="box-card cyan">
        <div class="box-title">3. SUPERVISORY DECISION</div>
        <p style="font-size:12.5px; color:var(--text-sub);">Stateflow macro-intents (MAINTAIN, FOLLOW, YIELD, OVERTAKE) with spatial feasibility gate.</p>
      </div>

      <div class="box-card cyan">
        <div class="box-title">4. DYNAMIC FREE SPACE</div>
        <p style="font-size:12.5px; color:var(--text-sub);">Unstructured road drivable corridor extraction ($y_{min}, y_{max}$) & dual topology choice.</p>
      </div>

      <div class="box-card cyan">
        <div class="box-title">5. QP-MPC OPTIMIZER</div>
        <p style="font-size:12.5px; color:var(--text-sub);">Convex trajectory optimization under kinematic dynamics, steering bounds, and road limits.</p>
      </div>

      <div class="box-card rose">
        <div class="box-title" style="color:var(--rose-bright);">6. SAFETY SUPERVISOR</div>
        <p style="font-size:12.5px; color:var(--text-sub);">Independent 2D OBB clearance verification & Layer 2 emergency deceleration fallback.</p>
      </div>
    </div>
  </section>

  <!-- SECTION 13: ENGINEERING AUDIT CHECKLIST -->
  <section class="section-card" id="section-status">
    <div class="section-header">
      <div class="section-title"><span class="section-num">13</span> Forensic Codebase Audit Checklist</div>
      <span class="badge b-green">Honest Technical Audit</span>
    </div>

    <div class="grid-2">
      <div>
        <h4 style="color:var(--green-bright); font-size:14px; font-weight:800; margin-bottom:12px;">CURRENTLY DEMONSTRATED IN CODEBASE:</h4>
        <ul style="font-size:12.5px; color:var(--text-sub); padding-left:20px; line-height:1.8;">
          <li>Closed-loop MATLAB simulation pipeline (Stages 0 through 5).</li>
          <li>Perception observation model with noise and 100ms delay buffers (<code>ObservationModel.m</code>).</li>
          <li>Relative multi-vehicle detection and static obstacle mapping (<code>MultiVehicleDetector.m</code>).</li>
          <li>Time-To-Conflict ($TTC$) risk assessment (<code>RiskPredictor.predictTTC</code>).</li>
          <li>6-class interaction classification (<code>InteractionClassifier.m</code>).</li>
          <li>Macro-intent decision layer with spatial feasibility gate and overtake latching (<code>CoordinationDecisionLayer.m</code>).</li>
          <li>2D Dynamic Free Space map and corridor bound extraction (<code>FreeSpaceMap.m</code>).</li>
          <li>Dual topology evaluation ($J_{left}$ vs $J_{right}$) and Hildreth Dual QP optimization (<code>CACRCPlanner.m</code>).</li>
          <li>Independent safety filter with 2D OBB footprint clearance & Layer 2 Emergency Braking ($a = -3.0$ m/s$^2$).</li>
          <li>Actuator uncertainty model (+0.8 deg steering bias) and kinematic bicycle plant step.</li>
        </ul>

        <h4 style="color:var(--amber-bright); font-size:14px; font-weight:800; margin-top:16px; margin-bottom:12px;">PARTIALLY IMPLEMENTED / DISCARDED:</h4>
        <ul style="font-size:12.5px; color:var(--text-sub); padding-left:20px; line-height:1.8;">
          <li><code>RiskPredictor.predictTrajectories()</code> multi-step predictions are generated but <strong>DISCARDED/UNUSED</strong> in MPC.</li>
          <li>RoadRunner co-simulation interface contains fallback logic; full closed-loop API coupling pending.</li>
          <li>Unstructured road boundaries modeled via hand-authored sinusoidal equations.</li>
        </ul>
      </div>

      <div>
        <h4 style="color:var(--cyan-bright); font-size:14px; font-weight:800; margin-bottom:12px;">PROPOSED SIH EXTENSIONS:</h4>
        <ul style="font-size:12.5px; color:var(--text-sub); padding-left:20px; line-height:1.8;">
          <li>True Camera, LiDAR, and Radar sensor point cloud perception pipeline.</li>
          <li>EKF / UKF sensor fusion and Multi-Object Tracking (MOT) state estimation.</li>
          <li>Downstream trajectory prediction consumption directly inside MPC constraints.</li>
          <li>Perception-derived 2D dynamic occupancy grid maps.</li>
          <li>Unsignalized intersection (CROSS) and informal highway merge (MERGE) supervisory behavior trees.</li>
          <li>Control Barrier Functions (CBF) for formal mathematical safety guarantees.</li>
        </ul>

        <h4 style="color:var(--rose-bright); font-size:14px; font-weight:800; margin-top:16px; margin-bottom:12px;">CRITICAL RISKS & UNCERTAINTIES:</h4>
        <ul style="font-size:12.5px; color:var(--text-sub); padding-left:20px; line-height:1.8;">
          <li>QP solver infeasibility under extreme corridor collapse (mitigated by Layer 1 Slack & Layer 2 Emergency Braking).</li>
          <li>High lateral acceleration / jerk during sudden aborts from overtaking under oncoming threats.</li>
          <li>Sensor occlusion and false negative dropouts in dense heterogeneous traffic.</li>
        </ul>
      </div>
    </div>
  </section>

  <!-- SECTION 14: RECOMMENDED FINAL ARCHITECTURE SUMMARY -->
  <section class="section-card" id="section-summary" style="background: linear-gradient(135deg, #0f172a 0%, #1e1b4b 100%); border-color: var(--cyan-bright);">
    <div class="section-header" style="border-bottom-color: rgba(255,255,255,0.1);">
      <div class="section-title" style="color:var(--text-main);"><span class="section-num">14</span> Recommended Final Architecture Summary</div>
      <span class="badge b-cyan">Master Conclusion</span>
    </div>
    
    <p style="font-size:14px; line-height:1.8; color:#cbd5e1;">
      The proposed complete solution architecture for <strong>SIH26037 ("Adaptive Path Planning and Collision Avoidance for Autonomous Vehicles on Unstructured Indian Roads")</strong> is a <strong>Hierarchical Stateflow / Supervisory State-Machine Architecture (HSMA)</strong> tightly integrated with a receding-horizon data-flow loop. The architecture decouples high-level discrete behavioral decisions (handled by a Stateflow supervisor evaluating spatial feasibility gates, Time-To-Conflict, and latched macro-intents) from low-level continuous trajectory optimization (handled by a Context-Adaptive Risk-Sensitive QP-MPC planner operating over dynamic free-space drivable corridors). An independent Layer 2 Safety Filter guarantees real-time collision verification and emergency deceleration fallback. This hybrid design ensures bounded computation time ($\approx 4-6.4$ms solver latency), strict road-bound compliance, and robust evasion capability in complex, unstructured traffic environments.
    </p>
  </section>

</div>
</body>
</html>
"""
    
    target_path = "/home/yeswanth/projects/sih_new_2026/artifacts/SIH26037_Slide2_Architecture.html"
    os.makedirs(os.path.dirname(target_path), exist_ok=True)
    with open(target_path, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"Successfully generated dark-mode UI html at {target_path}")

if __name__ == "__main__":
    build_html()
