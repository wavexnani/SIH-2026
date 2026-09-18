import os
import subprocess

def generate_slide2_artifacts():
    html_content = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>SIH26037 Slide 2 - Authoritative Engineering Architecture</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; }
  body { background-color: #0f172a; display: flex; justify-content: center; align-items: center; min-height: 100vh; padding: 0; overflow: hidden; }
  .canvas { width: 1920px; height: 1080px; background: #ffffff; position: relative; overflow: hidden; box-shadow: 0 25px 50px -12px rgba(0,0,0,0.5); }
  
  /* Typography & Titles */
  .title-header { position: absolute; top: 20px; left: 30px; }
  .main-title { font-size: 24px; font-weight: 800; color: #0f172a; letter-spacing: -0.5px; text-transform: uppercase; }
  .subtitle { font-size: 15px; font-weight: 700; color: #1d4ed8; margin-top: 2px; }
  .tagline { font-size: 12px; font-weight: 500; color: #64748b; margin-top: 3px; }

  /* Evidence Panel (Right side) */
  .evidence-panel { position: absolute; top: 20px; right: 30px; width: 260px; height: 860px; background: #f8fafc; border: 1.5px solid #cbd5e1; border-radius: 8px; padding: 14px; }
  .panel-header { font-size: 13px; font-weight: 800; color: #0f172a; text-transform: uppercase; border-bottom: 2px solid #e2e8f0; padding-bottom: 6px; margin-bottom: 12px; }
  .evidence-item { font-size: 11px; color: #1e293b; font-weight: 600; margin-bottom: 10px; line-height: 1.3; display: flex; align-items: flex-start; }
  .evidence-item span.check { color: #16a34a; font-weight: 800; margin-right: 6px; font-size: 12px; }
  .solver-sub { font-size: 9.5px; color: #64748b; font-weight: 500; display: block; margin-top: 1px; }
  .evidence-footer { position: absolute; bottom: 12px; left: 12px; right: 12px; background: #fef3c7; border: 1px solid #f59e0b; border-radius: 4px; padding: 6px; text-align: center; font-size: 9.5px; font-weight: 700; color: #b45309; }

  /* Diagram Legend (Right bottom) */
  .legend-box { position: absolute; top: 715px; right: 30px; width: 260px; background: #ffffff; border: 1.5px solid #cbd5e1; border-radius: 8px; padding: 10px; }
  .legend-title { font-size: 11px; font-weight: 800; color: #0f172a; text-transform: uppercase; margin-bottom: 6px; }
  .legend-row { display: flex; align-items: center; margin-bottom: 5px; font-size: 10px; color: #334155; font-weight: 600; }
  .legend-sample { width: 24px; height: 12px; margin-right: 8px; border-radius: 2px; }
  .sample-solid { background: #f1f5f9; border: 1.5px solid #475569; }
  .sample-dashed { background: #f8fafc; border: 1.5px dashed #94a3b8; }
  .sample-stateflow { background: #eff6ff; border: 1.5px solid #1d4ed8; }

  /* SVG Overlay for Connections & Geometry */
  svg.diagram-svg { position: absolute; top: 0; left: 0; width: 100%; height: 100%; pointer-events: none; z-index: 10; }
</style>
</head>
<body>
<div class="canvas">

  <!-- Title Header -->
  <div class="title-header">
    <div class="main-title">TECHNICAL APPROACH</div>
    <div class="subtitle">Hierarchical Supervisory Architecture for Adaptive Closed-Loop Planning</div>
    <div class="tagline">Perceive &rarr; Predict &rarr; Decide &rarr; Adapt &rarr; Optimize &rarr; Verify &rarr; Control (Closed-Loop Replanning)</div>
  </div>

  <!-- Evidence Panel -->
  <div class="evidence-panel">
    <div class="panel-header">PROTOTYPE EVIDENCE</div>
    <div class="evidence-item"><span class="check">&#10004;</span> Zero collision steps in tested deterministic scenarios</div>
    <div class="evidence-item"><span class="check">&#10004;</span> 150 m route completion in tested scenarios</div>
    <div class="evidence-item"><span class="check">&#10004;</span> QP solver &approx; 4&ndash;6.4 ms<br><span class="solver-sub">(solver time only)</span></div>
    <div class="evidence-item"><span class="check">&#10004;</span> Observation noise / FN / FP / delay modeled</div>
    <div class="evidence-item"><span class="check">&#10004;</span> Steering bias uncertainty modeled</div>

    <div style="margin-top: 15px; border-top: 1px solid #e2e8f0; padding-top: 10px;">
      <div style="font-size: 10px; font-weight: 800; color: #475569; text-transform: uppercase;">TECHNICAL HONESTY</div>
      <div style="font-size: 9.5px; color: #64748b; margin-top: 4px; line-height: 1.3;">
        &bull; Stateflow: Structurally verified SLX<br>
        &bull; Stage 5 execution: MATLAB decision layer<br>
        &bull; Hardware perception / fusion: Proposed
      </div>
    </div>

    <div class="evidence-footer">
      Prototype evidence &ne; production guarantee
    </div>
  </div>

  <!-- Legend Box -->
  <div class="legend-box">
    <div class="legend-title">ARCHITECTURE LEGEND</div>
    <div class="legend-row"><div class="legend-sample sample-solid"></div> Solid = Executed Prototype Abstraction</div>
    <div class="legend-row"><div class="legend-sample sample-dashed"></div> Dashed = Proposed SIH Extension</div>
    <div class="legend-row"><div class="legend-sample sample-stateflow"></div> Stateflow Supervisor (SLX Verified)</div>
  </div>

  <!-- Main Inline SVG Diagram -->
  <svg class="diagram-svg" viewBox="0 0 1920 1080">
    <defs>
      <!-- Arrowhead Definitions -->
      <marker id="arrow-blue" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto">
        <polygon points="0 0, 8 4, 0 8" fill="#1d4ed8"/>
      </marker>
      <marker id="arrow-slate" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto">
        <polygon points="0 0, 8 4, 0 8" fill="#475569"/>
      </marker>
      <marker id="arrow-red" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto">
        <polygon points="0 0, 8 4, 0 8" fill="#dc2626"/>
      </marker>
      <marker id="arrow-green" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto">
        <polygon points="0 0, 8 4, 0 8" fill="#16a34a"/>
      </marker>
    </defs>

    <!-- ===================================================================== -->
    <!-- PIPELINE BLOCK 1: PERCEPTION (X: 30, Y: 90, W: 220, H: 260) -->
    <!-- ===================================================================== -->
    <rect x="30" y="90" width="220" height="260" rx="8" fill="#f8fafc" stroke="#475569" stroke-width="2"/>
    <path d="M 30 90 L 250 90 L 250 120 L 30 120 Z" fill="#334155"/>
    <text x="140" y="111" fill="#ffffff" font-size="12" font-weight="bold" text-anchor="middle">PERCEPTION</text>

    <!-- Hardware Sub-boxes (Proposed Abstraction) -->
    <rect x="42" y="130" width="54" height="24" rx="4" fill="#f1f5f9" stroke="#94a3b8" stroke-width="1" stroke-dasharray="3,3"/>
    <text x="69" y="146" fill="#64748b" font-size="9" font-weight="bold" text-anchor="middle">Camera</text>
    <rect x="104" y="130" width="54" height="24" rx="4" fill="#f1f5f9" stroke="#94a3b8" stroke-width="1" stroke-dasharray="3,3"/>
    <text x="131" y="146" fill="#64748b" font-size="9" font-weight="bold" text-anchor="middle">LiDAR</text>
    <rect x="166" y="130" width="54" height="24" rx="4" fill="#f1f5f9" stroke="#94a3b8" stroke-width="1" stroke-dasharray="3,3"/>
    <text x="193" y="146" fill="#64748b" font-size="9" font-weight="bold" text-anchor="middle">Radar</text>

    <!-- Executed Prototype Modules -->
    <rect x="42" y="165" width="196" height="36" rx="4" fill="#ffffff" stroke="#475569" stroke-width="1.5"/>
    <text x="140" y="187" fill="#0f172a" font-size="11" font-weight="bold" text-anchor="middle">Observation Model</text>
    <rect x="42" y="210" width="196" height="36" rx="4" fill="#ffffff" stroke="#475569" stroke-width="1.5"/>
    <text x="140" y="232" fill="#0f172a" font-size="11" font-weight="bold" text-anchor="middle">Multi-Vehicle Detector</text>

    <rect x="42" y="256" width="196" height="82" rx="4" fill="#f1f5f9" stroke="#cbd5e1" stroke-width="1"/>
    <text x="50" y="272" fill="#475569" font-size="9.5" font-weight="bold">Prototype Abstraction:</text>
    <text x="50" y="288" fill="#64748b" font-size="8.5">&bull; Truth observations + noise</text>
    <text x="50" y="302" fill="#64748b" font-size="8.5">&bull; False pos / neg &amp; 0.1s delay</text>
    <text x="50" y="316" fill="#b45309" font-size="8.5" font-weight="bold">No direct camera/LiDAR hw</text>

    <!-- Connection: Perception -> State Estimation -->
    <line x1="250" y1="220" x2="272" y2="220" stroke="#475569" stroke-width="2" marker-end="url(#arrow-slate)"/>

    <!-- ===================================================================== -->
    <!-- PIPELINE BLOCK 2: STATE ESTIMATION (X: 275, Y: 90, W: 210, H: 260) -->
    <!-- ===================================================================== -->
    <rect x="275" y="90" width="210" height="260" rx="8" fill="#f8fafc" stroke="#475569" stroke-width="2"/>
    <path d="M 275 90 L 485 90 L 485 120 L 275 120 Z" fill="#334155"/>
    <text x="380" y="111" fill="#ffffff" font-size="12" font-weight="bold" text-anchor="middle">STATE ESTIMATION</text>

    <rect x="287" y="132" width="186" height="150" rx="6" fill="#ffffff" stroke="#cbd5e1" stroke-width="1.5"/>
    <text x="300" y="152" fill="#0f172a" font-size="10.5" font-weight="bold">&bull; Ego vehicle state [x, y, &theta;, v]</text>
    <text x="300" y="174" fill="#0f172a" font-size="10.5" font-weight="bold">&bull; Surrounding agent states</text>
    <text x="300" y="196" fill="#0f172a" font-size="10.5" font-weight="bold">&bull; Relative pos / vel [dx, dy, dvx]</text>
    <text x="300" y="218" fill="#0f172a" font-size="10.5" font-weight="bold">&bull; Agent heading &amp; alignment</text>
    <text x="300" y="240" fill="#0f172a" font-size="10.5" font-weight="bold">&bull; Tracking latency buffer</text>
    <text x="300" y="266" fill="#475569" font-size="9" font-style="italic">Prototype tracking abstraction</text>

    <rect x="287" y="292" width="186" height="46" rx="4" fill="#eff6ff" stroke="#93c5fd" stroke-width="1"/>
    <text x="380" y="310" fill="#1e40af" font-size="9" font-weight="bold" text-anchor="middle">Ego &amp; Agent State Vector</text>
    <text x="380" y="326" fill="#3b82f6" font-size="8.5" text-anchor="middle">Synchronized to t = k &middot; dt</text>

    <!-- Connection: State Estimation -> Prediction -->
    <line x1="485" y1="220" x2="507" y2="220" stroke="#475569" stroke-width="2" marker-end="url(#arrow-slate)"/>

    <!-- ===================================================================== -->
    <!-- PIPELINE BLOCK 3: PREDICTION + RISK (X: 510, Y: 90, W: 220, H: 260) -->
    <!-- ===================================================================== -->
    <rect x="510" y="90" width="220" height="260" rx="8" fill="#f8fafc" stroke="#475569" stroke-width="2"/>
    <path d="M 510 90 L 730 90 L 730 120 L 510 120 Z" fill="#334155"/>
    <text x="620" y="111" fill="#ffffff" font-size="12" font-weight="bold" text-anchor="middle">PREDICTION + RISK</text>

    <rect x="522" y="130" width="196" height="34" rx="4" fill="#ffffff" stroke="#475569" stroke-width="1.5"/>
    <text x="620" y="151" fill="#0f172a" font-size="11" font-weight="bold" text-anchor="middle">RiskPredictor</text>
    <rect x="522" y="170" width="196" height="34" rx="4" fill="#ffffff" stroke="#475569" stroke-width="1.5"/>
    <text x="620" y="191" fill="#0f172a" font-size="11" font-weight="bold" text-anchor="middle">InteractionClassifier</text>

    <rect x="522" y="212" width="196" height="70" rx="4" fill="#eff6ff" stroke="#3b82f6" stroke-width="1.5"/>
    <text x="620" y="230" fill="#1d4ed8" font-size="10" font-weight="bold" text-anchor="middle">EXACT HORIZON SPECIFICATION</text>
    <text x="620" y="248" fill="#1e293b" font-size="10" font-weight="bold" text-anchor="middle">CV Kinematic Model | Np = 10</text>
    <text x="620" y="264" fill="#1e293b" font-size="10" font-weight="bold" text-anchor="middle">dt = 0.10 s &rarr; 1.0 s Horizon</text>

    <text x="524" y="300" fill="#475569" font-size="9" font-weight="bold">Risk Outputs:</text>
    <text x="524" y="315" fill="#64748b" font-size="8.5">&bull; min_ttc &bull; lead_dx &bull; W_avail &bull; Class</text>

    <!-- Signal Arrows: Prediction -> Stateflow Inputs -->
    <path d="M 730 200 L 762 200" stroke="#1d4ed8" stroke-width="2.5" marker-end="url(#arrow-blue)"/>
    <text x="746" y="192" fill="#1d4ed8" font-size="8.5" font-weight="bold" text-anchor="middle">Signals</text>

    <!-- ===================================================================== -->
    <!-- CENTRAL STATEFLOW SUPERVISOR (X: 765, Y: 90, W: 560, H: 450) -->
    <!-- ===================================================================== -->
    <rect x="765" y="90" width="560" height="450" rx="10" fill="#eff6ff" stroke="#1d4ed8" stroke-width="2.5"/>
    <path d="M 765 90 L 1325 90 L 1325 125 L 765 125 Z" fill="#1d4ed8"/>
    <text x="1045" y="112" fill="#ffffff" font-size="13" font-weight="800" text-anchor="middle">STATEFLOW SUPERVISOR [SIH26037_SupervisoryArchitecture.slx]</text>
    <text x="1045" y="139" fill="#1e40af" font-size="10" font-weight="bold" text-anchor="middle">Hierarchical Behavioral Decision Layer &bull; EXCLUSIVE_OR Decomposition</text>

    <!-- Disclaimer Badge -->
    <rect x="780" y="148" width="530" height="24" rx="4" fill="#fef3c7" stroke="#f59e0b" stroke-width="1.2"/>
    <text x="1045" y="164" fill="#b45309" font-size="9.5" font-weight="bold" text-anchor="middle">STRUCTURALLY VERIFIED IN SLX  |  STAGE 5 RUNTIME: MATLAB CoordinationDecisionLayer.m</text>

    <!-- Top-Level State: INIT -->
    <rect x="780" y="185" width="85" height="35" rx="5" fill="#ffffff" stroke="#334155" stroke-width="1.5"/>
    <text x="822" y="206" fill="#0f172a" font-size="10" font-weight="bold" text-anchor="middle">INIT</text>

    <!-- Transition: INIT -> AUTONOMOUS_NAVIGATION -->
    <line x1="865" y1="202" x2="890" y2="202" stroke="#475569" stroke-width="1.5" marker-end="url(#arrow-slate)"/>
    <text x="877" y="196" fill="#475569" font-size="7.5" font-weight="bold" text-anchor="middle">[lead_dx &gt; 0]</text>

    <!-- Super-State: AUTONOMOUS_NAVIGATION (X: 892, Y: 185, W: 298, H: 340) -->
    <rect x="892" y="185" width="298" height="340" rx="8" fill="#ffffff" stroke="#2563eb" stroke-width="1.8"/>
    <path d="M 892 185 L 1190 185 L 1190 210 L 892 210 Z" fill="#dbeafe"/>
    <text x="1041" y="202" fill="#1e40af" font-size="10" font-weight="bold" text-anchor="middle">AUTONOMOUS_NAVIGATION (Super-State)</text>

    <!-- Sub-States inside AUTONOMOUS_NAVIGATION -->
    <!-- CRUISE -->
    <rect x="905" y="222" width="75" height="42" rx="4" fill="#f8fafc" stroke="#334155" stroke-width="1.2"/>
    <text x="942" y="240" fill="#0f172a" font-size="9.5" font-weight="bold" text-anchor="middle">CRUISE</text>
    <text x="942" y="254" fill="#64748b" font-size="8" text-anchor="middle">v_tgt = 8.0m/s</text>

    <!-- FOLLOW -->
    <rect x="1005" y="222" width="75" height="42" rx="4" fill="#f8fafc" stroke="#334155" stroke-width="1.2"/>
    <text x="1042" y="240" fill="#0f172a" font-size="9.5" font-weight="bold" text-anchor="middle">FOLLOW</text>
    <text x="1042" y="254" fill="#64748b" font-size="8" text-anchor="middle">Safe Headway</text>

    <!-- RECOVER -->
    <rect x="1102" y="222" width="75" height="42" rx="4" fill="#f8fafc" stroke="#334155" stroke-width="1.2"/>
    <text x="1139" y="240" fill="#0f172a" font-size="9.5" font-weight="bold" text-anchor="middle">RECOVER</text>
    <text x="1139" y="254" fill="#64748b" font-size="8" text-anchor="middle">Re-center</text>

    <!-- YIELD -->
    <rect x="905" y="322" width="75" height="42" rx="4" fill="#f8fafc" stroke="#334155" stroke-width="1.2"/>
    <text x="942" y="340" fill="#0f172a" font-size="9.5" font-weight="bold" text-anchor="middle">YIELD</text>
    <text x="942" y="354" fill="#64748b" font-size="8" text-anchor="middle">Decel / Yield</text>

    <!-- OVERTAKE -->
    <rect x="1005" y="322" width="75" height="42" rx="4" fill="#fef3c7" stroke="#d97706" stroke-width="1.8"/>
    <text x="1042" y="340" fill="#92400e" font-size="9.5" font-weight="bold" text-anchor="middle">OVERTAKE</text>
    <text x="1042" y="354" fill="#b45309" font-size="8" text-anchor="middle">Latched Pass</text>

    <!-- OVERTAKE GATE Side Callout -->
    <rect x="1090" y="302" width="92" height="85" rx="4" fill="#ffffff" stroke="#d97706" stroke-width="1.2" stroke-dasharray="3,2"/>
    <text x="1136" y="317" fill="#b45309" font-size="8.5" font-weight="bold" text-anchor="middle">OVERTAKE GATE</text>
    <text x="1095" y="331" fill="#78350f" font-size="7.5">&bull; W_avail &ge; 4.10m</text>
    <text x="1095" y="343" fill="#78350f" font-size="7.5">&bull; Oncoming TTC &gt; 6s</text>
    <text x="1095" y="355" fill="#78350f" font-size="7.5">&bull; 6 &le; dx &le; 28m</text>
    <text x="1095" y="367" fill="#78350f" font-size="7.5">&bull; v_ego &ge; 1.5m/s</text>
    <text x="1095" y="379" fill="#16a34a" font-size="7.5" font-weight="bold">&#10004; Gated Pass</text>

    <!-- Internal Transitions inside AUTONOMOUS_NAVIGATION -->
    <!-- CRUISE -> FOLLOW -->
    <line x1="980" y1="243" x2="1003" y2="243" stroke="#475569" stroke-width="1.2" marker-end="url(#arrow-slate)"/>
    <text x="991" y="238" fill="#475569" font-size="7" text-anchor="middle">28&lt;dx&le;40</text>

    <!-- CRUISE -> YIELD -->
    <line x1="930" y1="264" x2="930" y2="320" stroke="#475569" stroke-width="1.2" marker-end="url(#arrow-slate)"/>
    <text x="915" y="292" fill="#475569" font-size="7" text-anchor="middle">TTC&le;6s</text>

    <!-- FOLLOW -> OVERTAKE -->
    <line x1="1042" y1="264" x2="1042" y2="320" stroke="#d97706" stroke-width="1.5" marker-end="url(#arrow-slate)"/>
    <text x="1062" y="292" fill="#b45309" font-size="7" font-weight="bold">Gate Clear</text>

    <!-- OVERTAKE -> RECOVER -->
    <path d="M 1080 343 L 1139 343 L 1139 266" fill="none" stroke="#16a34a" stroke-width="1.2" marker-end="url(#arrow-green)"/>
    <text x="1145" y="310" fill="#16a34a" font-size="7" font-weight="bold">Pass dx&ge;7.5m</text>

    <!-- OVERTAKE -> YIELD (Abort) -->
    <line x1="1005" y1="343" x2="982" y2="343" stroke="#dc2626" stroke-width="1.2" marker-end="url(#arrow-red)"/>
    <text x="993" y="338" fill="#dc2626" font-size="7" text-anchor="middle">Abort</text>

    <!-- YIELD -> FOLLOW -->
    <path d="M 980 330 L 995 330 L 995 255 L 1003 255" fill="none" stroke="#475569" stroke-width="1.2" marker-end="url(#arrow-slate)"/>

    <!-- RECOVER -> CRUISE -->
    <path d="M 1139 222 L 1139 213 L 942 213 L 942 220" fill="none" stroke="#475569" stroke-width="1.2" marker-end="url(#arrow-slate)"/>

    <!-- Top-Level External States (Right side of Stateflow box) -->
    <!-- EMERGENCY -->
    <rect x="1205" y="215" width="105" height="50" rx="5" fill="#fff1f2" stroke="#dc2626" stroke-width="1.8"/>
    <text x="1257" y="237" fill="#991b1b" font-size="10" font-weight="bold" text-anchor="middle">EMERGENCY</text>
    <text x="1257" y="253" fill="#b91c1c" font-size="8" text-anchor="middle">Override Active</text>

    <!-- SAFE STOP -->
    <rect x="1205" y="315" width="105" height="50" rx="5" fill="#fff1f2" stroke="#dc2626" stroke-width="1.8"/>
    <text x="1257" y="337" fill="#991b1b" font-size="10" font-weight="bold" text-anchor="middle">SAFE_STOP</text>
    <text x="1257" y="353" fill="#b91c1c" font-size="8" text-anchor="middle">Vehicle Halted</text>

    <!-- MISSION COMPLETE -->
    <rect x="1205" y="420" width="105" height="45" rx="5" fill="#f0fdf4" stroke="#16a34a" stroke-width="1.8"/>
    <text x="1257" y="440" fill="#14532d" font-size="9.5" font-weight="bold" text-anchor="middle">MISSION</text>
    <text x="1257" y="453" fill="#15803d" font-size="9.5" font-weight="bold" text-anchor="middle">COMPLETE</text>

    <!-- Critical Override Transitions -->
    <!-- AUTONOMOUS_NAVIGATION -> EMERGENCY -->
    <path d="M 1190 240 L 1203 240" stroke="#dc2626" stroke-width="2" marker-end="url(#arrow-red)"/>
    <text x="1197" y="232" fill="#dc2626" font-size="7.5" font-weight="bold" text-anchor="middle">Risk</text>

    <!-- EMERGENCY -> SAFE_STOP -->
    <line x1="1257" y1="265" x2="1257" y2="313" stroke="#dc2626" stroke-width="2" marker-end="url(#arrow-red)"/>
    <text x="1272" y="290" fill="#dc2626" font-size="7.5" font-weight="bold">v&lt;0.1m/s</text>

    <!-- AUTONOMOUS_NAVIGATION -> MISSION COMPLETE -->
    <path d="M 1190 442 L 1203 442" stroke="#16a34a" stroke-width="2" marker-end="url(#arrow-green)"/>
    <text x="1196" y="435" fill="#16a34a" font-size="7.5" font-weight="bold" text-anchor="middle">x&ge;150m</text>

    <!-- Signal Outflow: Stateflow -> Planning -->
    <path d="M 1325 280 L 1352 280" stroke="#1d4ed8" stroke-width="2.5" marker-end="url(#arrow-blue)"/>
    <text x="1338" y="272" fill="#1d4ed8" font-size="8.5" font-weight="bold" text-anchor="middle">Commands</text>

    <!-- ===================================================================== -->
    <!-- PIPELINE BLOCK 4: ADAPTIVE FREE SPACE (X: 1355, Y: 90, W: 250, H: 170) -->
    <!-- ===================================================================== -->
    <rect x="1355" y="90" width="250" height="170" rx="8" fill="#f8fafc" stroke="#475569" stroke-width="2"/>
    <path d="M 1355 90 L 1605 90 L 1605 120 L 1355 120 Z" fill="#334155"/>
    <text x="1480" y="111" fill="#ffffff" font-size="11" font-weight="bold" text-anchor="middle">ADAPTIVE FREE-SPACE + TOPOLOGY</text>

    <text x="1367" y="137" fill="#0f172a" font-size="10" font-weight="bold">&bull; FreeSpaceMap &bull; FreeSpaceBoundProvider</text>

    <!-- Mini Road Corridor Drawing -->
    <rect x="1367" y="145" width="226" height="55" rx="4" fill="#ffffff" stroke="#cbd5e1" stroke-width="1"/>
    <!-- Road Bounds -->
    <line x1="1375" y1="155" x2="1585" y2="155" stroke="#94a3b8" stroke-width="1.5" stroke-dasharray="4,2"/>
    <line x1="1375" y1="190" x2="1585" y2="190" stroke="#94a3b8" stroke-width="1.5" stroke-dasharray="4,2"/>
    <text x="1380" y="152" fill="#64748b" font-size="7">y_max(s) Road Boundary</text>
    <text x="1380" y="198" fill="#64748b" font-size="7">y_min(s) Road Boundary</text>

    <!-- Obstacle -->
    <rect x="1460" y="172" width="25" height="15" fill="#ef4444" rx="2"/>
    <text x="1472" y="183" fill="#ffffff" font-size="7" text-anchor="middle">Obs</text>

    <!-- Available Corridor -->
    <path d="M 1375 162 L 1450 162 L 1490 157 L 1585 157" fill="none" stroke="#2563eb" stroke-width="1.8"/>
    <text x="1520" y="170" fill="#1d4ed8" font-size="8" font-weight="bold">Dynamic Corridor</text>

    <text x="1367" y="218" fill="#475569" font-size="9">&bull; Open-gap detection &bull; Road-bound constraints</text>
    <text x="1367" y="232" fill="#0f172a" font-size="9" font-weight="bold">TOPOLOGY: Left / Right / Follow</text>

    <!-- Connection: Free Space -> CA-CRC Planner -->
    <line x1="1480" y1="260" x2="1480" y2="280" stroke="#475569" stroke-width="2" marker-end="url(#arrow-slate)"/>

    <!-- ===================================================================== -->
    <!-- PIPELINE BLOCK 5: CA-CRC PLANNER (X: 1355, Y: 282, W: 250, H: 140) -->
    <!-- ===================================================================== -->
    <rect x="1355" y="282" width="250" height="140" rx="8" fill="#f8fafc" stroke="#475569" stroke-width="2"/>
    <path d="M 1355 282 L 1605 282 L 1605 312 L 1355 312 Z" fill="#334155"/>
    <text x="1480" y="303" fill="#ffffff" font-size="11.5" font-weight="bold" text-anchor="middle">CA-CRC PLANNER</text>
    <text x="1480" y="325" fill="#1e40af" font-size="9" font-weight="bold" text-anchor="middle">Context-Adaptive Collision-Risk Planning</text>

    <text x="1367" y="342" fill="#0f172a" font-size="9.5">&bull; Reference trajectory generation</text>
    <text x="1367" y="357" fill="#0f172a" font-size="9.5">&bull; Corridor &amp; topology bounds</text>
    <text x="1367" y="372" fill="#0f172a" font-size="9.5">&bull; Obstacle avoidance constraints</text>
    <text x="1367" y="387" fill="#0f172a" font-size="9.5">&bull; Soft slack handling for feasibility</text>

    <!-- Connection: CA-CRC -> QP-MPC -->
    <line x1="1480" y1="422" x2="1480" y2="442" stroke="#475569" stroke-width="2" marker-end="url(#arrow-slate)"/>

    <!-- ===================================================================== -->
    <!-- PIPELINE BLOCK 6: QP-MPC (X: 1355, Y: 444, W: 250, H: 150) -->
    <!-- ===================================================================== -->
    <rect x="1355" y="444" width="250" height="150" rx="8" fill="#f8fafc" stroke="#475569" stroke-width="2"/>
    <path d="M 1355 444 L 1605 444 L 1605 474 L 1355 474 Z" fill="#334155"/>
    <text x="1480" y="465" fill="#ffffff" font-size="12" font-weight="bold" text-anchor="middle">QP-MPC OPTIMIZER</text>

    <rect x="1367" y="482" width="226" height="42" rx="4" fill="#ffffff" stroke="#cbd5e1" stroke-width="1"/>
    <text x="1480" y="498" fill="#0f172a" font-size="10" font-weight="bold" text-anchor="middle">Hildreth Dual QP Solver</text>
    <text x="1480" y="514" fill="#1e293b" font-size="9.5" text-anchor="middle">&delta; &in; [-35&deg;, +35&deg;]  |  a &in; [-6, +3] m/s&sup2;</text>

    <rect x="1367" y="530" width="226" height="52" rx="4" fill="#f1f5f9" stroke="#cbd5e1" stroke-width="1"/>
    <text x="1375" y="546" fill="#15803d" font-size="10" font-weight="bold">Prototype Solver Time: &approx; 4&ndash;6.4 ms</text>
    <text x="1375" y="562" fill="#b45309" font-size="8.5" font-weight="bold">(Label: Hildreth QP solver execution time only)</text>
    <text x="1375" y="574" fill="#64748b" font-size="8">Output: u_mpc [steering, acceleration]</text>

    <!-- Connection: QP-MPC -> Safety Filter -->
    <path d="M 1355 520 L 1315 520 L 1315 598 L 1270 598" fill="none" stroke="#475569" stroke-width="2" marker-end="url(#arrow-slate)"/>

    <!-- ===================================================================== -->
    <!-- PIPELINE BLOCK 7: INDEPENDENT SAFETY FILTER (X: 1010, Y: 560, W: 250, H: 140) -->
    <!-- ===================================================================== -->
    <rect x="1010" y="560" width="250" height="140" rx="8" fill="#fff1f2" stroke="#dc2626" stroke-width="2"/>
    <path d="M 1010 560 L 1260 560 L 1260 590 L 1010 590 Z" fill="#b91c1c"/>
    <text x="1135" y="581" fill="#ffffff" font-size="11" font-weight="bold" text-anchor="middle">INDEPENDENT SAFETY FILTER</text>

    <text x="1022" y="608" fill="#0f172a" font-size="9.5" font-weight="bold">&bull; Layer 2 Emergency Override Safeguard</text>
    <text x="1022" y="623" fill="#0f172a" font-size="9">&bull; Forward trajectory projection &amp; OBB/SAT clearance</text>
    <text x="1022" y="637" fill="#0f172a" font-size="9">&bull; Evaluates same Stage 5 envelopes independently</text>

    <rect x="1022" y="646" width="226" height="42" rx="4" fill="#ffffff" stroke="#fca5a5" stroke-width="1"/>
    <text x="1030" y="661" fill="#15803d" font-size="9" font-weight="bold">SAFE: Execute u_mpc</text>
    <text x="1030" y="677" fill="#b91c1c" font-size="9" font-weight="bold">UNSAFE: Override &rarr; Hard Brake / Safe Stop</text>

    <!-- Connection: Safety Filter -> Vehicle Plant -->
    <line x1="1010" y1="630" x2="960" y2="630" stroke="#dc2626" stroke-width="2" marker-end="url(#arrow-red)"/>
    <text x="985" y="622" fill="#dc2626" font-size="8.5" font-weight="bold" text-anchor="middle">u_safe</text>

    <!-- ===================================================================== -->
    <!-- PIPELINE BLOCK 8: VEHICLE DYNAMICS & ENVIRONMENT (X: 710, Y: 560, W: 240, H: 140) -->
    <!-- ===================================================================== -->
    <rect x="710" y="560" width="240" height="140" rx="8" fill="#f8fafc" stroke="#475569" stroke-width="2"/>
    <path d="M 710 560 L 950 560 L 950 590 L 710 590 Z" fill="#334155"/>
    <text x="830" y="581" fill="#ffffff" font-size="11" font-weight="bold" text-anchor="middle">VEHICLE DYNAMICS &amp; PLANT</text>

    <text x="722" y="608" fill="#0f172a" font-size="10" font-weight="bold">&bull; BicycleModel (Ego Dynamics)</text>
    <text x="722" y="623" fill="#475569" font-size="9">State Vector: [x, y, &theta;, v, &delta;]</text>
    <text x="722" y="637" fill="#475569" font-size="9">Controls: acceleration, steering angle</text>

    <!-- Vehicle Graphic -->
    <rect x="722" y="648" width="50" height="24" rx="4" fill="#3b82f6" stroke="#1d4ed8" stroke-width="1"/>
    <circle cx="732" cy="672" r="4" fill="#0f172a"/>
    <circle cx="762" cy="672" r="4" fill="#0f172a"/>
    <text x="747" y="663" fill="#ffffff" font-size="8" font-weight="bold" text-anchor="middle">EGO</text>

    <text x="780" y="660" fill="#0f172a" font-size="9" font-weight="bold">ENVIRONMENT &amp; ROAD USERS:</text>
    <text x="780" y="674" fill="#64748b" font-size="8">Vehicles, Two-Wheelers, Pedestrians, Cattle</text>

    <!-- ===================================================================== -->
    <!-- CLOSED LOOP FEEDBACK ARROW (Vehicle -> Perception) -->
    <!-- ===================================================================== -->
    <path d="M 710 630 L 140 630 L 140 358" fill="none" stroke="#2563eb" stroke-width="2.5" stroke-dasharray="6,4" marker-end="url(#arrow-blue)"/>
    <rect x="360" y="618" width="220" height="24" rx="4" fill="#eff6ff" stroke="#3b82f6" stroke-width="1"/>
    <text x="470" y="634" fill="#1d4ed8" font-size="9.5" font-weight="bold" text-anchor="middle">CLOSED-LOOP REPLANNING &amp; FEEDBACK LOOP</text>

    <!-- ===================================================================== -->
    <!-- PROPOSED SIH COMPLETION EXTENSIONS CONTAINER (X: 30, Y: 715, W: 1575, H: 140) -->
    <!-- ===================================================================== -->
    <rect x="30" y="715" width="1575" height="140" rx="8" fill="#f8fafc" stroke="#94a3b8" stroke-width="2" stroke-dasharray="6,4"/>
    <path d="M 30 715 L 1605 715 L 1605 742 L 30 742 Z" fill="#64748b"/>
    <text x="817" y="733" fill="#ffffff" font-size="11" font-weight="bold" text-anchor="middle">PROPOSED SIH COMPLETION EXTENSIONS  [DASHED BORDER = NOT IN CURRENT PROTOTYPE RUNTIME]</text>

    <text x="817" y="757" fill="#475569" font-size="9" font-style="italic" text-anchor="middle">From prototype analytical abstraction &rarr; realistic closed-loop deployment model</text>

    <!-- 5 Dashed Modules -->
    <!-- 1 -->
    <rect x="45" y="768" width="290" height="75" rx="6" fill="#ffffff" stroke="#94a3b8" stroke-width="1.5" stroke-dasharray="4,3"/>
    <text x="190" y="788" fill="#334155" font-size="10" font-weight="bold" text-anchor="middle">1. Real Camera / LiDAR / Radar</text>
    <text x="190" y="804" fill="#64748b" font-size="8.5" text-anchor="middle">Physical Hardware Perception Suite</text>
    <text x="190" y="820" fill="#94a3b8" font-size="8" text-anchor="middle">(Point clouds, image object detection)</text>

    <!-- 2 -->
    <rect x="355" y="768" width="290" height="75" rx="6" fill="#ffffff" stroke="#94a3b8" stroke-width="1.5" stroke-dasharray="4,3"/>
    <text x="500" y="788" fill="#334155" font-size="10" font-weight="bold" text-anchor="middle">2. EKF / UKF Sensor Fusion</text>
    <text x="500" y="804" fill="#64748b" font-size="8.5" text-anchor="middle">Multi-Sensor Tracking &amp; Filtering</text>
    <text x="500" y="820" fill="#94a3b8" font-size="8" text-anchor="middle">(Covariance-weighted state estimation)</text>

    <!-- 3 -->
    <rect x="665" y="768" width="290" height="75" rx="6" fill="#ffffff" stroke="#94a3b8" stroke-width="1.5" stroke-dasharray="4,3"/>
    <text x="810" y="788" fill="#334155" font-size="10" font-weight="bold" text-anchor="middle">3. RoadRunner Co-Simulation</text>
    <text x="810" y="804" fill="#64748b" font-size="8.5" text-anchor="middle">High-Fidelity 3D Environment</text>
    <text x="810" y="820" fill="#94a3b8" font-size="8" text-anchor="middle">(Automotive synthetic scene generation)</text>

    <!-- 4 -->
    <rect x="975" y="768" width="290" height="75" rx="6" fill="#ffffff" stroke="#94a3b8" stroke-width="1.5" stroke-dasharray="4,3"/>
    <text x="1120" y="788" fill="#334155" font-size="10" font-weight="bold" text-anchor="middle">4. Intersection + Merge Logic</text>
    <text x="1120" y="804" fill="#64748b" font-size="8.5" text-anchor="middle">Complex Junction Coordination</text>
    <text x="1120" y="820" fill="#94a3b8" font-size="8" text-anchor="middle">(Multi-agent right-of-way negotiation)</text>

    <!-- 5 -->
    <rect x="1285" y="768" width="305" height="75" rx="6" fill="#ffffff" stroke="#94a3b8" stroke-width="1.5" stroke-dasharray="4,3"/>
    <text x="1437" y="788" fill="#334155" font-size="10" font-weight="bold" text-anchor="middle">5. CBF Safety Safeguard Layer</text>
    <text x="1437" y="804" fill="#64748b" font-size="8.5" text-anchor="middle">Control Barrier Function Layer</text>
    <text x="1437" y="820" fill="#94a3b8" font-size="8" text-anchor="middle">(Formal set-invariance safety bounds)</text>

    <!-- ===================================================================== -->
    <!-- BOTTOM SCENARIO STRIP (X: 30, Y: 870, W: 1860, H: 180) -->
    <!-- ===================================================================== -->
    <rect x="30" y="870" width="1860" height="180" rx="8" fill="#0f172a"/>
    <text x="50" y="895" fill="#f8fafc" font-size="13" font-weight="800" letter-spacing="0.5">SIH26037 TARGET SCENARIOS &bull; OPERATIONAL DESIGN DOMAINS (ODD)</text>
    <text x="1870" y="895" fill="#94a3b8" font-size="9" font-weight="bold" text-anchor="end">Slide 2 Engineering Artifact</text>

    <!-- 5 Scenario Cards -->
    <!-- Card 1 -->
    <rect x="50" y="910" width="345" height="120" rx="6" fill="#1e293b" stroke="#334155" stroke-width="1.5"/>
    <text x="65" y="932" fill="#38bdf8" font-size="11" font-weight="bold">01  UNMARKED VILLAGE ROAD</text>
    <text x="65" y="952" fill="#e2e8f0" font-size="9.5" font-weight="bold">Executed Stage 5 Baseline Coverage</text>
    <text x="65" y="970" fill="#94a3b8" font-size="8.5">&bull; No lane markings / irregular boundaries</text>
    <text x="65" y="985" fill="#94a3b8" font-size="8.5">&bull; FreeSpaceMap corridor derivation</text>
    <text x="65" y="1000" fill="#94a3b8" font-size="8.5">&bull; Single &amp; multi-agent interactions</text>

    <!-- Card 2 -->
    <rect x="415" y="910" width="345" height="120" rx="6" fill="#1e293b" stroke="#334155" stroke-width="1.5"/>
    <text x="430" y="932" fill="#94a3b8" font-size="11" font-weight="bold">02  UNSIGNALIZED URBAN INTERSECTION</text>
    <text x="430" y="952" fill="#cbd5e1" font-size="9.5" font-weight="bold">Proposed SIH Extension Coverage</text>
    <text x="430" y="970" fill="#94a3b8" font-size="8.5">&bull; Cross-traffic yield &amp; gap acceptance</text>
    <text x="430" y="985" fill="#94a3b8" font-size="8.5">&bull; Blind spot occlusion management</text>
    <text x="430" y="1000" fill="#94a3b8" font-size="8.5">&bull; Multi-directional conflict resolution</text>

    <!-- Card 3 -->
    <rect x="780" y="910" width="345" height="120" rx="6" fill="#1e293b" stroke="#334155" stroke-width="1.5"/>
    <text x="795" y="932" fill="#94a3b8" font-size="11" font-weight="bold">03  HIGHWAY MERGE</text>
    <text x="795" y="952" fill="#cbd5e1" font-size="9.5" font-weight="bold">Proposed SIH Extension Coverage</text>
    <text x="795" y="970" fill="#94a3b8" font-size="8.5">&bull; High-speed ramp trajectory matching</text>
    <text x="795" y="985" fill="#94a3b8" font-size="8.5">&bull; Cooperative headway insertion</text>
    <text x="795" y="1000" fill="#94a3b8" font-size="8.5">&bull; Variable speed corridor adaptation</text>

    <!-- Card 4 -->
    <rect x="1145" y="910" width="345" height="120" rx="6" fill="#1e293b" stroke="#334155" stroke-width="1.5"/>
    <text x="1160" y="932" fill="#38bdf8" font-size="11" font-weight="bold">04  DENSE MARKET TRAFFIC</text>
    <text x="1160" y="952" fill="#e2e8f0" font-size="9.5" font-weight="bold">Executed Stage 5 Baseline Coverage</text>
    <text x="1160" y="970" fill="#94a3b8" font-size="8.5">&bull; Slow multi-agent squeezing</text>
    <text x="1160" y="985" fill="#94a3b8" font-size="8.5">&bull; Two-wheelers &amp; pedestrian bypass</text>
    <text x="1160" y="1000" fill="#94a3b8" font-size="8.5">&bull; Stage 4 CA-CRC + SafetyFilter</text>

    <!-- Card 5 -->
    <rect x="1510" y="910" width="340" height="120" rx="6" fill="#1e293b" stroke="#334155" stroke-width="1.5"/>
    <text x="1525" y="932" fill="#38bdf8" font-size="11" font-weight="bold">05  CATTLE CROSSING</text>
    <text x="1525" y="952" fill="#e2e8f0" font-size="9.5" font-weight="bold">Executed Stage 5 Baseline Coverage</text>
    <text x="1525" y="970" fill="#94a3b8" font-size="8.5">&bull; Static &amp; low-speed obstacle intrusion</text>
    <text x="1525" y="985" fill="#94a3b8" font-size="8.5">&bull; Autonomous YIELD &amp; standstill stop</text>
    <text x="1525" y="1000" fill="#94a3b8" font-size="8.5">&bull; Resume navigation upon clear path</text>
  </svg>
</div>
</body>
</html>
"""

    output_dir = "artifacts"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    html_path = os.path.join(output_dir, "SIH26037_Slide2_Architecture.html")
    svg_path = os.path.join(output_dir, "SIH26037_Slide2_Architecture.svg")
    png_path = os.path.join(output_dir, "SIH26037_Slide2_Architecture.png")

    # Write HTML
    with open(html_path, "w", encoding="utf-8") as f:
        f.write(html_content)
    print(f"SUCCESS: Written HTML to {html_path}")

    # Extract pure SVG content for standalone SVG file
    svg_start = html_content.find('<svg class="diagram-svg"')
    svg_end = html_content.find('</svg>') + len('</svg>')
    pure_svg_body = html_content[svg_start:svg_end]
    
    # Wrap in standalone SVG header
    standalone_svg = f"""<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1920 1080" width="1920" height="1080" style="background-color: #ffffff;">
  <style>
    text {{ font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; }}
  </style>
  {pure_svg_body}
</svg>
"""

    with open(svg_path, "w", encoding="utf-8") as f:
        f.write(standalone_svg)
    print(f"SUCCESS: Written SVG to {svg_path}")

    # Render PNG using google-chrome headless
    cmd = [
        "google-chrome",
        "--headless",
        "--disable-gpu",
        "--hide-scrollbars",
        f"--screenshot={png_path}",
        "--window-size=1920,1080",
        f"file://{os.path.abspath(html_path)}"
    ]
    try:
        subprocess.run(cmd, check=True)
        print(f"SUCCESS: Rendered PNG to {png_path}")
    except Exception as e:
        print(f"WARNING: Google Chrome PNG export error: {e}")
        # Fallback to convert or matlab if chrome failed
        try:
            subprocess.run(["convert", svg_path, png_path], check=True)
            print(f"SUCCESS: Rendered PNG via convert to {png_path}")
        except Exception as e2:
            print(f"ERROR: Convert failed: {e2}")

if __name__ == "__main__":
    generate_slide2_artifacts()
