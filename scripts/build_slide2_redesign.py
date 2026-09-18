import os
import subprocess

def generate_slide2_redesign():
    html_content = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>SIH26037 Slide 2 - Systems & State Machine Architecture</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; }
  body { background-color: #0b0f19; display: flex; justify-content: center; align-items: center; min-height: 100vh; padding: 0; overflow: hidden; }
  .canvas { width: 1920px; height: 1080px; background: #ffffff; position: relative; overflow: hidden; box-shadow: 0 25px 50px -12px rgba(0,0,0,0.6); }
  
  /* Slide Header */
  .slide-header { position: absolute; top: 16px; left: 30px; z-index: 20; }
  .header-title { font-size: 20px; font-weight: 900; color: #0f172a; letter-spacing: -0.5px; text-transform: uppercase; }
  .header-subtitle { font-size: 13.5px; font-weight: 700; color: #1d4ed8; margin-top: 1px; }
  .header-tagline { font-size: 11px; font-weight: 600; color: #64748b; margin-top: 2px; }

  /* Floating Mode Toggle */
  .mode-toggle { position: absolute; top: 16px; left: 600px; z-index: 30; background: #0f172a; color: #ffffff; border: none; padding: 5px 12px; border-radius: 4px; font-size: 10px; font-weight: 700; cursor: pointer; display: flex; align-items: center; gap: 6px; box-shadow: 0 2px 4px rgba(0,0,0,0.2); }
  .mode-toggle:hover { background: #1e293b; }

  /* Tooltip Box */
  .tooltip-panel { position: absolute; bottom: 20px; left: 375px; width: 1180px; height: 32px; background: #0f172a; border-radius: 4px; color: #f8fafc; display: flex; align-items: center; padding: 0 16px; font-size: 11px; font-weight: 600; z-index: 25; transition: all 0.2s ease; opacity: 0.95; }
  .tooltip-panel span.accent { color: #38bdf8; font-weight: 700; margin-right: 8px; }

  /* SVG Overlay */
  svg.architecture-svg { position: absolute; top: 0; left: 0; width: 100%; height: 100%; z-index: 10; }
  
  /* Presentation mode hide class */
  .presentation-mode .tooltip-panel { display: none !important; }
</style>
</head>
<body>
<div class="canvas" id="canvas">

  <!-- Header -->
  <div class="slide-header">
    <div class="header-title">TECHNICAL APPROACH</div>
    <div class="header-subtitle">Hierarchical Supervisory Architecture for Adaptive Closed-Loop Planning</div>
    <div class="header-tagline">Perceive &rarr; Predict &rarr; Decide &rarr; Adapt &rarr; Optimize &rarr; Verify &rarr; Control (Closed-Loop Replanning)</div>
  </div>

  <!-- Interactive Controls -->
  <button class="mode-toggle" onclick="togglePresentationMode()">
    <span id="mode-icon">&#128065;</span> <span id="mode-text">TOGGLE PRESENTATION VIEW</span>
  </button>

  <!-- Interactive Tooltip Panel -->
  <div class="tooltip-panel" id="tooltip">
    <span class="accent">INTERACTIVE DIAGRAM MASTER:</span> Hover over any module or transition to inspect formal implementation parameters and evidence.
  </div>

  <!-- SVG Architecture Visual -->
  <svg class="architecture-svg" viewBox="0 0 1920 1080">
    <defs>
      <!-- Markers -->
      <marker id="arr-blue" markerWidth="7" markerHeight="7" refX="6" refY="3.5" orient="auto">
        <polygon points="0 0, 7 3.5, 0 7" fill="#1d4ed8"/>
      </marker>
      <marker id="arr-slate" markerWidth="7" markerHeight="7" refX="6" refY="3.5" orient="auto">
        <polygon points="0 0, 7 3.5, 0 7" fill="#475569"/>
      </marker>
      <marker id="arr-red" markerWidth="7" markerHeight="7" refX="6" refY="3.5" orient="auto">
        <polygon points="0 0, 7 3.5, 0 7" fill="#dc2626"/>
      </marker>
      <marker id="arr-green" markerWidth="7" markerHeight="7" refX="6" refY="3.5" orient="auto">
        <polygon points="0 0, 7 3.5, 0 7" fill="#16a34a"/>
      </marker>
    </defs>

    <!-- ===================================================================== -->
    <!-- ZONE 1 — LEFT: UNSTRUCTURED ROAD CONTEXT (X: 30, Y: 75, W: 325, H: 980) -->
    <!-- ===================================================================== -->
    <rect x="30" y="75" width="325" height="980" rx="8" fill="#f8fafc" stroke="#cbd5e1" stroke-width="1.5"/>
    <path d="M 30 75 L 355 75 L 355 110 L 30 110 Z" fill="#1e293b"/>
    <text x="192" y="97" fill="#ffffff" font-size="12" font-weight="900" text-anchor="middle" letter-spacing="0.5">WHY ADAPTIVE PLANNING?</text>

    <!-- Context Card 1: No Markings -->
    <rect x="42" y="122" width="301" height="64" rx="6" fill="#ffffff" stroke="#e2e8f0" stroke-width="1.2"/>
    <text x="54" y="142" fill="#0f172a" font-size="10.5" font-weight="800">NO RELIABLE LANE MARKINGS</text>
    <text x="54" y="158" fill="#475569" font-size="9.5">Mixed road boundaries and informal lanes.</text>
    <text x="54" y="172" fill="#64748b" font-size="9.5">Corridor derived dynamically from space.</text>

    <!-- Context Card 2: Mixed Traffic -->
    <rect x="42" y="196" width="301" height="64" rx="6" fill="#ffffff" stroke="#e2e8f0" stroke-width="1.2"/>
    <text x="54" y="216" fill="#0f172a" font-size="10.5" font-weight="800">MIXED HETEROGENEOUS TRAFFIC</text>
    <text x="54" y="232" fill="#475569" font-size="9.5">Cars &bull; buses &bull; two-wheelers &bull; pedestrians &bull; cattle.</text>
    <text x="54" y="246" fill="#64748b" font-size="9.5">High velocity &amp; footprint variance.</text>

    <!-- Context Card 3: Irregular Interaction -->
    <rect x="42" y="270" width="301" height="64" rx="6" fill="#ffffff" stroke="#e2e8f0" stroke-width="1.2"/>
    <text x="54" y="290" fill="#0f172a" font-size="10.5" font-weight="800">IRREGULAR INTERACTION</text>
    <text x="54" y="306" fill="#475569" font-size="9.5">Informal merges &bull; sudden lateral cuts &bull; oncoming.</text>
    <text x="54" y="320" fill="#64748b" font-size="9.5">Requires risk-gated behavioral intent.</text>

    <!-- Context Card 4: Dynamic Drivable Space -->
    <rect x="42" y="344" width="301" height="64" rx="6" fill="#ffffff" stroke="#e2e8f0" stroke-width="1.2"/>
    <text x="54" y="364" fill="#0f172a" font-size="10.5" font-weight="800">DYNAMIC DRIVABLE SPACE</text>
    <text x="54" y="380" fill="#475569" font-size="9.5">Obstacles continuously shrink corridor width.</text>
    <text x="54" y="394" fill="#64748b" font-size="9.5">Cannot blindly follow centerline.</text>

    <!-- Emphasized Key Statement Box -->
    <rect x="42" y="422" width="301" height="95" rx="6" fill="#fef3c7" stroke="#f59e0b" stroke-width="1.5"/>
    <text x="54" y="444" fill="#92400e" font-size="10" font-weight="900">KEY CONCEPTUAL STATEMENT:</text>
    <text x="54" y="464" fill="#78350f" font-size="10.5" font-weight="700">&ldquo;The problem is not only obstacle</text>
    <text x="54" y="482" fill="#78350f" font-size="10.5" font-weight="700">avoidance &mdash; it is deciding WHERE,</text>
    <text x="54" y="500" fill="#78350f" font-size="10.5" font-weight="700">WHEN and HOW to move safely.&rdquo;</text>

    <!-- Target Scenarios Header -->
    <rect x="42" y="532" width="301" height="28" rx="4" fill="#0f172a"/>
    <text x="192" y="550" fill="#ffffff" font-size="10" font-weight="800" text-anchor="middle">SIH26037 TARGET SCENARIOS</text>

    <!-- Compact Scenario Tags -->
    <g font-size="9.5" font-weight="700">
      <rect x="42" y="568" width="301" height="28" rx="4" fill="#ffffff" stroke="#cbd5e1" stroke-width="1"/>
      <text x="54" y="586" fill="#0369a1">&bull; Village Road (Unmarked)</text>
      <text x="240" y="586" fill="#16a34a" font-size="8.5">[Prototype]</text>

      <rect x="42" y="602" width="301" height="28" rx="4" fill="#ffffff" stroke="#cbd5e1" stroke-width="1"/>
      <text x="54" y="620" fill="#475569">&bull; Unsignalized Urban Intersection</text>
      <text x="252" y="620" fill="#b45309" font-size="8.5">[Proposed]</text>

      <rect x="42" y="636" width="301" height="28" rx="4" fill="#ffffff" stroke="#cbd5e1" stroke-width="1"/>
      <text x="54" y="654" fill="#475569">&bull; Highway Merge</text>
      <text x="252" y="654" fill="#b45309" font-size="8.5">[Proposed]</text>

      <rect x="42" y="670" width="301" height="28" rx="4" fill="#ffffff" stroke="#cbd5e1" stroke-width="1"/>
      <text x="54" y="688" fill="#0369a1">&bull; Dense Market Traffic</text>
      <text x="240" y="688" fill="#16a34a" font-size="8.5">[Prototype]</text>

      <rect x="42" y="704" width="301" height="28" rx="4" fill="#ffffff" stroke="#cbd5e1" stroke-width="1"/>
      <text x="54" y="722" fill="#0369a1">&bull; Cattle Crossing</text>
      <text x="240" y="722" fill="#16a34a" font-size="8.5">[Prototype]</text>
    </g>

    <!-- Context Side Connector Arrow -->
    <path d="M 355 470 L 373 470" stroke="#0284c7" stroke-width="2" marker-end="url(#arr-blue)"/>

    <!-- ===================================================================== -->
    <!-- ZONE 2 — CENTER: THE ENGINEERING CORE (X: 375, Y: 75, W: 1180, H: 980) -->
    <!-- ===================================================================== -->
    
    <!-- --------------------------------------------------------------------- -->
    <!-- LAYER A — SCENE UNDERSTANDING (X: 375, Y: 75, W: 1180, H: 130) -->
    <!-- --------------------------------------------------------------------- -->
    <rect x="375" y="75" width="1180" height="130" rx="8" fill="#f8fafc" stroke="#475569" stroke-width="1.8"
          onmouseover="showTooltip('LAYER A: SCENE UNDERSTANDING — Derives truth-based observations with noise, false positives/negatives, tracking latency, and constant-velocity 1.0s risk predictions.')"
          onmouseout="resetTooltip()"/>
    <path d="M 375 75 L 1555 75 L 1555 100 L 375 100 Z" fill="#334155"/>
    <text x="965" y="92" fill="#ffffff" font-size="11" font-weight="900" text-anchor="middle" letter-spacing="0.5">LAYER A &mdash; SCENE UNDERSTANDING &amp; PREDICTION</text>

    <!-- Module 1: Observation -->
    <rect x="390" y="110" width="365" height="85" rx="6" fill="#ffffff" stroke="#cbd5e1" stroke-width="1.2"/>
    <text x="402" y="128" fill="#0f172a" font-size="10.5" font-weight="800">1. OBSERVATION &amp; DETECTION</text>
    <text x="402" y="145" fill="#1e293b" font-size="9.5" font-weight="700">&bull; ObservationModel &bull; MultiVehicleDetector</text>
    <text x="402" y="161" fill="#64748b" font-size="8.5">Truth-derived observations + noise, FN/FP &amp; 0.1s delay.</text>
    <text x="402" y="175" fill="#b45309" font-size="8.5" font-weight="700">Prototype analytical abstraction (no camera/LiDAR hw)</text>

    <!-- Module 2: State / Tracking -->
    <rect x="770" y="110" width="370" height="85" rx="6" fill="#ffffff" stroke="#cbd5e1" stroke-width="1.2"/>
    <text x="782" y="128" fill="#0f172a" font-size="10.5" font-weight="800">2. STATE ESTIMATION &amp; TRACKING</text>
    <text x="782" y="145" fill="#1e293b" font-size="9.5" font-weight="700">&bull; Ego State [x, y, &theta;, v] &bull; Agent States [x, y, vx, vy]</text>
    <text x="782" y="161" fill="#64748b" font-size="8.5">Relative positions [dx, dy] &amp; relative velocity [dvx].</text>
    <text x="782" y="175" fill="#475569" font-size="8.5">Tracking latency buffer &amp; boundary alignment.</text>

    <!-- Module 3: Prediction + Risk -->
    <rect x="1155" y="110" width="385" height="85" rx="6" fill="#ffffff" stroke="#3b82f6" stroke-width="1.5"/>
    <text x="1167" y="128" fill="#1d4ed8" font-size="10.5" font-weight="800">3. PREDICTION &amp; RISK ESTIMATION</text>
    <text x="1167" y="145" fill="#0f172a" font-size="9.5" font-weight="700">&bull; RiskPredictor &bull; InteractionClassifier</text>
    <rect x="1167" y="152" width="361" height="34" rx="4" fill="#eff6ff" stroke="#93c5fd" stroke-width="1"/>
    <text x="1347" y="166" fill="#1e40af" font-size="9" font-weight="800" text-anchor="middle">CV Kinematic Prediction | Np = 10 | dt = 0.10 s &rarr; 1.0 s Horizon</text>
    <text x="1347" y="180" fill="#1d4ed8" font-size="8.5" font-weight="700" text-anchor="middle">Outputs: lead_dx &bull; min_ttc &bull; W_avail &bull; Class &bull; Risk</text>

    <!-- Signal Bus Line Down into Stateflow -->
    <path d="M 965 195 L 965 220" stroke="#1d4ed8" stroke-width="2.5" marker-end="url(#arr-blue)"/>
    <rect x="850" y="200" width="230" height="18" rx="3" fill="#eff6ff" stroke="#3b82f6" stroke-width="1"/>
    <text x="965" y="213" fill="#1d4ed8" font-size="8.5" font-weight="800" text-anchor="middle">SIGNAL BUS: lead_dx, min_ttc, W_avail, v_ego, clearance, mpc_status</text>

    <!-- --------------------------------------------------------------------- -->
    <!-- LAYER B — STATEFLOW SUPERVISOR HERO (X: 375, Y: 225, W: 1180, H: 450) -->
    <!-- --------------------------------------------------------------------- -->
    <rect x="375" y="225" width="1180" height="450" rx="10" fill="#eff6ff" stroke="#1d4ed8" stroke-width="2.5"
          onmouseover="showTooltip('LAYER B: STATEFLOW SUPERVISOR HERO — Structurally verified SLX chart. Enforces EXCLUSIVE_OR decomposition across INIT, AUTONOMOUS_NAVIGATION, EMERGENCY, SAFE_STOP, and MISSION_COMPLETE.')"
          onmouseout="resetTooltip()"/>
    <path d="M 375 225 L 1555 225 L 1555 262 L 375 262 Z" fill="#1d4ed8"/>
    <text x="965" y="249" fill="#ffffff" font-size="13" font-weight="900" text-anchor="middle" letter-spacing="0.5">LAYER B &mdash; SUPERVISORY BEHAVIOR &bull; STATEFLOW ARCHITECTURE [SIH26037_SupervisoryArchitecture.slx]</text>

    <!-- Status Note Banner -->
    <rect x="390" y="270" width="1150" height="26" rx="4" fill="#fef3c7" stroke="#f59e0b" stroke-width="1.2"/>
    <text x="965" y="287" fill="#b45309" font-size="9.5" font-weight="800" text-anchor="middle">STATEFLOW STATUS: STRUCTURALLY VERIFIED IN SLX  |  STAGE 5 RUNTIME: MATLAB CoordinationDecisionLayer.m</text>

    <!-- Top State: INIT -->
    <rect x="400" y="310" width="95" height="40" rx="5" fill="#ffffff" stroke="#334155" stroke-width="1.5"/>
    <text x="447" y="334" fill="#0f172a" font-size="11" font-weight="800" text-anchor="middle">INIT</text>

    <!-- Transition: INIT -> AUTONOMOUS_NAVIGATION -->
    <line x1="495" y1="330" x2="522" y2="330" stroke="#475569" stroke-width="1.5" marker-end="url(#arr-slate)"/>
    <text x="508" y="322" fill="#475569" font-size="7.5" font-weight="800" text-anchor="middle">[lead_dx&gt;0]</text>

    <!-- Super-State: AUTONOMOUS_NAVIGATION (X: 525, Y: 305, W: 785, H: 355) -->
    <rect x="525" y="305" width="785" height="355" rx="8" fill="#ffffff" stroke="#2563eb" stroke-width="2"/>
    <path d="M 525 305 L 1310 305 L 1310 332 L 525 332 Z" fill="#dbeafe"/>
    <text x="917" y="323" fill="#1e40af" font-size="11" font-weight="900" text-anchor="middle">AUTONOMOUS_NAVIGATION (Super-State &bull; EXCLUSIVE_OR Decomposition)</text>

    <!-- Nested Sub-States inside AUTONOMOUS_NAVIGATION -->
    <!-- CRUISE -->
    <rect x="545" y="348" width="115" height="60" rx="5" fill="#f8fafc" stroke="#334155" stroke-width="1.5"/>
    <text x="602" y="375" fill="#0f172a" font-size="11" font-weight="800" text-anchor="middle">CRUISE</text>
    <text x="602" y="393" fill="#64748b" font-size="9" text-anchor="middle">v_tgt = 8.0 m/s</text>

    <!-- FOLLOW -->
    <rect x="725" y="348" width="115" height="60" rx="5" fill="#f8fafc" stroke="#334155" stroke-width="1.5"/>
    <text x="782" y="375" fill="#0f172a" font-size="11" font-weight="800" text-anchor="middle">FOLLOW</text>
    <text x="782" y="393" fill="#64748b" font-size="9" text-anchor="middle">Car-Following</text>

    <!-- RECOVER -->
    <rect x="905" y="348" width="115" height="60" rx="5" fill="#f8fafc" stroke="#334155" stroke-width="1.5"/>
    <text x="962" y="375" fill="#0f172a" font-size="11" font-weight="800" text-anchor="middle">RECOVER</text>
    <text x="962" y="393" fill="#64748b" font-size="9" text-anchor="middle">Lane Re-center</text>

    <!-- YIELD -->
    <rect x="545" y="475" width="115" height="60" rx="5" fill="#f8fafc" stroke="#334155" stroke-width="1.5"/>
    <text x="602" y="502" fill="#0f172a" font-size="11" font-weight="800" text-anchor="middle">YIELD</text>
    <text x="602" y="520" fill="#64748b" font-size="9" text-anchor="middle">Decel / Yield</text>

    <!-- OVERTAKE (Gold Highlight) -->
    <rect x="725" y="475" width="115" height="60" rx="5" fill="#fef3c7" stroke="#d97706" stroke-width="2.2"
          onmouseover="showTooltip('OVERTAKE STATE: Initiates latched passing maneuver when corridor width, lead headway, ego speed, and oncoming TTC pass the feasibility gate.')"
          onmouseout="resetTooltip()"/>
    <text x="782" y="502" fill="#92400e" font-size="11" font-weight="900" text-anchor="middle">OVERTAKE</text>
    <text x="782" y="520" fill="#b45309" font-size="9" font-weight="700" text-anchor="middle">Latched Pass</text>

    <!-- HERO FEATURE: OVERTAKE GATE CALLOUT CARD -->
    <rect x="1050" y="450" width="240" height="195" rx="6" fill="#ffffff" stroke="#d97706" stroke-width="1.8" stroke-dasharray="4,2"/>
    <path d="M 1050 450 L 1290 450 L 1290 478 L 1050 478 Z" fill="#fef3c7"/>
    <text x="1170" y="469" fill="#92400e" font-size="10.5" font-weight="900" text-anchor="middle">OVERTAKE GATE FEASIBILITY</text>
    <text x="1062" y="496" fill="#78350f" font-size="9.5" font-weight="700">&#10004; Corridor Width W_avail &ge; 4.10 m</text>
    <text x="1062" y="514" fill="#78350f" font-size="9.5" font-weight="700">&#10004; Oncoming TTC min_ttc &gt; 6.0 s</text>
    <text x="1062" y="532" fill="#78350f" font-size="9.5" font-weight="700">&#10004; Lead Gap 6.0 &le; lead_dx &le; 28.0 m</text>
    <text x="1062" y="550" fill="#78350f" font-size="9.5" font-weight="700">&#10004; Ego Speed v_ego &ge; 1.5 m/s</text>
    <rect x="1062" y="562" width="216" height="72" rx="4" fill="#f0fdf4" stroke="#16a34a" stroke-width="1"/>
    <text x="1170" y="580" fill="#15803d" font-size="9.5" font-weight="900" text-anchor="middle">GATED MANEUVER DECISION</text>
    <text x="1170" y="596" fill="#166534" font-size="8.5" text-anchor="middle">Prevents premature lane changes &amp;</text>
    <text x="1170" y="610" fill="#166534" font-size="8.5" text-anchor="middle">ensures physical passing clearance</text>

    <!-- State Machine Transition Lines -->
    <!-- CRUISE -> FOLLOW -->
    <line x1="660" y1="378" x2="720" y2="378" stroke="#475569" stroke-width="1.5" marker-end="url(#arr-slate)"/>
    <text x="690" y="370" fill="#475569" font-size="8" font-weight="700" text-anchor="middle">28&lt;dx&le;40m</text>

    <!-- CRUISE -> YIELD -->
    <line x1="602" y1="408" x2="602" y2="470" stroke="#475569" stroke-width="1.5" marker-end="url(#arr-slate)"/>
    <text x="578" y="442" fill="#475569" font-size="8" font-weight="700">TTC&le;6s</text>

    <!-- FOLLOW -> OVERTAKE (Gate Pass) -->
    <line x1="782" y1="408" x2="782" y2="470" stroke="#d97706" stroke-width="2" marker-end="url(#arr-slate)"/>
    <text x="812" y="442" fill="#b45309" font-size="8" font-weight="800">Gate Passed</text>

    <!-- OVERTAKE -> RECOVER (Completion) -->
    <path d="M 840 505 L 962 505 L 962 413" fill="none" stroke="#16a34a" stroke-width="1.8" marker-end="url(#arr-green)"/>
    <text x="968" y="465" fill="#16a34a" font-size="8" font-weight="800">Pass dx&ge;7.5m</text>

    <!-- OVERTAKE -> YIELD (Abort) -->
    <line x1="725" y1="505" x2="665" y2="505" stroke="#dc2626" stroke-width="1.8" marker-end="url(#arr-red)"/>
    <text x="695" y="497" fill="#dc2626" font-size="8" font-weight="800" text-anchor="middle">Abort Threat</text>

    <!-- YIELD -> FOLLOW -->
    <path d="M 660 490 L 690 490 L 690 395 L 720 395" fill="none" stroke="#475569" stroke-width="1.5" marker-end="url(#arr-slate)"/>

    <!-- Separate Safety & Mission States (Right side of Stateflow) -->
    <!-- EMERGENCY -->
    <rect x="1330" y="325" width="105" height="55" rx="5" fill="#fff1f2" stroke="#dc2626" stroke-width="2"/>
    <text x="1382" y="350" fill="#991b1b" font-size="11" font-weight="900" text-anchor="middle">EMERGENCY</text>
    <text x="1382" y="367" fill="#b91c1c" font-size="8.5" text-anchor="middle">Override Active</text>

    <!-- SAFE STOP -->
    <rect x="1330" y="430" width="105" height="55" rx="5" fill="#fff1f2" stroke="#dc2626" stroke-width="2"/>
    <text x="1382" y="455" fill="#991b1b" font-size="11" font-weight="900" text-anchor="middle">SAFE_STOP</text>
    <text x="1382" y="472" fill="#b91c1c" font-size="8.5" text-anchor="middle">Vehicle Halted</text>

    <!-- MISSION COMPLETE -->
    <rect x="1330" y="535" width="105" height="55" rx="5" fill="#f0fdf4" stroke="#16a34a" stroke-width="2"/>
    <text x="1382" y="558" fill="#14532d" font-size="10" font-weight="900" text-anchor="middle">MISSION</text>
    <text x="1382" y="573" fill="#15803d" font-size="10" font-weight="900" text-anchor="middle">COMPLETE</text>

    <!-- Override Transitions -->
    <path d="M 1310 352 L 1325 352" stroke="#dc2626" stroke-width="2" marker-end="url(#arr-red)"/>
    <text x="1317" y="344" fill="#dc2626" font-size="7.5" font-weight="800" text-anchor="middle">Risk</text>

    <line x1="1382" y1="380" x2="1382" y2="425" stroke="#dc2626" stroke-width="2" marker-end="url(#arr-red)"/>
    <text x="1395" y="405" fill="#dc2626" font-size="7.5" font-weight="800">v&lt;0.1</text>

    <path d="M 1310 562 L 1325 562" stroke="#16a34a" stroke-width="2" marker-end="url(#arr-green)"/>

    <!-- Outgoing Commands Bus -->
    <path d="M 1435 457 L 1555 457 L 1555 685 L 965 685 L 965 700" fill="none" stroke="#1d4ed8" stroke-width="2" stroke-dasharray="5,3"/>
    <rect x="1445" y="442" width="105" height="28" rx="4" fill="#eff6ff" stroke="#3b82f6" stroke-width="1"/>
    <text x="1497" y="459" fill="#1d4ed8" font-size="8.5" font-weight="800" text-anchor="middle">SUPERVISORY CMD</text>
    <text x="1497" y="468" fill="#1e40af" font-size="7.5" text-anchor="middle">intent, target_v, side</text>

    <!-- --------------------------------------------------------------------- -->
    <!-- LAYER C — ADAPTIVE MOTION GENERATION (X: 375, Y: 690, W: 1180, H: 175) -->
    <!-- --------------------------------------------------------------------- -->
    <rect x="375" y="690" width="1180" height="175" rx="8" fill="#f8fafc" stroke="#475569" stroke-width="1.8"
          onmouseover="showTooltip('LAYER C: ADAPTIVE MOTION GENERATION — FreeSpaceMap derives corridor bounds, CA-CRC shapes references & topology, QP-MPC optimizes trajectory subject to Hildreth dual solver constraints.')"
          onmouseout="resetTooltip()"/>
    <path d="M 375 690 L 1555 690 L 1555 715 L 375 715 Z" fill="#334155"/>
    <text x="965" y="707" fill="#ffffff" font-size="11" font-weight="900" text-anchor="middle" letter-spacing="0.5">LAYER C &mdash; ADAPTIVE MOTION GENERATION &amp; OPTIMIZATION</text>

    <!-- 1. FREE SPACE & TOPOLOGY -->
    <rect x="390" y="725" width="370" height="128" rx="6" fill="#ffffff" stroke="#cbd5e1" stroke-width="1.2"/>
    <text x="402" y="743" fill="#0f172a" font-size="10.5" font-weight="800">1. FREE SPACE &amp; TOPOLOGY</text>
    <text x="402" y="758" fill="#475569" font-size="9">&bull; FreeSpaceMap &bull; FreeSpaceBoundProvider</text>

    <!-- Mini Corridor Visual -->
    <rect x="402" y="765" width="346" height="48" rx="4" fill="#f8fafc" stroke="#cbd5e1" stroke-width="1"/>
    <line x1="410" y1="775" x2="740" y2="775" stroke="#94a3b8" stroke-width="1.5" stroke-dasharray="4,2"/>
    <line x1="410" y1="803" x2="740" y2="803" stroke="#94a3b8" stroke-width="1.5" stroke-dasharray="4,2"/>
    <text x="415" y="772" fill="#64748b" font-size="7">y_max(s) Road Boundary</text>
    <text x="415" y="810" fill="#64748b" font-size="7">y_min(s) Road Boundary</text>
    <rect x="550" y="785" width="35" height="15" fill="#ef4444" rx="2"/>
    <text x="567" y="796" fill="#ffffff" font-size="7.5" font-weight="700" text-anchor="middle">Obstacle</text>
    <path d="M 410 782 L 530 782 L 580 778 L 740 778" fill="none" stroke="#2563eb" stroke-width="2"/>
    <text x="630" y="790" fill="#1d4ed8" font-size="8" font-weight="800">Available Corridor</text>

    <text x="402" y="828" fill="#1d4ed8" font-size="9" font-weight="800">TOPOLOGY CHOICE: LEFT  |  FOLLOW  |  RIGHT</text>
    <text x="402" y="842" fill="#64748b" font-size="8.5">&ldquo;Corridor bounds adapt dynamically to road users &amp; obstacles.&rdquo;</text>

    <!-- Arrow 1 -> 2 -->
    <line x1="760" y1="788" x2="778" y2="788" stroke="#475569" stroke-width="1.8" marker-end="url(#arr-slate)"/>

    <!-- 2. CA-CRC PLANNER -->
    <rect x="780" y="725" width="365" height="128" rx="6" fill="#ffffff" stroke="#cbd5e1" stroke-width="1.2"/>
    <text x="792" y="743" fill="#0f172a" font-size="10.5" font-weight="800">2. CA-CRC PLANNER</text>
    <text x="792" y="758" fill="#1e40af" font-size="9" font-weight="700">Context-Adaptive Collision-Risk Planning</text>
    <text x="792" y="776" fill="#0f172a" font-size="9.5">&bull; Reference trajectory corridor shaping</text>
    <text x="792" y="792" fill="#0f172a" font-size="9.5">&bull; Candidate topology feasibility evaluation</text>
    <text x="792" y="808" fill="#0f172a" font-size="9.5">&bull; Soft slack-assisted feasibility preservation</text>
    <text x="792" y="824" fill="#0f172a" font-size="9.5">&bull; Stage 4 baseline planner authority integration</text>
    <text x="792" y="842" fill="#64748b" font-size="8.5" font-style="italic">Evaluates corridor constraints prior to QP optimization</text>

    <!-- Arrow 2 -> 3 -->
    <line x1="1145" y1="788" x2="1163" y2="788" stroke="#475569" stroke-width="1.8" marker-end="url(#arr-slate)"/>

    <!-- 3. QP-MPC OPTIMIZER -->
    <rect x="1165" y="725" width="375" height="128" rx="6" fill="#ffffff" stroke="#cbd5e1" stroke-width="1.2"/>
    <text x="1177" y="743" fill="#0f172a" font-size="10.5" font-weight="800">3. QP-MPC OPTIMIZER</text>
    <text x="1177" y="758" fill="#1d4ed8" font-size="9" font-weight="700">Convex Trajectory Optimization</text>
    <text x="1177" y="776" fill="#0f172a" font-size="9.5">&bull; Hildreth Dual QP Solver Implementation</text>
    <text x="1177" y="792" fill="#0f172a" font-size="9.5">&bull; Constraints: &delta; &in; &plusmn;35&deg;, a &in; [-6, +3] m/s&sup2;</text>

    <rect x="1177" y="800" width="351" height="44" rx="4" fill="#f1f5f9" stroke="#cbd5e1" stroke-width="1"/>
    <text x="1185" y="817" fill="#15803d" font-size="9.5" font-weight="900">Prototype Solver Execution Time: &approx; 4&ndash;6.4 ms</text>
    <text x="1185" y="833" fill="#b45309" font-size="8.5" font-weight="700">(Explicitly labeled: Hildreth QP solver execution time only)</text>

    <!-- Connection: Layer C -> Independent Safety Layer D -->
    <path d="M 965 865 L 965 880" stroke="#dc2626" stroke-width="2" marker-end="url(#arr-red)"/>

    <!-- --------------------------------------------------------------------- -->
    <!-- LAYER D — INDEPENDENT SAFETY & VEHICLE PLANT (X: 375, Y: 885, W: 1180, H: 170) -->
    <!-- --------------------------------------------------------------------- -->
    <rect x="375" y="885" width="1180" height="170" rx="8" fill="#fff1f2" stroke="#dc2626" stroke-width="2"
          onmouseover="showTooltip('LAYER D: INDEPENDENT SAFETY FILTER — Layer 2 hard safeguard evaluating OBB/SAT geometric clearance and road containment independently after optimization.')"
          onmouseout="resetTooltip()"/>
    <path d="M 375 885 L 1555 885 L 1555 910 L 375 910 Z" fill="#b91c1c"/>
    <text x="965" y="902" fill="#ffffff" font-size="11" font-weight="900" text-anchor="middle" letter-spacing="0.5">LAYER D &mdash; INDEPENDENT SAFETY FILTER &amp; VEHICLE PLANT</text>

    <!-- Independent Safety Box -->
    <rect x="390" y="920" width="480" height="122" rx="6" fill="#ffffff" stroke="#fca5a5" stroke-width="1.5"/>
    <text x="402" y="938" fill="#991b1b" font-size="11" font-weight="900">INDEPENDENT SAFETY FILTER (Layer 2 Safeguard)</text>
    <text x="402" y="955" fill="#0f172a" font-size="9.5">&bull; Forward trajectory projection &amp; OBB/SAT clearance</text>
    <text x="402" y="970" fill="#0f172a" font-size="9.5">&bull; Road containment &amp; actuator physical limit check</text>

    <!-- Decision Fork Visual -->
    <rect x="402" y="980" width="220" height="50" rx="4" fill="#f0fdf4" stroke="#16a34a" stroke-width="1.2"/>
    <text x="512" y="1000" fill="#15803d" font-size="10" font-weight="900" text-anchor="middle">&#10004; SAFE: Execute u_mpc</text>
    <text x="512" y="1018" fill="#166534" font-size="8.5" text-anchor="middle">Control commands to vehicle plant</text>

    <rect x="635" y="980" width="225" height="50" rx="4" fill="#fff1f2" stroke="#dc2626" stroke-width="1.2"/>
    <text x="747" y="1000" fill="#991b1b" font-size="10" font-weight="900" text-anchor="middle">&#10008; UNSAFE: Emergency Brake</text>
    <text x="747" y="1018" fill="#b91c1c" font-size="8.5" text-anchor="middle">Override active &rarr; Safe stop halt</text>

    <!-- Concept Quote -->
    <text x="402" y="1036" fill="#7f1d1d" font-size="8.5" font-weight="700">&ldquo;Optimization proposes. Safety independently validates.&rdquo; (Not a formal guarantee)</text>

    <!-- Arrow Safety -> Vehicle -->
    <line x1="870" y1="981" x2="895" y2="981" stroke="#dc2626" stroke-width="2" marker-end="url(#arr-red)"/>

    <!-- Vehicle Dynamics Box -->
    <rect x="897" y="920" width="643" height="122" rx="6" fill="#ffffff" stroke="#cbd5e1" stroke-width="1.5"/>
    <text x="910" y="938" fill="#0f172a" font-size="11" font-weight="900">VEHICLE DYNAMICS &amp; CLOSED-LOOP ENVIRONMENT</text>
    <text x="910" y="955" fill="#1e293b" font-size="9.5" font-weight="700">&bull; BicycleModel (Ego Dynamics) State Vector: [x, y, &theta;, v, &delta;]</text>
    <text x="910" y="970" fill="#64748b" font-size="9">Actuator uncertainty &amp; steering bias model integrated.</text>

    <!-- Ego Graphic -->
    <rect x="910" y="982" width="60" height="30" rx="4" fill="#2563eb" stroke="#1d4ed8" stroke-width="1.5"/>
    <circle cx="922" cy="1012" r="5" fill="#0f172a"/>
    <circle cx="958" cy="1012" r="5" fill="#0f172a"/>
    <text x="940" y="1001" fill="#ffffff" font-size="9" font-weight="900" text-anchor="middle">EGO</text>

    <text x="985" y="995" fill="#0f172a" font-size="9.5" font-weight="800">ENVIRONMENT &amp; SURROUNDING ROAD USERS:</text>
    <text x="985" y="1012" fill="#475569" font-size="9">Unstructured road &bull; Vehicles &bull; Two-wheelers &bull; Pedestrians &bull; Cattle &bull; Static Obstacles</text>

    <!-- Closed-Loop Feedback Arrow -->
    <path d="M 1540 981 L 1570 981 L 1570 140 L 1555 140" fill="none" stroke="#2563eb" stroke-width="2.5" stroke-dasharray="6,4" marker-end="url(#arr-blue)"/>
    <rect x="1465" y="560" width="130" height="40" rx="4" fill="#eff6ff" stroke="#3b82f6" stroke-width="1.2"/>
    <text x="1530" y="577" fill="#1d4ed8" font-size="9" font-weight="900" text-anchor="middle">REAL-TIME REPLANNING</text>
    <text x="1530" y="591" fill="#1e40af" font-size="8" font-weight="700" text-anchor="middle">Feedback Loop ↺</text>

    <!-- ===================================================================== -->
    <!-- ZONE 3 — RIGHT: EVIDENCE & PROPOSED SIH SYSTEM (X: 1570, Y: 75, W: 320, H: 980) -->
    <!-- ===================================================================== -->

    <!-- Top Card: Prototype Evidence (X: 1570, Y: 75, W: 320, H: 410) -->
    <rect x="1570" y="75" width="320" height="410" rx="8" fill="#f8fafc" stroke="#cbd5e1" stroke-width="1.8"/>
    <path d="M 1570 75 L 1890 75 L 1890 110 L 1570 110 Z" fill="#0f172a"/>
    <text x="1730" y="97" fill="#ffffff" font-size="12" font-weight="900" text-anchor="middle" letter-spacing="0.5">PROTOTYPE EVIDENCE</text>

    <!-- Stat 1: 0 Collisions -->
    <rect x="1585" y="122" width="290" height="70" rx="6" fill="#ffffff" stroke="#16a34a" stroke-width="1.8"/>
    <text x="1600" y="162" fill="#15803d" font-size="32" font-weight="900">0</text>
    <text x="1635" y="148" fill="#0f172a" font-size="11" font-weight="800">Collision steps in tested</text>
    <text x="1635" y="164" fill="#0f172a" font-size="11" font-weight="800">deterministic scenarios</text>
    <text x="1635" y="180" fill="#16a34a" font-size="8.5" font-weight="700">&#10004; Evaluated on Stage 5 baseline stack</text>

    <!-- Stat 2: 150m Completion -->
    <rect x="1585" y="202" width="290" height="70" rx="6" fill="#ffffff" stroke="#0284c7" stroke-width="1.8"/>
    <text x="1600" y="242" fill="#0369a1" font-size="26" font-weight="900">150m</text>
    <text x="1675" y="228" fill="#0f172a" font-size="11" font-weight="800">Route completion in</text>
    <text x="1675" y="244" fill="#0f172a" font-size="11" font-weight="800">tested scenarios</text>
    <text x="1675" y="260" fill="#0284c7" font-size="8.5" font-weight="700">&#10004; Continuous closed-loop navigation</text>

    <!-- Stat 3: 4-6.4ms Solver -->
    <rect x="1585" y="282" width="290" height="75" rx="6" fill="#ffffff" stroke="#d97706" stroke-width="1.8"/>
    <text x="1595" y="320" fill="#b45309" font-size="20" font-weight="900">&approx;4-6.4ms</text>
    <text x="1692" y="306" fill="#0f172a" font-size="11" font-weight="800">QP solver execution</text>
    <text x="1692" y="322" fill="#b45309" font-size="9" font-weight="800">(solver time only)</text>
    <text x="1692" y="338" fill="#475569" font-size="8" font-style="italic">Not end-to-end latency</text>

    <!-- Additional Noise Details -->
    <rect x="1585" y="367" width="290" height="85" rx="5" fill="#ffffff" stroke="#cbd5e1" stroke-width="1"/>
    <text x="1595" y="385" fill="#475569" font-size="9.5" font-weight="800">&bull; Obs noise / FN / FP / delay modeled</text>
    <text x="1595" y="401" fill="#475569" font-size="9.5" font-weight="800">&bull; Steering bias uncertainty modeled</text>
    <rect x="1595" y="412" width="270" height="28" rx="3" fill="#fef3c7" stroke="#f59e0b" stroke-width="1"/>
    <text x="1730" y="430" fill="#b45309" font-size="9" font-weight="800" text-anchor="middle">Prototype evidence &ne; production guarantee</text>

    <!-- Second Card: From Prototype -> SIH Complete System (X: 1570, Y: 495, W: 320, H: 450) -->
    <rect x="1570" y="495" width="320" height="450" rx="8" fill="#f8fafc" stroke="#94a3b8" stroke-width="1.8" stroke-dasharray="6,4"/>
    <path d="M 1570 495 L 1890 495 L 1890 530 L 1570 530 Z" fill="#475569"/>
    <text x="1730" y="517" fill="#ffffff" font-size="11" font-weight="900" text-anchor="middle" letter-spacing="0.5">FROM PROTOTYPE &rarr; SIH COMPLETE</text>

    <!-- 5 Maturity Progression Items -->
    <g font-size="9.5" font-weight="800">
      <!-- 01 -->
      <rect x="1585" y="542" width="290" height="52" rx="5" fill="#ffffff" stroke="#94a3b8" stroke-width="1.2" stroke-dasharray="4,2"/>
      <text x="1597" y="562" fill="#334155">01. Real Camera / LiDAR / Radar</text>
      <text x="1597" y="578" fill="#94a3b8" font-size="8.5" font-weight="600">[PROPOSING SIH EXTENSION]</text>

      <!-- Down arrow -->
      <path d="M 1730 596 L 1730 604" stroke="#94a3b8" stroke-width="1.5" marker-end="url(#arr-slate)"/>

      <!-- 02 -->
      <rect x="1585" y="606" width="290" height="52" rx="5" fill="#ffffff" stroke="#94a3b8" stroke-width="1.2" stroke-dasharray="4,2"/>
      <text x="1597" y="626" fill="#334155">02. EKF / UKF Sensor Fusion</text>
      <text x="1597" y="642" fill="#94a3b8" font-size="8.5" font-weight="600">[PROPOSING SIH EXTENSION]</text>

      <!-- Down arrow -->
      <path d="M 1730 660 L 1730 668" stroke="#94a3b8" stroke-width="1.5" marker-end="url(#arr-slate)"/>

      <!-- 03 -->
      <rect x="1585" y="670" width="290" height="52" rx="5" fill="#ffffff" stroke="#94a3b8" stroke-width="1.2" stroke-dasharray="4,2"/>
      <text x="1597" y="690" fill="#334155">03. RoadRunner Co-Simulation</text>
      <text x="1597" y="706" fill="#94a3b8" font-size="8.5" font-weight="600">[PROPOSING SIH EXTENSION]</text>

      <!-- Down arrow -->
      <path d="M 1730 724 L 1730 732" stroke="#94a3b8" stroke-width="1.5" marker-end="url(#arr-slate)"/>

      <!-- 04 -->
      <rect x="1585" y="734" width="290" height="52" rx="5" fill="#ffffff" stroke="#94a3b8" stroke-width="1.2" stroke-dasharray="4,2"/>
      <text x="1597" y="754" fill="#334155">04. Intersection &amp; Merge Behavior</text>
      <text x="1597" y="770" fill="#94a3b8" font-size="8.5" font-weight="600">[PROPOSING SIH EXTENSION]</text>

      <!-- Down arrow -->
      <path d="M 1730 788 L 1730 796" stroke="#94a3b8" stroke-width="1.5" marker-end="url(#arr-slate)"/>

      <!-- 05 -->
      <rect x="1585" y="798" width="290" height="52" rx="5" fill="#ffffff" stroke="#94a3b8" stroke-width="1.2" stroke-dasharray="4,2"/>
      <text x="1597" y="818" fill="#334155">05. Formal Safety Layer / CBF</text>
      <text x="1597" y="834" fill="#94a3b8" font-size="8.5" font-weight="600">[PROPOSING SIH EXTENSION]</text>
    </g>

    <!-- Prototype Status Legend Box (X: 1570, Y: 955, W: 320, H: 100) -->
    <rect x="1570" y="955" width="320" height="100" rx="8" fill="#ffffff" stroke="#cbd5e1" stroke-width="1.5"/>
    <text x="1582" y="974" fill="#0f172a" font-size="10.5" font-weight="900">PROTOTYPE STATUS LEGEND</text>
    <rect x="1582" y="983" width="18" height="10" fill="#ffffff" stroke="#334155" stroke-width="1.5"/>
    <text x="1608" y="992" fill="#0f172a" font-size="9" font-weight="700">Solid = Implemented Prototype</text>

    <rect x="1582" y="999" width="18" height="10" fill="#ffffff" stroke="#94a3b8" stroke-width="1.5" stroke-dasharray="3,2"/>
    <text x="1608" y="1008" fill="#64748b" font-size="9" font-weight="700">Dashed = Proposed SIH Extension</text>

    <rect x="1582" y="1015" width="18" height="10" fill="#eff6ff" stroke="#1d4ed8" stroke-width="1.5"/>
    <text x="1608" y="1024" fill="#1d4ed8" font-size="9" font-weight="700">Blue = Stateflow SLX Verified</text>

    <text x="1582" y="1044" fill="#b45309" font-size="8" font-weight="800">Current Stage 5 runtime: MATLAB CoordinationDecisionLayer</text>
  </svg>
</div>

<script>
  function showTooltip(text) {
    var tooltip = document.getElementById('tooltip');
    tooltip.innerHTML = '<span class="accent">TECHNICAL DETAIL:</span> ' + text;
    tooltip.style.background = '#1e293b';
    tooltip.style.borderLeft = '4px solid #38bdf8';
  }

  function resetTooltip() {
    var tooltip = document.getElementById('tooltip');
    tooltip.innerHTML = '<span class="accent">INTERACTIVE DIAGRAM MASTER:</span> Hover over any module or transition to inspect formal implementation parameters and evidence.';
    tooltip.style.background = '#0f172a';
    tooltip.style.borderLeft = 'none';
  }

  function togglePresentationMode() {
    var canvas = document.getElementById('canvas');
    canvas.classList.toggle('presentation-mode');
    var isPres = canvas.classList.contains('presentation-mode');
    document.getElementById('mode-text').innerText = isPres ? 'SHOW INTERACTIVE DETAILS' : 'TOGGLE PRESENTATION VIEW';
  }
</script>
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
    print(f"SUCCESS: Written Redesigned HTML to {html_path}")

    # Extract pure SVG content for standalone SVG file
    svg_start = html_content.find('<svg class="architecture-svg"')
    svg_end = html_content.find('</svg>') + len('</svg>')
    pure_svg_body = html_content[svg_start:svg_end]
    
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
    print(f"SUCCESS: Written Redesigned SVG to {svg_path}")

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
        print(f"SUCCESS: Rendered Redesigned PNG to {png_path}")
    except Exception as e:
        print(f"WARNING: Google Chrome PNG export error: {e}")
        try:
            subprocess.run(["convert", svg_path, png_path], check=True)
            print(f"SUCCESS: Rendered PNG via convert to {png_path}")
        except Exception as e2:
            print(f"ERROR: Convert failed: {e2}")

if __name__ == "__main__":
    generate_slide2_redesign()
