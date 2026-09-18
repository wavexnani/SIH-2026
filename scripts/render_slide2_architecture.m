function render_slide2_architecture()
    % RENDER_SLIDE2_ARCHITECTURE Generates Slide 2 Engineering Architecture Artifacts
    %
    % Creates high-resolution SVG, PDF, and 300 DPI PNG diagrams of the
    % SIH26037 Supervisory Architecture and Closed-Loop Data-Flow.
    
    fprintf('=== Generating Slide 2 Architecture Engineering Diagram ===\n');
    
    % Set up figure (16:9 Landscape aspect ratio)
    fig = figure('Visible', 'off', 'Color', [1 1 1], 'Units', 'pixels', 'Position', [100 100 1920 1080]);
    ax = axes('Parent', fig, 'Position', [0 0 1 1], 'XLim', [0 100], 'YLim', [0 100]);
    hold(ax, 'on');
    axis(ax, 'off');
    
    % Color Palette Definitions
    c_navy       = [15 23 42] / 255;    % #0F172A - Primary dark navy header
    c_blue_bg    = [239 246 255] / 255; % #EFF6FF - Stateflow background fill
    c_blue_border= [29 78 216] / 255;  % #1D4ED8 - Stateflow primary border
    c_slate_bg   = [248 250 252] / 255; % #F8FAFC - Subsystem background fill
    c_slate_border=[71 85 105] / 255;   % #475569 - Solid subsystem border
    c_state_bg   = [255 255 255] / 255; % #FFFFFF - State card fill
    c_state_border=[30 41 59] / 255;    % #1E293B - State card border
    c_red_border = [225 29 72] / 255;   % #E11D48 - Emergency state border
    c_red_bg     = [255 241 242] / 255; % #FFF1F2 - Emergency fill
    c_green_bg   = [240 253 244] / 255; % #F0FDF4 - Complete fill
    c_green_border=[22 163 74] / 255;   % #16A34A - Complete border
    c_yellow_bg  = [254 252 232] / 255; % #FEFCE8 - Callout fill
    c_yellow_bdr = [202 138 4] / 255;   % #CA8A04 - Callout border
    c_prop_bg    = [249 250 251] / 255; % #F9FAFB - Proposed fill
    c_prop_border= [156 163 175] / 255; % #9CA3AF - Proposed dashed border
    c_text_dark  = [15 23 42] / 255;    % #0F172A
    c_text_muted = [100 116 139] / 255; % #64748B
    
    % -------------------------------------------------------------------------
    % 1. TOP HEADER BANNER
    % -------------------------------------------------------------------------
    rectangle('Position', [1 92 98 7], 'FaceColor', c_navy, 'EdgeColor', 'none', 'Curvature', [0.05 0.2]);
    text(3, 96.2, 'SIH26037 AUTHORITATIVE ENGINEERING ARCHITECTURE', 'FontSize', 18, 'FontWeight', 'bold', 'Color', [1 1 1], 'Interpreter', 'none');
    text(3, 93.8, 'Slide 2: Supervisory Stateflow Intelligence & Multi-Layer Closed-Loop Data-Flow', 'FontSize', 12, 'Color', [203 213 225]/255, 'Interpreter', 'none');
    text(76, 95.0, 'PPT SOURCE ARTIFACT', 'FontSize', 11, 'FontWeight', 'bold', 'Color', [251 191 36]/255, 'Interpreter', 'none');

    % -------------------------------------------------------------------------
    % 2. LEFT SIDE PIPELINE: PERCEPTION & PREDICTION (Solid Border)
    % -------------------------------------------------------------------------
    % Perception Block
    draw_box(2, 67, 22, 22, c_slate_bg, c_slate_border, 2, '-');
    draw_header_bar(2, 84, 22, 5, [30 41 59]/255, 'PERCEPTION / WORLD MODEL');
    text(3, 80, '\bullet ObservationModel', 'FontSize', 10, 'FontWeight', 'bold', 'Color', c_text_dark);
    text(3, 76, '\bullet MultiVehicleDetector', 'FontSize', 10, 'FontWeight', 'bold', 'Color', c_text_dark);
    text(3, 71, 'Inputs: Sensor / World State', 'FontSize', 8, 'FontAngle', 'italic', 'Color', c_text_muted);

    % Arrow: Perception -> Prediction
    draw_arrow([13 13], [67 62], c_slate_border, 2, 'down');

    % Prediction Block
    draw_box(2, 40, 22, 22, c_slate_bg, c_slate_border, 2, '-');
    draw_header_bar(2, 57, 22, 5, [30 41 59]/255, 'PREDICTION & RISK');
    text(3, 53, '\bullet RiskPredictor (Np=10, dt=0.10s)', 'FontSize', 9.5, 'FontWeight', 'bold', 'Color', c_text_dark);
    text(3, 49, '\bullet TTC Estimator', 'FontSize', 9.5, 'FontWeight', 'bold', 'Color', c_text_dark);
    text(3, 45, '\bullet InteractionClassifier', 'FontSize', 9.5, 'FontWeight', 'bold', 'Color', c_text_dark);
    text(3, 42, 'Outputs: lead_dx, min_ttc, W_avail', 'FontSize', 8, 'FontAngle', 'italic', 'Color', c_text_muted);

    % Arrow: Prediction -> Stateflow
    draw_arrow([24 26.5], [51 51], c_blue_border, 2.5, 'right');
    text(24.2, 53, 'Perception/Risk Data', 'FontSize', 8, 'FontWeight', 'bold', 'Color', c_blue_border);

    % -------------------------------------------------------------------------
    % 3. CENTERPIECE: STATEFLOW SUPERVISORY ARCHITECTURE (Solid Border)
    % -------------------------------------------------------------------------
    draw_box(27, 40, 47, 49, c_blue_bg, c_blue_border, 2.5, '-');
    draw_header_bar(27, 84, 47, 5, c_blue_border, 'STATEFLOW SUPERVISOR  [SIH26037_SupervisoryArchitecture.slx]');

    % Callout Annotation Box (Important Disclaimer)
    draw_box(28.5, 76.5, 44, 6.5, c_yellow_bg, c_yellow_bdr, 1.5, '-');
    text(29.5, 81.2, '[!] Supervisory Architecture — Generated & Structurally Verified in SLX', 'FontSize', 9, 'FontWeight', 'bold', 'Color', [146 64 14]/255);
    text(29.5, 78.2, '[!] Current Stage 5 Prototype Execution: MATLAB Decision Layer (Stage5CoordinationController.m)', 'FontSize', 8.5, 'Color', [180 83 9]/255);

    % Super-State: AUTONOMOUS_NAVIGATION
    draw_box(28.5, 42, 30.5, 33, [255 255 255]/255, [37 99 235]/255, 1.5, '-');
    rectangle('Position', [28.5 71.5 30.5 3.5], 'FaceColor', [219 234 254]/255, 'EdgeColor', 'none');
    text(29.5, 73.2, 'AUTONOMOUS_NAVIGATION (Super-State)', 'FontSize', 9.5, 'FontWeight', 'bold', 'Color', [30 58 138]/255);

    % Sub-States inside AUTONOMOUS_NAVIGATION
    % CRUISE
    draw_state_card(30, 63, 8.5, 6.5, 'CRUISE', 'v_tgt = 8.0 m/s', [241 245 249]/255, c_state_border);
    % FOLLOW
    draw_state_card(40, 63, 8.5, 6.5, 'FOLLOW', 'Car-Following', [241 245 249]/255, c_state_border);
    % RECOVER
    draw_state_card(50, 63, 8.5, 6.5, 'RECOVER', 'Lane Re-center', [241 245 249]/255, c_state_border);
    % YIELD
    draw_state_card(30, 44, 8.5, 6.5, 'YIELD', 'Decel / Yield', [241 245 249]/255, c_state_border);
    % OVERTAKE
    draw_state_card(40, 44, 8.5, 6.5, 'OVERTAKE', 'Latched Pass', [254 243 199]/255, [217 119 6]/255);

    % Inner Sub-State Transitions
    draw_arrow([38.5 40], [66.25 66.25], [51 65 85]/255, 1.2, 'right'); % CRUISE -> FOLLOW
    draw_arrow([48.5 50], [66.25 66.25], [51 65 85]/255, 1.2, 'right'); % FOLLOW -> RECOVER (via OVERTAKE)
    draw_arrow([44.25 44.25], [63 50.5], [51 65 85]/255, 1.2, 'down');   % FOLLOW -> OVERTAKE
    draw_arrow([40 38.5], [47.25 47.25], [51 65 85]/255, 1.2, 'left');   % OVERTAKE -> YIELD (Abort)
    draw_arrow([34.25 34.25], [50.5 63], [51 65 85]/255, 1.2, 'up');     % YIELD -> CRUISE / FOLLOW
    draw_arrow([38.5 40], [47.25 47.25], [51 65 85]/255, 1.2, 'right'); % YIELD -> OVERTAKE
    draw_arrow([54.25 54.25 34.25 34.25], [63 71 71 69.5], [51 65 85]/255, 1.2, 'up'); % RECOVER -> CRUISE

    % Outside Top-Level States
    % INIT
    draw_state_card(60.5, 70, 12, 5, 'INIT', 'State Reset', [241 245 249]/255, c_state_border);
    draw_arrow([66.5 59], [70 70], [51 65 85]/255, 1.2, 'left'); text(60, 71.5, '[lead_dx>0]', 'FontSize', 7.5, 'Color', c_text_muted);

    % EMERGENCY
    draw_state_card(60.5, 59, 12, 7, 'EMERGENCY', 'Override Active', c_red_bg, c_red_border);
    % SAFE_STOP
    draw_state_card(60.5, 49, 12, 7, 'SAFE_STOP', 'Vehicle Halt', c_red_bg, c_red_border);
    % MISSION_COMPLETE
    draw_state_card(60.5, 42, 12, 5, 'MISSION_COMPLETE', 'Goal Reached', c_green_bg, c_green_border);

    % Critical Transitions
    draw_arrow([59 60.5], [54 62.5], c_red_border, 2, 'right');
    text(53.5, 57, '[mpc_status==0 || min_ttc<1s]', 'FontSize', 7.5, 'FontWeight', 'bold', 'Color', c_red_border);
    draw_arrow([66.5 66.5], [59 56], c_red_border, 2, 'down');
    text(67, 57.5, '[v_ego<0.1]', 'FontSize', 7.5, 'Color', c_red_border);

    % Arrow: Stateflow -> Adaptive Planning
    draw_arrow([74 76.5], [51 51], c_blue_border, 2.5, 'right');
    text(74.2, 53, 'Intent + Target V', 'FontSize', 8, 'FontWeight', 'bold', 'Color', c_blue_border);

    % -------------------------------------------------------------------------
    % 4. RIGHT SIDE PIPELINE: PLANNING, MPC & SAFETY (Solid Border)
    % -------------------------------------------------------------------------
    % Adaptive Planning Block
    draw_box(76.5, 71, 21.5, 18, c_slate_bg, c_slate_border, 2, '-');
    draw_header_bar(76.5, 84, 21.5, 5, [30 41 59]/255, 'ADAPTIVE PLANNING');
    text(77.5, 80, '\bullet FreeSpaceMap', 'FontSize', 9.5, 'FontWeight', 'bold', 'Color', c_text_dark);
    text(77.5, 76, '\bullet Corridor Topology', 'FontSize', 9.5, 'FontWeight', 'bold', 'Color', c_text_dark);
    text(77.5, 72.5, '\bullet CACRCPlanner Corridor', 'FontSize', 9.5, 'FontWeight', 'bold', 'Color', c_text_dark);

    % Arrow: Planning -> QP-MPC
    draw_arrow([87.25 87.25], [71 66], c_slate_border, 2, 'down');

    % QP-MPC Block
    draw_box(76.5, 50, 21.5, 16, c_slate_bg, c_slate_border, 2, '-');
    draw_header_bar(76.5, 61, 21.5, 5, [30 41 59]/255, 'QP-MPC OPTIMIZER');
    text(77.5, 57, '\bullet QPMPCPlanner', 'FontSize', 10, 'FontWeight', 'bold', 'Color', c_text_dark);
    text(77.5, 53, '\bullet Convex Trajectory Opt.', 'FontSize', 9, 'Color', c_text_dark);
    text(77.5, 51, 'Output: u_mpc [steer, accel]', 'FontSize', 8, 'FontAngle', 'italic', 'Color', c_text_muted);

    % Arrow: QP-MPC -> Safety Filter
    draw_arrow([87.25 87.25], [50 45], c_slate_border, 2, 'down');

    % Safety Filter Block
    draw_box(76.5, 29, 21.5, 16, c_slate_bg, c_slate_border, 2, '-');
    draw_header_bar(76.5, 40, 21.5, 5, [185 28 28]/255, 'SAFETY FILTER (LAYER 2)');
    text(77.5, 36, '\bullet SafetyFilter', 'FontSize', 10, 'FontWeight', 'bold', 'Color', c_text_dark);
    text(77.5, 32, '\bullet Emergency Override Guard', 'FontSize', 9, 'Color', c_text_dark);
    text(77.5, 30, 'Output: u_safe command', 'FontSize', 8, 'FontAngle', 'italic', 'Color', c_text_muted);

    % Arrow: Safety Filter -> Vehicle Plant
    draw_arrow([76.5 73.5], [37 37], c_slate_border, 2, 'left');

    % -------------------------------------------------------------------------
    % 5. BOTTOM PIPELINE: VEHICLE PLANT & CLOSED-LOOP FEEDBACK (Solid Border)
    % -------------------------------------------------------------------------
    draw_box(51, 25, 22.5, 14, c_slate_bg, c_slate_border, 2, '-');
    draw_header_bar(51, 34, 22.5, 5, [30 41 59]/255, 'VEHICLE & ENVIRONMENT PLANT');
    text(52, 30, '\bullet ActuatorUncertaintyModel', 'FontSize', 9.5, 'FontWeight', 'bold', 'Color', c_text_dark);
    text(52, 26.5, '\bullet BicycleModel Dynamics', 'FontSize', 9.5, 'FontWeight', 'bold', 'Color', c_text_dark);

    % Feedback Loop Arrow (Vehicle -> Perception)
    draw_arrow([51 13 13], [32 32 67], [100 116 139]/255, 2, 'up');
    text(24, 30.5, 'Closed-Loop State Feedback (Ego / World State)', 'FontSize', 9, 'FontWeight', 'bold', 'Color', [71 85 105]/255);

    % -------------------------------------------------------------------------
    % 6. PROPOSED SIH EXTENSIONS (Dashed Border = Not in Prototype Runtime)
    % -------------------------------------------------------------------------
    draw_box(2, 4, 46, 32, c_prop_bg, c_prop_border, 2, '--');
    draw_header_bar(2, 31, 46, 5, [100 116 139]/255, 'PROPOSED SIH EXTENSIONS  [DASHED BORDER = NOT IN PROTOTYPE RUNTIME]');

    % Extension Cards
    draw_box(3.5, 20.5, 20, 9, [255 255 255]/255, c_prop_border, 1.2, '--');
    text(4.5, 27, '\bullet Real Camera/LiDAR/Radar', 'FontSize', 8.5, 'FontWeight', 'bold', 'Color', c_text_muted);
    text(4.5, 24, '   Hardware Perception Suite', 'FontSize', 8, 'Color', c_text_muted);

    draw_box(25.5, 20.5, 21, 9, [255 255 255]/255, c_prop_border, 1.2, '--');
    text(26.5, 27, '\bullet EKF / UKF Sensor Fusion', 'FontSize', 8.5, 'FontWeight', 'bold', 'Color', c_text_muted);
    text(26.5, 24, '   Multi-Sensor State Estimator', 'FontSize', 8, 'Color', c_text_muted);

    draw_box(3.5, 9.5, 20, 9, [255 255 255]/255, c_prop_border, 1.2, '--');
    text(4.5, 16, '\bullet RoadRunner Co-Simulation', 'FontSize', 8.5, 'FontWeight', 'bold', 'Color', c_text_muted);
    text(4.5, 13, '   High-Fidelity 3D Environment', 'FontSize', 8, 'Color', c_text_muted);

    draw_box(25.5, 9.5, 21, 9, [255 255 255]/255, c_prop_border, 1.2, '--');
    text(26.5, 16, '\bullet Intersection / Merge Layer', 'FontSize', 8.5, 'FontWeight', 'bold', 'Color', c_text_muted);
    text(26.5, 13, '   Multi-Agent Junction Logic', 'FontSize', 8, 'Color', c_text_muted);

    draw_box(14.5, 5, 21, 4, [255 255 255]/255, c_prop_border, 1.2, '--');
    text(15.5, 7, '\bullet CBF Safety Safeguard Layer', 'FontSize', 8, 'FontWeight', 'bold', 'Color', c_text_muted);

    % -------------------------------------------------------------------------
    % 7. ARCHITECTURAL CLASSIFICATION LEGEND (Bottom Right)
    % -------------------------------------------------------------------------
    draw_box(76.5, 4, 21.5, 22, [255 255 255]/255, [203 213 225]/255, 1.5, '-');
    draw_header_bar(76.5, 22, 21.5, 4, [71 85 105]/255, 'CLASSIFICATION LEGEND');

    % Legend Items
    % Solid Box
    rectangle('Position', [78 17 4 3], 'FaceColor', c_slate_bg, 'EdgeColor', c_slate_border, 'LineWidth', 1.5, 'LineStyle', '-');
    text(83, 18.5, 'Executed Stage 5 Prototype (MATLAB)', 'FontSize', 8, 'FontWeight', 'bold', 'Color', c_text_dark);

    % Dashed Box
    rectangle('Position', [78 12.5 4 3], 'FaceColor', c_prop_bg, 'EdgeColor', c_prop_border, 'LineWidth', 1.5, 'LineStyle', '--');
    text(83, 14, 'Proposed SIH Extensions (Conceptual)', 'FontSize', 8, 'FontWeight', 'bold', 'Color', c_text_muted);

    % Blue Box
    rectangle('Position', [78 8 4 3], 'FaceColor', c_blue_bg, 'EdgeColor', c_blue_border, 'LineWidth', 1.5, 'LineStyle', '-');
    text(83, 9.5, 'Stateflow Supervisor (SLX Verified)', 'FontSize', 8, 'FontWeight', 'bold', 'Color', c_blue_border);

    % Red Line
    plot([78 82], [5.5 5.5], 'Color', c_red_border, 'LineWidth', 2);
    text(83, 5.5, 'High-Priority Emergency Override', 'FontSize', 8, 'FontWeight', 'bold', 'Color', c_red_border);

    % -------------------------------------------------------------------------
    % EXPORT HIGH-RESOLUTION ARTIFACTS
    % -------------------------------------------------------------------------
    output_dir = fullfile(pwd, 'artifacts');
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    png_path = fullfile(output_dir, 'SIH26037_Slide2_Architecture.png');
    svg_path = fullfile(output_dir, 'SIH26037_Slide2_Architecture.svg');
    pdf_path = fullfile(output_dir, 'SIH26037_Slide2_Architecture.pdf');

    % Export PNG at 300 DPI
    print(fig, png_path, '-dpng', '-r300');
    fprintf('SUCCESS: Exported 300 DPI PNG to: %s\n', png_path);

    % Export SVG
    try
        print(fig, svg_path, '-dsvg');
        fprintf('SUCCESS: Exported SVG to: %s\n', svg_path);
    catch ME
        fprintf('WARNING: SVG export via print failed: %s\n', ME.message);
    end

    % Export PDF (Vector format)
    try
        exportgraphics(fig, pdf_path, 'ContentType', 'vector');
        fprintf('SUCCESS: Exported Vector PDF to: %s\n', pdf_path);
    catch ME
        fprintf('WARNING: PDF export failed: %s\n', ME.message);
    end

    close(fig);
    fprintf('=== Architecture Diagram Generation Complete ===\n');
end

% -----------------------------------------------------------------------------
% HELPER GRAPHICS FUNCTIONS
% -----------------------------------------------------------------------------
function draw_box(x, y, w, h, bg_color, border_color, lw, style)
    rectangle('Position', [x y w h], 'FaceColor', bg_color, 'EdgeColor', border_color, ...
              'LineWidth', lw, 'LineStyle', style, 'Curvature', [0.03 0.05]);
end

function draw_header_bar(x, y, w, h, bg_color, title_str)
    rectangle('Position', [x y w h], 'FaceColor', bg_color, 'EdgeColor', 'none', 'Curvature', [0.05 0.2]);
    text(x + w/2, y + h/2, title_str, 'FontSize', 9, 'FontWeight', 'bold', ...
         'Color', [1 1 1], 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Interpreter', 'none');
end

function draw_state_card(x, y, w, h, title_str, sub_str, bg_color, border_color)
    rectangle('Position', [x y w h], 'FaceColor', bg_color, 'EdgeColor', border_color, ...
              'LineWidth', 1.2, 'Curvature', [0.15 0.2]);
    text(x + w/2, y + h*0.65, title_str, 'FontSize', 8.5, 'FontWeight', 'bold', ...
         'Color', [15 23 42]/255, 'HorizontalAlignment', 'center', 'Interpreter', 'none');
    text(x + w/2, y + h*0.30, sub_str, 'FontSize', 7.5, 'Color', [100 116 139]/255, ...
         'HorizontalAlignment', 'center', 'Interpreter', 'none');
end

function draw_arrow(x_vec, y_vec, color_rgb, lw, head_dir)
    plot(x_vec, y_vec, 'Color', color_rgb, 'LineWidth', lw);
    x_end = x_vec(end);
    y_end = y_vec(end);
    
    % Draw arrowhead based on direction
    sz = 0.8;
    switch head_dir
        case 'right'
            patch([x_end x_end-sz x_end-sz], [y_end y_end+sz*0.6 y_end-sz*0.6], color_rgb, 'EdgeColor', 'none');
        case 'left'
            patch([x_end x_end+sz x_end+sz], [y_end y_end+sz*0.6 y_end-sz*0.6], color_rgb, 'EdgeColor', 'none');
        case 'down'
            patch([x_end x_end+sz*0.6 x_end-sz*0.6], [y_end y_end+sz y_end+sz], color_rgb, 'EdgeColor', 'none');
        case 'up'
            patch([x_end x_end+sz*0.6 x_end-sz*0.6], [y_end y_end-sz y_end-sz], color_rgb, 'EdgeColor', 'none');
    end
end
