classdef TopDownTrafficVisualizer < handle
    % TOPDOWNTRAFFICVISUALIZER 2D Top-Down Autonomous Driving Simulation Renderer
    %
    % Purpose:
    %   Provides top-down visual simulation rendering for baseline and proposed scenarios.
    %   Renders asphalt road surface, grass shoulders, detailed vehicle/obstacle assets
    %   (Ego sedan, Auto-Rickshaw, Roadside Stall, Cattle), translucent free-space corridor
    %   overlays, camera tracking, and a minimal clean HUD without plot/engineering clutter.
    %   Operates strictly on logged simulation telemetry without changing physics/controller logic.
    
    properties
        fig             matlab.ui.Figure
        ax              matlab.graphics.axis.Axes
        mode_title      char = 'PROPOSED'
        sub_title       char = 'FREE SPACE'
        window_ahead    double = 30.0
        window_behind   double = 10.0
        is_freespace    logical = true
        show_corridor   logical = true
        show_trail      logical = true
    end
    
    methods
        function obj = TopDownTrafficVisualizer(varargin)
            % Constructor: Set up figure canvas
            p = inputParser();
            addParameter(p, 'Visible', 'off', @ischar);
            addParameter(p, 'Position', [50, 50, 1100, 500], @isnumeric);
            addParameter(p, 'ModeTitle', 'PROPOSED', @ischar);
            addParameter(p, 'SubTitle', 'FREE SPACE', @ischar);
            addParameter(p, 'IsFreeSpace', true, @islogical);
            parse(p, varargin{:});
            
            obj.mode_title = p.Results.ModeTitle;
            obj.sub_title = p.Results.SubTitle;
            obj.is_freespace = p.Results.IsFreeSpace;
            
            obj.fig = figure('Visible', p.Results.Visible, ...
                             'Position', p.Results.Position, ...
                             'Color', [0.12, 0.14, 0.16], ...
                             'InvertHardcopy', 'off');
                         
            obj.ax = axes('Parent', obj.fig, 'Position', [0, 0, 1, 1]);
            hold(obj.ax, 'on');
            axis(obj.ax, 'off');
            axis(obj.ax, 'equal');
        end
        
        function renderFrame(obj, world, map_obj, telemetry, step_idx, cfg)
            % RENDERFRAME Render top-down 2D simulation frame at step_idx
            if nargin < 6 || isempty(cfg), cfg = SimulationConfig(); end
            
            cla(obj.ax);
            hold(obj.ax, 'on');
            
            % Extract ego pose
            if isfield(telemetry, 'x') && length(telemetry.x) >= step_idx
                ego_x = telemetry.x(step_idx);
                ego_y = telemetry.y(step_idx);
                ego_v = telemetry.v(step_idx);
                if isfield(telemetry, 'theta') && length(telemetry.theta) >= step_idx
                    ego_th = telemetry.theta(step_idx);
                else
                    ego_th = world.ego.theta;
                end
            else
                ego_x = world.ego.x;
                ego_y = world.ego.y;
                ego_v = world.ego.v;
                ego_th = world.ego.theta;
            end
            
            % Camera tracking bounds
            x_min_cam = max(0, ego_x - obj.window_behind);
            x_max_cam = ego_x + obj.window_ahead;
            
            % 1. Render Environment: Grass shoulders and Asphalt road surface
            obj.drawEnvironment(map_obj, x_min_cam, x_max_cam);
            
            % 2. Render Corridor / Reference Representation
            if obj.is_freespace && obj.show_corridor && nargin >= 3 && ~isempty(map_obj)
                obj.drawFreeSpaceCorridor(map_obj, world, x_min_cam, x_max_cam, ego_x, ego_y, ego_th);
            else
                obj.drawFixedLaneCenterline(x_min_cam, x_max_cam);
            end
            
            % 3. Render Trajectory Trail (subtle fading line)
            if obj.show_trail && isfield(telemetry, 'x') && step_idx > 1
                trail_len = min(step_idx, 30);
                start_k = max(1, step_idx - trail_len);
                x_trail = telemetry.x(start_k:step_idx);
                y_trail = telemetry.y(start_k:step_idx);
                
                if obj.is_freespace
                    trail_col = [0.20, 0.65, 1.00];
                else
                    trail_col = [1.00, 0.35, 0.35];
                end
                
                plot(obj.ax, x_trail, y_trail, '-', 'Color', [trail_col, 0.60], ...
                    'LineWidth', 2.2);
            end
            
            % 4. Render Static Obstacles (Autorickshaw & Roadside Stall)
            if isprop(world, 'n_static_obs') && world.n_static_obs > 0
                for i = 1:world.n_static_obs
                    obs_x = world.static_obs(i, 1);
                    obs_y = world.static_obs(i, 2);
                    if obs_x < x_min_cam - 10 || obs_x > x_max_cam + 10, continue; end
                    
                    obs_L = world.static_obs(i, 3);
                    obs_W = world.static_obs(i, 4);
                    if obs_L <= 0, continue; end
                    
                    if i == 1 && abs(obs_x - 38.0) < 2.0
                        % Obstacle 1: Parked Auto-Rickshaw
                        obj.drawAutoRickshaw(obs_x, obs_y, 0.0, obs_L, obs_W);
                    elseif i == 2 && abs(obs_x - 48.0) < 5.0
                        % Obstacle 2: Roadside Stall / Structure
                        obj.drawRoadsideStall(obs_x, obs_y, 0.0, obs_L, obs_W);
                    else
                        % Generic structured obstacle
                        obj.drawGenericBoxObstacle(obs_x, obs_y, 0.0, obs_L, obs_W);
                    end
                end
            end
            
            % 5. Render Dynamic Agents (Cattle A, Cattle B, Motorcycle, Goat Herd, Herder)
            if isprop(world, 'agents') && ~isempty(world.agents)
                for a_i = 1:world.n_agents
                    ag = world.agents(a_i);
                    if ag.x < x_min_cam - 10 || ag.x > x_max_cam + 10 || ag.x <= 0, continue; end
                    ag_th = atan2(ag.vy, ag.vx + 1e-6);
                    if a_i == 1 || a_i == 2
                        % Cattle Agents
                        obj.drawCattleAgent(ag.x, ag.y, ag_th);
                    elseif a_i == 3 || ag.vx < -2.0
                        % Oncoming Motorcycle
                        obj.drawMotorcycle(ag.x, ag.y, ag_th);
                    elseif a_i >= 4 && a_i <= 8
                        % Goat Herd
                        obj.drawGoat(ag.x, ag.y, ag_th);
                    elseif a_i == 9
                        % Herder Guide
                        obj.drawHerder(ag.x, ag.y, ag_th);
                    else
                        % Generic obstacle
                        obj.drawGenericBoxObstacle(ag.x, ag.y, ag_th, ag.length, ag.width);
                    end
                end
            elseif isfield(telemetry, 'cattle_x') && length(telemetry.cattle_x) >= step_idx && telemetry.cattle_x(step_idx) > 0
                cat_x = telemetry.cattle_x(step_idx);
                cat_y = telemetry.cattle_y(step_idx);
                cat_th = telemetry.cattle_theta(step_idx);
                obj.drawCattleAgent(cat_x, cat_y, cat_th);
            end
            
            % 6. Render Ego Vehicle (Passenger Sedan)
            body_center_offset = 1.35; % m forward from rear axle
            body_x = ego_x + body_center_offset * cos(ego_th);
            body_y = ego_y + body_center_offset * sin(ego_th);
            obj.drawEgoSedan(body_x, body_y, ego_th, cfg.vehicle_length, cfg.vehicle_width);
            
            % 7. Render Minimal Clean HUD
            obj.drawMinimalHUD(step_idx, ego_x, ego_y, ego_v, telemetry, cfg);
            
            % Set View Bounds & Formatting
            xlim(obj.ax, [x_min_cam, x_max_cam]);
            ylim(obj.ax, [-1.2, 7.8]);
            axis(obj.ax, 'equal');
            axis(obj.ax, 'off');
            drawnow;
        end
        
        function drawEnvironment(obj, map_obj, x_min, x_max)
            % Draw organic background grass, dirt shoulder, and asymmetric asphalt road surface
            N_pts = 150;
            x_vec = linspace(x_min, x_max, N_pts);
            y_lo = zeros(1, N_pts);
            y_hi = zeros(1, N_pts);
            
            for i = 1:N_pts
                if ~isempty(map_obj)
                    [y_lo(i), y_hi(i)] = map_obj.getRoadBoundsAt(x_vec(i));
                else
                    y_lo(i) = 0.0; y_hi(i) = 6.0;
                end
            end
            
            % Organic Grass background fill across visible camera window
            x_bg = [x_min, x_max, x_max, x_min];
            y_bg = [-5.0, -5.0, 12.0, 12.0];
            fill(obj.ax, x_bg, y_bg, [0.25, 0.42, 0.20], 'EdgeColor', 'none');
            
            % Dirt shoulder band (asymmetric village road margins)
            x_dirt = [x_vec, fliplr(x_vec)];
            y_dirt_hi = [y_hi + 0.60, fliplr(y_hi)];
            y_dirt_lo = [y_lo - 0.40, fliplr(y_lo)];
            fill(obj.ax, x_dirt, y_dirt_hi, [0.55, 0.45, 0.32], 'EdgeColor', 'none');
            fill(obj.ax, x_dirt, y_dirt_lo, [0.55, 0.45, 0.32], 'EdgeColor', 'none');
            
            % Dark asphalt road polygon
            x_road = [x_vec, fliplr(x_vec)];
            y_road = [y_lo, fliplr(y_hi)];
            fill(obj.ax, x_road, y_road, [0.22, 0.24, 0.26], 'EdgeColor', 'none');
            
            % Road shoulder edges (dirt/gravel curbs)
            plot(obj.ax, x_vec, y_hi, '-', 'Color', [0.75, 0.68, 0.52], 'LineWidth', 2.0);
            plot(obj.ax, x_vec, y_lo, '-', 'Color', [0.75, 0.68, 0.52], 'LineWidth', 2.0);
            
            % Draw potholes & road decay patches
            obj.drawPotholes(x_min, x_max);
            
            % Draw roadside poles, signboards & small debris
            obj.drawRoadsideDetails(x_min, x_max);
        end
        
        function drawFreeSpaceCorridor(obj, map_obj, world, x_min, x_max, ego_x, ego_y, ego_th)
            % Draw Local Perception Window & Forward Sensing Fan centered at Ego
            if nargin < 6 || isempty(ego_x), ego_x = world.ego.x; ego_y = world.ego.y; ego_th = world.ego.theta; end
            
            % Sensing Fan Range (Local 25m Perception Horizon)
            sense_range = 25.0;
            x_sense_max = min(x_max, ego_x + sense_range);
            
            if x_sense_max > ego_x
                % 1. Render LiDAR / Radar Forward Sensing Cone / Fan
                fov_deg = 45;
                angles = linspace(-fov_deg/2, fov_deg/2, 17) * (pi/180) + ego_th;
                for a = 1:length(angles)
                    r_line = sense_range;
                    ray_x = [ego_x, ego_x + r_line * cos(angles(a))];
                    ray_y = [ego_y, ego_y + r_line * sin(angles(a))];
                    plot(obj.ax, ray_x, ray_y, ':', 'Color', [0.20, 0.85, 0.95, 0.18], 'LineWidth', 1.0);
                end
                
                % Sensing Horizon Arc
                arc_angles = linspace(-fov_deg/2, fov_deg/2, 25) * (pi/180) + ego_th;
                arc_x = ego_x + sense_range * cos(arc_angles);
                arc_y = ego_y + sense_range * sin(arc_angles);
                plot(obj.ax, arc_x, arc_y, '--', 'Color', [0.20, 0.85, 0.95, 0.45], 'LineWidth', 1.2);
                
                % 2. Local Perception Free-Space Corridor (within sensing range)
                N_pts = 60;
                x_vec = linspace(ego_x, x_sense_max, N_pts);
                
                half_W = 1.05;
                obs_world = world;
                [y_min_vec, y_max_vec] = map_obj.extractLocalBounds(x_vec', obs_world, half_W);
                
                y_lo = y_min_vec'; y_hi = y_max_vec';
                invalid = y_lo >= y_hi;
                if any(invalid)
                    y_lo(invalid) = 2.75 - 0.10;
                    y_hi(invalid) = 2.75 + 0.10;
                end
                
                % Translucent local drivable corridor polygon fill
                x_poly = [x_vec, fliplr(x_vec)];
                y_poly = [y_lo, fliplr(y_hi)];
                fill(obj.ax, x_poly, y_poly, [0.15, 0.75, 0.35], ...
                    'FaceAlpha', 0.25, 'EdgeColor', 'none');
                
                % Boundary outline curves of local free space
                plot(obj.ax, x_vec, y_hi, ':', 'Color', [0.30, 0.90, 0.45, 0.70], 'LineWidth', 1.5);
                plot(obj.ax, x_vec, y_lo, ':', 'Color', [0.30, 0.90, 0.45, 0.70], 'LineWidth', 1.5);
                
                % Corridor centerline
                plot(obj.ax, [ego_x, x_sense_max], [2.75, 2.75], '--', ...
                    'Color', [0.20, 0.85, 0.40, 0.50], 'LineWidth', 1.5);
            end
        end
        
        function drawFixedLaneCenterline(obj, x_min, x_max)
            % Draw subtle fixed-lane centerline (y = 1.80m)
            plot(obj.ax, [x_min, x_max], [1.80, 1.80], '--', ...
                'Color', [0.95, 0.40, 0.40, 0.60], 'LineWidth', 1.6);
        end
        
        function drawEgoSedan(obj, cx, cy, theta, L, W)
            % Draw high-fidelity 2D top-down passenger sedan
            % Footprint dimensions: L = 4.70m, W = 1.80m centered at (cx, cy)
            
            % Wheels (4 corners)
            wheel_L = 0.75; wheel_W = 0.25;
            w_offsets = [ L*0.30,  W*0.48;   % Front Left
                          L*0.30, -W*0.48;   % Front Right
                         -L*0.32,  W*0.48;   % Rear Left
                         -L*0.32, -W*0.48];  % Rear Right
            for w = 1:4
                w_xy = obj.rotateAndTranslate(w_offsets(w,1), w_offsets(w,2), cx, cy, theta);
                obj.drawOrientedPatch(w_xy(1), w_xy(2), theta, wheel_L, wheel_W, ...
                    [0.10, 0.10, 0.12], [0.05, 0.05, 0.05], 1.0);
            end
            
            % Main Sedan Body Shell
            obj.drawOrientedPatch(cx, cy, theta, L, W, ...
                [0.10, 0.45, 0.85], [0.04, 0.22, 0.50], 0.95);
            
            % Cabin & Roof
            roof_L = L * 0.48; roof_W = W * 0.72;
            obj.drawOrientedPatch(cx - L*0.02, cy, theta, roof_L, roof_W, ...
                [0.15, 0.55, 0.92], [0.08, 0.30, 0.60], 0.95);
            
            % Windshield (Front) & Rear Window
            wind_L = L * 0.14; wind_W = W * 0.65;
            fw_xy = obj.rotateAndTranslate(L*0.16, 0, cx, cy, theta);
            obj.drawOrientedPatch(fw_xy(1), fw_xy(2), theta, wind_L, wind_W, ...
                [0.12, 0.16, 0.22], [0.08, 0.12, 0.18], 0.90);
                
            rw_xy = obj.rotateAndTranslate(-L*0.20, 0, cx, cy, theta);
            obj.drawOrientedPatch(rw_xy(1), rw_xy(2), theta, wind_L*0.9, wind_W*0.9, ...
                [0.12, 0.16, 0.22], [0.08, 0.12, 0.18], 0.90);
                
            % Front Headlights (Bright Yellow/White)
            hl_offsets = [L*0.48, W*0.36; L*0.48, -W*0.36];
            for h = 1:2
                hl_xy = obj.rotateAndTranslate(hl_offsets(h,1), hl_offsets(h,2), cx, cy, theta);
                obj.drawOrientedPatch(hl_xy(1), hl_xy(2), theta, 0.25, 0.25, ...
                    [1.00, 0.92, 0.45], [0.85, 0.75, 0.20], 1.0);
            end
        end
        
        function drawAutoRickshaw(obj, cx, cy, theta, L, W)
            % Draw recognizable Indian Auto-Rickshaw (3-wheeler silhouette)
            % Bounding box: L = 2.20m, W = 1.50m centered at (cx, cy)
            
            % Main Body Base
            obj.drawOrientedPatch(cx, cy, theta, L, W, ...
                [0.88, 0.65, 0.08], [0.50, 0.35, 0.05], 0.95);
            
            % Canopy Top (Black/Dark Green Roof)
            roof_L = L * 0.70; roof_W = W * 0.90;
            roof_xy = obj.rotateAndTranslate(-L*0.08, 0, cx, cy, theta);
            obj.drawOrientedPatch(roof_xy(1), roof_xy(2), theta, roof_L, roof_W, ...
                [0.15, 0.18, 0.16], [0.08, 0.10, 0.08], 0.95);
            
            % Front Tapered Nose & Windshield
            nose_xy = obj.rotateAndTranslate(L*0.32, 0, cx, cy, theta);
            obj.drawOrientedPatch(nose_xy(1), nose_xy(2), theta, L*0.28, W*0.50, ...
                [0.18, 0.70, 0.65], [0.10, 0.40, 0.35], 0.95);
            
            % Wheels (1 Front Center, 2 Rear)
            f_wheel = obj.rotateAndTranslate(L*0.42, 0, cx, cy, theta);
            obj.drawOrientedPatch(f_wheel(1), f_wheel(2), theta, 0.40, 0.18, [0.1, 0.1, 0.1], [0,0,0], 1.0);
            
            r1_wheel = obj.rotateAndTranslate(-L*0.35, W*0.45, cx, cy, theta);
            obj.drawOrientedPatch(r1_wheel(1), r1_wheel(2), theta, 0.45, 0.20, [0.1, 0.1, 0.1], [0,0,0], 1.0);
            
            r2_wheel = obj.rotateAndTranslate(-L*0.35, -W*0.45, cx, cy, theta);
            obj.drawOrientedPatch(r2_wheel(1), r2_wheel(2), theta, 0.45, 0.20, [0.1, 0.1, 0.1], [0,0,0], 1.0);
            
            % Label / Symbol
            plot(obj.ax, cx, cy, 'k.', 'MarkerSize', 6);
        end
        
        function drawRoadsideStall(obj, cx, cy, theta, L, W)
            % Draw recognizable Roadside Stall / Encroaching Structure
            % Bounding box: L = 4.50m, W = 1.50m centered at (cx, cy)
            
            % Wooden Frame / Platform Base
            obj.drawOrientedPatch(cx, cy, theta, L, W, ...
                [0.52, 0.34, 0.18], [0.30, 0.18, 0.08], 0.95);
            
            % Striped Awning Roof Canopy
            N_stripes = 6;
            stripe_w = L / N_stripes;
            start_x = -L/2 + stripe_w/2;
            for s = 1:N_stripes
                x_rel = start_x + (s-1)*stripe_w;
                s_xy = obj.rotateAndTranslate(x_rel, 0, cx, cy, theta);
                if mod(s, 2) == 1
                    s_col = [0.85, 0.20, 0.20]; % Red
                else
                    s_col = [0.95, 0.95, 0.92]; % White
                end
                obj.drawOrientedPatch(s_xy(1), s_xy(2), theta, stripe_w*0.95, W*0.85, ...
                    s_col, [0.3, 0.1, 0.1], 0.95);
            end
            
            % Support Poles at Corners
            corner_offsets = [L*0.45, W*0.42; L*0.45, -W*0.42; -L*0.45, W*0.42; -L*0.45, -W*0.42];
            for c = 1:4
                c_xy = obj.rotateAndTranslate(corner_offsets(c,1), corner_offsets(c,2), cx, cy, theta);
                plot(obj.ax, c_xy(1), c_xy(2), 'ko', 'MarkerFaceColor', [0.2, 0.1, 0.0], 'MarkerSize', 4);
            end
        end
        
        function drawCattleAgent(obj, cx, cy, theta)
            % Draw bovine top-down silhouette for dynamic cattle agent
            % Body dimensions ~ 2.0m x 0.9m centered at (cx, cy)
            L_cat = 2.0; W_cat = 0.90;
            
            % Torso / Body (Brown / Off-White)
            obj.drawOrientedPatch(cx, cy, theta, L_cat*0.65, W_cat*0.85, ...
                [0.72, 0.52, 0.32], [0.42, 0.28, 0.14], 0.95);
            
            % Head & Horns at front
            head_xy = obj.rotateAndTranslate(L_cat*0.42, 0, cx, cy, theta);
            obj.drawOrientedPatch(head_xy(1), head_xy(2), theta, L_cat*0.30, W_cat*0.60, ...
                [0.65, 0.45, 0.26], [0.35, 0.20, 0.10], 0.95);
                
            % Horns
            h1 = obj.rotateAndTranslate(L_cat*0.52, W_cat*0.40, cx, cy, theta);
            h2 = obj.rotateAndTranslate(L_cat*0.52, -W_cat*0.40, cx, cy, theta);
            plot(obj.ax, [head_xy(1), h1(1)], [head_xy(2), h1(2)], 'w-', 'LineWidth', 2.0);
            plot(obj.ax, [head_xy(1), h2(1)], [head_xy(2), h2(2)], 'w-', 'LineWidth', 2.0);
        end
        
        function drawGenericBoxObstacle(obj, cx, cy, theta, L, W)
            obj.drawOrientedPatch(cx, cy, theta, L, W, ...
                [0.60, 0.60, 0.62], [0.30, 0.30, 0.32], 0.90);
        end
        
        function drawMinimalHUD(obj, step_idx, ego_x, ego_y, ego_v, telemetry, cfg)
            % Render clean top-left HUD panel overlay
            t_sec = (step_idx - 1) * cfg.dt;
            v_kmh = ego_v * 3.6;
            
            % Determine status badge
            if isfield(telemetry, 'sf_active') && step_idx <= length(telemetry.sf_active) && telemetry.sf_active(step_idx)
                status_txt = 'STATUS: SAFETY STOP';
                status_col = [1.00, 0.30, 0.30];
            elseif isfield(telemetry, 'intent') && step_idx <= length(telemetry.intent) && strcmp(telemetry.intent{step_idx}, 'YIELD')
                status_txt = 'STATUS: YIELDING';
                status_col = [1.00, 0.70, 0.20];
            elseif ego_x >= 48.0
                status_txt = 'STATUS: BOTTLENECK PASSED';
                status_col = [0.20, 0.85, 0.40];
            else
                status_txt = 'STATUS: NAVIGATING';
                status_col = [0.30, 0.80, 1.00];
            end
            
            % Draw HUD Card background annotation
            card_str = sprintf('\\bf%s  |  %s\\rm\nSpeed: %.1f km/h  |  Dist: %.1f m  |  t: %.1fs\n%s', ...
                obj.mode_title, obj.sub_title, v_kmh, ego_x, t_sec, status_txt);
            
            annotation(obj.fig, 'textbox', [0.03, 0.76, 0.32, 0.19], ...
                'String', card_str, ...
                'BackgroundColor', [0.08, 0.10, 0.12, 0.82], ...
                'EdgeColor', [0.25, 0.30, 0.35], ...
                'Color', [0.95, 0.95, 0.97], ...
                'FontName', 'Helvetica', 'FontSize', 10, ...
                'FitBoxToText', 'on');
        end
        
        function drawOrientedPatch(obj, cx, cy, theta, L, W, col_fill, col_edge, alpha_val)
            cos_th = cos(theta); sin_th = sin(theta);
            dx = L / 2; dy = W / 2;
            corners_local = [ dx,  dy;
                             -dx,  dy;
                             -dx, -dy;
                              dx, -dy]';
            R = [cos_th, -sin_th; sin_th, cos_th];
            corners_world = R * corners_local + [cx; cy];
            
            patch(obj.ax, corners_world(1, :), corners_world(2, :), col_fill, ...
                'FaceAlpha', alpha_val, 'EdgeColor', col_edge, 'LineWidth', 1.2);
        end
        
        function pt_out = rotateAndTranslate(obj, x_local, y_local, cx, cy, theta)
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            res = R * [x_local; y_local] + [cx; cy];
            pt_out = res';
        end
        
        function drawMotorcycle(obj, cx, cy, theta)
            % Draw 2D top-down motorcycle primitive (rider torso, handlebars, chassis)
            L_mc = 2.0; W_mc = 0.80;
            % Chassis / Frame
            obj.drawOrientedPatch(cx, cy, theta, L_mc, W_mc*0.35, ...
                [0.15, 0.15, 0.18], [0.05, 0.05, 0.05], 1.0);
            % Rider Torso
            obj.drawOrientedPatch(cx - L_mc*0.1, cy, theta, L_mc*0.4, W_mc*0.75, ...
                [0.85, 0.35, 0.15], [0.50, 0.15, 0.05], 1.0);
            % Rider Helmet
            h_xy = obj.rotateAndTranslate(L_mc*0.05, 0, cx, cy, theta);
            plot(obj.ax, h_xy(1), h_xy(2), 'ko', 'MarkerFaceColor', [0.9, 0.9, 0.1], 'MarkerSize', 6);
            % Front/Rear Wheels
            fw = obj.rotateAndTranslate(L_mc*0.45, 0, cx, cy, theta);
            rw = obj.rotateAndTranslate(-L_mc*0.45, 0, cx, cy, theta);
            obj.drawOrientedPatch(fw(1), fw(2), theta, 0.45, 0.15, [0.1,0.1,0.1], [0,0,0], 1.0);
            obj.drawOrientedPatch(rw(1), rw(2), theta, 0.45, 0.15, [0.1,0.1,0.1], [0,0,0], 1.0);
        end
        
        function drawGoat(obj, cx, cy, theta)
            % Draw goat quadruped avatar (length ~ 1.0m, width ~ 0.45m)
            L_g = 1.0; W_g = 0.45;
            % Body (White / Off-white / Speckled)
            obj.drawOrientedPatch(cx, cy, theta, L_g*0.65, W_g*0.85, ...
                [0.92, 0.88, 0.80], [0.55, 0.50, 0.42], 0.95);
            % Head
            head_xy = obj.rotateAndTranslate(L_g*0.40, 0, cx, cy, theta);
            obj.drawOrientedPatch(head_xy(1), head_xy(2), theta, L_g*0.30, W_g*0.60, ...
                [0.85, 0.78, 0.70], [0.45, 0.40, 0.32], 0.95);
            % Small Horns
            h1 = obj.rotateAndTranslate(L_g*0.48, W_g*0.35, cx, cy, theta);
            h2 = obj.rotateAndTranslate(L_g*0.48, -W_g*0.35, cx, cy, theta);
            plot(obj.ax, [head_xy(1), h1(1)], [head_xy(2), h1(2)], '-', 'Color', [0.3, 0.2, 0.1], 'LineWidth', 1.5);
            plot(obj.ax, [head_xy(1), h2(1)], [head_xy(2), h2(2)], '-', 'Color', [0.3, 0.2, 0.1], 'LineWidth', 1.5);
        end
        
        function drawHerder(obj, cx, cy, theta)
            % Draw top-down human herder avatar (length ~ 0.6m, width ~ 0.5m)
            L_h = 0.60; W_h = 0.50;
            % Shoulders / Torso (Bright Red Shirt)
            obj.drawOrientedPatch(cx, cy, theta, L_h*0.50, W_h*0.90, ...
                [0.85, 0.25, 0.20], [0.45, 0.10, 0.08], 0.95);
            % Head (Tan / Skin Tone circle)
            h_xy = obj.rotateAndTranslate(L_h*0.10, 0, cx, cy, theta);
            plot(obj.ax, h_xy(1), h_xy(2), 'o', 'MarkerFaceColor', [0.85, 0.65, 0.45], ...
                'MarkerEdgeColor', [0.4, 0.2, 0.1], 'MarkerSize', 6);
            % Herder Walking Stick
            s1 = obj.rotateAndTranslate(L_h*0.45, W_h*0.60, cx, cy, theta);
            s2 = obj.rotateAndTranslate(-L_h*0.30, W_h*0.60, cx, cy, theta);
            plot(obj.ax, [s1(1), s2(1)], [s1(2), s2(2)], '-', 'Color', [0.4, 0.25, 0.1], 'LineWidth', 2.0);
        end
        
        function drawPotholes(obj, x_min, x_max)
            % Draw realistic irregular potholes on asphalt
            potholes = [28.0, 1.80, 1.80, 0.90;
                        72.0, 3.80, 2.20, 1.10;
                        105.0, 2.20, 1.90, 0.85;
                        145.0, 3.50, 2.50, 1.20];
            for p = 1:size(potholes, 1)
                px = potholes(p, 1); py = potholes(p, 2);
                pL = potholes(p, 3); pW = potholes(p, 4);
                if px < x_min - 5 || px > x_max + 5, continue; end
                obj.drawOrientedPatch(px, py, 0.15, pL, pW, ...
                    [0.12, 0.13, 0.14], [0.06, 0.07, 0.08], 0.85);
            end
        end
        
        function drawRoadsideDetails(obj, x_min, x_max)
            % Draw utility poles, signboards & small debris along shoulders
            pole_x = [15.0, 55.0, 95.0, 135.0, 175.0];
            for k = 1:length(pole_x)
                if pole_x(k) >= x_min - 5 && pole_x(k) <= x_max + 5
                    plot(obj.ax, pole_x(k), 5.80, 's', 'MarkerFaceColor', [0.3, 0.3, 0.35], ...
                        'MarkerEdgeColor', [0.1, 0.1, 0.1], 'MarkerSize', 5);
                end
            end
            if 20.0 >= x_min - 5 && 20.0 <= x_max + 5
                obj.drawOrientedPatch(20.0, 5.70, 0.0, 1.2, 0.3, ...
                    [0.2, 0.6, 0.3], [0.1, 0.3, 0.1], 1.0);
            end
        end
        
        function saveFrame(obj, filename)
            saveas(obj.fig, filename);
        end
        
        function close(obj)
            if isvalid(obj.fig)
                close(obj.fig);
            end
        end
    end
end
