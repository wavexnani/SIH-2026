classdef SceneVisualizer < handle
    % SCENEVISUALIZER High-Fidelity 2D Scenario & Trajectory Renderer
    %
    % Purpose:
    %   Renders drivable FreeSpaceMap boundaries, oriented vehicle body footprints,
    %   dynamic agents, static obstacles, MPC predicted horizons, and active topology overlays.
    %   Operates WITHOUT modifying any planner or controller mathematics.
    
    properties
        fig             matlab.ui.Figure
        ax              matlab.graphics.axis.Axes
        show_prediction logical = true
        show_legend     logical = true
        window_width    double = 30.0
    end
    
    methods
        function obj = SceneVisualizer(varargin)
            % Constructor: Set up figure canvas
            p = inputParser();
            addParameter(p, 'Visible', 'off', @ischar);
            addParameter(p, 'Position', [100, 100, 1200, 500], @isnumeric);
            parse(p, varargin{:});
            
            obj.fig = figure('Visible', p.Results.Visible, ...
                             'Position', p.Results.Position, ...
                             'Color', [0.96, 0.96, 0.97]);
            obj.ax = axes('Parent', obj.fig);
            hold(obj.ax, 'on');
            box(obj.ax, 'on');
            grid(obj.ax, 'on');
            set(obj.ax, 'FontName', 'Helvetica', 'FontSize', 11);
        end
        
        function render(obj, world, map_obj, pred_states, info, ref_path, step_idx)
            % RENDER Render complete 2D scene for current timestep
            if nargin < 7, step_idx = 1; end
            
            cla(obj.ax);
            hold(obj.ax, 'on');
            
            ego_x = world.ego.x;
            % A compact, equal-scale camera avoids visually distorting the
            % 4.7 m x 1.8 m footprint across an 80 m wide scene.
            x_min_view = max(0, ego_x - 10.0);
            x_max_view = ego_x + obj.window_width;
            
            % 1. Render Drivable Free-Space Region & Boundaries from FreeSpaceMap
            if nargin >= 3 && ~isempty(map_obj)
                obj.drawFreeSpaceMap(map_obj, x_min_view, x_max_view);
            else
                obj.drawCorridorRoad(world, x_min_view, x_max_view);
            end
            
            % 2. Render Reference Trajectory
            if nargin >= 6 && ~isempty(ref_path)
                valid_idx = ref_path(:, 1) >= x_min_view & ref_path(:, 1) <= x_max_view;
                plot(obj.ax, ref_path(valid_idx, 1), ref_path(valid_idx, 2), ...
                    'k:', 'LineWidth', 1.4, 'DisplayName', 'Reference Path');
            end
            
            % 3. Render Static Obstacles (Oriented Rectangular Body Footprints)
            if isprop(world, 'n_static_obs') && world.n_static_obs > 0
                for i = 1:world.n_static_obs
                    obs_x = world.static_obs(i, 1);
                    obs_y = world.static_obs(i, 2);
                    if obs_x < x_min_view - 10 || obs_x > x_max_view + 10, continue; end
                    
                    if size(world.static_obs, 2) >= 4
                        obs_L = world.static_obs(i, 3);
                        obs_W = world.static_obs(i, 4);
                    else
                        obs_L = 1.8; obs_W = 1.8;
                    end
                    obj.drawOrientedBox(obs_x, obs_y, 0.0, obs_L, obs_W, ...
                        [0.85, 0.33, 0.10], [0.55, 0.15, 0.05], sprintf('Obstacle %d', i));
                end
            end
            
            % 4. Render Dynamic Agents
            if isprop(world, 'dynamic_agents') && ~isempty(world.dynamic_agents)
                for a_i = 1:length(world.dynamic_agents)
                    ag = world.dynamic_agents(a_i);
                    if ag.x < x_min_view - 10 || ag.x > x_max_view + 10, continue; end
                    ag_L = 4.5; ag_W = 1.8;
                    if isfield(ag, 'length'), ag_L = ag.length; end
                    if isfield(ag, 'width'), ag_W = ag.width; end
                    
                    ag_th = 0.0;
                    if isfield(ag, 'theta'), ag_th = ag.theta; end
                    
                    color_fill = [0.49, 0.18, 0.56];
                    color_edge = [0.30, 0.10, 0.35];
                    obj.drawOrientedBox(ag.x, ag.y, ag_th, ag_L, ag_W, ...
                        color_fill, color_edge, sprintf('Agent %d', a_i));
                end
            end
            
            % 5. Render MPC Predicted Trajectory & Horizon Preview (Body Center Coordinates)
            if obj.show_prediction && nargin >= 4 && ~isempty(pred_states)
                body_center_offset = 1.35;
                Np_pred = size(pred_states, 1);
                pred_bc = zeros(Np_pred, 2);
                for k = 1:Np_pred
                    pth_k = 0.0;
                    if size(pred_states, 2) >= 3, pth_k = pred_states(k, 3); end
                    pred_bc(k, 1) = pred_states(k, 1) + body_center_offset * cos(pth_k);
                    pred_bc(k, 2) = pred_states(k, 2) + body_center_offset * sin(pth_k);
                end
                plot(obj.ax, pred_bc(:, 1), pred_bc(:, 2), ...
                    'g-o', 'LineWidth', 2.0, 'MarkerSize', 4, ...
                    'MarkerFaceColor', 'g', 'DisplayName', 'Optimizer horizon');
            end
            
            % 6. Render Ego Vehicle Footprint about its physical body center.
            % The bicycle state is the rear-axle reference point, while the
            % body centre lies 1.35 m ahead for the configured sedan geometry.
            body_center_offset = 1.35;
            body_x = world.ego.x + body_center_offset * cos(world.ego.theta);
            body_y = world.ego.y + body_center_offset * sin(world.ego.theta);
            obj.drawOrientedBox(body_x, body_y, world.ego.theta, ...
                4.7, 1.8, [0.0, 0.45, 0.74], [0.0, 0.20, 0.50], 'Ego Vehicle');
            
            % Draw velocity vector arrow
            arrow_len = max(1.5, world.ego.v * 0.5);
            quiver(obj.ax, body_x, body_y, ...
                arrow_len * cos(world.ego.theta), arrow_len * sin(world.ego.theta), ...
                0, 'Color', [0.0, 0.20, 0.50], 'LineWidth', 2.0, 'MaxHeadSize', 0.8);
            
            % 7. Render Status Overlay Text Box & HUD Panel
            topology_str = 'N/A';
            filter_str = 'PASSIVE';
            event_str = '';
            if nargin >= 5 && isstruct(info)
                if isfield(info, 'selected_topology'), topology_str = info.selected_topology; end
                if isfield(info, 'macro_intent'), intent_str = info.macro_intent; else, intent_str = 'N/A'; end
                if isfield(info, 'filter_active') && info.filter_active
                    filter_str = sprintf('ACTIVE (%s)', info.filter_reason);
                end
                if isfield(info, 'event_banner'), event_str = info.event_banner; end
            else
                intent_str = 'N/A';
            end
            
            min_clr = world.getMinClearance(SimulationConfig());
            t_curr = (step_idx - 1) * 0.1;
            status_text = sprintf(['[HUD TELEMETRY] t = %.1fs | Step %d\n' ...
                'Ego Position: (%.1fm, %.1fm) | Speed: %.1fm/s | Steering: %+.1f deg\n' ...
                'Macro Intent: %s | Topology: %s\n' ...
                'Min Clearance: %+.2fm | Safety Filter: %s'], ...
                t_curr, step_idx, world.ego.x, world.ego.y, world.ego.v, ...
                rad2deg(world.ego.delta), intent_str, topology_str, min_clr, filter_str);
            
            if isfield(info, 'delta_cmd') && isfield(info, 'delta_actual')
                status_text = sprintf(['%s\n' ...
                    'Actuator Cmd/Act: Steer (%+.1f / %+.1f deg) | Accel (%+.1f / %+.1f m/s²)'], ...
                    status_text, rad2deg(info.delta_cmd), rad2deg(info.delta_actual), ...
                    info.a_cmd, info.a_actual);
            end
            
            annotation(obj.fig, 'textbox', [0.14, 0.72, 0.42, 0.18], ...
                'String', status_text, 'BackgroundColor', [1.0, 1.0, 1.0, 0.90], ...
                'EdgeColor', [0.2, 0.2, 0.2], 'FontName', 'Helvetica', ...
                'FontSize', 9, 'FitBoxToText', 'on');
            
            if ~isempty(event_str)
                annotation(obj.fig, 'textbox', [0.14, 0.63, 0.42, 0.07], ...
                    'String', sprintf('EVENT: %s', event_str), 'BackgroundColor', [1.0, 0.95, 0.80, 0.95], ...
                    'EdgeColor', [0.8, 0.5, 0.0], 'FontName', 'Helvetica', ...
                    'FontSize', 10, 'FontWeight', 'bold', 'FitBoxToText', 'on');
            end
            
            % Canvas Formatting
            xlabel(obj.ax, 'Longitudinal Position x (m)', 'FontSize', 12);
            ylabel(obj.ax, 'Lateral Position y (m)', 'FontSize', 12);
            title(obj.ax, 'Autonomous Safety Coordination Simulation', 'FontSize', 13, 'FontWeight', 'bold');
            xlim(obj.ax, [x_min_view, x_max_view]);
            ylim(obj.ax, [-1.5, 7.5]);
            axis(obj.ax, 'equal');
            
            if obj.show_legend
                legend(obj.ax, 'Location', 'northeast');
            end
            drawnow;
        end
        
        function drawFreeSpaceMap(obj, map_obj, x_min, x_max)
            % Draw drivable polygon fill and boundary curves from FreeSpaceMap
            N_pts = 200;
            x_vec = linspace(x_min, x_max, N_pts);
            y_min_vec = zeros(1, N_pts);
            y_max_vec = zeros(1, N_pts);
            
            for i = 1:N_pts
                [y_min_vec(i), y_max_vec(i)] = map_obj.getRoadBoundsAt(x_vec(i));
            end
            
            % Create closed boundary polygon for drivable region fill
            x_poly = [x_vec, fliplr(x_vec)];
            y_poly = [y_min_vec, fliplr(y_max_vec)];
            
            fill(obj.ax, x_poly, y_poly, [0.88, 0.93, 0.96], ...
                'EdgeColor', 'none', 'DisplayName', 'Drivable Free Space');
            
            % Draw solid road boundaries
            plot(obj.ax, x_vec, y_max_vec, 'r-', 'LineWidth', 2.2, 'DisplayName', 'Left Road Boundary');
            plot(obj.ax, x_vec, y_min_vec, 'r-', 'LineWidth', 2.2, 'DisplayName', 'Right Road Boundary');
        end
        
        function drawCorridorRoad(obj, world, x_min, x_max)
            bounds = world.getRoadBounds();
            y_min = bounds(3); y_max = bounds(4);
            x_poly = [x_min, x_max, x_max, x_min];
            y_poly = [y_min, y_min, y_max, y_max];
            
            fill(obj.ax, x_poly, y_poly, [0.92, 0.92, 0.94], ...
                'EdgeColor', 'none', 'DisplayName', 'Fixed Corridor');
            yline(obj.ax, y_max, 'r-', 'LineWidth', 2.2, 'DisplayName', 'Road Bounds');
            yline(obj.ax, y_min, 'r-', 'LineWidth', 2.2, 'HandleVisibility', 'off');
        end
        
        function drawOrientedBox(obj, cx, cy, theta, L, W, color_fill, color_edge, label_str)
            % Compute 4 rotated corner coordinates for Oriented Bounding Box
            cos_th = cos(theta); sin_th = sin(theta);
            dx = L / 2; dy = W / 2;
            
            % Local corner offsets
            corners_local = [ dx,  dy;
                             -dx,  dy;
                             -dx, -dy;
                              dx, -dy]';
            
            % Rotation matrix
            R = [cos_th, -sin_th; sin_th, cos_th];
            corners_world = R * corners_local + [cx; cy];
            
            patch(obj.ax, corners_world(1, :), corners_world(2, :), color_fill, ...
                'FaceAlpha', 0.85, 'EdgeColor', color_edge, 'LineWidth', 1.8, ...
                'DisplayName', label_str);
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
