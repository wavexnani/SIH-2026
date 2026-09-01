classdef WorldState
    % WORLDSTATE Complete world representation for Stage 0
    %
    % Contains:
    %   - Ego vehicle state
    %   - Dynamic agents
    %   - Static obstacles
    %   - Road boundaries
    %   - Simulation time
    %
    % Usage:
    %   world = WorldState(cfg);
    %   world = world.updateFromScenario(scenario, cfg);
    %   world = world.step(dt, L);
    %   world.visualize();
    
    properties
        % Time
        t               % Current simulation time (s)
        step_count      % Current step number
        
        % Ego state
        ego             % EgoState object
        
        % Dynamic agents
        agents          % Array of Agent objects
        n_agents        % Number of agents
        dynamic_agents  % Array of DynamicAgent objects (Phase 12A)
        
        % Static obstacles (only [x, y, L, W] stored)
        static_obs      % (n_obs x 5): [x, y, L, W, id]
        n_static_obs    % Number of static obstacles
        
        % Road
        road_length     % Total length (m)
        road_width      % Total width (m)
        road_center_y   % Center Y coordinate (m)
    end
    
    methods
        function obj = WorldState(cfg)
            % Constructor: Initialize from config
            %
            % Input: SimulationConfig object
            
            obj.t = 0;
            obj.step_count = 0;
            
            % Initialize ego at configured position
            obj.ego = EgoState(cfg.ego_x_init, cfg.ego_y_init, ...
                               cfg.ego_theta_init, cfg.ego_v_init, 0);
            
            % Initialize agents array
            obj.n_agents = cfg.n_agents;
            obj.agents = Agent.empty(cfg.n_agents, 0);
            for i = 1:cfg.n_agents
                obj.agents(i) = Agent(i, 0, 0, 0, 0);
            end
            
            % Initialize static obstacles
            obj.n_static_obs = cfg.n_static_obs;
            obj.static_obs = zeros(cfg.n_static_obs, 5);  % [x, y, L, W, id]
            
            % Road geometry
            obj.road_length = cfg.road_length;
            obj.road_width = cfg.road_width;
            obj.road_center_y = cfg.road_center_y;
        end
        
        function obj = setEgoState(obj, x, y, theta, v)
            % Set ego vehicle state
            obj.ego = EgoState(x, y, theta, v, obj.t);
        end
        
        function obj = setAgentState(obj, agent_id, x, y, vx, vy, sigma)
            % Set agent state
            %
            % Input:
            %   agent_id: agent ID (1 to n_agents)
            %   x, y:     position (m)
            %   vx, vy:   velocity (m/s)
            %   sigma:    uncertainty (m)
            
            if agent_id < 1 || agent_id > obj.n_agents
                error('Invalid agent ID: %d', agent_id);
            end
            
            obj.agents(agent_id) = Agent(agent_id, x, y, vx, vy, sigma);
        end
        
        function obj = setStaticObstacle(obj, obs_id, x, y, L, W)
            % Set static obstacle
            %
            % Input:
            %   obs_id: obstacle ID (1 to n_static_obs)
            %   x, y:   center position (m)
            %   L:      length along x (m)
            %   W:      width along y (m)
            
            if obs_id < 1 || obs_id > obj.n_static_obs
                error('Invalid obstacle ID: %d', obs_id);
            end
            
            obj.static_obs(obs_id, :) = [x, y, L, W, obs_id];
        end
        
        function ego_out = getEgoState(obj)
            % Get current ego state
            ego_out = obj.ego;
        end
        
        function agent_out = getAgent(obj, agent_id)
            % Get agent state
            agent_out = obj.agents(agent_id);
        end
        
        function obs_out = getStaticObstacle(obj, obs_id)
            % Get static obstacle
            % Returns: [x, y, L, W]
            obs_out = obj.static_obs(obs_id, 1:4);
        end
        
        function road_bounds = getRoadBounds(obj)
            % Get road boundary constraints
            % Returns: [x_min, x_max, y_min, y_max]
            x_min = 0;
            x_max = obj.road_length;
            y_min = obj.road_center_y - obj.road_width/2;
            y_max = obj.road_center_y + obj.road_width/2;
            road_bounds = [x_min, x_max, y_min, y_max];
        end
        
        function is_in_bounds = isEgoInBounds(obj, cfg)
            % Check if ego vehicle footprint stays within road bounds
            bounds = obj.getRoadBounds();
            x_min = bounds(1); x_max = bounds(2);
            y_min = bounds(3); y_max = bounds(4);
            
            % Half dimensions including safety margin
            half_L = cfg.vehicle_length / 2 + cfg.safety_margin;
            half_W = cfg.vehicle_width / 2 + cfg.safety_margin;
            
            % Oriented bounding box projections along axes
            r_x = abs(half_L * cos(obj.ego.theta)) + abs(half_W * sin(obj.ego.theta));
            r_y = abs(half_L * sin(obj.ego.theta)) + abs(half_W * cos(obj.ego.theta));
            
            is_in_bounds = (obj.ego.x - r_x > x_min) && ...
                           (obj.ego.x + r_x < x_max) && ...
                           (obj.ego.y - r_y > y_min) && ...
                           (obj.ego.y + r_y < y_max);
        end
        
        function min_clearance = getMinClearance(obj, cfg)
            % Compute ground-truth minimum footprint-aware clearance distance (meters) via 2D OBB SAT.
            % Ground-truth collision occurs ONLY when min_clearance <= 0.
            L_ego = cfg.vehicle_length;
            W_ego = cfg.vehicle_width;
            min_clearance = inf;
            
            % Check vs static obstacles
            for i = 1:obj.n_static_obs
                obs = obj.static_obs(i, :);
                if obs(1) > -50 && obs(3) > 0 && obs(4) > 0  % Active obstacle
                    obs_x = obs(1); obs_y = obs(2);
                    obs_L = obs(3); obs_W = obs(4);
                    
                    clearance = obj.computeOBBClearance(obj.ego.x, obj.ego.y, obj.ego.theta, ...
                                                        L_ego, W_ego, obs_x, obs_y, obs_L, obs_W, 0.0);
                    
                    if clearance < min_clearance
                        min_clearance = clearance;
                    end
                end
            end
            
            % Check vs dynamic agents
            for i = 1:obj.n_agents
                agent = obj.agents(i);
                if agent.x > -50
                    ag_x = agent.x; ag_y = agent.y;
                    ag_L = agent.length; ag_W = agent.width;
                    ag_theta = atan2(agent.vy, agent.vx + 1e-6);
                    
                    clearance = obj.computeOBBClearance(obj.ego.x, obj.ego.y, obj.ego.theta, ...
                                                        L_ego, W_ego, ag_x, ag_y, ag_L, ag_W, ag_theta);
                    
                    if clearance < min_clearance
                        min_clearance = clearance;
                    end
                end
            end
        end
        
        function is_collision = checkCollision(obj, cfg)
            % Check if ego collides with any obstacle
            is_collision = (obj.getMinClearance(cfg) <= 0);
        end
        
        function obj = stepEgo(obj, dt, L)
            % Update ego vehicle state (kinematic model)
            % Input:
            %   dt: time step (s)
            %   L:  wheelbase (m)
            
            obj.ego = obj.ego.kinematicUpdate(dt, L);
            obj.t = obj.ego.t;
            obj.step_count = obj.step_count + 1;
        end
        
        function obj = stepAgents(obj, dt)
            % Update all agents (DynamicAgent 2nd-order model or constant velocity)
            % Input: dt - time step (s)
            
            if ~isempty(obj.dynamic_agents)
                for i = 1:length(obj.dynamic_agents)
                    if ~isempty(obj.dynamic_agents(i)) && obj.dynamic_agents(i).id > 0
                        % Closed-loop update for DynamicAgent
                        obj.dynamic_agents(i).step(dt);
                        if i <= obj.n_agents
                            obj.agents(i) = obj.dynamic_agents(i).toLegacyAgent();
                        end
                    end
                end
            else
                for i = 1:obj.n_agents
                    obj.agents(i) = obj.agents(i).constantVelocityUpdate(dt);
                end
            end
        end
        
        function obj = step(obj, dt, L)
            % Single simulation step
            % Updates both ego and agents
            
            obj = obj.stepEgo(dt, L);
            obj = obj.stepAgents(dt);
        end
        
        function display(obj)
            % Display current world state
            fprintf('\n=== WORLD STATE (t=%.2f s, step %d) ===\n', obj.t, obj.step_count);
            fprintf('\nEgo:\n');
            obj.ego.display();
            fprintf('\nAgents (%d):\n', obj.n_agents);
            for i = 1:obj.n_agents
                fprintf('  ');
                obj.agents(i).display();
            end
            fprintf('\nStatic Obstacles (%d):\n', obj.n_static_obs);
            for i = 1:obj.n_static_obs
                obs = obj.static_obs(i, :);
                fprintf('  Obs %d: (%.2f, %.2f), %.1f m x %.1f m\n', ...
                        i, obs(1), obs(2), obs(3), obs(4));
            end
        end
        
        function visualize(obj, cfg, fig_handle)
            % Visualize current world state
            %
            % Input:
            %   cfg: SimulationConfig
            %   fig_handle: figure handle (optional)
            
            if nargin < 3
                figure('Name', 'SIH World State', 'NumberTitle', 'off');
                fig_handle = gcf;
            end
            
            clf(fig_handle);
            set(fig_handle, 'CurrentAxes', axes(fig_handle));
            
            hold on;
            axis equal;
            grid on;
            
            % Road
            bounds = obj.getRoadBounds();
            road_x = [bounds(1), bounds(2), bounds(2), bounds(1), bounds(1)];
            road_y = [bounds(3), bounds(3), bounds(4), bounds(4), bounds(3)];
            plot(road_x, road_y, 'k-', 'LineWidth', 2, 'DisplayName', 'Road');
            fill(road_x, road_y, [0.9, 0.9, 0.9], 'EdgeColor', 'k', 'FaceAlpha', 0.1);
            
            % Static obstacles
            for i = 1:obj.n_static_obs
                obs = obj.static_obs(i, :);
                x_c = obs(1); y_c = obs(2);
                L = obs(3); W = obs(4);
                
                % Rectangle corners
                x_rect = [x_c - L/2, x_c + L/2, x_c + L/2, x_c - L/2, x_c - L/2];
                y_rect = [y_c - W/2, y_c - W/2, y_c + W/2, y_c + W/2, y_c - W/2];
                
                plot(x_rect, y_rect, 'r-', 'LineWidth', 2);
                fill(x_rect, y_rect, 'r', 'FaceAlpha', 0.3);
                text(x_c, y_c, sprintf('O%d', i), 'HorizontalAlignment', 'center');
            end
            
            % Agents
            for i = 1:obj.n_agents
                agent = obj.agents(i);
                plot(agent.x, agent.y, 'bs', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
                
                % Velocity vector
                scale = 0.5;
                plot([agent.x, agent.x + scale * agent.vx], ...
                     [agent.y, agent.y + scale * agent.vy], 'b-');
                
                text(agent.x, agent.y - 0.3, sprintf('A%d', agent.id), ...
                     'HorizontalAlignment', 'center', 'FontSize', 8);
            end
            
            % Ego vehicle
            ego = obj.ego;
            r = cfg.collision_radius;
            theta_circle = linspace(0, 2*pi, 20);
            x_circle = ego.x + r * cos(theta_circle);
            y_circle = ego.y + r * sin(theta_circle);
            plot(x_circle, y_circle, 'g-', 'LineWidth', 2);
            fill(x_circle, y_circle, 'g', 'FaceAlpha', 0.3);
            
            % Heading arrow
            arrow_len = 1.5;
            plot([ego.x, ego.x + arrow_len * cos(ego.theta)], ...
                 [ego.y, ego.y + arrow_len * sin(ego.theta)], 'g-', 'LineWidth', 2);
            plot(ego.x, ego.y, 'g*', 'MarkerSize', 15);
            
            text(ego.x, ego.y - 0.5, sprintf('EGO (v=%.1f)', ego.v), ...
                 'HorizontalAlignment', 'center', 'FontSize', 9, 'FontWeight', 'bold');
            
            % Labels
            xlabel('X (m)');
            ylabel('Y (m)');
            title(sprintf('World State (t=%.2f s, step %d)', obj.t, obj.step_count));
            legend({'Road'}, 'Location', 'NorthEast');
            
            xlim([bounds(1) - 5, bounds(2) + 5]);
            ylim([bounds(3) - 2, bounds(4) + 2]);
            
            hold off;
            drawnow;
        end
    end
    
    methods (Static)
        function clearance = computeOBBClearance(ego_x, ego_y, ego_theta, L_e, W_e, obs_x, obs_y, L_o, W_o, obs_theta)
            % Separating Axis Theorem (SAT) 2D OBB vs OBB/AABB Clearance calculation
            if nargin < 10, obs_theta = 0.0; end
            
            u1 = [cos(ego_theta), sin(ego_theta)];
            u2 = [-sin(ego_theta), cos(ego_theta)];
            u3 = [cos(obs_theta), sin(obs_theta)];
            u4 = [-sin(obs_theta), cos(obs_theta)];
            
            axes = [u1; u2; u3; u4];
            D = [ego_x - obs_x, ego_y - obs_y];
            
            gaps = zeros(4, 1);
            for a = 1:4
                n = axes(a, :);
                d = abs(dot(D, n));
                r_e = (L_e/2) * abs(dot(u1, n)) + (W_e/2) * abs(dot(u2, n));
                r_o = (L_o/2) * abs(dot(u3, n)) + (W_o/2) * abs(dot(u4, n));
                gaps(a) = d - (r_e + r_o);
            end
            
            clearance = max(gaps);
        end
    end
end

