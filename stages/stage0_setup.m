function stage0_setup(varargin)
    % STAGE0_SETUP - Simulation Infrastructure Test
    %
    % Purpose: Verify that world model, scenario loading, and visualization work
    % This is Stage 0 - NO planning, NO collision checking, NO prediction
    %
    % Success criteria:
    %   ✓ Vehicles initialize correctly
    %   ✓ Vehicles update states over time
    %   ✓ Visualization displays correctly
    %   ✓ Simulation runs for 10 seconds without errors
    %
    % Usage:
    %   stage0_setup();
    %   stage0_setup('scenario', 'moderate', 'verbose', true);
    
    % === SET UP EXPLICIT MATLAB PATHS ===
    current_dir = fileparts(mfilename('fullpath'));
    if isempty(current_dir)
        projectRoot = pwd;
    else
        % Since this file is in stages/, its parent is the project root
        projectRoot = fileparts(current_dir);
    end
    addpath(fullfile(projectRoot, 'config'));
    addpath(fullfile(projectRoot, 'core'));
    addpath(fullfile(projectRoot, 'environment'));
    addpath(fullfile(projectRoot, 'vehicle'));
    addpath(fullfile(projectRoot, 'planning'));
    addpath(fullfile(projectRoot, 'metrics'));
    addpath(fullfile(projectRoot, 'stages'));
    
    % Parse arguments
    parser = inputParser;
    addParameter(parser, 'scenario', 'moderate', @ischar);
    addParameter(parser, 'verbose', true, @islogical);
    addParameter(parser, 'visualize', true, @islogical);
    addParameter(parser, 'duration', 10.0, @isnumeric);  % seconds
    
    parse(parser, varargin{:});
    
    scenario_name = parser.Results.scenario;
    verbose = parser.Results.verbose;
    visualize = parser.Results.visualize;
    T_sim = parser.Results.duration;
    
    % =====================================================
    % INITIALIZATION
    % =====================================================
    
    if verbose
        fprintf('\n');
        fprintf('╔════════════════════════════════════════════════════════╗\n');
        fprintf('║          STAGE 0: SIMULATION INFRASTRUCTURE            ║\n');
        fprintf('║                                                        ║\n');
        fprintf('║  Testing: World model, Scenario loading, Visualization║\n');
        fprintf('╚════════════════════════════════════════════════════════╝\n\n');
    end
    
    % Load configuration
    cfg = SimulationConfig();
    if verbose
        fprintf('[CONFIG] Simulation parameters loaded\n');
        cfg.display();
    end
    
    % Create scenario
    if verbose
        fprintf('[SCENARIO] Loading scenario: "%s"\n', scenario_name);
    end
    world = ScenarioDefinitions(scenario_name, cfg);
    
    if verbose
        fprintf('[WORLD] World initialized\n');
        world.display();
    end
    
    % =====================================================
    % SIMULATION LOOP
    % =====================================================
    
    % Calculate number of steps
    N_steps = ceil(T_sim / cfg.dt);
    
    % Storage for history
    history.t = zeros(N_steps, 1);
    history.ego_x = zeros(N_steps, 1);
    history.ego_y = zeros(N_steps, 1);
    history.ego_theta = zeros(N_steps, 1);
    history.ego_v = zeros(N_steps, 1);
    
    % Agent history
    history.agent_x = zeros(N_steps, cfg.n_agents);
    history.agent_y = zeros(N_steps, cfg.n_agents);
    history.agent_v = zeros(N_steps, cfg.n_agents);
    
    % Create figure for visualization
    if visualize
        fig_handle = figure('Name', 'Stage 0: World State Visualization', ...
                           'NumberTitle', 'off', ...
                           'Position', [100, 100, 1000, 600]);
    end
    
    if verbose
        fprintf('\n[SIMULATION] Starting %d steps (%.1f seconds at dt=%.2f s)\n', ...
                N_steps, T_sim, cfg.dt);
        fprintf('[SIMULATION] Ego initial state: (%.2f, %.2f, %.3f rad, %.2f m/s)\n', ...
                world.ego.x, world.ego.y, world.ego.theta, world.ego.v);
    end
    
    % Set constant control for ego
    a_constant = 1.0;  % 1 m/s² acceleration
    delta_constant = 0;  % Straight ahead
    world.ego = world.ego.setControl(a_constant, delta_constant);
    
    % Simulation loop
    for k = 1:N_steps
        % Store history
        history.t(k) = world.t;
        history.ego_x(k) = world.ego.x;
        history.ego_y(k) = world.ego.y;
        history.ego_theta(k) = world.ego.theta;
        history.ego_v(k) = world.ego.v;
        
        for i = 1:cfg.n_agents
            history.agent_x(k, i) = world.agents(i).x;
            history.agent_y(k, i) = world.agents(i).y;
            history.agent_v(k, i) = world.agents(i).getSpeed();
        end
        
        % Visualize every 5th step
        if visualize && mod(k, 5) == 0
            world.visualize(cfg, fig_handle);
        end
        
        % Update world state
        world = world.step(cfg.dt, cfg.wheelbase);
        
        % Print progress every 50 steps
        if verbose && mod(k, 50) == 0
            fprintf('[STEP %4d] t=%.2f s, ego=(%.2f, %.2f), v=%.2f m/s\n', ...
                    k, world.t, world.ego.x, world.ego.y, world.ego.v);
        end
        
        % Check if ego goes out of bounds
        if ~world.isEgoInBounds(cfg)
            if verbose
                fprintf('[WARNING] Ego vehicle out of road bounds at t=%.2f s\n', world.t);
            end
            break;
        end
    end
    
    % =====================================================
    % RESULTS & VALIDATION
    % =====================================================
    
    if verbose
        fprintf('\n[SIMULATION] Complete!\n');
        fprintf('[RESULTS] Final ego state:\n');
        fprintf('  Position:  (%.2f, %.2f) m\n', world.ego.x, world.ego.y);
        fprintf('  Heading:   %.3f rad\n', world.ego.theta);
        fprintf('  Speed:     %.2f m/s\n', world.ego.v);
        fprintf('  Time:      %.2f s\n', world.t);
        fprintf('[RESULTS] Steps completed: %d / %d\n', world.step_count, N_steps);
    end
    
    % =====================================================
    % VISUALIZATION OF TRAJECTORY
    % =====================================================
    
    if visualize
        % Create trajectory figure
        figure('Name', 'Stage 0: Trajectory Analysis', 'NumberTitle', 'off');
        
        % Plot 1: Ego trajectory
        subplot(2, 2, 1);
        plot(history.ego_x, history.ego_y, 'g-', 'LineWidth', 2);
        hold on;
        plot(history.ego_x(1), history.ego_y(1), 'go', 'MarkerSize', 8, ...
             'MarkerFaceColor', 'g', 'DisplayName', 'Start');
        plot(history.ego_x(end), history.ego_y(end), 'r*', 'MarkerSize', 12, ...
             'DisplayName', 'End');
        
        % Plot agents
        for i = 1:cfg.n_agents
            plot(history.agent_x(:, i), history.agent_y(:, i), '--', 'LineWidth', 1.5);
        end
        
        % Plot road
        bounds = world.getRoadBounds();
        plot([bounds(1), bounds(2)], [bounds(3), bounds(3)], 'k-', 'LineWidth', 2);
        plot([bounds(1), bounds(2)], [bounds(4), bounds(4)], 'k-', 'LineWidth', 2);
        
        % Plot static obstacles
        for i = 1:world.n_static_obs
            obs = world.static_obs(i, :);
            x_c = obs(1); y_c = obs(2);
            L = obs(3); W = obs(4);
            if x_c > -50  % Skip dummy obstacles
                x_rect = [x_c - L/2, x_c + L/2, x_c + L/2, x_c - L/2, x_c - L/2];
                y_rect = [y_c - W/2, y_c - W/2, y_c + W/2, y_c + W/2, y_c - W/2];
                plot(x_rect, y_rect, 'r-', 'LineWidth', 2);
                fill(x_rect, y_rect, 'r', 'FaceAlpha', 0.2);
            end
        end
        
        grid on;
        axis equal;
        xlabel('X (m)'); ylabel('Y (m)');
        title('Ego Trajectory');
        legend('Ego path', 'Start', 'End');
        
        % Plot 2: Ego speed over time
        subplot(2, 2, 2);
        plot(history.t, history.ego_v, 'b-', 'LineWidth', 2);
        grid on;
        xlabel('Time (s)'); ylabel('Speed (m/s)');
        title('Ego Speed vs Time');
        
        % Plot 3: Agent speeds
        subplot(2, 2, 3);
        for i = 1:cfg.n_agents
            plot(history.t, history.agent_v(:, i), '--', 'LineWidth', 1.5, ...
                 'DisplayName', sprintf('Agent %d', i));
        end
        grid on;
        xlabel('Time (s)'); ylabel('Speed (m/s)');
        title('Agent Speeds vs Time');
        legend;
        
        % Plot 4: Heading over time
        subplot(2, 2, 4);
        plot(history.t, rad2deg(history.ego_theta), 'g-', 'LineWidth', 2);
        grid on;
        xlabel('Time (s)'); ylabel('Heading (degrees)');
        title('Ego Heading vs Time');
    end
    
    % =====================================================
    % VALIDATION
    % =====================================================
    
    if verbose
        fprintf('\n[VALIDATION] Checking success criteria...\n');
        
        % Criterion 1: Ego moved
        dist_traveled = sqrt((history.ego_x(end) - history.ego_x(1))^2 + ...
                             (history.ego_y(end) - history.ego_y(1))^2);
        if dist_traveled > 0.1
            fprintf('  ✓ Ego vehicle moved (%.2f m)\n', dist_traveled);
        else
            fprintf('  ✗ Ego vehicle did NOT move\n');
        end
        
        % Criterion 2: Speed increased
        if history.ego_v(end) > history.ego_v(1) + 0.5
            fprintf('  ✓ Speed controller working (%.2f → %.2f m/s)\n', ...
                    history.ego_v(1), history.ego_v(end));
        else
            fprintf('  ✗ Speed NOT increased\n');
        end
        
        % Criterion 3: Stayed in bounds
        if world.isEgoInBounds(cfg)
            fprintf('  ✓ Ego stayed within road bounds\n');
        else
            fprintf('  ✗ Ego went out of bounds\n');
        end
        
        % Criterion 4: Simulation completed
        if world.step_count == N_steps
            fprintf('  ✓ Full simulation completed (%d steps)\n', N_steps);
        else
            fprintf('  ⚠ Simulation ended early at step %d/%d\n', ...
                    world.step_count, N_steps);
        end
        
        fprintf('\n[SUCCESS] Stage 0 infrastructure test complete!\n\n');
    end
end
