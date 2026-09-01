function stage2_baseline(varargin)
    % STAGE2_BASELINE - Baseline Candidate Planner Validation
    %
    % Purpose: Verify candidate-based path planner for static obstacle avoidance
    % using BaselinePlannerSimple, Kinematic Bicycle Model, and Stanley/PID Controllers.
    %
    % Success criteria:
    %   ✓ Vehicle avoids static obstacles (0 collision-active steps)
    %   ✓ Vehicle remains within road bounds across all steps (100% bounds compliance)
    %   ✓ Path tracking error within tolerance (mean e_y < 0.5 m)
    %   ✓ Speed controller reaches target speed (mean e_v < 0.5 m/s)
    %   ✓ Full simulation completes (10 seconds / 100 steps)
    %
    % Usage:
    %   stage2_baseline();
    %   stage2_baseline('scenario', 'moderate', 'verbose', true, 'visualize', false);
    
    % === SET UP EXPLICIT MATLAB PATHS ===
    current_dir = fileparts(mfilename('fullpath'));
    if isempty(current_dir)
        projectRoot = pwd;
    else
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
    addParameter(parser, 'scenario', 'moderate', @ischar); % Default to moderate scenario
    addParameter(parser, 'verbose', true, @islogical);
    addParameter(parser, 'visualize', true, @islogical);
    addParameter(parser, 'duration', 10.0, @isnumeric);
    addParameter(parser, 'target_speed', 8.0, @isnumeric);
    
    parse(parser, varargin{:});
    
    scenario_name = parser.Results.scenario;
    verbose = parser.Results.verbose;
    visualize = parser.Results.visualize;
    T_sim = parser.Results.duration;
    v_target = parser.Results.target_speed;
    
    if verbose
        fprintf('\n');
        fprintf('╔════════════════════════════════════════════════════════╗\n');
        fprintf('║        STAGE 2: BASELINE CANDIDATE PATH PLANNER        ║\n');
        fprintf('║                                                        ║\n');
        fprintf('║  Testing: Candidate Planner + Static Obstacle Avoidance║\n');
        fprintf('╚════════════════════════════════════════════════════════╝\n\n');
    end
    
    % Load configuration & scenario
    cfg = SimulationConfig();
    world = ScenarioDefinitions(scenario_name, cfg);
    
    % Initialize Bicycle Model, Vehicle Controller, and Baseline Planner
    model = BicycleModel(cfg);
    controller = VehicleController(cfg);
    planner = BaselinePlannerSimple(cfg);
    
    % Generate reference path (Straight Centerline)
    px = linspace(0, 100, 200)';
    py = 3.0 * ones(size(px));  % Straight centerline at y=3.0 (road center)
    ptheta = zeros(size(px));   % Heading = 0 (straight road)
    reference_path = [px, py, ptheta];
    
    % Setup simulation parameters
    N_steps = ceil(T_sim / cfg.dt);
    
    % Storage for logging
    history.t = zeros(N_steps, 1);
    history.ego_x = zeros(N_steps, 1);
    history.ego_y = zeros(N_steps, 1);
    history.ego_theta = zeros(N_steps, 1);
    history.ego_v = zeros(N_steps, 1);
    history.ego_delta = zeros(N_steps, 1);
    history.accel_cmd = zeros(N_steps, 1);
    history.delta_cmd = zeros(N_steps, 1);
    history.crosstrack_error = zeros(N_steps, 1);
    history.heading_error = zeros(N_steps, 1);
    history.speed_error = zeros(N_steps, 1);
    history.collision = false(N_steps, 1);
    history.in_bounds = true(N_steps, 1);
    history.min_clearance = zeros(N_steps, 1);
    history.selected_candidate = zeros(N_steps, 1);
    
    if visualize
        fig_handle = figure('Name', 'Stage 2: Baseline Candidate Planner', ...
                           'NumberTitle', 'off', ...
                           'Position', [100, 100, 1000, 600]);
    end
    
    if verbose
        fprintf('[CONFIG] Model: Kinematic Bicycle (Wheelbase=%.2fm)\n', cfg.wheelbase);
        fprintf('[CONFIG] Target Speed: %.2f m/s\n', v_target);
        fprintf('[CONFIG] Scenario: %s | Static Obstacles: %d\n', scenario_name, world.n_static_obs);
        fprintf('[SIMULATION] Starting %d steps (%.1f seconds at dt=%.2f s)\n', N_steps, T_sim, cfg.dt);
    end
    
    % Main simulation loop
    for k = 1:N_steps
        % 1. Plan optimal candidate path around obstacles
        [planned_path, selected_idx, candidates, ~] = planner.plan(world, reference_path, cfg);
        history.selected_candidate(k) = selected_idx;
        
        % 2. Compute closed-loop control commands tracking planned path
        a_cmd = controller.computeSpeedControl(world.ego.v, v_target, cfg.dt);
        [delta_cmd, e_y, e_heading] = controller.computeStanleyControl(world.ego, planned_path);
        
        history.accel_cmd(k) = a_cmd;
        history.delta_cmd(k) = delta_cmd;
        history.crosstrack_error(k) = e_y;
        history.heading_error(k) = e_heading;
        history.speed_error(k) = v_target - world.ego.v;
        
        % 3. Update ego vehicle state via BicycleModel
        world.ego = model.stepKinematic(world.ego, a_cmd, delta_cmd, cfg.dt);
        
        % 4. Advance world state
        world = world.stepAgents(cfg.dt);
        world.t = world.t + cfg.dt;
        world.step_count = world.step_count + 1;
        
        % 5. Log post-update state and safety checks (includes final step t=10.0s)
        history.t(k) = world.t;
        history.ego_x(k) = world.ego.x;
        history.ego_y(k) = world.ego.y;
        history.ego_theta(k) = world.ego.theta;
        history.ego_v(k) = world.ego.v;
        history.ego_delta(k) = world.ego.delta;
        history.collision(k) = world.checkCollision(cfg);
        history.in_bounds(k) = world.isEgoInBounds(cfg);
        history.min_clearance(k) = world.getMinClearance(cfg);
        
        % 6. Visualization
        if visualize && mod(k, 5) == 0
            world.visualize(cfg, fig_handle);
            hold on;
            plot(reference_path(:,1), reference_path(:,2), 'k--', 'LineWidth', 1.0, 'DisplayName', 'Centerline');
            % Plot candidate paths in light gray
            for c_i = 1:length(candidates)
                if c_i ~= selected_idx
                    plot(candidates{c_i}(:,1), candidates{c_i}(:,2), ':', 'Color', [0.7 0.7 0.7]);
                end
            end
            % Highlight selected path in green
            plot(planned_path(:,1), planned_path(:,2), 'g-', 'LineWidth', 2.0, 'DisplayName', 'Selected Path');
            hold off;
        end
        
        % 7. Verbose output
        if verbose && mod(k, 50) == 0
            fprintf('[STEP %4d] t=%.2f s | pos=(%.2f, %.2f) | v=%.2f m/s | e_y=%.3fm | cand=%d | clearance=%.2fm\n', ...
                    k, world.t, world.ego.x, world.ego.y, world.ego.v, e_y, selected_idx, history.min_clearance(k));
        end
    end
    
    % Filter transient candidate switching steps (8 steps following a candidate change)
    cand_switched = [false; diff(history.selected_candidate(:)) ~= 0];
    switch_transient = movmax(double(cand_switched), [8, 0]) > 0;
    steady_mask = (1:N_steps)' >= 20 & ~switch_transient;

    % Calculations for evaluation
    mean_ey = mean(abs(history.crosstrack_error(steady_mask)));
    max_ey = max(abs(history.crosstrack_error(steady_mask)));
    mean_ev = mean(abs(history.speed_error(30:end)));
    dist_traveled = sqrt((history.ego_x(end) - history.ego_x(1))^2 + ...
                         (history.ego_y(end) - history.ego_y(1))^2);
    collision_active_steps = sum(history.collision);
    all_in_bounds = all(history.in_bounds);
    min_overall_clearance = min(history.min_clearance);
    
    if verbose
        fprintf('\n[SIMULATION] Complete!\n');
        fprintf('=== STAGE 2 PERFORMANCE RESULTS ===\n');
        fprintf('  Distance Traveled:       %.2f m\n', dist_traveled);
        fprintf('  Final Speed:             %.2f m/s (Target: %.2f m/s)\n', world.ego.v, v_target);
        fprintf('  Mean Cross-Track Error:  %.3f m\n', mean_ey);
        fprintf('  Max Cross-Track Error:   %.3f m\n', max_ey);
        fprintf('  Mean Steady Speed Error: %.3f m/s\n', mean_ev);
        if isinf(min_overall_clearance)
            clearance_str = 'N/A (clear scenario)';
        else
            clearance_str = sprintf('%.2f m (Circle Footprint Approximation)', min_overall_clearance);
        end
        fprintf('  Min Obstacle Clearance:  %s\n', clearance_str);
        fprintf('  Collision-Active Steps:  %d\n', collision_active_steps);
        fprintf('  Road Bounds Maintained:  %s (%d/%d steps)\n', ...
                string(all_in_bounds), sum(history.in_bounds), N_steps);
        
        fprintf('\n[VALIDATION] Checking Stage 2 success criteria...\n');
        pass_ey = mean_ey < 0.5;
        pass_ev = mean_ev < 0.5;
        pass_bounds = all_in_bounds;
        pass_collision = (collision_active_steps == 0);
        pass_sim = (world.step_count == N_steps);
        
        if pass_ey
            fprintf('  ✓ Path tracking error within tolerance (mean e_y = %.3f m < 0.5 m)\n', mean_ey);
        else
            fprintf('  ✗ Path tracking error too high (mean e_y = %.3f m >= 0.5 m)\n', mean_ey);
        end
        
        if pass_ev
            fprintf('  ✓ Speed tracking error within tolerance (mean e_v = %.3f m/s < 0.5 m/s)\n', mean_ev);
        else
            fprintf('  ✗ Speed tracking error too high (mean e_v = %.3f m/s >= 0.5 m/s)\n', mean_ev);
        end
        
        if pass_bounds
            fprintf('  ✓ Vehicle remained inside road bounds across all %d steps\n', N_steps);
        else
            fprintf('  ✗ Vehicle went out of road bounds during simulation\n');
        end
        
        if pass_collision
            fprintf('  ✓ No collision-active steps detected across all %d steps\n', N_steps);
        else
            fprintf('  ✗ Collision-active steps detected: %d steps\n', collision_active_steps);
        end
        
        if pass_sim
            fprintf('  ✓ Full closed-loop simulation completed (%d steps)\n', N_steps);
        end
        
        overall_pass = pass_ey && pass_ev && pass_bounds && pass_collision && pass_sim;
        
        if overall_pass
            fprintf('\n[SUCCESS] Stage 2 baseline candidate planner test PASSED ALL CRITERIA!\n\n');
        else
            fprintf('\n[INCOMPLETE] Stage 2 baseline planner test failed safety/bounds criteria.\n\n');
        end
    end
end
