function [passed, metrics] = stage4_cacrc_safety(varargin)
    % STAGE4_CACRC_SAFETY Stage 4: Context-Adaptive Risk-Sensitive Control + Hard Safety Layer
    %
    % Purpose:
    %   Evaluates Stage 4 integrated prediction, multi-topology corridor optimization
    %   (J_left vs J_right), and multi-layered hard safety filter layer.
    
    p = inputParser;
    addParameter(p, 'scenario', 'moderate', @ischar);
    addParameter(p, 'verbose', true, @islogical);
    addParameter(p, 'perturbed', false, @islogical);
    parse(p, varargin{:});
    
    scenario_name = p.Results.scenario;
    verbose = p.Results.verbose;
    perturbed = p.Results.perturbed;
    
    if verbose
        fprintf('\n╔════════════════════════════════════════════════════════╗\n');
        fprintf('║  STAGE 4: CONTEXT-ADAPTIVE CRC + HARD SAFETY LAYER    ║\n');
        fprintf('║                                                        ║\n');
        fprintf('║  Testing: Multi-Topology (J_left/J_right) + Safety     ║\n');
        fprintf('╚════════════════════════════════════════════════════════╝\n\n');
    end
    
    % Initialize Configuration & Modules
    config = SimulationConfig();
    controller = VehicleController(config);
    planner = CACRCPlanner(config);
    safety_filter = SafetyFilter(config);
    
    v_target = 8.0;
    dt = config.dt;
    N_steps = 100;
    
    % Initialize World State & Vehicle Model
    world = ScenarioDefinitions(scenario_name, config);
    vehicle = BicycleModel(config);
    world.ego.x = 10.0;
    world.ego.v = 5.0;
    
    if perturbed
        world.ego.y = 3.5;
        world.ego.theta = 0.05;
        if verbose, fprintf('[CONFIG] Perturbed Initial State: y0=3.5m, theta0=0.05rad, v0=5.0m/s\n'); end
    else
        world.ego.y = 3.0;
        world.ego.theta = 0.0;
    end
    
    % Reference path (straight centerline y=3.0)
    x_ref = (0:0.5:150)';
    y_ref = 3.0 * ones(size(x_ref));
    theta_ref = zeros(size(x_ref));
    ref_path = [x_ref, y_ref, theta_ref];
    
    % History Arrays
    history = struct();
    history.t = zeros(N_steps, 1);
    history.ego_x = zeros(N_steps, 1); history.ego_y = zeros(N_steps, 1);
    history.ego_v = zeros(N_steps, 1); history.ego_theta = zeros(N_steps, 1);
    history.crosstrack_error = zeros(N_steps, 1); history.speed_error = zeros(N_steps, 1);
    history.min_clearance = zeros(N_steps, 1); history.is_collision = false(N_steps, 1);
    history.inside_bounds = true(N_steps, 1); history.solver_status = zeros(N_steps, 1);
    history.solve_time_ms = zeros(N_steps, 1); history.filter_active = false(N_steps, 1);
    history.selected_topology = cell(N_steps, 1);
    
    planner.reset();
    
    if verbose
        fprintf('[SIMULATION] Starting %d steps (%.1f seconds at dt=%.2f s)\n\n', ...
            N_steps, N_steps * dt, dt);
    end
    
    filter_overrides = 0;
    hard_qp_count = 0;
    soft_qp_count = 0;
    safety_rejected_count = 0;
    emergency_braking_count = 0;
    
    for k = 1:N_steps
        t = (k - 1) * dt;
        
        % Generate Plan
        [u_mpc, pred_states, status, info] = planner.plan(world, ref_path, v_target);
        
        % Apply Safety Filter Layer
        [u_cmd, filter_active, filter_reason] = safety_filter.filter(u_mpc, status, world, pred_states);
        
        % Track 4-State Control Outcome Breakdown
        if status == 1
            if filter_active
                safety_rejected_count = safety_rejected_count + 1;
                filter_overrides = filter_overrides + 1;
            elseif info.is_soft
                soft_qp_count = soft_qp_count + 1;
            else
                hard_qp_count = hard_qp_count + 1;
            end
        else
            emergency_braking_count = emergency_braking_count + 1;
            if filter_active, filter_overrides = filter_overrides + 1; end
        end
        
        % Step Closed-Loop Dynamics
        delta_cmd = u_cmd(1);
        a_cmd = u_cmd(2);
        world.ego = vehicle.stepKinematic(world.ego, a_cmd, delta_cmd, dt);
        
        % Record Diagnostics
        cte = world.ego.y - 3.0;
        clearance = world.getMinClearance(config);
        is_coll = clearance <= 0;
        in_bounds = world.isEgoInBounds(config);
        
        history.t(k) = t;
        history.ego_x(k) = world.ego.x; history.ego_y(k) = world.ego.y;
        history.ego_v(k) = world.ego.v; history.ego_theta(k) = world.ego.theta;
        history.crosstrack_error(k) = cte;
        history.speed_error(k) = abs(world.ego.v - v_target);
        history.min_clearance(k) = clearance;
        history.is_collision(k) = is_coll;
        history.inside_bounds(k) = in_bounds;
        history.solver_status(k) = status;
        history.solve_time_ms(k) = info.solve_time_ms;
        history.filter_active(k) = filter_active;
        history.selected_topology{k} = info.selected_topology;
        
        if ~in_bounds && verbose
            bounds = world.getRoadBounds();
            half_L = config.vehicle_length / 2 + config.safety_margin;
            half_W = config.vehicle_width / 2 + config.safety_margin;
            r_y = abs(half_L * sin(world.ego.theta)) + abs(half_W * cos(world.ego.theta));
            fprintf('[OUT_OF_BOUNDS STEP %3d] y=%.3f | th=%.3f rad | r_y=%.3f | y-r_y=%.3f (min=%.2f) | y+r_y=%.3f (max=%.2f)\n', ...
                k, world.ego.y, world.ego.theta, r_y, world.ego.y - r_y, bounds(3), world.ego.y + r_y, bounds(4));
        end
    end
    
    % Compute Metrics
    dist_traveled = world.ego.x - 10.0;
    final_v = world.ego.v;
    mean_ey = mean(abs(history.crosstrack_error));
    max_ey = max(abs(history.crosstrack_error));
    mean_ev = mean(history.speed_error(50:end));
    collision_steps = sum(history.is_collision);
    bounds_steps = sum(history.inside_bounds);
    feasibility_rate = (sum(history.solver_status == 1) / N_steps) * 100;
    mean_solve_time = mean(history.solve_time_ms);
    min_clr = min(history.min_clearance);
    
    % 4 Explicit Status Criteria
    longitudinal_passed = world.ego.x > 40.0;
    collision_free_run = collision_steps == 0;
    road_compliant_run = bounds_steps == N_steps;
    safe_obstacle_passage = longitudinal_passed && collision_free_run && road_compliant_run;
    controlled_stop_executed = final_v < 0.1 && collision_free_run;
    
    metrics = struct();
    metrics.dist_traveled = dist_traveled;
    metrics.final_v = final_v;
    metrics.mean_ey = mean_ey; metrics.max_ey = max_ey;
    metrics.mean_ev = mean_ev; metrics.collision_steps = collision_steps;
    metrics.bounds_steps = bounds_steps; metrics.feasibility_rate = feasibility_rate;
    metrics.mean_solve_time = mean_solve_time; metrics.min_clr = min_clr;
    metrics.filter_overrides = filter_overrides;
    metrics.hard_qp_count = hard_qp_count;
    metrics.soft_qp_count = soft_qp_count;
    metrics.safety_rejected_count = safety_rejected_count;
    metrics.emergency_braking_count = emergency_braking_count;
    metrics.longitudinal_passed = longitudinal_passed;
    metrics.collision_free_run = collision_free_run;
    metrics.safe_obstacle_passage = safe_obstacle_passage;
    metrics.controlled_stop_executed = controlled_stop_executed;
    
    % Pass Criteria
    pass_ey = mean_ey < 0.50;
    pass_ev = mean_ev < 0.50;
    pass_bounds = bounds_steps == N_steps;
    pass_collision = collision_steps == 0;
    pass_feasibility = feasibility_rate > 90.0;
    
    if strcmp(scenario_name, 'passable_marginal')
        passed = pass_collision && longitudinal_passed;
    elseif strcmp(scenario_name, 'impassable_center')
        passed = pass_collision && controlled_stop_executed;
    elseif strcmp(scenario_name, 'multi_obstacle_sequence')
        passed = pass_collision && pass_bounds && longitudinal_passed;
    else
        passed = pass_ey && pass_ev && pass_bounds && pass_collision && pass_feasibility;
    end
    
    % Scenario-Specific Scientific Assertion Structure
    switch scenario_name
        case 'passable_moderate'
            assert(feasibility_rate >= 90.0, 'Audit Fail (passable_moderate): Feasibility rate < 90%%');
            assert(collision_steps == 0, 'Audit Fail (passable_moderate): Collision steps != 0');
            assert(bounds_steps == N_steps, 'Audit Fail (passable_moderate): Bounds compliance != 100%%');
            assert(final_v > 7.0, 'Audit Fail (passable_moderate): Final speed < 7.0 m/s');
            assert(longitudinal_passed, 'Audit Fail (passable_moderate): Obstacle not passed');
            
        case 'passable_marginal'
            assert(collision_steps == 0, 'Audit Fail (passable_marginal): Collision steps != 0');
            assert(longitudinal_passed, 'Audit Fail (passable_marginal): Obstacle not passed');
            
        case 'impassable_center'
            assert(collision_steps == 0, 'Audit Fail (impassable_center): Collision steps != 0');
            assert(bounds_steps == N_steps, 'Audit Fail (impassable_center): Bounds compliance != 100%%');
            assert(controlled_stop_executed, 'Audit Fail (impassable_center): Controlled stop not executed');
            assert(~longitudinal_passed, 'Audit Fail (impassable_center): Obstacle passed when impassable');
            
        case 'multi_obstacle_sequence'
            assert(collision_steps == 0, 'Audit Fail (multi_obstacle_sequence): Collision steps != 0');
            assert(longitudinal_passed, 'Audit Fail (multi_obstacle_sequence): Obstacles not passed');
    end
    
    if verbose
        fprintf('\n[SIMULATION] Complete!\n');
        fprintf('=== STAGE 4 PERFORMANCE RESULTS ===\n');
        fprintf('  Distance Traveled:          %.2f m\n', world.ego.x - 10.0);
        fprintf('  Final Speed:                %.2f m/s (Target: %.2f m/s)\n', world.ego.v, v_target);
        fprintf('  Mean Cross-Track Error:     %.3f m\n', mean_ey);
        fprintf('  Max Cross-Track Error:      %.3f m\n', max_ey);
        fprintf('  Mean Steady Speed Error:    %.3f m/s\n', mean_ev);
        fprintf('  QP Feasibility Rate:        %.1f %%\n', feasibility_rate);
        fprintf('  Planner Runtime:            %.2f ms/step\n', mean_solve_time);
        fprintf('  Safety Filter Overrides:    %d steps\n', filter_overrides);
        fprintf('  Min Obstacle Clearance:     %.2f m\n', min_clr);
        fprintf('  Collision-Active Steps:     %d\n', collision_steps);
        fprintf('  Road Bounds Maintained:     %s (%d/%d steps)\n', ...
            mat2str(pass_bounds), bounds_steps, N_steps);
        
        fprintf('\n[VALIDATION] Checking Stage 4 criteria...\n');
        fprintf('  %s Path tracking error (mean e_y = %.3f m < 0.5 m)\n', check_mark(pass_ey), mean_ey);
        fprintf('  %s Speed tracking error (mean e_v = %.3f m/s < 0.5 m/s)\n', check_mark(pass_ev), mean_ev);
        fprintf('  %s Vehicle inside road bounds (100/100 steps)\n', check_mark(pass_bounds));
        fprintf('  %s No collision-active steps (0 steps)\n', check_mark(pass_collision));
        fprintf('  %s Feasibility rate (> 90.0 %%)\n', check_mark(pass_feasibility));
        
        if passed
            fprintf('\n[SUCCESS] Stage 4 CA-CRC + Hard Safety Layer test PASSED ALL CRITERIA!\n\n');
        else
            fprintf('\n[FAIL] Stage 4 failed one or more criteria.\n\n');
        end
    end
end

function str = check_mark(val)
    if val, str = '✓'; else, str = '✗'; end
end
