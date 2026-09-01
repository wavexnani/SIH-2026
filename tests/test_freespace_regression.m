function test_freespace_regression()
    % TEST_FREESPACE_REGRESSION Execution of Regression Test A
    %
    % Purpose:
    %   Verifies that Stage 5.1 using FreeSpaceBoundProvider with an exact corridor map
    %   reproduces IDENTICAL planning behavior and metrics as the original CorridorBoundProvider.
    %
    % Criteria:
    %   - 0 metric discrepancy across all 3 nominal scenarios.
    %   - Discrepancy > 1e-4 fails the regression gate.
    
    addpath('planning', 'config', 'vehicle', 'core', 'environment', 'stages');
    
    fprintf('\n========================================================================================\n');
    fprintf('        REGRESSION TEST A: FREESPACE BOUND PROVIDER VS CORRIDOR BOUND PROVIDER          \n');
    fprintf('========================================================================================\n\n');
    
    scenarios = {'multi_vehicle_following', 'multi_vehicle_yield_overtake', 'multi_vehicle_oncoming_conflict'};
    all_passed = true;
    
    for s_idx = 1:length(scenarios)
        scen_name = scenarios{s_idx};
        fprintf('----------------------------------------------------------------------------------------\n');
        fprintf('  Evaluating Scenario: %s\n', scen_name);
        fprintf('----------------------------------------------------------------------------------------\n');
        
        % 1. Run with Baseline CorridorBoundProvider
        ctrl_corridor = Stage5CoordinationController(SimulationConfig());
        ctrl_corridor.cacrc_planner.bound_provider = CorridorBoundProvider();
        [res_corr, history_corr] = run_scenario_with_controller(scen_name, ctrl_corridor);
        
        % 2. Run with FreeSpaceBoundProvider representing exact same corridor (0.0m to 6.0m)
        ctrl_freespace = Stage5CoordinationController(SimulationConfig());
        corridor_map = FreeSpaceMap.createCorridorMap(0.0, 6.0);
        ctrl_freespace.cacrc_planner.bound_provider = FreeSpaceBoundProvider(corridor_map);
        [res_free, history_free] = run_scenario_with_controller(scen_name, ctrl_freespace);
        
        % 3. Calculate Metric Discrepancies
        diff_bounds = abs(res_corr.bounds_steps - res_free.bounds_steps);
        diff_collisions = abs(res_corr.collision_steps - res_free.collision_steps);
        diff_clr = abs(res_corr.min_clr - res_free.min_clr);
        diff_emergency = abs(res_corr.emergency_braking_count - res_free.emergency_braking_count);
        max_y_diff = max(abs(history_corr.ego_y - history_free.ego_y));
        max_v_diff = max(abs(history_corr.ego_v - history_free.ego_v));
        
        fprintf('  [METRIC COMPARISON]\n');
        fprintf('    Road Bounds Compliance (Steps) : Corridor=%d | FreeSpace=%d | Diff=%d\n', ...
            res_corr.bounds_steps, res_free.bounds_steps, diff_bounds);
        fprintf('    Collision Active Steps          : Corridor=%d | FreeSpace=%d | Diff=%d\n', ...
            res_corr.collision_steps, res_free.collision_steps, diff_collisions);
        fprintf('    Minimum Clearance (m)           : Corridor=%.4f | FreeSpace=%.4f | Diff=%.6f\n', ...
            res_corr.min_clr, res_free.min_clr, diff_clr);
        fprintf('    Emergency Braking Count         : Corridor=%d | FreeSpace=%d | Diff=%d\n', ...
            res_corr.emergency_braking_count, res_free.emergency_braking_count, diff_emergency);
        fprintf('    Max Trajectory Position Diff    : %.6f m\n', max_y_diff);
        fprintf('    Max Speed Profile Diff         : %.6f m/s\n', max_v_diff);
        
        scen_pass = (diff_bounds == 0) && (diff_collisions == 0) && ...
                    (diff_clr < 1e-4) && (diff_emergency == 0) && (max_y_diff < 1e-4);
        
        if scen_pass
            fprintf('  => RESULT: PASS (Identical behavior within 1e-4 tolerance)\n\n');
        else
            fprintf('  => RESULT: FAIL (Metric discrepancy exceeds tolerance)\n\n');
            all_passed = false;
        end
    end
    
    fprintf('========================================================================================\n');
    if all_passed
        fprintf('  OVERALL REGRESSION TEST A: PASS (FreeSpaceBoundProvider validated on corridor!)\n');
    else
        fprintf('  OVERALL REGRESSION TEST A: FAIL (Stopping pipeline as mandated by protocol)\n');
        error('Regression Test A failed: Discrepancy detected between Corridor and FreeSpace providers!');
    end
    fprintf('========================================================================================\n\n');
end

function [res, history] = run_scenario_with_controller(scenario_name, ctrl)
    cfg = SimulationConfig();
    world = ScenarioDefinitions(scenario_name, cfg);
    vehicle = BicycleModel(cfg);
    
    ctrl.reset();
    ctrl.setOvertakeEnabled(~strcmp(scenario_name, 'multi_vehicle_following'));
    
    dt = cfg.dt; N_steps = 150;
    world.ego.v = 5.0;
    
    ref_path = zeros(500, 5);
    ref_path(:, 1) = linspace(0, 150, 500)';
    ref_path(:, 2) = world.ego.y;
    ref_path(:, 4) = 0.0;
    ref_path(:, 5) = 8.0;
    
    history.ego_x = zeros(N_steps, 1);
    history.ego_y = zeros(N_steps, 1);
    history.ego_v = zeros(N_steps, 1);
    
    clr_list = zeros(N_steps, 1);
    coll_list = false(N_steps, 1);
    bounds_list = false(N_steps, 1);
    emergency_count = 0;
    
    for k = 1:N_steps
        [u_cmd, ~, status, info] = ctrl.step(world, ref_path, 8.0);
        if status == 0
            emergency_count = emergency_count + 1;
        end
        
        world.ego = vehicle.stepKinematic(world.ego, u_cmd(2), u_cmd(1), dt);
        
        history.ego_x(k) = world.ego.x;
        history.ego_y(k) = world.ego.y;
        history.ego_v(k) = world.ego.v;
        
        clr_list(k) = world.getMinClearance(cfg);
        coll_list(k) = clr_list(k) <= 0;
        bounds_list(k) = world.isEgoInBounds(cfg);
        
        % Step dynamic agents forward
        world = world.stepAgents(dt);
    end
    
    res.bounds_steps = sum(bounds_list);
    res.collision_steps = sum(coll_list);
    res.min_clr = min(clr_list);
    res.emergency_braking_count = emergency_count;
end
