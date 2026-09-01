function run_controlled_ab_experiment()
    % RUN_CONTROLLED_AB_EXPERIMENT Controlled A/B Baseline vs Free-Space Comparison
    %
    % Purpose:
    %   Evaluates Baseline (Fixed-Lane / CorridorBoundProvider) vs Proposed
    %   (Perception-Derived FreeSpaceBoundProvider) on the Indian Unstructured Road Scenario.
    %
    % Requirements:
    %   - Baseline is NOT artificially tuned to fail or succeed.
    %   - Free-Space navigation derives lateral bounds directly from perception.
    
    addpath('planning', 'config', 'core', 'environment', 'stages', 'vehicle');
    
    fprintf('\n========================================================================================\n');
    fprintf('   CONTROLLED A/B EXPERIMENT: BASELINE (LANE-BASED) VS PROPOSED (FREE-SPACE)            \n');
    fprintf('   Scenario: scenario_indian_unstructured                                               \n');
    fprintf('========================================================================================\n\n');
    
    cfg = SimulationConfig();
    map_obj = FreeSpaceMap.createIndianUnstructuredMap();
    
    % -------------------------------------------------------------------------
    % 1. BASELINE MODEL: Fixed-Lane Corridor Controller
    % -------------------------------------------------------------------------
    fprintf('  Running BASELINE (Fixed-Lane / CorridorBoundProvider)...\n');
    ctrl_baseline = Stage5CoordinationController(cfg);
    ctrl_baseline.reset();
    ctrl_baseline.cacrc_planner.bound_provider = CorridorBoundProvider();
    ctrl_baseline.setOvertakeEnabled(true);
    [res_base, hist_base] = run_experiment_sim(cfg, ctrl_baseline, map_obj, 'baseline');
    
    % -------------------------------------------------------------------------
    % 2. PROPOSED MODEL: Perception-Derived Free-Space Controller
    % -------------------------------------------------------------------------
    fprintf('  Running PROPOSED (Perception-Derived FreeSpaceBoundProvider)...\n');
    ctrl_proposed = Stage5CoordinationController(cfg);
    ctrl_proposed.reset();
    ctrl_proposed.cacrc_planner.bound_provider = FreeSpaceBoundProvider(map_obj);
    ctrl_proposed.setOvertakeEnabled(true);
    [res_prop, hist_prop] = run_experiment_sim(cfg, ctrl_proposed, map_obj, 'proposed');
    
    % -------------------------------------------------------------------------
    % 3. SIDE-BY-SIDE METRIC COMPARISON TABLE
    % -------------------------------------------------------------------------
    fprintf('\n========================================================================================\n');
    fprintf('                        CONTROLLED A/B EXPERIMENTAL RESULTS                             \n');
    fprintf('========================================================================================\n');
    fprintf('  %-35s | %-18s | %-18s\n', 'Metric', 'Baseline (Fixed Lane)', 'Proposed (Free Space)');
    fprintf('  ------------------------------------+--------------------+--------------------\n');
    fprintf('  %-35s | %-18s | %-18s\n', 'Completion Status', res_base.status_str, res_prop.status_str);
    fprintf('  %-35s | %-18d | %-18d\n', 'Total Steps Evaluated', res_base.total_steps, res_prop.total_steps);
    fprintf('  %-35s | %d / %d (%.1f%%)   | %d / %d (%.1f%%)\n', 'Drivable-Space Compliance (Steps)', ...
        res_base.bounds_steps, res_base.total_steps, (res_base.bounds_steps/res_base.total_steps)*100, ...
        res_prop.bounds_steps, res_prop.total_steps, (res_prop.bounds_steps/res_prop.total_steps)*100);
    fprintf('  %-35s | %-18d | %-18d\n', 'Road Boundary Violation Steps', res_base.boundary_violations, res_prop.boundary_violations);
    fprintf('  %-35s | %-18d | %-18d\n', 'Collision Active Steps', res_base.collision_steps, res_prop.collision_steps);
    fprintf('  %-35s | %-+18.4f m | %-+18.4f m\n', 'Minimum Clearance to Obstacles', res_base.min_clr, res_prop.min_clr);
    fprintf('  %-35s | %-+18.4f m | %-+18.4f m\n', 'Minimum Road-Boundary Clearance', res_base.min_road_clr, res_prop.min_road_clr);
    fprintf('  %-35s | %-18d | %-18d\n', 'QP Infeasibility / Interventions', res_base.emergency_count, res_prop.emergency_count);
    fprintf('  %-35s | %-18.2f m | %-18.2f m\n', 'Total Trajectory Length', res_base.traj_length, res_prop.traj_length);
    fprintf('  %-35s | %-18.2f m/s | %-18.2f m/s\n', 'Average Travel Velocity', res_base.avg_v, res_prop.avg_v);
    fprintf('========================================================================================\n\n');
    
    % Evaluation Summary
    if res_prop.collision_steps == 0 && res_prop.boundary_violations == 0 && res_prop.min_clr > 0
        if res_base.collision_steps > 0 || res_base.boundary_violations > 0 || res_base.emergency_count > 0
            fprintf('  => HYPOTHESIS CONFIRMED: Baseline fails due to fixed-lane rigidity, while Proposed Free-Space successfully navigates the corridor!\n');
        else
            fprintf('  => HYPOTHESIS CAPABILITY COMPARISON: Both systems completed the scenario.\n');
        end
    else
        fprintf('  => SUMMARY: PROPOSED MODEL REQUIRES FURTHER ATTENTION.\n');
    end
    fprintf('========================================================================================\n\n');
end

function [res, history] = run_experiment_sim(cfg, ctrl, map_obj, mode)
    gt_world = ScenarioDefinitions('indian_unstructured', cfg);
    vehicle = BicycleModel(cfg);
    obs_model = ObservationModel('ideal', 42);
    
    N_steps = 350; dt = cfg.dt;
    
    N_path = 700;
    ref_path = zeros(N_path, 5);
    ref_path(:, 1) = linspace(0, 150, N_path)';
    if strcmp(mode, 'baseline')
        % Baseline expects fixed lane centerline y = 1.80m
        ref_path(:, 2) = 1.80;
    else
        % Proposed uses dynamic center of road boundaries
        for r = 1:N_path
            [y_lo, y_hi] = map_obj.getRoadBoundsAt(ref_path(r, 1));
            ref_path(r, 2) = 0.5 * (y_lo + y_hi);
        end
    end
    ref_path(:, 4) = 0.0; ref_path(:, 5) = 5.0;
    
    history.ego_x = zeros(N_steps, 1);
    history.ego_y = zeros(N_steps, 1);
    history.ego_v = zeros(N_steps, 1);
    
    clr_list = zeros(N_steps, 1);
    road_clr_list = zeros(N_steps, 1);
    coll_list = false(N_steps, 1);
    bounds_list = false(N_steps, 1);
    emergency_count = 0;
    traj_dist = 0.0;
    half_W = cfg.vehicle_width / 2.0; % 0.90m
    
    for k = 1:N_steps
        [obs_world, ~] = obs_model.observe(gt_world, dt);
        [u_cmd, ~, status, ~] = ctrl.step(obs_world, ref_path, 5.0);
        if status == 0
            emergency_count = emergency_count + 1;
        end
        
        prev_x = gt_world.ego.x; prev_y = gt_world.ego.y;
        gt_world.ego = vehicle.stepKinematic(gt_world.ego, u_cmd(2), u_cmd(1), dt);
        
        history.ego_x(k) = gt_world.ego.x;
        history.ego_y(k) = gt_world.ego.y;
        history.ego_v(k) = gt_world.ego.v;
        
        if k > 1
            traj_dist = traj_dist + sqrt((history.ego_x(k) - prev_x)^2 + (history.ego_y(k) - prev_y)^2);
        end
        
        bounds_list(k) = map_obj.isDrivable(gt_world.ego.x, gt_world.ego.y);
        clr_list(k) = gt_world.getMinClearance(cfg);
        coll_list(k) = clr_list(k) <= 0;
        
        [y_road_lo, y_road_hi] = map_obj.getRoadBoundsAt(gt_world.ego.x);
        dist_to_lo = (gt_world.ego.y - half_W) - y_road_lo;
        dist_to_hi = y_road_hi - (gt_world.ego.y + half_W);
        road_clr_list(k) = min(dist_to_lo, dist_to_hi);
        
        gt_world = gt_world.stepAgents(dt);
    end
    
    res.total_steps = N_steps;
    res.bounds_steps = sum(bounds_list);
    res.boundary_violations = N_steps - sum(bounds_list);
    res.collision_steps = sum(coll_list);
    res.min_clr = min(clr_list);
    res.min_road_clr = min(road_clr_list);
    res.emergency_count = emergency_count;
    res.avg_v = mean(history.ego_v);
    res.traj_length = traj_dist;
    
    if res.collision_steps > 0
        res.status_str = 'COLLISION';
    elseif res.boundary_violations > 0
        res.status_str = 'OUT_OF_BOUNDS';
    else
        res.status_str = 'COMPLETED';
    end
end
