function run_bottleneck_ab_experiment()
    % RUN_BOTTLENECK_AB_EXPERIMENT Experiment A: Isolated Static Bottleneck Free-Space Test
    %
    % Purpose:
    %   Evaluates Baseline (Fixed-Lane / CorridorBoundProvider) vs Proposed
    %   (Perception-Derived FreeSpaceBoundProvider) on the isolated static
    %   unstructured-road bottleneck scenario (WITHOUT distant cattle confound).
    %
    % Scientific Hypothesis:
    %   Fixed-lane architectures fail at unstructured bottlenecks due to rigid
    %   centerline assumptions and boundary breaches, whereas perception-derived
    %   free-space bounds enable safe, continuous bottleneck traversal.

    addpath('planning', 'config', 'core', 'environment', 'stages', 'vehicle', 'tests');
    
    fprintf('\n========================================================================================\n');
    fprintf('   EXPERIMENT A: ISOLATED STATIC BOTTLENECK FREE-SPACE A/B BENCHMARK                   \n');
    fprintf('   Scenario: scenario_indian_unstructured_bottleneck                                   \n');
    fprintf('========================================================================================\n\n');
    
    cfg = SimulationConfig();
    map_obj = FreeSpaceMap.createIndianUnstructuredMap();
    
    % 1. PROPOSED MODEL (Perception-Derived FreeSpaceBoundProvider)
    fprintf('  Running PROPOSED (Perception-Derived FreeSpaceBoundProvider)...\n');
    planner_prop = CACRCPlanner(cfg);
    bp_prop = FreeSpaceBoundProvider(map_obj);
    planner_prop.bound_provider = bp_prop;
    sf_prop = SafetyFilter(cfg);
    
    gt_world_prop = ScenarioDefinitions('indian_unstructured_bottleneck', cfg);
    gt_world_prop.n_static_obs = 0; % Static geometry represented via FreeSpaceMap
    
    vehicle = BicycleModel(cfg);
    obs_model = ObservationModel('ideal', 42);
    
    N_steps = 350; dt = cfg.dt;
    
    N_path = 700;
    ref_path_prop = zeros(N_path, 5);
    ref_path_prop(:, 1) = linspace(0, 150, N_path)';
    ref_path_prop(:, 2) = 2.75; % Free-space corridor center
    ref_path_prop(:, 4) = 0.0; ref_path_prop(:, 5) = 5.0;
    
    [res_prop, trace_prop] = run_bottleneck_sim(cfg, planner_prop, bp_prop, sf_prop, gt_world_prop, ref_path_prop, vehicle, obs_model, map_obj, N_steps, dt);
    
    % 2. BASELINE MODEL (Fixed-Lane / CorridorBoundProvider)
    fprintf('  Running BASELINE (Fixed-Lane / CorridorBoundProvider)...\n');
    planner_base = CACRCPlanner(cfg);
    bp_base = CorridorBoundProvider();
    planner_base.bound_provider = bp_base;
    sf_base = SafetyFilter(cfg);
    
    gt_world_base = ScenarioDefinitions('indian_unstructured_bottleneck', cfg);
    
    ref_path_base = ref_path_prop;
    ref_path_base(:, 2) = 1.80; % Fixed right-lane centerline
    
    [res_base, trace_base] = run_bottleneck_sim(cfg, planner_base, bp_base, sf_base, gt_world_base, ref_path_base, vehicle, obs_model, map_obj, N_steps, dt);
    
    % 3. SIDE-BY-SIDE METRIC COMPARISON TABLE
    fprintf('\n========================================================================================\n');
    fprintf('              EXPERIMENT A: BOTTLENECK A/B RESULTS (CONFOUND ISOLATED)                  \n');
    fprintf('========================================================================================\n');
    fprintf('  %-38s | %-18s | %-18s\n', 'Metric', 'Baseline (Fixed Lane)', 'Proposed (Free Space)');
    fprintf('  ----------------------------------------+--------------------+--------------------\n');
    fprintf('  %-38s | %-18s | %-18s\n', 'Completion Status', res_base.status_str, res_prop.status_str);
    fprintf('  %-38s | %-18d | %-18d\n', 'Total Steps Evaluated', res_base.total_steps, res_prop.total_steps);
    fprintf('  %-38s | %-18.2f m | %-18.2f m\n', 'Maximum x Position Reached', res_base.max_x, res_prop.max_x);
    fprintf('  %-38s | %d / %d (%.1f%%)   | %d / %d (%.1f%%)\n', 'Drivable-Space Compliance (Steps)', ...
        res_base.bounds_steps, res_base.total_steps, (res_base.bounds_steps/res_base.total_steps)*100, ...
        res_prop.bounds_steps, res_prop.total_steps, (res_prop.bounds_steps/res_prop.total_steps)*100);
    fprintf('  %-38s | %-18d | %-18d\n', 'Footprint Boundary Violations', res_base.boundary_violations, res_prop.boundary_violations);
    fprintf('  %-38s | %-18d | %-18d\n', 'Collision Active Steps (OBB SAT)', res_base.collision_steps, res_prop.collision_steps);
    fprintf('  %-38s | %-+18.4f m | %-+18.4f m\n', 'Minimum Obstacle Clearance', res_base.min_clr, res_prop.min_clr);
    fprintf('  %-38s | %-+18.4f m | %-+18.4f m\n', 'Minimum Road-Boundary Clearance', res_base.min_road_clr, res_prop.min_road_clr);
    fprintf('  %-38s | %-18d | %-18d\n', 'SafetyFilter Interventions', res_base.emergency_count, res_prop.emergency_count);
    fprintf('  %-38s | %-18.2f m | %-18.2f m\n', 'Total Trajectory Length', res_base.traj_length, res_prop.traj_length);
    fprintf('  %-38s | %-18.2f m/s | %-18.2f m/s\n', 'Average Travel Velocity', res_base.avg_v, res_prop.avg_v);
    fprintf('  %-38s | k=%-15s | k=%-15s\n', 'Bottleneck Entry Timestep (x=38m)', res_base.entry_k_str, res_prop.entry_k_str);
    fprintf('  %-38s | k=%-15s | k=%-15s\n', 'Bottleneck Exit Timestep (x=48m)', res_base.exit_k_str, res_prop.exit_k_str);
    fprintf('========================================================================================\n\n');
    
    % Evaluation Verdict
    if res_base.max_x < 38.0 && res_prop.max_x < 38.0
        fprintf('  => VERDICT: EXPERIMENT INVALID — Bottleneck not reached!\n');
    elseif res_prop.collision_steps == 0 && res_prop.boundary_violations == 0 && res_prop.max_x >= 48.0
        if res_base.boundary_violations > 0 || res_base.collision_steps > 0 || res_base.max_x < 48.0
            fprintf('  => SCIENTIFIC VERDICT: HYPOTHESIS SUPPORTED!\n');
            fprintf('     Fixed-lane baseline fails at bottleneck due to rigid target adherence,\n');
            fprintf('     while Proposed Free-Space successfully navigates the corridor!\n');
        else
            fprintf('  => SCIENTIFIC VERDICT: HYPOTHESIS NOT DEMONSTRATED (Both succeeded).\n');
        end
    else
        fprintf('  => SCIENTIFIC VERDICT: HYPOTHESIS NOT DEMONSTRATED (Proposed failed).\n');
    end
    fprintf('========================================================================================\n\n');
end

function [res, trace] = run_bottleneck_sim(cfg, planner, bp, sf, gt_world, ref_path, vehicle, obs_model, map_obj, N_steps, dt)
    trace.ego_x = zeros(N_steps, 1);
    trace.ego_y = zeros(N_steps, 1);
    trace.ego_v = zeros(N_steps, 1);
    
    clr_list = zeros(N_steps, 1);
    road_clr_list = zeros(N_steps, 1);
    coll_list = false(N_steps, 1);
    bounds_list = false(N_steps, 1);
    emergency_count = 0;
    traj_dist = 0.0;
    half_W = cfg.vehicle_width / 2.0; % 0.90m
    
    entry_k = -1; exit_k = -1;
    eval_world_true = ScenarioDefinitions('indian_unstructured_bottleneck', cfg);
    
    for k = 1:N_steps
        [obs_world, ~] = obs_model.observe(gt_world, dt);
        
        [u_mpc, pred_states, status, info] = planner.plan(obs_world, ref_path, 5.0);
        
        eval_world_true.ego = gt_world.ego;
        [u_cmd, filter_active, filter_reason] = sf.filter(u_mpc, status, eval_world_true, pred_states, bp);
        
        if filter_active
            emergency_count = emergency_count + 1;
        end
        
        prev_x = gt_world.ego.x; prev_y = gt_world.ego.y;
        gt_world.ego = vehicle.stepKinematic(gt_world.ego, u_cmd(2), u_cmd(1), dt);
        
        trace.ego_x(k) = gt_world.ego.x;
        trace.ego_y(k) = gt_world.ego.y;
        trace.ego_v(k) = gt_world.ego.v;
        
        if k > 1
            traj_dist = traj_dist + sqrt((trace.ego_x(k) - prev_x)^2 + (trace.ego_y(k) - prev_y)^2);
        end
        
        bounds_list(k) = map_obj.isDrivable(gt_world.ego.x, gt_world.ego.y);
        
        eval_world_true.ego = gt_world.ego;
        clr_list(k) = eval_world_true.getMinClearance(cfg);
        coll_list(k) = clr_list(k) <= 0;
        
        [y_road_lo, y_road_hi] = map_obj.getRoadBoundsAt(gt_world.ego.x);
        dist_to_lo = (gt_world.ego.y - half_W) - y_road_lo;
        dist_to_hi = y_road_hi - (gt_world.ego.y + half_W);
        road_clr_list(k) = min(dist_to_lo, dist_to_hi);
        
        if gt_world.ego.x >= 38.0 && entry_k == -1
            entry_k = k;
        end
        if gt_world.ego.x >= 48.0 && exit_k == -1
            exit_k = k;
        end
        
        gt_world = gt_world.stepAgents(dt);
    end
    
    res.total_steps = N_steps;
    res.max_x = max(trace.ego_x);
    res.bounds_steps = sum(bounds_list);
    res.boundary_violations = N_steps - sum(bounds_list);
    res.collision_steps = sum(coll_list);
    res.min_clr = min(clr_list);
    res.min_road_clr = min(road_clr_list);
    res.emergency_count = emergency_count;
    res.avg_v = mean(trace.ego_v);
    res.traj_length = traj_dist;
    
    if entry_k == -1, res.entry_k_str = 'NEVER_REACHED'; else res.entry_k_str = sprintf('%d', entry_k); end
    if exit_k == -1, res.exit_k_str = 'NEVER_REACHED'; else res.exit_k_str = sprintf('%d', exit_k); end
    
    if res.collision_steps > 0
        res.status_str = 'COLLISION';
    elseif res.boundary_violations > 0
        res.status_str = 'OUT_OF_BOUNDS';
    elseif res.max_x < 48.0
        res.status_str = 'STOPPED_EARLY';
    else
        res.status_str = 'COMPLETED';
    end
end
