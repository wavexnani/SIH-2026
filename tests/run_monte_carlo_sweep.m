function run_monte_carlo_sweep()
    % RUN_MONTE_CARLO_SWEEP Repeatable Multi-Tier Monte Carlo Evaluation
    %
    % Purpose:
    %   Evaluates frozen Stage 5.1 controller across 20 randomized scenarios spanning
    %   Level 1 (Nominal) to Level 4 (Infeasible) difficulty tiers.
    %   Logs 13 quantitative metrics and performs automated failure categorization.
    
    addpath('planning', 'config', 'core', 'environment', 'stages', 'tests', 'visualization');
    
    if ~exist('docs', 'dir'), mkdir('docs'); end
    
    fprintf('\n========================================================================================\n');
    fprintf('        PHASE 6 & 7: MONTE CARLO SCENARIO SWEEP & FAILURE CLASSIFICATION                 \n');
    fprintf('========================================================================================\n\n');
    
    rng(42); % Fixed random seed for strict reproducibility
    
    scenario_types = {'nominal_straight', 'gradual_narrowing', 'scurve_road', 'tight_corridor', 'infeasible_blocked'};
    N_scenarios = 20;
    
    sweep_results = cell(N_scenarios, 1);
    
    for s_idx = 1:N_scenarios
        scen_type = scenario_types{mod(s_idx - 1, length(scenario_types)) + 1};
        cfg = SimulationConfig();
        
        [world, map_obj, level_num, name_str] = ScenarioGenerator.createScenario(scen_type, cfg);
        
        % Randomize obstacle lateral/longitudinal positions slightly for variability
        if world.n_static_obs > 0 && world.static_obs(1, 1) > 0
            world.static_obs(1, 1) = world.static_obs(1, 1) + (rand() - 0.5) * 4.0;
            world.static_obs(1, 2) = world.static_obs(1, 2) + (rand() - 0.5) * 0.4;
        end
        
        res = evaluate_scenario_run(s_idx, scen_type, level_num, name_str, world, map_obj, cfg);
        sweep_results{s_idx} = res;
    end
    
    % Print Quantitative Summary Table
    fprintf('\n========================================================================================\n');
    fprintf('                  MONTE CARLO SWEEP QUANTITATIVE RESULTS SUMMARY                        \n');
    fprintf('========================================================================================\n');
    fprintf('| ID | Difficulty Level | Scenario Type      | Outcome | Coll. | Bounds Viol. | Min Clr (m) | Emergency Steps |\n');
    fprintf('|----|------------------|--------------------|---------|-------|--------------|-------------|-----------------|\n');
    for i = 1:N_scenarios
        r = sweep_results{i};
        fprintf('| %-2d | Level %-10d | %-18s | %-7s | %-5d | %-12d | %-+11.4f | %-15d |\n', ...
            r.id, r.level, r.scen_type, r.outcome, r.collisions, r.bounds_violations, r.min_clr, r.emergency_steps);
    end
    fprintf('========================================================================================\n\n');
    
    % Export docs/evaluation_results.md and docs/failure_analysis.md
    export_evaluation_docs(sweep_results);
end

function res = evaluate_scenario_run(id, scen_type, level, name_str, world, map_obj, cfg)
    vehicle = BicycleModel(cfg);
    ctrl = Stage5CoordinationController(cfg);
    ctrl.reset();
    ctrl.cacrc_planner.bound_provider = FreeSpaceBoundProvider(map_obj, 1.05);
    ctrl.setOvertakeEnabled(true);
    
    N_steps = 150;
    dt = cfg.dt;
    
    N_path = 500;
    ref_path = zeros(N_path, 5);
    ref_path(:, 1) = linspace(0, 150, N_path)';
    for r = 1:N_path
        [y_min_r, y_max_r] = map_obj.getRoadBoundsAt(ref_path(r, 1));
        ref_path(r, 2) = 0.5 * (y_min_r + y_max_r);
    end
    for r = 1:N_path
        r1 = max(1, r - 1); r2 = min(N_path, r + 1);
        ref_path(r, 3) = atan2(ref_path(r2, 2) - ref_path(r1, 2), ref_path(r2, 1) - ref_path(r1, 1));
    end
    ref_path(:, 4) = 0.0; ref_path(:, 5) = 8.0;
    
    drivable_mask = true(N_steps, 1);
    clr_list = zeros(N_steps, 1);
    coll_list = false(N_steps, 1);
    emergency_count = 0;
    hard_qp_count = 0;
    soft_qp_count = 0;
    infeasibility_count = 0;
    solve_times = zeros(N_steps, 1);
    ego_x_hist = zeros(N_steps, 1);
    ego_y_hist = zeros(N_steps, 1);
    
    for k = 1:N_steps
        t0 = tic;
        [u_cmd, ~, status, info] = ctrl.step(world, ref_path, 8.0);
        solve_times(k) = toc(t0) * 1000;
        
        if status == 0
            emergency_count = emergency_count + 1;
            infeasibility_count = infeasibility_count + 1;
        else
            if info.is_soft
                soft_qp_count = soft_qp_count + 1;
            else
                hard_qp_count = hard_qp_count + 1;
            end
        end
        
        world.ego = vehicle.stepKinematic(world.ego, u_cmd(2), u_cmd(1), dt);
        
        ego_x_hist(k) = world.ego.x;
        ego_y_hist(k) = world.ego.y;
        
        drivable_mask(k) = map_obj.isDrivable(world.ego.x, world.ego.y);
        clr_list(k) = world.getMinClearance(cfg);
        coll_list(k) = clr_list(k) <= 0;
    end
    
    res.id = id;
    res.scen_type = scen_type;
    res.level = level;
    res.name = name_str;
    res.bounds_violations = sum(~drivable_mask);
    res.collisions = sum(coll_list);
    res.min_clr = min(clr_list);
    res.emergency_steps = emergency_count;
    res.hard_qp = hard_qp_count;
    res.soft_qp = soft_qp_count;
    res.infeasibility = infeasibility_count;
    res.avg_solve_time = mean(solve_times);
    res.dist_traveled = ego_x_hist(end) - ego_x_hist(1);
    res.max_lat_dev = max(abs(ego_y_hist - 3.0));
    
    % Failure Categorization Protocol
    if res.collisions > 0
        res.outcome = 'FAIL';
        res.failure_category = 'obstacle_conflict';
    elseif res.bounds_violations > 0
        res.outcome = 'FAIL';
        res.failure_category = 'drivable_space_breach';
    elseif level == 4 && res.emergency_steps > 0
        res.outcome = 'EXPECTED_FAIL';
        res.failure_category = 'scenario_genuinely_infeasible';
    elseif res.emergency_steps > 0
        res.outcome = 'FAIL';
        res.failure_category = 'safety_filter_emergency_intervention';
    else
        res.outcome = 'PASS';
        res.failure_category = 'none';
    end
end

function export_evaluation_docs(sweep_results)
    fid_eval = fopen('docs/evaluation_results.md', 'w');
    fprintf(fid_eval, '# Monte Carlo Quantitative Evaluation Results (Phase 6)\n\n');
    fprintf(fid_eval, '| ID | Tier | Scenario Type | Outcome | Collisions | Bounds Viol. | Min Clearance (m) | Emergency Steps | Hard QP | Soft QP | Avg Solve Time (ms) |\n');
    fprintf(fid_eval, '|---|---|---|---|---|---|---|---|---|---|---|\n');
    for i = 1:length(sweep_results)
        r = sweep_results{i};
        fprintf(fid_eval, '| %d | Level %d | %s | %s | %d | %d | %+.4f | %d | %d | %d | %.2f |\n', ...
            r.id, r.level, r.scen_type, r.outcome, r.collisions, r.bounds_violations, r.min_clr, ...
            r.emergency_steps, r.hard_qp, r.soft_qp, r.avg_solve_time);
    end
    fclose(fid_eval);
    
    fid_fail = fopen('docs/failure_analysis.md', 'w');
    fprintf(fid_fail, '# Systematic Failure Analysis (Phase 7)\n\n');
    fprintf(fid_fail, '## Summary of Evaluated Scenario Failures\n\n');
    fprintf(fid_fail, '| ID | Level | Scenario | Category | Description & System Diagnosis |\n');
    fprintf(fid_fail, '|---|---|---|---|---|\n');
    for i = 1:length(sweep_results)
        r = sweep_results{i};
        if ~strcmp(r.outcome, 'PASS')
            fprintf(fid_fail, '| %d | Level %d | %s | %s | %s |\n', ...
                r.id, r.level, r.scen_type, r.failure_category, ...
                get_failure_description(r));
        end
    end
    fclose(fid_fail);
    
    fprintf('[SUCCESS] Exported docs/evaluation_results.md and docs/failure_analysis.md\n');
end

function desc = get_failure_description(r)
    if strcmp(r.failure_category, 'scenario_genuinely_infeasible')
        desc = 'Level 4 total road blockage. Planner and SafetyFilter correctly identified physical impossibility and executed emergency stopping.';
    elseif strcmp(r.failure_category, 'safety_filter_emergency_intervention')
        desc = 'Constrained road constriction caused temporary QP reachability infeasibility, triggering Layer 2 emergency deceleration.';
    else
        desc = sprintf('Metric breach detected: %s', r.failure_category);
    end
end
